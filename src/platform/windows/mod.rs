// Copyright 2015 The Servo Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

use std::cell::RefCell;
use std::cmp::PartialEq;
use std::default::Default;
use std::ffi::CString;
use std::io::{Error, ErrorKind};
use std::marker::{Send, Sync};
use std::mem;
use std::ops::Deref;
use std::env;
use std::ptr;
use std::slice;

use libc::{intptr_t, memcmp};

use uuid::Uuid;

use serde;
use bincode;
use bincode::serde::DeserializeError;

use winapi;
use winapi::{HANDLE, INVALID_HANDLE_VALUE, LPVOID};
use kernel32;

lazy_static! {
    static ref DD_ENABLED: bool = match env::var_os("DD") {
        Some(_) => true,
        None => false,
    };
    static ref DD2_ENABLED: bool = match env::var_os("DD2") {
        Some(_) => true,
        None => false,
    };
}

macro_rules! dd { ($($rest:tt)*) => { if *DD_ENABLED { println!($($rest)*); } } }
macro_rules! dd2 { ($($rest:tt)*) => { if *DD_ENABLED || *DD2_ENABLED { println!($($rest)*); } } }

// We stick these in WinErrors to indicate various things
// that aren't quite errors.
const MAGIC_CODE_BASE: u32 = 0xdeadbee0u32;
const MAGIC_NO_OUTSTANDING_DATA_CODE: u32 = MAGIC_CODE_BASE + 0;
const MAGIC_CHANNEL_CLOSED_CODE: u32 = MAGIC_CODE_BASE + 1;

// When we create the pipe, how big of a write buffer do we specify?
// This is reserved in the nonpaged pool.  The fragment size is the
// max we can write to the pipe without fragmentation, and the
// buffer size is what we tell the pipe it is, so we have room
// for out of band data etc.  We can probably do way less than
// 16k extra.
const MAX_FRAGMENT_SIZE: usize = 64 * 1024;
const WRITE_BUFFER_SIZE: usize = MAX_FRAGMENT_SIZE + 16*1024;

// When we read, how big of a heap buffer do we allocate?
const READ_BUFFER_SIZE: usize = 128 * 1024; //128k;
const READ_BUFFER_MAX_GROWTH: usize = 1 * 1024 * 1024; // 1MB max growth

// The message header is 2xu32 -- first one is the length of the data,
// second one is the length of any out of band data.  If the first
// field is INVALID_HEADER_DATA_SIZE, then the second one signifies
// a control code (currently only MAGIC_CHANNEL_CLOSED_CODE is used)
const HEADER_SIZE: usize = 8; //mem::size_of::<u32>()*2;
const INVALID_HEADER_DATA_SIZE: u32 = 0xffffffffu32;

#[allow(non_snake_case)]
fn GetLastError() -> u32 {
    unsafe {
        kernel32::GetLastError()
    }
}

pub fn channel() -> Result<(OsIpcSender, OsIpcReceiver),WinError> {
    let mut receiver = try!(OsIpcReceiver::new());
    let sender = try!(receiver.sender());
    Ok((sender, receiver))
}

// If we have any channel handles or shmem segments,
// then we'll send an OsIpcOutOfBandMessage after
// the data message.
#[derive(Debug)]
struct OsIpcOutOfBandMessage {
    channel_is_sender: Vec<bool>,
    channel_handles: Vec<intptr_t>,
    shmem_sizes: Vec<u64>,
    shmem_handles: Vec<intptr_t>,
    big_data_receiver_handle: intptr_t,
}

impl OsIpcOutOfBandMessage {
    fn new() -> OsIpcOutOfBandMessage {
        OsIpcOutOfBandMessage {
            channel_is_sender: vec![],
            channel_handles: vec![],
            shmem_sizes: vec![],
            shmem_handles: vec![],
            big_data_receiver_handle: 0,
        }
    }

    fn needs_to_be_sent(&self) -> bool {
        self.channel_handles.len() > 0 ||
        self.shmem_handles.len() > 0 ||
        self.big_data_receiver_handle != 0
    }

    fn add_sender(&mut self, handle: HANDLE) {
        self.channel_is_sender.push(true);
        self.channel_handles.push(handle as intptr_t);
    }

    fn add_receiver(&mut self, handle: HANDLE) {
        self.channel_is_sender.push(false);
        self.channel_handles.push(handle as intptr_t);
    }

    fn add_shmem(&mut self, handle: HANDLE, size: usize) {
        self.shmem_sizes.push(size as u64);
        self.shmem_handles.push(handle as intptr_t);
    }

    fn set_big_data_receiver_handle(&mut self, handle: HANDLE) {
        self.big_data_receiver_handle = handle as intptr_t;
    }
}

impl serde::Serialize for OsIpcOutOfBandMessage {
    fn serialize<S>(&self, serializer: &mut S) -> Result<(), S::Error>
        where S: serde::Serializer
    {
        ((&self.channel_is_sender, &self.channel_handles,
          &self.shmem_sizes, &self.shmem_handles,
          &self.big_data_receiver_handle)).serialize(serializer)
    }
}

impl serde::Deserialize for OsIpcOutOfBandMessage {
    fn deserialize<D>(deserializer: &mut D) -> Result<OsIpcOutOfBandMessage, D::Error>
        where D: serde::Deserializer
    {
        let (channel_is_sender, channel_handles,
             shmem_sizes, shmem_handles, big_data_receiver_handle) =
            try!(serde::Deserialize::deserialize(deserializer));
        Ok(OsIpcOutOfBandMessage {
            channel_is_sender: channel_is_sender, channel_handles: channel_handles,
            shmem_sizes: shmem_sizes, shmem_handles: shmem_handles,
            big_data_receiver_handle: big_data_receiver_handle
        })
    }
}

fn make_pipe_id() -> Uuid {
    Uuid::new_v4()
}

fn make_pipe_name(pipe_id: &Uuid) -> CString {
    CString::new(format!("\\\\.\\pipe\\rust-ipc-{}", pipe_id.to_string())).unwrap()
}

// Duplicate a given handle from this process to the target one, passing the
// given flags to DuplicateHandle.
//
// Unlike win32 DuplicateHandle, this will preserve INVALID_HANDLE_VALUE (which is
// also the pseudohandle for the current process).
#[allow(dead_code)]
fn dup_handle_to_process_with_flags(handle: HANDLE, other_process: HANDLE, flags: winapi::DWORD)
                                    -> Result<HANDLE,WinError>
{
    if handle == INVALID_HANDLE_VALUE {
        return Ok(INVALID_HANDLE_VALUE);
    }

    unsafe {
        let mut new_handle: HANDLE = INVALID_HANDLE_VALUE;
        let ok = kernel32::DuplicateHandle(kernel32::GetCurrentProcess(), handle,
                                           other_process, &mut new_handle,
                                           0, winapi::FALSE, flags);
        if ok == winapi::FALSE {
            Err(WinError::last("DuplicateHandle"))
        } else {
            Ok(new_handle)
        }
    }
}

// duplicate a handle in the current process
fn dup_handle(handle: HANDLE) -> Result<HANDLE,WinError> {
    dup_handle_to_process(handle, unsafe { kernel32::GetCurrentProcess() })
}

// duplicate a handle to the target process
fn dup_handle_to_process(handle: HANDLE, other_process: HANDLE) -> Result<HANDLE,WinError> {
    dup_handle_to_process_with_flags(handle, other_process, winapi::DUPLICATE_SAME_ACCESS)
}

// duplicate a handle to the target process, closing the source handle
fn move_handle_to_process(handle: HANDLE, other_process: HANDLE) -> Result<HANDLE,WinError> {
    dup_handle_to_process_with_flags(handle, other_process, winapi::DUPLICATE_CLOSE_SOURCE | winapi::DUPLICATE_SAME_ACCESS)
}

// get the current process handle as a handle in another process
#[allow(dead_code)]
fn current_process_handle_to_process(other_process: HANDLE) -> Result<HANDLE,WinError> {
    unsafe {
        let mut new_handle: HANDLE = INVALID_HANDLE_VALUE;
        let ok = kernel32::DuplicateHandle(kernel32::GetCurrentProcess(), kernel32::GetCurrentProcess(),
                                           other_process, &mut new_handle,
                                           winapi::PROCESS_DUP_HANDLE, winapi::FALSE, 0);
        if ok == winapi::FALSE {
            Err(WinError::last("DuplicateHandle"))
        } else {
            Ok(new_handle)
        }
    }
}

fn create_overlapped() -> Box<winapi::OVERLAPPED> {
    Box::new(winapi::OVERLAPPED {
        Internal: 0,
        InternalHigh: 0,
        Offset: 0,
        OffsetHigh: 0,
        hEvent: 0 as HANDLE
    })
}

fn reset_overlapped(ov: &mut winapi::OVERLAPPED) {
    ov.Internal = 0;
    ov.InternalHigh = 0;
    ov.Offset = 0;
    ov.OffsetHigh = 0;
    ov.hEvent = 0 as HANDLE;
}


#[derive(Debug, PartialEq)]
struct WinHandle {
    h: HANDLE
}

unsafe impl Send for WinHandle { }
unsafe impl Sync for WinHandle { }

impl Drop for WinHandle {
    fn drop(&mut self) {
        self.close();
    }
}

impl Default for WinHandle {
    fn default() -> WinHandle {
        WinHandle { h: INVALID_HANDLE_VALUE }
    }
}

impl Deref for WinHandle {
    type Target = HANDLE;

    #[inline]
    fn deref(&self) -> &HANDLE {
        &self.h
    }
}

impl WinHandle {
    fn new(h: HANDLE) -> WinHandle {
        WinHandle { h: h }
    }

    fn invalid() -> WinHandle {
        WinHandle { h: INVALID_HANDLE_VALUE }
    }

    fn is_valid(&self) -> bool {
        self.h != INVALID_HANDLE_VALUE
    }

    fn close(&mut self) {
        unsafe {
            dd!("WinHandle drop: {:?}", self.h);
            let h = self.take();
            kernel32::CloseHandle(h);
        }
    }

    fn take(&mut self) -> HANDLE {
        mem::replace(&mut self.h, INVALID_HANDLE_VALUE)
    }
}

// MessageReader implements blocking/nonblocking reads of messages
// from the handle
#[derive(Debug)]
struct MessageReader {
    // The reader handle; same as the one on OsIpcReceiver, here for
    // convenience
    handle: HANDLE,

    // The OVERLAPPED struct for async IO on this receiver; we'll only
    // ever have one in flight
    ov: Box<winapi::OVERLAPPED>,

    // A read buffer for any pending reads
    read_buf: Vec<u8>,

    // If we have already issued an async read
    read_in_progress: bool,

    // If we received a BROKEN_PIPE or other error
    // indicating that the remote end has closed the pipe
    closed: bool,

    // this is ONLY here for debugging purposes.
    // (for printfs so we know which iocp this is part of)
    iocp: HANDLE,
}

impl MessageReader {
    fn new(handle: HANDLE) -> MessageReader {
        MessageReader {
            handle: handle,
            ov: create_overlapped(),
            read_buf: Vec::with_capacity(READ_BUFFER_SIZE),
            read_in_progress: false,
            closed: false,
            iocp: 0 as HANDLE,
        }
    }

    fn ov_ptr(&self) -> *mut winapi::OVERLAPPED {
        self.ov.deref() as *const _ as *mut winapi::OVERLAPPED
    }

    fn reset_overlapped(&mut self) {
        reset_overlapped(&mut self.ov);
    }

    fn cancel_io(&mut self) {
        unsafe {
            if self.read_in_progress {
                kernel32::CancelIoEx(self.handle, self.ov_ptr());
                self.read_in_progress = false;
            }
        }
    }

    fn ensure_is_reading(&mut self) -> Result<(),WinError> {
        self.start_read()
    }

    // Called when we receive an IO Completion Packet for this handle.
    fn notify_completion(&mut self, err: u32) -> Result<(),WinError> {
        dd2!("[$ {:?}:{:?}] notify_completion", self.iocp, self.handle);

        if err == winapi::ERROR_BROKEN_PIPE {
            assert!(!self.closed, "we shouldn't get an async BROKEN_PIPE after we already got one");
            self.closed = true;
            return Ok(());
        }

        let nbytes = self.ov.InternalHigh as u32;
        let offset = self.ov.Offset;

        assert!(offset == 0);

        // if the remote end closed...
        if err != winapi::ERROR_SUCCESS {
            panic!("[$ {:?}:{:?}] *** notify_completion: need to handle error! {}", self.iocp, self.handle, err);
        }

        unsafe {
            let new_size = self.read_buf.len() + nbytes as usize;
            dd!("nbytes: {}, offset {}, buf len {}->{}, capacity {}",
                nbytes, offset, self.read_buf.len(), new_size, self.read_buf.capacity());
            assert!(new_size <= self.read_buf.capacity());
            self.read_buf.set_len(new_size);
        }

        Ok(())
    }

    // kick off an asynchronous read
    fn start_read(&mut self) -> Result<(),WinError> {
        if self.read_in_progress || self.closed {
            return Ok(());
        }

        dd2!("[$ {:?}:{:?}] start_read ov {:?}", self.iocp, self.handle, self.ov_ptr());
        let mut bytes_read: u32 = 0;

        // if the buffer is full, add more space
        let buf_len = self.read_buf.len();
        let mut buf_cap = self.read_buf.capacity();
        if buf_cap == buf_len {
            let more =
                if buf_cap == 0 { READ_BUFFER_SIZE }
            else if buf_cap < READ_BUFFER_MAX_GROWTH { buf_cap }
            else { READ_BUFFER_MAX_GROWTH };
            self.read_buf.reserve(more);
            buf_cap = self.read_buf.capacity();
        }

        // issue the read to the buffer, at the current length offset
        unsafe {
            self.reset_overlapped();
            let buf_ptr = self.read_buf.as_mut_ptr() as LPVOID;
            let max_read_bytes = buf_cap - buf_len;
            let ok = kernel32::ReadFile(self.handle,
                                        buf_ptr.offset(buf_len as isize),
                                        max_read_bytes as u32,
                                        &mut bytes_read,
                                        self.ov_ptr());

            // ReadFile can return TRUE; if it does, an IO completion
            // packet is still posted to any port, and the OVERLAPPED
            // structure has the IO operation flagged as complete.
            if ok == winapi::FALSE {
                let err = GetLastError();
                if err == winapi::ERROR_BROKEN_PIPE {
                    dd!("[$ {:?}:{:?}] BROKEN_PIPE straight from ReadFile", self.iocp, self.handle);
                    self.closed = true;
                    return Ok(());
                }

                if err == winapi::ERROR_IO_PENDING {
                    self.read_in_progress = true;
                    return Ok(());
                }

                Err(WinError::last("ReadFile"))
            } else {
                self.read_in_progress = true;
                Ok(())
            }
        }
    }

    fn message_length(&self) -> Option<(usize,usize)> {
        dd!("[$ {:?}:{:?}] message_length read_buf: {} bytes", self.iocp, self.handle, self.read_buf.len());
        if self.read_buf.len() < HEADER_SIZE {
            return None;
        }

        let buf_header: &[u32] = unsafe { slice::from_raw_parts(self.read_buf.as_ptr() as *const u32, 2) };
        let data_bytes = buf_header[0] as usize;
        let oob_bytes = buf_header[1] as usize;
        let bytes_needed = HEADER_SIZE + data_bytes + oob_bytes;
        if self.read_buf.len() >= bytes_needed {
            Some((data_bytes, oob_bytes))
        } else {
            None
        }
    }

    // Err(false) -> something really failed
    // Err(true) -> no message
    // XXX This is dumb, we should return
    //   Result<Option<(...)>,WinError>
    fn get_message(&mut self) -> Result<(Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>),bool> {
        let message_lengths = self.message_length();
        if message_lengths.is_none() {
            return Err(false)
        }
        let (data_bytes, oob_bytes) = message_lengths.unwrap();

        let bytes_needed = HEADER_SIZE + data_bytes as usize + oob_bytes as usize;

        // remove this message's bytes from read_buf, or just take read_buf
        // if it contains exactly one message
        //dd!("[$ {:?}:{:?}] rb {:?}", self.iocp, self.handle, self.read_buf);
        let msg_buf = if self.read_buf.len() == bytes_needed {
            mem::replace(&mut self.read_buf, Vec::with_capacity(READ_BUFFER_SIZE))
        } else {
            self.read_buf.drain(0..bytes_needed).collect::<Vec<u8>>()
        };
        dd!("[$ {:?}:{:?}] consumed {} bytes, left {}", self.iocp, self.handle, bytes_needed, self.read_buf.len());

        // We have a full message available.
        let data_end = HEADER_SIZE + data_bytes as usize;
        let mut buf_data = msg_buf[HEADER_SIZE..data_end].to_vec();

        let mut channels: Vec<OsOpaqueIpcChannel> = vec![];
        let mut shmems: Vec<OsIpcSharedMemory> = vec![];

        if oob_bytes > 0 {
            let buf_oob = &msg_buf[data_end..];
            let msg = bincode::serde::deserialize::<OsIpcOutOfBandMessage>(&buf_oob).unwrap();

            assert!(msg.channel_is_sender.len() == msg.channel_handles.len());
            assert!(msg.shmem_sizes.len() == msg.shmem_handles.len());

            for (handle, is_sender) in msg.channel_handles.iter().zip(msg.channel_is_sender.iter()) {
                channels.push(OsOpaqueIpcChannel::new(*is_sender, *handle as HANDLE));
            }

            for (handle, size) in msg.shmem_handles.iter().zip(msg.shmem_sizes.iter()) {
                shmems.push(OsIpcSharedMemory::from_handle(*handle as HANDLE, *size as usize).unwrap());
            }

            if msg.big_data_receiver_handle != 0 {
                let receiver = OsIpcReceiver::from_handle(msg.big_data_receiver_handle as HANDLE);
                let big_msg = try!(receiver.recv().map_err(|_| panic!("Failed to receive subchannel big data")));
                buf_data = big_msg.0;
            }
        }

        dd!("[$ {:?}:{:?}] get_message success -> {} bytes, {} channels, {} shmems",
            self.iocp, self.handle, buf_data.len(), channels.len(), shmems.len());
        //dd!("[$ {:?}:{:?}] bd {:?}", self.iocp, self.handle, buf_data);
        Ok((buf_data, channels, shmems))
    }
}

#[derive(Debug)]
pub struct OsIpcReceiver {
    // The handle to the receiver end of a pipe
    handle: WinHandle,

    // The handle to the sender end of a pipe, if it hasn't been taken
    // yet; None if it has.
    sender: Option<OsIpcSender>,

    // The IOCP that this is a part of.  A bare HANDLE, since we don't
    // own it.
    iocp: HANDLE,

    // A pipe_id; if None, then this isn't a named
    // pipe server (or is no longer one), and accept() will fail
    pipe_id: Option<Uuid>,

    // A MessageReader that's used to read from this OsIpcRecever.
    // XXX can this be an UnsafeCell?
    reader: RefCell<MessageReader>,
}

unsafe impl Send for OsIpcReceiver { }
unsafe impl Sync for OsIpcReceiver { }

impl PartialEq for OsIpcReceiver {
    fn eq(&self, other: &OsIpcReceiver) -> bool {
        self.handle == other.handle
        // XXX should check others? but handle
        // equivalency should be enough..
    }
}

impl Drop for OsIpcReceiver {
    fn drop(&mut self) {
        dd!("[$ {:?}:{:?}] OsIpcReceiver drop", self.iocp, *self.handle);
        self.reader.borrow_mut().cancel_io();
    }
}

impl OsIpcReceiver {
    fn new() -> Result<OsIpcReceiver,WinError> {
        let mut r = try!(OsIpcReceiver::new_named());
        let pipe_id = mem::replace(&mut r.pipe_id, None).unwrap();
        r.sender = Some(try!(OsIpcSender::connect_pipe_id(pipe_id)));
        Ok(r)
    }

    fn from_handle(handle: HANDLE) -> OsIpcReceiver {
        OsIpcReceiver {
            handle: WinHandle::new(handle),
            sender: None,
            iocp: INVALID_HANDLE_VALUE,
            pipe_id: None,
            reader: RefCell::new(MessageReader::new(handle)),
        }
    }

    fn new_named() -> Result<OsIpcReceiver,WinError> {
        unsafe {
            let pipe_id = make_pipe_id();
            let pipe_name = make_pipe_name(&pipe_id);

            // create the pipe server
            let handle =
                kernel32::CreateNamedPipeA(pipe_name.as_ptr(),
                                           winapi::PIPE_ACCESS_INBOUND | winapi::FILE_FLAG_OVERLAPPED,
                                           winapi::PIPE_TYPE_BYTE | winapi::PIPE_READMODE_BYTE | winapi::PIPE_REJECT_REMOTE_CLIENTS,
                                           // 1 max instance of this pipe
                                           1,
                                           // out/in buffer sizes
                                           0, WRITE_BUFFER_SIZE as u32,
                                           0, // default timeout for WaitNamedPipe (0 == 50ms as default)
                                           ptr::null_mut());
            if handle == INVALID_HANDLE_VALUE {
                return Err(WinError::last("CreateNamedPipeA"));
            }

            Ok(OsIpcReceiver {
                handle: WinHandle::new(handle),
                sender: None,
                iocp: INVALID_HANDLE_VALUE,
                pipe_id: Some(pipe_id),
                reader: RefCell::new(MessageReader::new(handle)),
            })
        }
    }

    fn sender(&mut self) -> Result<OsIpcSender,WinError> {
        Ok(mem::replace(&mut self.sender, None).unwrap())
    }

    fn is_closed(&self) -> bool {
        self.reader.borrow().closed
    }

    fn prepare_for_transfer(&mut self) -> Result<bool,WinError> {
        let mut reader = self.reader.borrow_mut();
        // cancel any outstanding IO request
        reader.cancel_io();
        // this is only okay if we have nothing in the read buf
        Ok(reader.read_buf.len() == 0)
    }

    pub fn consume(&self) -> OsIpcReceiver {
        let handle = dup_handle(*self.handle).unwrap();
        OsIpcReceiver::from_handle(handle)
    }

    fn receive_message(&self, block: bool)
                       -> Result<(Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>),WinError> {
        // This is only used for recv/try_recv.  When this is added to an IpcReceiverSet, then
        // the implementation in select() is used.  It does much the same thing, but across multiple
        // channels.
        assert!(self.iocp == INVALID_HANDLE_VALUE, "receive_message is only valid before this OsIpcReceiver was added to a Set");

        // This function loops, because in the case of a blocking read, we may need to
        // read multiple sets of bytes from the pipe to receive a complete message.
        unsafe {
            let mut reader = self.reader.borrow_mut();
            loop {
                // First, try to fetch a message, in case we have one in the pipe
                if let Ok(msg) = reader.get_message() {
                    return Ok(msg);
                }

                // If the pipe was already closed, we're done -- we've
                // already drained all incoming bytes
                if reader.closed {
                    return Err(WinError::ok_channel_closed());
                }

                // Then, issue a read if we don't have one already in flight.
                // We must not issue a read if we have unconsumed messages,
                // because getting a message modifies the read_buf.
                //
                // XXX is there some rust-y safe way to do this?  I'd like to
                // transfer ownership of the read buffer to something that can
                // either consume a message, from the buffer, or have it in flight,
                // but not both.
                try!(reader.start_read());

                // If the last read flagged us closed we're done; we've already
                // drained all incoming bytes earlier in the loop.
                if reader.closed {
                    return Err(WinError::ok_channel_closed());
                }

                // Then, get the overlapped result, blocking if we need to.
                let mut nbytes: u32 = 0;
                let mut err = winapi::ERROR_SUCCESS;
                let ok = kernel32::GetOverlappedResult(reader.handle, reader.ov_ptr(), &mut nbytes,
                                                       if block { winapi::TRUE } else { winapi::FALSE });
                if ok == winapi::FALSE {
                    err = GetLastError();
                    if !block && err == winapi::ERROR_IO_INCOMPLETE {
                        // Nonblocking read, no message, read's in flight, we're
                        // done.
                        return Err(WinError::ok_no_data());
                    }
                    // We pass err through to notify_completion so
                    // that it can handle other errors.
                }

                // The IO completed, so the read is no longer in progress
                reader.read_in_progress = false;

                // Notify that the read completed, which will update the
                // read pointers
                try!(reader.notify_completion(err));

                // If we're blocking, keep looping around until we have a complete message.
                // If we're not blocking, keep looping until GetOverlappedResult gives us
                // IO_INCOMPLETE.
            }
        }
    }

    pub fn recv(&self)
                -> Result<(Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>),WinError> {
        dd!("recv");
        self.receive_message(true)
    }

    pub fn try_recv(&self)
                    -> Result<(Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>),WinError> {
        dd!("try_recv");
        self.receive_message(false)
    }

    fn add_to_iocp(&mut self, iocp: HANDLE) -> Result<(),WinError> {
        unsafe {
            assert!(self.sender.is_none(), "Adding to OsIpcReceiverSet, but sender was never taken!");
            assert!(self.iocp == INVALID_HANDLE_VALUE);

            let ret = kernel32::CreateIoCompletionPort(*self.handle,
                                                       iocp,
                                                       *self.handle as winapi::ULONG_PTR,
                                                       0);
            if ret == ptr::null_mut() {
                return Err(WinError::last("CreateIoCompletionPort"));
            }

            self.iocp = iocp;

            {
                let mut reader = self.reader.borrow_mut();
                // make sure that the reader has a read in flight,
                // otherwise a later select() will hang
                try!(reader.ensure_is_reading());
                reader.iocp = iocp;
            }

            Ok(())
        }
    }

    // Do a pipe connect.  Only used for one-shot servers
    fn accept(&mut self) -> Result<(),WinError> {
        unsafe {
            let mut ov = create_overlapped();
            reset_overlapped(&mut ov);
            let ov_ptr = ov.deref() as *const _ as *mut winapi::OVERLAPPED;

            let ok = kernel32::ConnectNamedPipe(*self.handle, ov_ptr);

            // we should always get FALSE with async IO
            assert!(ok == winapi::FALSE);
            let err = GetLastError();

            // did we successfully connect? (it's reported as an error [ok==false])
            if err == winapi::ERROR_PIPE_CONNECTED {
                dd!("[$ {:?}:{:?}] accept (PIPE_CONNECTED)", self.iocp, *self.handle);
                return Ok(());
            }

            // This is a weird one -- if we create a named pipe (like we do
            // in new(), the client connects, sends data, then drops its handle,
            // a Connect here will get ERROR_NO_DATA -- but there may be data in
            // the pipe that we'll be able to read.  So we need to go do some reads
            // like normal and wait until ReadFile gives us ERROR_NO_DATA.
            if err == winapi::ERROR_NO_DATA {
                dd!("[$ {:?}:{:?}] accept (ERROR_NO_DATA)", self.iocp, *self.handle);
                return Ok(());
            }

            // was it an actual error?
            if err != winapi::ERROR_IO_PENDING {
                dd!("[$ {:?}:{:?}] accept error -> {}", self.iocp, *self.handle, err);
                return Err(WinError::last("ConnectNamedPipe"));
            }

            // The connect is pending; wait for it to complete
            let mut nbytes: u32 = 0;
            let ok = kernel32::GetOverlappedResult(*self.handle, ov_ptr, &mut nbytes, winapi::TRUE);
            if ok == winapi::FALSE {
                return Err(WinError::last("GetOverlappedResult[ConnectNamedPipe]"));
            }

            Ok(())
        }
    }
}

#[derive(Debug)]
pub struct OsIpcSender {
    // The client hande itself
    handle: WinHandle,
}

unsafe impl Send for OsIpcSender { }
unsafe impl Sync for OsIpcSender { }

impl PartialEq for OsIpcSender {
    fn eq(&self, other: &OsIpcSender) -> bool {
        self.handle == other.handle
    }
}

impl Clone for OsIpcSender {
    fn clone(&self) -> OsIpcSender {
        OsIpcSender::from_handle(dup_handle(*self.handle).unwrap())
    }
}

unsafe fn write_buf(handle: HANDLE, bytes: &[u8]) -> Result<(),WinError> {
    let mut ntowrite: u32 = bytes.len() as u32;
    if ntowrite == 0 {
        return Ok(());
    }
    let mut nwritten: u32 = 0;
    //dd!("[c {:?}] writing: {:?}", handle, bytes);
    while nwritten < ntowrite {
        let mut nwrote: u32 = 0;
        if kernel32::WriteFile(handle,
                               bytes.as_ptr().offset(nwritten as isize) as LPVOID,
                               ntowrite,
                               &mut nwrote,
                               ptr::null_mut())
            == winapi::FALSE
        {
            return Err(WinError::last("WriteFile"));
        }
        nwritten += nwrote;
        ntowrite -= nwrote;
        //dd!("[c {:?}] ... wrote {} bytes, total {}/{} err {}", handle, nwrote, nwritten, bytes.len(), GetLastError());
    }

    Ok(())
}

impl OsIpcSender {
    pub fn connect(name: String) -> Result<OsIpcSender,WinError> {
        OsIpcSender::connect_pipe_id(Uuid::parse_str(&name).unwrap())
    }

    pub fn get_max_fragment_size() -> usize {
        MAX_FRAGMENT_SIZE
    }

    fn from_handle(handle: HANDLE) -> OsIpcSender {
        OsIpcSender {
            handle: WinHandle::new(handle),
        }
    }

    // Connect to a pipe server
    fn connect_pipe_id(pipe_id: Uuid) -> Result<OsIpcSender,WinError> {
        unsafe {
            dd!("[c ----] connect_to_server {:?}", pipe_id);
            let pipe_name = make_pipe_name(&pipe_id);
            let handle =
                kernel32::CreateFileA(pipe_name.as_ptr(),
                                      winapi::GENERIC_WRITE,
                                      0,
                                      ptr::null_mut(), // lpSecurityAttributes
                                      winapi::OPEN_EXISTING,
                                      winapi::FILE_ATTRIBUTE_NORMAL,
                                      ptr::null_mut());
            if handle == INVALID_HANDLE_VALUE {
                return Err(WinError::last("CreateFileA"));
            }

            dd!("[c {:?}] connect_to_server success", handle);

            Ok(OsIpcSender {
                handle: WinHandle::new(handle)
            })
        }
    }

    fn get_pipe_server_process_handle(&self) -> Result<WinHandle,WinError> {
        unsafe {
            let mut server_pid: winapi::ULONG = 0;
            if kernel32::GetNamedPipeServerProcessId(*self.handle, &mut server_pid) == winapi::FALSE {
                return Err(WinError::last("GetNamedPipeServerProcessId"));
            }
            let raw_handle = kernel32::OpenProcess(winapi::PROCESS_DUP_HANDLE,
                                                   winapi::FALSE,
                                                   server_pid as winapi::DWORD);
            if raw_handle == ptr::null_mut() {
                return Err(WinError::last("OpenProcess"));
            }

            Ok(WinHandle::new(raw_handle))
        }
    }

    fn maximum_message_data_size(oob_data_len: u64) -> usize {
        assert!((oob_data_len as usize) < (WRITE_BUFFER_SIZE-HEADER_SIZE), "too much oob data");
        (WRITE_BUFFER_SIZE-HEADER_SIZE) - (oob_data_len as usize)
    }

    fn needs_fragmentation(data_len: usize, oob: &OsIpcOutOfBandMessage) -> bool {
        let oob_size = if oob.needs_to_be_sent() { bincode::serde::serialized_size(oob) } else { 0 };
        data_len > OsIpcSender::maximum_message_data_size(oob_size)
    }

    // An internal-use-only send method that sends just data, but doesn't
    // fragment -- we use this for the secondary channel fragmentation
    // creates.
    fn send_unfragmented(&self, data: &[u8]) -> Result<(),WinError> {
        assert!(data.len() < INVALID_HEADER_DATA_SIZE as usize);
        unsafe {
            let header: [u32; 2] = [ data.len() as u32, 0 ];
            let header_bytes: &[u8] = slice::from_raw_parts(header.as_ptr() as *const u8, HEADER_SIZE);
            try!(write_buf(*self.handle, &header_bytes));
            try!(write_buf(*self.handle, data));
        }
        Ok(())
    }

    pub fn send(&self,
                data: &[u8],
                ports: Vec<OsIpcChannel>,
                shared_memory_regions: Vec<OsIpcSharedMemory>)
                -> Result<(),WinError>
    {
        // We limit the max size we can send here; we can fix this
        // just by upping the header to be 2x u64 if we really want
        // to.
        assert!(data.len() < INVALID_HEADER_DATA_SIZE as usize);

        let server_process_handle =
            if ports.len() > 0 || shared_memory_regions.len() > 0 {
                try!(self.get_pipe_server_process_handle())
            } else {
                WinHandle::invalid()
            };

        let mut oob = OsIpcOutOfBandMessage::new();

        for ref shmem in shared_memory_regions {
            // shmem.handle, shmem.length
            let remote_handle = try!(dup_handle_to_process(*shmem.handle, *server_process_handle));
            oob.add_shmem(remote_handle, shmem.length);
        }

        for port in ports {
            match port {
                OsIpcChannel::Sender(mut s) => {
                    let raw_remote_handle = try!(move_handle_to_process(s.handle.take(), *server_process_handle));
                    oob.add_sender(raw_remote_handle);
                },
                OsIpcChannel::Receiver(mut r) => {
                    if try!(r.prepare_for_transfer()) == false {
                        panic!("Sending receiver with outstanding partial read buffer, noooooo!  What should even happen?");
                    }

                    let raw_remote_handle = try!(move_handle_to_process(r.handle.take(), *server_process_handle));
                    oob.add_receiver(raw_remote_handle);
                },
            }
        }

        // Do we need to fragment?
        let mut big_data_sender: Option<OsIpcSender> = None;
        if OsIpcSender::needs_fragmentation(data.len(), &oob) {
            // We need to create a channel for the big data
            let mut receiver = try!(OsIpcReceiver::new());
            big_data_sender = Some(try!(receiver.sender()));

            // Put the receiver in the OOB data
            let raw_receiver_handle = try!(move_handle_to_process(receiver.handle.take(), *server_process_handle));
            oob.set_big_data_receiver_handle(raw_receiver_handle);
        }

        // If we need to send OOB data, serialize it
        let mut oob_data: Vec<u8> = vec![];
        if oob.needs_to_be_sent() {
            oob_data = bincode::serde::serialize(&oob, bincode::SizeLimit::Infinite).unwrap();
        }

        unsafe {
            // if we need to use the fragment channel, then we send a 0 as the data size;
            // the receiver will see that there's a big data receiver in the OOB portion
            let header: [u32; 2] = [ if big_data_sender.is_some() { 0 } else { data.len() } as u32, oob_data.len() as u32];
            let header_bytes: &[u8] = slice::from_raw_parts(header.as_ptr() as *const u8, HEADER_SIZE);

            // We need to write the main message first, which will indicate to the receiver
            // that it should receive a message on the OOB big data channel.  If we don't
            // send this first we'll block
            try!(write_buf(*self.handle, &header_bytes));
            if big_data_sender.is_none() {
                try!(write_buf(*self.handle, data));
            }
            try!(write_buf(*self.handle, &oob_data));
            if big_data_sender.is_some() {
                try!(big_data_sender.unwrap().send_unfragmented(data));
            }
        }

        Ok(())
    }
}

pub struct OsIpcReceiverSet {
    // the IOCP that we select on
    iocp: WinHandle,

    // the set of receivers
    receivers: Vec<OsIpcReceiver>,
}

pub enum OsIpcSelectionResult {
    DataReceived(i64, Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>),
    ChannelClosed(i64),
}

impl OsIpcReceiverSet {
    pub fn new() -> Result<OsIpcReceiverSet,WinError> {
        unsafe {
            let iocp = kernel32::CreateIoCompletionPort(INVALID_HANDLE_VALUE,
                                                        ptr::null_mut(),
                                                        0 as winapi::ULONG_PTR,
                                                        0);
            if iocp == ptr::null_mut() {
                return Err(WinError::last("CreateIoCompletionPort"));
            }

            Ok(OsIpcReceiverSet {
                iocp: WinHandle::new(iocp),
                receivers: vec![],
            })
        }
    }

    pub fn add(&mut self, mut receiver: OsIpcReceiver) -> Result<i64,WinError> {
        // use this to identify the receiver
        let receiver_handle = *receiver.handle;

        // XXX we'll need a mutex here... at least while we loop through
        // receivers to find a matching handle when we get a IOCP
        try!(receiver.add_to_iocp(*self.iocp));
        self.receivers.push(receiver);

        dd!("[# {:?}] ReceiverSet add {:?}", *self.iocp, receiver_handle);

        Ok(receiver_handle as i64)
    }

    pub fn select(&mut self) -> Result<Vec<OsIpcSelectionResult>,WinError> {
        assert!(self.receivers.len() > 0, "selecting with no objects?");
        dd!("[# {:?}] select() with {} receivers", *self.iocp, self.receivers.len());

        unsafe {
            // the ultimate results
            let mut selection_results = Vec::new();

            // make a local var for this, because in the future
            // we'll take it from a mutex
            let ref mut receivers = &mut self.receivers;

            // Make a quick first-run check for any closed receivers.
            // This will only happen if we have a receiver that
            // gets added to the Set after it was closed (the
            // router_drops_callbacks_on_cloned_sender_shutdown test
            // causes this.)
            receivers.retain(|ref r| {
                if r.is_closed() {
                    selection_results.push(OsIpcSelectionResult::ChannelClosed(*r.handle as i64));
                    false
                } else {
                    true
                }
            });

            // if we had prematurely closed elements, just process them first
            if selection_results.len() > 0 {
                return Ok(selection_results);
            }

            // Do this in a loop, because we may need to dequeue multiple packets to
            // read a complete message.
            loop {
                let mut nbytes: u32 = 0;
                let mut completion_key: HANDLE = INVALID_HANDLE_VALUE;
                let mut ov_ptr: *mut winapi::OVERLAPPED = ptr::null_mut();
                // XXX use GetQueuedCompletionStatusEx to dequeue multiple CP at once!
                let ok = kernel32::GetQueuedCompletionStatus(*self.iocp,
                                                             &mut nbytes,
                                                             &mut completion_key as *mut _ as *mut u64,
                                                             &mut ov_ptr,
                                                             winapi::INFINITE);
                dd!("[# {:?}] GetQueuedCS -> ok:{} nbytes:{} key:{:?}", *self.iocp, ok, nbytes, completion_key);
                let mut io_err = winapi::ERROR_SUCCESS;
                if ok == winapi::FALSE {
                    // If the OVERLAPPED result is NULL, then the
                    // function call itself failed or timed out.
                    // Otherwise, the async IO operation failed, and
                    // we want to hand io_err to notify_completion below.
                    if ov_ptr == ptr::null_mut() {
                        return Err(WinError::last("GetQueuedCompletionStatus"));
                    }

                    io_err = GetLastError();
                }

                assert!(ov_ptr != ptr::null_mut());
                assert!(completion_key != INVALID_HANDLE_VALUE);

                // If we get notification that a channel was closed, we can remove it
                // from our array.
                let mut remove_index = receivers.len();

                for (index, ref mut receiver) in receivers.iter_mut().enumerate() {
                    // find the matching receiver
                    if completion_key != *receiver.handle {
                        continue;
                    }

                    dd!("[# {:?}] result for receiver {:?}", *self.iocp, *receiver.handle);

                    // XXX we need to put this inside the receiver API
                    let mut reader = receiver.reader.borrow_mut();

                    // The IO completed, so the read is no longer in progress
                    reader.read_in_progress = false;

                    // tell it about the completed IO op
                    try!(reader.notify_completion(io_err));

                    // then drain as many messages as we can
                    loop {
                        if let Ok(msg) = reader.get_message() {
                            dd!("[# {:?}] receiver {:?} got a message", *self.iocp, *receiver.handle);
                            selection_results.push(OsIpcSelectionResult::DataReceived(completion_key as i64, msg.0, msg.1, msg.2));
                        } else {
                            dd!("[# {:?}] receiver {:?} -- no message!", *self.iocp, *receiver.handle);
                            break;
                        }
                    }

                    // if it wasn't closed, kick off a read
                    try!(reader.start_read());

                    // we may have already been closed, or the read resulted in us being closed
                    if reader.closed {
                        dd!("[# {:?}] receiver {:?} -- now closed!", *self.iocp, *receiver.handle);
                        selection_results.push(OsIpcSelectionResult::ChannelClosed(completion_key as i64));
                        remove_index = index;
                    }

                    break;
                }

                // if we were told to remove this receiver, do so
                if remove_index < receivers.len() {
                    receivers.swap_remove(remove_index);
                }

                // if we didn't dequeue at least one complete message -- we need to loop through GetQueuedCS again;
                // otherwise we're done.
                if selection_results.len() > 0 {
                    break;
                }
            }

            dd!("select() -> {} results", selection_results.len());
            Ok(selection_results)
        }
    }
}

impl OsIpcSelectionResult {
    pub fn unwrap(self) -> (i64, Vec<u8>, Vec<OsOpaqueIpcChannel>, Vec<OsIpcSharedMemory>) {
        match self {
            OsIpcSelectionResult::DataReceived(id, data, channels, shared_memory_regions) => {
                (id, data, channels, shared_memory_regions)
            }
            OsIpcSelectionResult::ChannelClosed(id) => {
                panic!("OsIpcSelectionResult::unwrap(): receiver ID {} was closed!", id)
            }
        }
    }
}

#[derive(Debug)]
pub struct OsIpcSharedMemory {
    handle: WinHandle,
    ptr: *mut u8,
    length: usize,
}

unsafe impl Send for OsIpcSharedMemory {}
unsafe impl Sync for OsIpcSharedMemory {}

impl Drop for OsIpcSharedMemory {
    fn drop(&mut self) {
        unsafe {
            if self.handle.is_valid() {
                kernel32::UnmapViewOfFile(self.ptr as LPVOID);
            }
        }
    }
}

impl Clone for OsIpcSharedMemory {
    fn clone(&self) -> OsIpcSharedMemory {
        let handle = dup_handle(*self.handle).unwrap();
        OsIpcSharedMemory::from_handle(handle, self.length).unwrap()
    }
}

impl PartialEq for OsIpcSharedMemory {
    fn eq(&self, other: &OsIpcSharedMemory) -> bool {
        unsafe {
            self.length == other.length &&
            memcmp(self.ptr as *const _, other.ptr as *const _, self.length) == 0
        }
    }
}

impl Deref for OsIpcSharedMemory {
    type Target = [u8];

    #[inline]
    fn deref(&self) -> &[u8] {
        assert!(!self.ptr.is_null() && self.handle.is_valid());
        unsafe {
            slice::from_raw_parts(self.ptr, self.length)
        }
    }
}

impl OsIpcSharedMemory {
    fn new(length: usize) -> Result<OsIpcSharedMemory,WinError> {
        unsafe {
            let lhigh = (length >> 32) as u32;
            let llow = (length & 0xffffffffusize) as u32;
            let handle =
                kernel32::CreateFileMappingA(INVALID_HANDLE_VALUE,
                                             ptr::null_mut(),
                                             winapi::PAGE_READWRITE | winapi::SEC_COMMIT,
                                             lhigh, llow,
                                             ptr::null_mut());
            if handle == INVALID_HANDLE_VALUE {
                return Err(WinError::last("CreateFileMapping"));
            }

            OsIpcSharedMemory::from_handle(handle, length)
        }
    }

    // There is no easy way to query the size of the mapping -- you
    // can use NtQuerySection, but that's an undocumented NT kernel
    // API.  Instead we'll just always pass the length along.
    //
    // This function takes ownership of the handle, and will close it
    // when finished.
    fn from_handle(handle_raw: HANDLE, length: usize) -> Result<OsIpcSharedMemory,WinError> {
        // turn this into a WinHandle, because that will
        // take care of closing it
        let handle = WinHandle::new(handle_raw);
        let address = unsafe {
            kernel32::MapViewOfFile(handle_raw,
                                    winapi::FILE_MAP_ALL_ACCESS,
                                    0, 0, 0)
        };
        if address == ptr::null_mut() {
            return Err(WinError::last("MapViewOfFile"));
        }

        Ok(OsIpcSharedMemory {
            handle: handle,
            ptr: address as *mut u8,
            length: length
        })
    }

    pub fn from_byte(byte: u8, length: usize) -> OsIpcSharedMemory {
        unsafe {
            // panic if we can't create it
            let mem = OsIpcSharedMemory::new(length).unwrap();
            for element in slice::from_raw_parts_mut(mem.ptr, mem.length) {
                *element = byte;
            }
            mem
        }
    }

    pub fn from_bytes(bytes: &[u8]) -> OsIpcSharedMemory {
        unsafe {
            // panic if we can't create it
            let mem = OsIpcSharedMemory::new(bytes.len()).unwrap();
            ptr::copy_nonoverlapping(bytes.as_ptr(), mem.ptr, bytes.len());
            mem
        }
    }
}

pub struct OsIpcOneShotServer {
    receiver: RefCell<Option<OsIpcReceiver>>,
}

impl OsIpcOneShotServer {
    pub fn new() -> Result<(OsIpcOneShotServer, String),WinError> {
        let receiver = try!(OsIpcReceiver::new_named());
        let pipe_id = receiver.pipe_id.unwrap().to_string();
        Ok((
            OsIpcOneShotServer {
                receiver: RefCell::new(Some(receiver)),
            },
            pipe_id
        ))
    }

    pub fn accept(&self) -> Result<(OsIpcReceiver,
                                    Vec<u8>,
                                    Vec<OsOpaqueIpcChannel>,
                                    Vec<OsIpcSharedMemory>),WinError> {
        let mut receiver = self.receiver.borrow_mut().take().unwrap();
        try!(receiver.accept());
        let (data, channels, shmems) = try!(receiver.recv());
        Ok((receiver, data, channels, shmems))
    }
}

pub enum OsIpcChannel {
    Sender(OsIpcSender),
    Receiver(OsIpcReceiver),
}

impl OsIpcChannel {
}

#[derive(PartialEq, Debug)]
pub struct OsOpaqueIpcChannel {
    is_sender: bool,
    handle: HANDLE,
}

impl OsOpaqueIpcChannel {
    fn new(is_sender: bool, handle: HANDLE) -> OsOpaqueIpcChannel {
        OsOpaqueIpcChannel {
            is_sender: is_sender,
            handle: handle,
        }
    }

    pub fn to_receiver(&self) -> OsIpcReceiver {
        assert!(!self.is_sender);
        OsIpcReceiver::from_handle(self.handle)
    }

    pub fn to_sender(&self) -> OsIpcSender {
        assert!(self.is_sender);
        OsIpcSender::from_handle(self.handle)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct WinError(pub u32);

impl WinError {
    pub fn error_string(errnum: u32) -> String {
        // This value is calculated from the macro
        // MAKELANGID(LANG_SYSTEM_DEFAULT, SUBLANG_SYS_DEFAULT)
        let lang_id = 0x0800 as winapi::DWORD;
        let mut buf = [0 as winapi::WCHAR; 2048];

        unsafe {
            let res = kernel32::FormatMessageW(winapi::FORMAT_MESSAGE_FROM_SYSTEM |
                                               winapi::FORMAT_MESSAGE_IGNORE_INSERTS,
                                               ptr::null_mut(),
                                               errnum as winapi::DWORD,
                                               lang_id,
                                               buf.as_mut_ptr(),
                                               buf.len() as winapi::DWORD,
                                               ptr::null_mut()) as usize;
            if res == 0 {
                // Sometimes FormatMessageW can fail e.g. system doesn't like lang_id,
                let fm_err = kernel32::GetLastError();
                return format!("OS Error {} (FormatMessageW() returned error {})",
                               errnum, fm_err);
            }

            match String::from_utf16(&buf[..res]) {
                Ok(mut msg) => {
                    // Trim trailing CRLF inserted by FormatMessageW
                    let len = msg.trim_right().len();
                    msg.truncate(len);
                    msg
                },
                Err(..) => format!("OS Error {} (FormatMessageW() returned \
                                    invalid UTF-16)", errnum),
            }
        }
    }

    fn from_system(err: u32, f: &str) -> WinError {
        // Don't print on the magic token, since it's not really
        // an error.
        // XXX I don't like that try_recv() is expected to return an error
        // when there was no data.
        if err < MAGIC_CODE_BASE {
            dd!("WinError: {} ({}) from {}", WinError::error_string(err), err, f);
        }
        WinError(err)
    }

    fn last(f: &str) -> WinError {
        WinError::from_system(GetLastError(), f)
    }

    pub fn channel_is_closed(&self) -> bool {
        self.0 == MAGIC_CHANNEL_CLOSED_CODE
    }

    #[allow(dead_code)]
    fn no_data(&self) -> bool {
        self.0 == MAGIC_NO_OUTSTANDING_DATA_CODE
    }

    fn ok_channel_closed() -> WinError {
        WinError::from_system(MAGIC_CHANNEL_CLOSED_CODE, "channel closed")
    }

    fn ok_no_data() -> WinError {
        WinError::from_system(MAGIC_NO_OUTSTANDING_DATA_CODE, "no data pending")
    }
}

impl From<WinError> for DeserializeError {
    fn from(mpsc_error: WinError) -> DeserializeError {
        DeserializeError::IoError(mpsc_error.into())
    }
}

impl From<WinError> for Error {
    fn from(mpsc_error: WinError) -> Error {
        //Error::new(ErrorKind::Other, format!("Win channel error ({} from {})", mpsc_error.0, mpsc_error.1))
        Error::new(ErrorKind::Other, format!("Win channel error ({})", mpsc_error.0))
    }
}
