// Copyright 2015 The Servo Project Developers. See the COPYRIGHT
// file at the top-level directory of this distribution.
//
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

#![cfg_attr(any(feature = "force-inprocess", target_os = "android"),
	    feature(mpsc_select))]

#[macro_use]
extern crate lazy_static;

extern crate bincode;
extern crate libc;
extern crate rand;
extern crate serde;

#[cfg(any(feature = "force-inprocess", target_os = "windows", target_os = "android"))]
extern crate uuid;

#[cfg(all(not(feature = "force-inprocess"), target_os = "windows"))]
extern crate winapi;
#[cfg(all(not(feature = "force-inprocess"), target_os = "windows"))]
extern crate kernel32;
#[cfg(all(not(feature = "force-inprocess"), target_os = "windows"))]
extern crate user32;

pub mod ipc;
pub mod platform;
mod refcell;
pub mod router;

#[cfg(test)]
mod test;

