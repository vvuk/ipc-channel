extern crate ipc_channel;

use ipc_channel::platform;
use std::thread;

fn main() {
    let num_senders = 3;

    let (tx, rx) = platform::channel().unwrap();

    let threads: Vec<_> = (0..num_senders).map(|i| {
        let tx = tx.clone();
        thread::spawn(move || {
            let data: Vec<u8> = (0.. 1024 * 1024).map(|j| (j % 13) as u8 | i << 4).collect();
            let data: &[u8] = &data[..];
            println!("=== send: #{}", i);
            tx.send(data, vec![], vec![]).unwrap();
            println!("=== send: #{} done", i);
        })
    }).collect();

    let mut received_vals: Vec<u8> = vec![];
    for k in 0..num_senders {
        println!("=== recv: #{}", k);
        let (mut received_data, received_channels, received_shared_memory_regions) =
            rx.recv().unwrap();
        println!("=== recv: #{} done", k);
        let val = received_data[0] >> 4;
        received_vals.push(val);
        let data: Vec<u8> = (0.. 1024 * 1024).map(|j| (j % 13) as u8 | val << 4).collect();
        let data: &[u8] = &data[..];
        received_data.truncate(1024 * 1024);
        assert_eq!(received_data.len(), data.len());
        assert_eq!((&received_data[..], received_channels, received_shared_memory_regions),
                   (&data[..], vec![], vec![]));
    }
    assert!(rx.try_recv().is_err()); // There should be no further messages pending.
    received_vals.sort();
    assert_eq!(received_vals, (0..num_senders).collect::<Vec<_>>()); // Got exactly the values we sent.

    for thread in threads {
        thread.join().unwrap();
    }
}
