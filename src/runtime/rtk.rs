﻿use super::send_async;
use async_std::{
    channel::{unbounded, Receiver},
    fs::File,
    io::{prelude::BufReadExt, BufReader},
    path::PathBuf,
    sync::{Arc, Mutex},
    task,
};
use lazy_static::lazy_static;
use pm1_sdk::driver::{SupervisorEventForSingle::*, SupervisorForSingle};
use rtk_qxwz::{
    encode_base64, Gpgga, GpggaParseError::*, GpggaSender, QXWZAccount, QXWZService, RTCMReceiver,
    RTKBoard,
};
use std::time::{Duration, Instant};

pub(super) enum Event {
    SerialConnected,
    SerialDisconnected,
    TcpConnected,
    TcpDisconnected,
    Gpgga(Instant, Gpgga),
}

pub(super) fn supervisor(dir: PathBuf) -> Receiver<Event> {
    let _ = *task::block_on(UPDATE_TIME.lock());
    *task::block_on(FILE_PATH.lock()) = dir;
    let gpgga: Arc<Mutex<Option<GpggaSender>>> = Arc::new(Mutex::new(None));
    let rtcm: Arc<Mutex<Option<RTCMReceiver>>> = Arc::new(Mutex::new(None));
    let (sender, receiver) = unbounded();
    {
        let gpgga = gpgga.clone();
        let rtcm = rtcm.clone();
        let sender = sender.clone();

        task::spawn_blocking(move || {
            let mut account = String::new();
            loop {
                let time = Instant::now();
                SupervisorForSingle::<QXWZService<AuthFile>>::default().join(|e| {
                    task::block_on(async {
                        match e {
                            Connected(key, stream) => {
                                send_async!(Event::TcpConnected => sender).await;
                                *gpgga.lock().await = Some(stream.get_sender());
                                account = key;
                            }
                            Disconnected => {
                                send_async!(Event::TcpDisconnected => sender).await;
                                *gpgga.lock().await = None;
                            }
                            Event(_, Some((_, buf))) => {
                                if let Some(ref mut receiver) = *rtcm.lock().await {
                                    receiver.receive(buf.as_slice());
                                }
                            }
                            Event(_, None) => {}
                            ConnectFailed => {
                                task::sleep(Duration::from_secs(3)).await;
                            }
                        }
                        *UPDATE_TIME.lock().await < time
                            && time < Instant::now() + Duration::from_secs(300)
                    })
                });
                task::block_on(send_async!(Event::TcpDisconnected => sender));
            }
        });
    }
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTKBoard>::default().join(|e| {
            task::block_on(async {
                match e {
                    Connected(_, board) => {
                        send_async!(Event::SerialConnected => sender).await;
                        *rtcm.lock().await = Some(board.get_receiver());
                    }
                    Disconnected => {
                        send_async!(Event::SerialDisconnected => sender).await;
                        *rtcm.lock().await = None;
                    }
                    Event(_, Some((t, line))) => match line.parse::<Gpgga>() {
                        Ok(body) => {
                            send_async!(Event::Gpgga(t, body) => sender).await;
                            if let Some(ref mut s) = *gpgga.lock().await {
                                s.send(&line).await;
                            }
                        }
                        Err(WrongHead) => {
                            if let Some(ref mut s) = *gpgga.lock().await {
                                s.send(&line).await;
                            }
                        }
                        Err(_) => {}
                    },
                    Event(_, _) => {}
                    ConnectFailed => {
                        task::sleep(Duration::from_secs(1)).await;
                    }
                }
            });
            true
        })
    });
    receiver
}

lazy_static! {
    static ref UPDATE_TIME: Mutex<Instant> = Mutex::new(Instant::now());
    static ref FILE_PATH: Mutex<PathBuf> = Default::default();
}

#[inline]
pub async fn reauth() {
    *UPDATE_TIME.lock().await = Instant::now();
}

struct AuthFile;

impl QXWZAccount for AuthFile {
    fn get() -> Option<String> {
        task::block_on(async {
            let path = FILE_PATH.lock().await.join("auth");
            match File::open(&path).await {
                Ok(file) => {
                    let mut reader = BufReader::new(file);
                    let mut line = String::new();
                    match reader.read_line(&mut line).await {
                        Ok(0) | Err(_) => None,
                        Ok(_) => Some(encode_base64(line.trim_end())),
                    }
                }
                Err(_) => None,
            }
        })
    }
}
