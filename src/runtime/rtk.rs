use super::send_async;
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
use rtk_ins570::{Solution, RTK};
use rtk_qxwz::{
    encode_base64, nmea::NmeaLine, GpggaSender, QXWZAccount, QXWZService, RTCMReceiver, RTKBoard,
};
use std::time::{Duration, Instant};

pub(super) enum Event {
    Connected,
    Disconnected,
    SolutionUpdated(Instant, Solution),
}

pub(super) fn supervisor(dir: PathBuf) -> Receiver<Event> {
    *task::block_on(FILE_PATH.lock()) = dir;
    spawn_qxwz();
    let (sender, receiver) = unbounded();
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTK>::default().join(|e| {
            task::block_on(async {
                match e {
                    Connected(_, _) => {
                        send_async!(Event::Connected => sender).await;
                    }
                    ConnectFailed => {
                        task::sleep(Duration::from_secs(1)).await;
                    }
                    Disconnected => {
                        send_async!(Event::Disconnected => sender).await;
                    }
                    Event(_, Some((time, solution))) => {
                        send_async!(Event::SolutionUpdated(time, solution) => sender).await;
                    }
                    Event(_, None) => {}
                };
            });
            true
        });
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
            match File::open(FILE_PATH.lock().await.clone()).await {
                Ok(file) => {
                    let mut reader = BufReader::new(file);
                    let mut line = String::new();
                    match reader.read_line(&mut line).await {
                        Ok(0) | Err(_) => None,
                        Ok(_) => Some(encode_base64(line)),
                    }
                }
                Err(_) => None,
            }
        })
    }
}

fn spawn_qxwz() {
    let sender: Arc<Mutex<Option<GpggaSender>>> = Arc::new(Mutex::new(None));
    let receiver: Arc<Mutex<Option<RTCMReceiver>>> = Arc::new(Mutex::new(None));
    {
        let sender = sender.clone();
        let receiver = receiver.clone();

        task::spawn_blocking(move || {
            let mut account = String::new();
            loop {
                let time = Instant::now();
                SupervisorForSingle::<QXWZService<AuthFile>>::default().join(|e| {
                    task::block_on(async {
                        match e {
                            Connected(key, stream) => {
                                println!("CONNECTED    qxwz tcp");
                                *sender.lock().await = Some(stream.get_sender());
                                account = key;
                            }
                            Disconnected => {
                                eprintln!("DISCONNECTED qxwz tcp");
                                *sender.lock().await = None;
                            }
                            Event(_, Some((_, buf))) => {
                                if let Some(ref mut receiver) = *receiver.lock().await {
                                    receiver.receive(buf.as_slice());
                                }
                            }
                            Event(_, None) => {}
                            ConnectFailed => {
                                task::sleep(Duration::from_secs(3)).await;
                            }
                        }
                        *UPDATE_TIME.lock().await < time
                    })
                });
            }
        });
    }
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTKBoard>::default().join(|e| {
            task::block_on(async {
                match e {
                    Connected(_, board) => {
                        println!("CONNECTED    qxwz serial");
                        *receiver.lock().await = Some(board.get_receiver());
                    }
                    Disconnected => {
                        eprintln!("DISCONNECTED qxwz serial");
                        *receiver.lock().await = None;
                    }
                    Event(_, Some((_, (NmeaLine::GPGGA(_, tail), cs)))) => {
                        if let Some(ref mut s) = *sender.lock().await {
                            s.send(tail.as_str(), cs).await;
                        }
                    }
                    Event(_, _) => {}
                    ConnectFailed => {
                        task::sleep(Duration::from_secs(1)).await;
                    }
                }
            });
            true
        });
    });
}
