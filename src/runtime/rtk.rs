use super::send_async;
use async_std::{
    channel::{unbounded, Receiver},
    sync::{Arc, Mutex},
    task,
};
use pm1_sdk::driver::{SupervisorEventForSingle::*, SupervisorForSingle};
use rtk_ins570::{Solution, RTK};
use rtk_qxwz::{
    encode_base64, nmea::NmeaLine, GpggaSender, QXWZAccount, QXWZService, RTCMReceiver, RTKBoard,
};
use std::{
    thread,
    time::{Duration, Instant},
};

pub(super) enum Event {
    Connected,
    Disconnected,
    SolutionUpdated(Instant, Solution),
}

pub(super) fn supervisor() -> Receiver<Event> {
    spawn_qxwz();
    let (sender, receiver) = unbounded();
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTK>::default().join(|e| {
            match e {
                Connected(_, _) => {
                    task::block_on(send_async!(Event::Connected => sender));
                }
                ConnectFailed => {
                    thread::sleep(Duration::from_secs(1));
                }
                Disconnected => {
                    task::block_on(send_async!(Event::Disconnected => sender));
                }
                Event(_, Some((time, solution))) => {
                    task::block_on(send_async!(Event::SolutionUpdated(time, solution) => sender));
                }
                Event(_, None) => {}
            };
            true
        });
    });
    receiver
}

struct AuthFile;

impl QXWZAccount for AuthFile {
    fn get() -> Option<String> {
        std::env::current_exe()
            .ok()
            .and_then(|path| path.parent().map(|temp| temp.to_path_buf()))
            .and_then(|mut path| {
                path.push("auth");
                std::fs::read_to_string(path).ok()
            })
            .and_then(|text| text.lines().next().map(|line| encode_base64(line)))
    }
}

fn spawn_qxwz() {
    let sender: Arc<Mutex<Option<GpggaSender>>> = Arc::new(Mutex::new(None));
    let receiver: Arc<Mutex<Option<RTCMReceiver>>> = Arc::new(Mutex::new(None));
    {
        let sender = sender.clone();
        let receiver = receiver.clone();
        task::spawn_blocking(move || {
            SupervisorForSingle::<QXWZService<AuthFile>>::default().join(|e| {
                task::block_on(async {
                    match e {
                        Connected(_, stream) => {
                            *sender.lock().await = Some(stream.get_sender());
                        }
                        Disconnected => {
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
                });
                true
            });
        });
    }
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTKBoard>::default().join(|e| {
            task::block_on(async {
                match e {
                    Connected(_, board) => {
                        *receiver.lock().await = Some(board.get_receiver());
                    }
                    Disconnected => {
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
