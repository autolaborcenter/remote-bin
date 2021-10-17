use super::macros::send_anyway;
use async_std::{
    channel::{unbounded, Receiver},
    task,
};
use pm1_sdk::driver::{SupersivorEventForSingle::*, SupervisorForSingle};
use rtk_ins570::{Solution, RTK};
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
    let (sender, receiver) = unbounded();
    task::spawn_blocking(move || {
        SupervisorForSingle::<RTK>::new().join(|e| {
            match e {
                Connected(k, _) => {
                    send_anyway!(Event::Connected => sender);
                }
                ConnectFailed => {
                    thread::sleep(Duration::from_secs(1));
                }
                Disconnected => {
                    send_anyway!(Event::Disconnected => sender);
                }
                Event(_, Some((time, solution))) => {
                    send_anyway!(Event::SolutionUpdated(time, solution) => sender);
                }
                Event(_, None) => {}
            };
            true
        });
    });
    receiver
}
