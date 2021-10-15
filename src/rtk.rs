use super::MsgToFollower::{self, *};
use parry2d::na::{Isometry2, Vector2};
use pm1_sdk::driver::{SupersivorEventForSingle::*, SupervisorForSingle};
use rtk_ins570::{Solution, RTK};
use std::{sync::mpsc::Sender, thread, time::Duration};

pub fn supervisor(to_follower: Sender<MsgToFollower>) {
    SupervisorForSingle::<RTK>::new().join(|e| {
        match e {
            Connected(k, _) => eprintln!("RTK at COM {}", k),
            ConnectFailed | Disconnected => thread::sleep(Duration::from_secs(1)),
            Event(_, Some((time, Solution::Data { state, enu, dir }))) => {
                if state.state_pos >= 40 && state.state_dir >= 30 {
                    let _ = to_follower.send(Absolute(
                        time,
                        Isometry2::new(Vector2::new(enu.e as f32, enu.n as f32), dir as f32),
                    ));
                }
            }
            Event(_, _) => {}
        };
        true
    });
}
