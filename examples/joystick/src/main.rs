use async_std::{
    channel::{unbounded, Sender},
    io,
    sync::Arc,
    task,
};
use joystick_win::JoyStick;
use robot_bin::{Physical, Robot};
use std::{
    f32::consts::FRAC_PI_2,
    sync::atomic::{AtomicU32, Ordering},
};

fn main() {
    task::block_on(async {
        let level = Arc::new(AtomicU32::new(0.3f32.to_bits()));
        keyboard(level.clone());

        let (sender, receiver) = unbounded();
        joystick(sender.clone());

        let (robot, _) = Robot::spawn(false).await;
        while let Ok(mut target) = receiver.recv().await {
            target.speed *= f32::from_bits(level.load(Ordering::Relaxed));
            robot.drive(target).await;
        }
    });
}

fn joystick(sender: Sender<Physical>) -> task::JoinHandle<()> {
    task::spawn(async move {
        let mut joystick = JoyStick::default();
        loop {
            let (duration, event) = joystick.read();
            if let Some((x, y)) = event {
                fn map(x: f32, y: f32) -> f32 {
                    let rad = x.atan2(y);
                    let map = rad.signum() * FRAC_PI_2;
                    (rad / map).powi(2) * map
                }

                let speed = f32::max(x.abs(), y.abs());
                let command = if speed < 0.01 {
                    Physical::RELEASED
                } else if y >= 0.0 {
                    Physical {
                        speed,
                        rudder: map(x, y),
                    }
                } else {
                    Physical {
                        speed: -speed,
                        rudder: map(x, -y),
                    }
                };
                let _ = sender.send(command).await;
            }
            task::sleep(duration).await;
        }
    })
}

fn keyboard(level: Arc<AtomicU32>) -> task::JoinHandle<()> {
    task::spawn(async move {
        let mut line = String::new();
        loop {
            line.clear();
            if let Ok(_) = io::stdin().read_line(&mut line).await {
                if let Ok(k) = line.trim().parse::<f32>() {
                    println!("k = {}", k);
                    if 0.0 <= k && k <= 1.0 {
                        level.store(k.to_bits(), Ordering::Relaxed);
                    }
                }
            }
        }
    })
}
