use robot_bin::Robot;

#[async_std::main]
async fn main() {
    let (_robot, events) = Robot::spawn(false).await;
    while let Ok(e) = events.recv().await {
        use robot_bin::Event::*;
        match e {
            ConnectionModified(_) => {}
            ChassisStatusUpdated(_) => {}
            ChassisOdometerUpdated(_, _) => {}
            PoseUpdated(_) => {}
            LidarFrameEncoded(_) => {}
            CollisionDetected(_) => {}
        }
    }
}
