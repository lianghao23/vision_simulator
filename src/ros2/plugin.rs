use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::{self, JoinHandle},
};

use bevy::{
    prelude::*,
    render::view::screenshot::{Screenshot, ScreenshotCaptured},
};

use r2r::{
    Clock, Context, Node, Publisher, QosProfile, sensor_msgs::msg::Image, std_msgs::msg::Header,
};

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);
#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Component)]
struct CaptureCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

#[derive(Resource)]
pub struct RawCameraPublisher(pub Arc<Mutex<Publisher<Image>>>);

fn capture_frame(mut commands: Commands, input: Res<ButtonInput<KeyCode>>) {
    // if input.just_pressed(KeyCode::KeyG) {
    commands.spawn(Screenshot::primary_window()).observe(
        |ev: On<ScreenshotCaptured>,
         clock: ResMut<RoboMasterClock>,
         publ: Res<RawCameraPublisher>| {
            let dyn_img = ev.image.clone().try_into_dynamic().unwrap();
            let rgb8 = dyn_img.to_rgb8();
            let publisher = publ.0.lock().unwrap();
            let mut clock = clock.0.lock().unwrap();

            publisher
                .publish(&Image {
                    header: Header {
                        stamp: Clock::to_builtin_time(&clock.get_now().unwrap()),
                        frame_id: "idk".to_string(),
                    },
                    height: rgb8.height(),
                    width: rgb8.width(),
                    encoding: "rgb8".to_string(),
                    is_bigendian: 0,
                    step: rgb8.width() as u32 * 3,
                    data: rgb8.into_raw(),
                })
                .unwrap();
        },
    );
    // }
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>, // 监听 AppExit 事件
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    // 只有在接收到 AppExit 事件时才设置信号
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            println!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => println!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => eprintln!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let ctx = Context::create().unwrap();
        let mut node = Node::create(ctx, "node", "namespace").unwrap();
        let publisher = node
            .create_publisher("/camera_raw", QosProfile::default())
            .unwrap();
        let node = Arc::new(Mutex::new(node));

        let arc = node.clone();
        let stop_signal = Arc::new(AtomicBool::new(false));
        let signal_arc = stop_signal.clone();
        let handle = thread::spawn(move || {
            while !signal_arc.load(Ordering::Acquire) {
                let mut node = arc.lock().unwrap();
                node.spin_once(std::time::Duration::from_millis(10));
            }
        });

        app.insert_resource(RoboMasterClock(Arc::new(Mutex::new(
            Clock::create(r2r::ClockType::RosTime).unwrap(),
        ))))
        .insert_resource(RawCameraPublisher(Arc::new(Mutex::new(publisher))))
        .insert_resource(StopSignal(stop_signal))
        .insert_resource(SpinThreadHandle(Some(handle)))
        .add_systems(Update, capture_frame)
        .add_systems(Last, cleanup_ros2_system);
    }
}
