use crate::ros2::capture::{CaptureConfig, RosCaptureContext, RosCapturePlugin};
use crate::ros2::topic::*;
use crate::{
    arc_mutex, publisher,
    robomaster::power_rune::{PowerRune, RuneIndex},
    InfantryChassis, InfantryGimbal, InfantryRoot, InfantryViewOffset, LocalInfantry,
};
use avian3d::prelude::*;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use r2r::geometry_msgs::msg::{Pose, PoseStamped};
use r2r::{Clock, ClockType::SystemTime, Context, Node};
use r2r::{std_msgs::msg::Header, tf2_msgs::msg::TFMessage};
use std::f32::consts::PI;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::thread::{self, JoinHandle};
use std::time::Duration;

pub const M_ALIGN_MAT3: Mat3 = Mat3::from_cols(
    Vec3::new(0.0, -1.0, 0.0), // M[0,0], M[1,0], M[2,0]
    Vec3::new(0.0, 0.0, 1.0),  // M[0,1], M[1,1], M[2,1]
    Vec3::new(-1.0, 0.0, 0.0), // M[0,2], M[1,2], M[2,2]
);

#[inline]
pub fn transform(bevy_transform: Transform) -> r2r::geometry_msgs::msg::Transform {
    let align_rot_mat = M_ALIGN_MAT3;
    let align_quat = Quat::from_mat3(&align_rot_mat);
    let new_rotation = align_quat * bevy_transform.rotation * align_quat.inverse();
    let new_translation = align_rot_mat * bevy_transform.translation;
    r2r::geometry_msgs::msg::Transform {
        translation: r2r::geometry_msgs::msg::Vector3 {
            x: new_translation.x as f64,
            y: new_translation.y as f64,
            z: new_translation.z as f64,
        },
        rotation: r2r::geometry_msgs::msg::Quaternion {
            x: new_rotation.x as f64,
            y: new_rotation.y as f64,
            z: new_rotation.z as f64,
            w: new_rotation.w as f64,
        },
    }
}

macro_rules! res_unwrap {
    ($res:tt) => {
        $res.0.lock().unwrap()
    };
}

#[derive(Resource)]
struct StopSignal(Arc<AtomicBool>);

#[derive(Resource)]
struct SpinThreadHandle(Option<JoinHandle<()>>);

#[derive(Component)]
pub struct MainCamera;

#[derive(Resource)]
pub struct RoboMasterClock(pub Arc<Mutex<Clock>>);

/// Vision system configuration parameters
#[derive(Resource, Clone)]
pub struct VisionConfig {
    pub bullet_speed: u8,     // BulletSpeed value (default: INFANTRY15 = 15)
    pub self_color: u8,       // SelfColor value (default: BLUE = 2)
    pub work_mode: u8,        // WorkMode value (default: AUTO_SHOOT = 0)
    pub game_progress: f32,
    pub stage_remain_time: f32,
    pub current_hp: f32,
    pub current_enemy_sentry_hp: f32,
    pub current_enemy_base_hp: f32,
    pub current_virtual_shield: f32,
    pub current_base_hp: f32,
}

impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            bullet_speed: 15,      // INFANTRY15
            self_color: 1,         // BLUE
            work_mode: 0,          // AUTO_SHOOT
            game_progress: 0.0,
            stage_remain_time: 420.0,
            current_hp: 600.0,
            current_enemy_sentry_hp: 600.0,
            current_enemy_base_hp: 5000.0,
            current_virtual_shield: 100.0,
            current_base_hp: 5000.0,
        }
    }
}

/// Simple smooth controller for gimbal motion
/// Uses exponential smoothing with velocity limiting, more stable than PID for vision tracking
#[derive(Clone)]
struct SmoothController {
    smoothing_factor: f32, // Smoothing factor (0-1), higher = faster response
    deadband: f32,         // Deadband threshold in radians
}

impl SmoothController {
    fn new(smoothing_factor: f32) -> Self {
        Self {
            smoothing_factor: smoothing_factor.clamp(0.0, 1.0),
            deadband: 0.002, // ~0.11 degrees deadband
        }
    }

    /// Calculate desired angular velocity based on error
    fn update(&self, current: f32, target: f32, dt: f32) -> f32 {
        let error = target - current;

        // Apply deadband to prevent jitter near target
        if error.abs() < self.deadband {
            return 0.0;
        }

        // Calculate desired velocity: larger error = faster speed
        error * self.smoothing_factor / dt.max(0.001)
    }

    fn reset(&mut self) {
        // No state to reset for this simple controller
    }
}

/// Resource to store gimbal control commands received from vision_send_data topic
#[derive(Resource, Clone)]
pub struct GimbalControl {
    pub target_pitch: f32,
    pub target_yaw: f32,
    pub target_distance: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub vel_yaw: f32,
    pub enabled: bool,
    pub last_update_time: f64,
    pub pitch_controller: SmoothController,
    pub yaw_controller: SmoothController,
}

impl Default for GimbalControl {
    fn default() -> Self {
        // Smoothing factor 0.2: compensates 20% of error per second
        // Provides good balance between responsiveness and smoothness
        Self {
            target_pitch: 0.0,
            target_yaw: 0.0,
            target_distance: 0.0,
            vel_x: 0.0,
            vel_y: 0.0,
            vel_yaw: 0.0,
            enabled: false,
            last_update_time: 0.0,
            pitch_controller: SmoothController::new(0.2),
            yaw_controller: SmoothController::new(0.2),
        }
    }
}

/// Resource for vision_send_data subscriber
#[derive(Resource)]
pub struct VisionSendDataSubscriber {
    receiver: Arc<Mutex<crossbeam_channel::Receiver<r2r::hnurm_interfaces::msg::VisionSendData>>>,
}

#[macro_export]
macro_rules! add_tf_frame {
    ($ls:ident, $hdr:expr, $id:expr, $translation:expr, $rotation:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform(
                Transform::IDENTITY
                    .with_translation($translation)
                    .with_rotation($rotation),
            ),
        });
    };
    ($ls:ident, $hdr:expr, $id:expr, $transform:expr) => {
        $ls.push(::r2r::geometry_msgs::msg::TransformStamped {
            header: $hdr.clone(),
            child_frame_id: $id.to_string(),
            transform: crate::ros2::plugin::transform($transform),
        });
    };
}

#[macro_export]
macro_rules! pose {
    ($hdr:expr) => {
        PoseStamped {
            header: $hdr.clone(),
            pose: Pose {
                position: r2r::geometry_msgs::msg::Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                orientation: r2r::geometry_msgs::msg::Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                },
            },
        }
    };
}

fn capture_rune(
    camera: Single<&GlobalTransform, With<MainCamera>>,
    gimbal: Single<&GlobalTransform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    _view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,

    runes: Query<(&GlobalTransform, &PowerRune)>,
    targets: Query<(&GlobalTransform, &RuneIndex, &Name)>,

    clock: ResMut<RoboMasterClock>,
    tf_publisher: ResMut<TopicPublisher<GlobalTransformTopic>>,
    gimbal_pose_pub: ResMut<TopicPublisher<GimbalPoseTopic>>,
    odom_pose_pub: ResMut<TopicPublisher<OdomPoseTopic>>,
    camera_pose_pub: ResMut<TopicPublisher<CameraPoseTopic>>,
) {
    let cam_transform = camera.into_inner();
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    let mut transform_stamped = vec![];
    let map_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "map".to_string(),
    };
    let odom_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "odom".to_string(),
    };
    let gimbal_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "gimbal_link".to_string(),
    };
    let camera_hdr = Header {
        stamp: stamp.clone(),
        frame_id: "camera_link".to_string(),
    };

    gimbal_pose_pub.publish(pose!(gimbal_hdr));
    odom_pose_pub.publish(pose!(odom_hdr));
    camera_pose_pub.publish(pose!(camera_hdr));

    add_tf_frame!(
        transform_stamped,
        map_hdr.clone(),
        "odom",
        gimbal.translation(),
        Quat::IDENTITY
    );
    add_tf_frame!(
        transform_stamped,
        odom_hdr.clone(),
        "gimbal_link",
        Vec3::ZERO,
        gimbal.rotation()
    );
    let cam_rel = cam_transform.reparented_to(gimbal.into_inner());
    add_tf_frame!(
        transform_stamped,
        gimbal_hdr.clone(),
        "camera_link",
        cam_rel.translation,
        cam_rel.rotation
    );
    add_tf_frame!(
        transform_stamped,
        camera_hdr.clone(),
        "camera_optical_frame",
        Vec3::ZERO,
        Quat::from_euler(EulerRot::ZYX, -PI / 2.0, PI, PI / 2.0)
    );
    for (transform, rune) in runes {
        add_tf_frame!(
            transform_stamped,
            map_hdr.clone(),
            format!("power_rune_{:?}", rune.mode)
                .to_string()
                .to_lowercase(),
            transform.compute_transform()
        );
    }
    for (target_transform, target, name) in targets {
        if !name.contains("_ACTIVATED") {
            continue;
        }
        if let Ok((_rune_transform, rune)) = runes.get(target.1) {
            add_tf_frame!(
                transform_stamped,
                Header {
                    stamp: stamp.clone(),
                    frame_id: format!("power_rune_{:?}", rune.mode)
                        .to_string()
                        .to_lowercase(),
                },
                format!("power_rune_{:?}_{:?}", rune.mode, target.0)
                    .to_string()
                    .to_lowercase(),
                target_transform.reparented_to(_rune_transform)
            );
        }
    }
    tf_publisher.publish(TFMessage {
        transforms: transform_stamped,
    });
}

fn publish_vision_recv_data(
    gimbal_query: Query<(&GlobalTransform, &InfantryGimbal), (With<LocalInfantry>, Without<InfantryChassis>)>,
    chassis_query: Query<&InfantryChassis, With<LocalInfantry>>,
    root_query: Query<(&LinearVelocity, &AngularVelocity), (With<InfantryRoot>, With<LocalInfantry>)>,
    
    clock: Res<RoboMasterClock>,
    config: Res<VisionConfig>,
    publisher: Res<TopicPublisher<VisionRecvDataTopic>>,
    mut logged: Local<bool>,
) {
    // Check if entities are ready
    let Some((gimbal_transform, gimbal_data)) = gimbal_query.iter().next() else {
        if !*logged {
            debug!("Waiting for gimbal entity to be ready...");
            *logged = true;
        }
        return; // Gimbal not ready yet
    };
    let Some(chassis_data) = chassis_query.iter().next() else {
        if !*logged {
            debug!("Waiting for chassis entity to be ready...");
            *logged = true;
        }
        return; // Chassis not ready yet
    };
    let Some((linear_vel, angular_vel)) = root_query.iter().next() else {
        if !*logged {
            debug!("Waiting for root entity with velocity components to be ready...");
            *logged = true;
        }
        return; // Root not ready yet
    };
    
    if !*logged {
        info!("Vision recv data publisher started successfully!");
        *logged = true;
    }
    
    let stamp = Clock::to_builtin_time(&res_unwrap!(clock).get_now().unwrap());
    
    // Get gimbal angles relative to chassis (what vision algorithm needs)
    // Convert from radians to degrees for ROS message
    let pitch = gimbal_data.pitch.to_degrees();
    let yaw = gimbal_data.local_yaw.to_degrees();
    
    // Extract roll from global transform (usually 0), convert to degrees
    let (_, _, roll_rad) = gimbal_transform.rotation().to_euler(EulerRot::YXZ);
    let roll = roll_rad.to_degrees();
    
    // Extract linear and angular velocities
    let vel_x = linear_vel.x;
    let vel_y = linear_vel.y;
    let vel_yaw = angular_vel.z;
    
    // Create VisionRecvData message using hnurm_interfaces
    let msg = r2r::hnurm_interfaces::msg::VisionRecvData {
        header: Header {
            stamp: stamp.clone(),
            frame_id: "gimbal_link".to_string(),
        },
        self_color: r2r::hnurm_interfaces::msg::SelfColor {
            data: config.self_color,
        },
        work_mode: r2r::hnurm_interfaces::msg::WorkMode {
            data: config.work_mode,
        },
        bullet_speed: r2r::hnurm_interfaces::msg::BulletSpeed {
            data: config.bullet_speed,
        },
        roll,
        pitch,
        yaw,
        vel_x,
        vel_y,
        vel_yaw,
        control_id: 1.0, // right controller by default
        game_progress: config.game_progress,
        stage_remain_time: config.stage_remain_time,
        current_hp: config.current_hp,
        current_enemy_sentry_hp: config.current_enemy_sentry_hp,
        current_enemy_base_hp: config.current_enemy_base_hp,
        current_virtual_shield: config.current_virtual_shield,
        current_base_hp: config.current_base_hp,
    };
    
    publisher.publish(msg);
}

/// System to receive vision_send_data messages and update gimbal control targets
fn receive_vision_send_data(
    subscriber: Res<VisionSendDataSubscriber>,
    mut gimbal_control: ResMut<GimbalControl>,
    time: Res<Time>,
) {
    let receiver = subscriber.receiver.lock().unwrap();
    
    // Process all available messages, keeping only the latest
    let mut latest_msg = None;
    while let Ok(msg) = receiver.try_recv() {
        latest_msg = Some(msg);
    }
    
    if let Some(msg) = latest_msg {
        // Check if target is valid: pitch=0 and yaw=0 means no target
        const EPSILON: f32 = 0.01;
        let has_target = msg.pitch.abs() > EPSILON || msg.yaw.abs() > EPSILON;
        
        if has_target {
            let was_enabled = gimbal_control.enabled;
            
            // pitch and yaw are absolute angles relative to chassis (in degrees)
            // Vision algorithm calculates target angles based on current angles we send
            // Convert to radians for internal use
            gimbal_control.target_pitch = msg.pitch.to_radians();
            gimbal_control.target_yaw = msg.yaw.to_radians();
            gimbal_control.target_distance = msg.target_distance;
            gimbal_control.vel_x = msg.vel_x;
            gimbal_control.vel_y = msg.vel_y;
            gimbal_control.vel_yaw = msg.vel_yaw;
            gimbal_control.enabled = true;
            gimbal_control.last_update_time = time.elapsed_secs_f64();
            
            // Reset controllers when enabling vision control
            if !was_enabled {
                gimbal_control.pitch_controller.reset();
                gimbal_control.yaw_controller.reset();
            }
            
            info!(
                "Vision target: pitch={:.2}°, yaw={:.2}°, distance={:.2}m",
                msg.pitch, msg.yaw, msg.target_distance
            );
        } else {
            // No target: disable auto control and switch to manual mode
            if gimbal_control.enabled {
                gimbal_control.enabled = false;
                gimbal_control.pitch_controller.reset();
                gimbal_control.yaw_controller.reset();
                info!("No target detected (pitch=0, yaw=0), switching to manual control");
            }
        }
    }
}

/// System to apply vision-based gimbal control with smooth motion
fn apply_gimbal_control(
    time: Res<Time>,
    mut gimbal_control: ResMut<GimbalControl>,
    mut gimbal_query: Query<
        (&mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let Some((mut gimbal_transform, mut gimbal_data)) = gimbal_query.iter_mut().next() else {
        return; // Gimbal not ready yet
    };

    // Check for timeout: disable auto control if no new messages for 0.5 seconds
    const VISION_TIMEOUT: f64 = 0.5;
    let current_time = time.elapsed_secs_f64();

    if gimbal_control.enabled {
        if current_time - gimbal_control.last_update_time > VISION_TIMEOUT {
            gimbal_control.enabled = false;
            gimbal_control.pitch_controller.reset();
            gimbal_control.yaw_controller.reset();
            info!("Vision control timeout, switching to manual control");
        }
    }

    // Only apply auto control when vision control is enabled
    if !gimbal_control.enabled {
        return; // Allow manual control
    }

    let dt = time.delta_secs();
    if dt <= 0.0 {
        return; // Avoid division by zero
    }

    // Calculate desired angular velocities using smooth controller
    let pitch_velocity = gimbal_control.pitch_controller.update(
        gimbal_data.pitch,
        gimbal_control.target_pitch,
        dt,
    );
    let yaw_velocity = gimbal_control.yaw_controller.update(
        gimbal_data.local_yaw,
        gimbal_control.target_yaw,
        dt,
    );

    // Limit maximum angular velocity to prevent jerky motion
    const MAX_ANGULAR_SPEED: f32 = 2.5; // rad/s (~143°/s)
    let pitch_velocity = pitch_velocity.clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    let yaw_velocity = yaw_velocity.clamp(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

    // Apply angular velocities to gimbal angles
    gimbal_data.pitch += pitch_velocity * dt;
    gimbal_data.local_yaw += yaw_velocity * dt;

    // Clamp pitch to valid range (±45°)
    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    // Update gimbal transform
    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
    gimbal_transform.rotation = gimbal_rotation;
}

fn cleanup_ros2_system(
    mut exit: MessageReader<AppExit>,
    stop_signal: Res<StopSignal>,
    mut handle_res: ResMut<SpinThreadHandle>,
) {
    if exit.read().len() > 0 {
        stop_signal.0.store(true, Ordering::Release);
        if let Some(handle) = handle_res.0.take() {
            info!("Waiting for ROS 2 spin thread to join...");
            match handle.join() {
                Ok(_) => info!("ROS 2 thread successfully joined. Safe to exit."),
                Err(_) => error!("WARNING: ROS 2 thread panicked or failed to join."),
            }
        }
    }
}

#[derive(Default)]
pub struct ROS2Plugin {}

impl Plugin for ROS2Plugin {
    fn build(&self, app: &mut App) {
        let mut node = Node::create(Context::create().unwrap(), "simulator", "robomaster").unwrap();
        let signal_arc = Arc::new(AtomicBool::new(false));

        publisher!(
            signal_arc,
            app,
            node,
            GlobalTransformTopic,
            GimbalPoseTopic,
            OdomPoseTopic,
            CameraPoseTopic,
            VisionRecvDataTopic
        );
        let camera_info = Arc::new(publisher!(signal_arc, node, CameraInfoTopic));
        let image_raw = Arc::new(publisher!(signal_arc, node, ImageRawTopic));
        let image_compressed = Arc::new(publisher!(signal_arc, node, ImageCompressedTopic));

        // Create subscriber for vision_send_data topic (uses hnurm_interfaces)
        let (tx, rx) =
            crossbeam_channel::unbounded::<r2r::hnurm_interfaces::msg::VisionSendData>();
        let mut subscriber = node
            .subscribe::<r2r::hnurm_interfaces::msg::VisionSendData>(
                "/vision_send_data",
                r2r::QosProfile::default(),
            )
            .unwrap();

        let rx_arc = Arc::new(Mutex::new(rx));

        // Spawn dedicated thread for ROS 2 subscriber
        let signal_clone = signal_arc.clone();
        thread::spawn(move || {
            use futures::stream::StreamExt;
            let rt = tokio::runtime::Runtime::new().unwrap();
            rt.block_on(async {
                while !signal_clone.load(Ordering::Acquire) {
                    match tokio::time::timeout(Duration::from_millis(100), subscriber.next()).await
                    {
                        Ok(Some(msg)) => {
                            let _ = tx.send(msg);
                        }
                        Ok(None) => break,
                        Err(_) => continue, // Timeout, check stop signal and retry
                    }
                }
            });
        });

        let clock = arc_mutex!(Clock::create(SystemTime).unwrap());

        app.insert_resource(RoboMasterClock(clock.clone()))
            .insert_resource(StopSignal(signal_arc.clone()))
            .insert_resource(VisionConfig::default())
            .insert_resource(GimbalControl::default())
            .insert_resource(VisionSendDataSubscriber {
                receiver: rx_arc,
            })
            .add_plugins(RosCapturePlugin {
                config: CaptureConfig {
                    width: 1440,
                    height: 1080,
                    texture_format: TextureFormat::bevy_default(),
                    fov_y: PI / 180.0 * 45.0,
                },
                context: RosCaptureContext {
                    clock,
                    camera_info,
                    image_raw,
                    image_compressed,
                },
            })
            .add_systems(Last, cleanup_ros2_system)
            .add_systems(
                Update,
                (
                    capture_rune.after(TransformSystems::Propagate),
                    publish_vision_recv_data.after(TransformSystems::Propagate),
                    receive_vision_send_data,
                    apply_gimbal_control,
                ),
            )
            .insert_resource(SpinThreadHandle(Some(thread::spawn(move || {
                while !signal_arc.load(Ordering::Acquire) {
                    node.spin_once(Duration::from_millis(1000));
                }
            }))));
    }
}
