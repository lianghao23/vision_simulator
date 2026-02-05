mod dataset;
mod handler;
mod robomaster;
mod ros2;
mod statistic;
mod util;
mod vision_msgs;

use crate::dataset::prelude::DatasetPlugin;
use crate::ros2::plugin::{AutoAimMode, ROS2Plugin, TargetEuler};
use crate::util::bevy::insert_all_child;
use crate::{
    handler::{on_activate, on_hit},
    robomaster::power_rune::{PowerRunePlugin, PowerRuneRoot, Projectile},
    statistic::{accurate_count, accurate_pct, increase_launch, launch_count},
};
use avian3d::prelude::*;
use bevy::asset::embedded_asset;
use bevy::camera::Exposure;
use bevy::light::light_consts::lux;
use bevy::render::view::screenshot::{Capturing, Screenshot, save_to_disk};
use bevy::window::{CursorIcon, PresentMode, SystemCursorIcon};
use bevy::{
    anti_alias::fxaa::Fxaa,
    input::mouse::MouseMotion,
    prelude::*,
    scene::{SceneInstance, SceneInstanceReady},
};
use std::collections::{HashMap, HashSet, VecDeque};

#[derive(Component)]
struct MainCamera {
    follow_offset: Vec3,
}

#[derive(Component)]
struct LocalInfantry;

#[derive(Component)]
pub struct RemoteInfantry {
    pub id: u8,
}

#[derive(Component)]
struct InfantryRoot;

// 自动运动配置参数
#[derive(Resource)]
struct AutoMoveConfig {
    range: f32,           // 往复运动范围（米）
    min_speed: f32,       // 最小平均速度（米/秒）
    rotation_speed: f32,  // 自转速度（转/秒）
}

impl Default for AutoMoveConfig {
    fn default() -> Self {
        Self {
            range: 5.0,
            min_speed: 1.0,
            rotation_speed: 0.4,
        }
    }
}

// 自动运动状态
#[derive(Resource, Default)]
struct AutoMove {
    enabled: bool,
    start_position: Vec3,
    direction: Vec3,      // 运动方向
    current_distance: f32, // 当前已移动距离
    moving_forward: bool,  // true = 正向，false = 反向
}

impl AutoMove {
    fn new() -> Self {
        Self {
            enabled: false,
            start_position: Vec3::ZERO,
            direction: Vec3::X, // 默认沿 X 轴运动
            current_distance: 0.0,
            moving_forward: true,
        }
    }
}

#[derive(Resource, PartialEq)]
struct CameraMode(pub FollowingType);

#[derive(PartialEq)]
enum FollowingType {
    Free,
    Robot,
    ThirdPerson,
}

#[derive(Component, Default)]
struct InfantryChassis {
    yaw: f32,
}

#[derive(Component, Default)]
struct InfantryGimbal {
    local_yaw: f32,
    pitch: f32,
}

#[derive(Component)]
struct InfantryViewOffset(Transform);

#[derive(Component)]
struct InfantryLaunchOffset(Transform);

#[derive(PhysicsLayer, Default, Clone, Copy, Debug)]
enum GameLayer {
    #[default]
    Default,
    Vehicle,
    ProjectileSelf,
    ProjectileOther,
    Environment,
}

#[derive(Resource)]
struct Cooldown(Timer);

/// Creates help text at the bottom of the screen.
fn create_help_text(auto_aim: &AutoAimMode) -> Text {
    format!(
        "total={} accurate={} pct={} | mode={}\nControls: F2-Screenshot F3-Camera T-AutoAim | WASD-Move Mouse-Look Space-Shoot\n2025 Actor&Thinker",
        launch_count(),
        accurate_count(),
        accurate_pct(),
        if auto_aim.enabled { "AUTO-AIM" } else { "MANUAL" }
    )
        .into()
}

/// Spawns the help text at the bottom of the screen.
fn spawn_text(commands: &mut Commands, auto_aim: &AutoAimMode) {
    commands.spawn((
        create_help_text(auto_aim),
        Node {
            position_type: PositionType::Absolute,
            bottom: px(12),
            left: px(12),
            ..default()
        },
        HelpText,
    ));
}

#[derive(Component)]
struct HelpText;

/// Update help text when auto-aim mode changes
fn update_help_text(
    auto_aim: Res<AutoAimMode>,
    mut text_query: Query<&mut Text, With<HelpText>>,
) {
    if auto_aim.is_changed() {
        for mut text in text_query.iter_mut() {
            *text = create_help_text(&auto_aim);
        }
    }
}

struct EmbeddedAssetPlugin;

impl Plugin for EmbeddedAssetPlugin {
    fn build(&self, app: &mut App) {
        embedded_asset!(app, "assets/GROUND_DISPLAY.glb");
        embedded_asset!(app, "assets/POWER.glb");
        embedded_asset!(app, "assets/projectile_launch.ogg");
        embedded_asset!(app, "assets/rune_activated.ogg");
        embedded_asset!(app, "assets/vehicle.glb");
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    present_mode: PresentMode::AutoVsync,
                    name: Some("RoboMaster Simulator | Actor&Thinker".to_string()),
                    title: "RoboMaster Simulator | Actor&Thinker".to_string(),
                    fit_canvas_to_parent: true,
                    ..default()
                }),
                ..default()
            }),
            EmbeddedAssetPlugin,
            PhysicsPlugins::default(),
        ))
        .add_plugins(ROS2Plugin::default())
        .add_plugins(PowerRunePlugin)
        .add_plugins(DatasetPlugin)
        .insert_resource(CameraMode(FollowingType::Robot))
        .insert_resource(AutoMoveConfig::default())
        .insert_resource(AutoMove::new())
        .insert_resource(Gravity(Vec3::NEG_Y * 9.81))
        .insert_resource(SubstepCount(10))
        .insert_resource(Cooldown(Timer::from_seconds(0.1, TimerMode::Once)))
        .add_systems(Startup, (setup, setup_projectile))
        .add_observer(setup_vehicle)
        .add_observer(setup_collision)
        .add_observer(on_hit)
        .add_observer(on_activate)
        .add_systems(
            Update,
            (
                update_auto_aim_mode,
                update_help_text,
                following_controls,
                vehicle_controls,
                remote_vehicle_controls,
                auto_move_system,
                toggle_auto_move,
                gimbal_controls,
                remote_gimbal_controls,
                freecam_controls,
                update_camera_follow,
                screenshot_on_f2,
                screenshot_saving,
            ),
        )
        .add_systems(
            PostUpdate,
            projectile_launch.after(TransformSystems::Propagate),
        )
        .run();
}

#[derive(Component)]
struct PreciousCollision(
    HashMap<String, (ColliderConstructorHierarchy, CollisionLayers, Visibility)>,
);

fn setup(mut commands: Commands, asset_server: Res<AssetServer>, auto_aim: Res<AutoAimMode>) {
    spawn_text(&mut commands, &auto_aim);
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.9, 0.95, 1.0),
            shadows_enabled: true,
            illuminance: lux::DIRECT_SUNLIGHT,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    let layer_env = CollisionLayers::new(
        [GameLayer::Environment],
        [
            GameLayer::Default,
            GameLayer::Vehicle,
            GameLayer::ProjectileSelf,
            GameLayer::ProjectileOther,
        ],
    );

    let trimesh = ColliderConstructorHierarchy::new(
        ColliderConstructor::TrimeshFromMeshWithConfig(TrimeshFlags::all()),
    );
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
    };

    commands.spawn((
        SceneRoot(
            asset_server
                .load("embedded://daedalus/assets/GROUND_DISPLAY.glb#Scene0"),
        ),
        Transform::IDENTITY,
        PreciousCollision(HashMap::from([(
            "GROUND_LOW".to_string(),
            (trimesh.clone(), layer_env, Visibility::Hidden),
        )])),
    ));

    let mut power_rune_col = HashMap::from([]);
    for i in 1..=2 {
        /*power_rune_col.insert(
            format!("FACE_{}", i).to_string(),
            (trimesh.clone(), layer_env, Visibility::Visible),
        );*/
        for j in 1..=5 {
            for k in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                power_rune_col.insert(
                    format!("FACE_{}_TARGET_{}_{}", i, j, k).to_string(),
                    (voxel(0.015), layer_env, Visibility::Visible),
                );
            }
        }
    }
    commands.spawn((
        RigidBody::Static,
        CollisionMargin(0.001),
        Restitution::ZERO,
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/POWER.glb#Scene0"),
        ),
        Transform::IDENTITY,
        PowerRuneRoot,
        PreciousCollision(power_rune_col),
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
        CollisionMargin(0.001),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0"),
        ),
        Transform::from_xyz(0.0, 1.0, 0.0),
        InfantryRoot,
        LocalInfantry,
    ));

    commands.spawn((
        RigidBody::Dynamic,
        Collider::cylinder(0.2593615, 0.433951),
        CollisionLayers::new(
            GameLayer::Vehicle,
            [
                GameLayer::Default,
                GameLayer::Vehicle,
                GameLayer::ProjectileOther,
                GameLayer::Environment,
            ],
        ),
        Mass(20.0),
        Friction::new(0.5),
        Restitution::ZERO,
        LinearDamping(0.0),
        AngularDamping(109.8),
        LockedAxes::new().lock_rotation_x().lock_rotation_z(),
        SceneRoot(
            asset_server.load("embedded://daedalus/assets/vehicle.glb#Scene0"),
        ),
        Transform::from_xyz(1.0, 1.0, 1.0),
        InfantryRoot,
        RemoteInfantry { id: 1 },
    ));

    commands.spawn((
        Camera3d::default(),
        Camera::default(),
        Projection::Perspective(PerspectiveProjection {
            fov: std::f32::consts::PI / 180.0 * 45.0,
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Exposure::SUNLIGHT,
        Msaa::Off,
        Fxaa::default(),
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::new(0.0, 3.0, 2.0),
        },
        ros2::plugin::MainCamera,
    ));
}

#[derive(Component, Clone)]
pub struct Armor(String);

fn setup_vehicle(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    root_query: Query<(Entity, Option<&LocalInfantry>, Option<&RemoteInfantry>), With<InfantryRoot>>,
    secondary_query: Query<&ChildOf, (Without<InfantryRoot>, Without<SceneInstance>)>,
    node_query: Query<
        (Entity, &Name, &ChildOf, &Transform),
        (Without<InfantryRoot>, Without<SceneInstance>),
    >,
) {
    let root = events.entity;
    if root_query.get(root).is_err() {
        return;
    }
    let (is_local, remote_id) = match root_query.get(root).unwrap() {
        (_, Some(_), _) => (true, None),
        (_, _, Some(remote)) => (false, Some(remote.id)),
        _ => (false, None),
    };

    let mut despawn = HashSet::new();

    for (node, name, &ChildOf(secondary), transform) in node_query {
        let Ok(&ChildOf(root2)) = secondary_query.get(secondary) else {
            continue;
        };
        if root != root2 {
            continue;
        }
        despawn.insert(secondary);
        commands.entity(secondary).remove_child(node);
        commands.entity(root).add_child(node);
        let mut ent = commands.entity(node);
        if is_local {
            ent.insert(LocalInfantry);
        } else if let Some(id) = remote_id {
            ent.insert(RemoteInfantry { id });
        }
        match name.as_str() {
            "BASE" => {
                ent.insert(InfantryChassis::default());
                let mut stack = VecDeque::from([(node, name)]);
                let mut set = HashSet::new();
                while let Some((e, name)) = stack.pop_front() {
                    if !set.insert(e) {
                        continue;
                    }
                    if name.starts_with("ARMOR_") && name.ends_with("_P") {
                        insert_all_child(&mut commands, e, &children, || Armor(name.to_string()));
                        commands.entity(e).insert(ColliderConstructorHierarchy::new(
                            ColliderConstructor::TrimeshFromMeshWithConfig(
                                TrimeshFlags::MERGE_DUPLICATE_VERTICES,
                            ),
                        ));
                    }
                    for (ee, n, &ChildOf(r), _) in node_query {
                        if r == e {
                            stack.push_back((ee, n));
                        }
                    }
                }
            }
            "GIMBAL" => {
                ent.insert(InfantryGimbal::default());
            }
            "SHOT_DIRECTION" => {
                ent.insert(InfantryLaunchOffset(transform.clone()));
            }
            "CAM_DIRECTION" => {
                ent.insert(InfantryViewOffset(transform.clone()));
            }
            _ => {}
        }
    }

    for ent in despawn {
        commands.entity(ent).despawn();
    }
}

fn setup_projectile(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.insert_resource(ProjectileSetting(
        meshes.add(Sphere::new(44.5 * 0.001 / 2.0)),
        materials.add(StandardMaterial {
            base_color: Color::srgba(0.132866, 1.0, 0.132869, 0.55),
            emissive: LinearRgba::new(0.132866, 1.0, 0.132869, 0.55),
            emissive_exposure_weight: -1.0,
            alpha_mode: AlphaMode::Blend,
            ..default()
        }),
    ));
}

#[derive(Resource)]
struct ProjectileSetting(Handle<Mesh>, Handle<StandardMaterial>);

fn projectile_launch(
    time: Res<Time>,
    _asset_server: Res<AssetServer>,
    mut cooldown: ResMut<Cooldown>,
    mut commands: Commands,
    setting: Res<ProjectileSetting>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<
        (&Transform, &LinearVelocity, &AngularVelocity),
        (With<InfantryRoot>, With<LocalInfantry>),
    >,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    launch_offset: Single<&InfantryLaunchOffset, With<LocalInfantry>>,
) {
    cooldown.0.tick(time.delta());
    if !cooldown.0.is_finished() {
        return;
    }
    cooldown.0.reset();
    if keyboard.pressed(KeyCode::Space) {
        increase_launch();
        let direction = (gimbal.0.rotation() * launch_offset.0.rotation)
            .mul_vec3(Vec3::Y)
            .normalize_or_zero();
        if direction == Vec3::ZERO {
            return;
        }
        let vel = infantry.1.0 + direction * 25.0;
        commands.spawn((
            RigidBody::Dynamic,
            Collider::sphere(44.5 * 0.001 / 2.0),
            Mass(44.5 * 0.001),
            Friction::new(1.1),
            Restitution::ZERO,
            LinearDamping(0.05),
            CollisionLayers::new(
                GameLayer::ProjectileSelf,
                [
                    GameLayer::Default,
                    GameLayer::Vehicle,
                    GameLayer::ProjectileSelf,
                    GameLayer::ProjectileOther,
                    GameLayer::Environment,
                ],
            ),
            Mesh3d(setting.0.clone()),
            MeshMaterial3d(setting.1.clone()),
            LinearVelocity(vel),
            AngularVelocity(infantry.2.0),
            Transform::IDENTITY.with_translation(
                infantry.0.translation + (gimbal.0.rotation() * launch_offset.0.translation),
            ),
            //AudioPlayer::new(asset_server.load("projectile_launch.ogg")),
            Projectile,
        ));
    }
}

fn setup_collision(
    events: On<SceneInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name, With<Children>>,
    root_query: Query<(Entity, &PreciousCollision)>,
) {
    let Ok((_, PreciousCollision(map))) = root_query.get(events.entity) else {
        return;
    };
    for e in children.iter_descendants(events.entity) {
        let Ok(name) = name.get(e) else {
            continue;
        };
        if let Some((constructor, layer, visibility)) = map.get(&name.to_string()) {
            commands.entity(e).insert((
                RigidBody::Static,
                Mass(0.0),
                Restitution::ZERO,
                constructor.clone(),
                CollisionMargin(0.001),
                *layer,
            ));
            if visibility == Visibility::Hidden {
                commands.entity(e).insert(*visibility);
            }
        }
    }
    commands.entity(events.entity).remove::<PreciousCollision>();
}

// 单位 m/s^2
const VEHICLE_ACCEL: f32 = 10.0;
// 单位 rad/s
const VEHICLE_ROTATION_SPEED: f32 = 3.0;
const GIMBAL_ROTATION_SPEED: f32 = 10.0;

const MAX_VEHICLE_VELOCITY: f32 = 6.0;

fn vehicle_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<(Forces, &Mass), (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (With<LocalInfantry>, Without<InfantryGimbal>),
    >,
) {
    if mode.0 == FollowingType::Free {
        return;
    }

    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyW) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let (mut forces, mass) = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_impulse(mass.0 * dt * desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_impulse(mass.0 * dt * -brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_impulse(mass.0 * dt * -linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyQ) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyE) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

fn remote_vehicle_controls(
    time: Res<Time>,
    _mode: Res<CameraMode>,
    keyboard: Res<ButtonInput<KeyCode>>,
    infantry: Single<(Forces, &Mass), (With<InfantryRoot>, Without<LocalInfantry>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (Without<LocalInfantry>, Without<InfantryGimbal>),
    >,
    auto_move: Res<AutoMove>,
) {
    // 如果自动运动启用，则不执行手动控制
    if auto_move.enabled {
        return;
    }

    let mut input = Vec2::ZERO;
    if keyboard.pressed(KeyCode::KeyI) {
        input.y += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyK) {
        input.y -= 1.0;
    }
    if keyboard.pressed(KeyCode::KeyL) {
        input.x += 1.0;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        input.x -= 1.0;
    }

    let dt = time.delta_secs();
    let (mut forces, mass) = infantry.into_inner();

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    let (gimbal_transform, _gimbal) = gimbal.into_inner();

    let forward = gimbal_transform.forward().with_y(0.0);
    let right = gimbal_transform.right().with_y(0.0);
    let forward_xz = forward.with_y(0.0).normalize_or_zero();
    let right_xz = right.with_y(0.0).normalize_or_zero();

    let desired_dir = (forward_xz * input.y + right_xz * input.x).normalize_or_zero();
    forces.apply_linear_impulse(mass.0 * dt * desired_dir * VEHICLE_ACCEL);
    let linear_vel = forces.linear_velocity();
    let current_velocity = linear_vel.length();
    if current_velocity > MAX_VEHICLE_VELOCITY {
        let brake_force = linear_vel.normalize() * (current_velocity - MAX_VEHICLE_VELOCITY) * 50.0;
        forces.apply_linear_impulse(mass.0 * dt * -brake_force);
    } else if input == Vec2::ZERO {
        forces.apply_linear_impulse(mass.0 * dt * -linear_vel * 10.0);
    }

    if keyboard.pressed(KeyCode::KeyU) {
        chassis_data.yaw += VEHICLE_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyO) {
        chassis_data.yaw -= VEHICLE_ROTATION_SPEED * dt;
    }

    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis_data.yaw, 0.0, 0.0);
}

// 自动运动控制系统
fn auto_move_system(
    time: Res<Time>,
    config: Res<AutoMoveConfig>,
    mut auto_move: ResMut<AutoMove>,
    mut root_query: Query<
        (Forces, &Mass, &GlobalTransform),
        (With<InfantryRoot>, With<RemoteInfantry>),
    >,
    mut chassis_query: Query<
        (&mut Transform, &mut InfantryChassis),
        (With<RemoteInfantry>, Without<InfantryRoot>),
    >,
) {
    if !auto_move.enabled {
        return;
    }

    let Ok((mut forces, mass, global_transform)) = root_query.single_mut() else {
        warn!("Failed to get remote vehicle root for auto move");
        return;
    };

    let Ok((mut chassis_transform, mut chassis)) = chassis_query.single_mut() else {
        warn!("Failed to get remote vehicle chassis for auto move");
        return;
    };

    let dt = time.delta_secs();

    // 初始化起始位置（第一次启用时）
    if auto_move.start_position == Vec3::ZERO {
        auto_move.start_position = global_transform.translation();
        info!("Auto move initialized at position: {:?}", auto_move.start_position);
    }

    // 计算当前位置相对于起点的偏移
    let current_pos = global_transform.translation();
    let offset = current_pos - auto_move.start_position;
    let distance_along_direction = offset.dot(auto_move.direction);

    // 检查是否需要反向（基于实际位置）
    if distance_along_direction >= config.range && auto_move.moving_forward {
        auto_move.moving_forward = false;
        info!("Auto move: reversing direction (reached +{} meters)", distance_along_direction);
    } else if distance_along_direction <= 0.0 && !auto_move.moving_forward {
        auto_move.moving_forward = true;
        info!("Auto move: reversing direction (reached start position)");
    }

    // 计算目标速度（确保平均速度 >= min_speed）
    let target_speed = config.min_speed.max(1.0);

    // 计算运动方向
    let direction = if auto_move.moving_forward {
        auto_move.direction
    } else {
        -auto_move.direction
    };

    // 应用线性运动
    let velocity_xz = direction.normalize_or_zero() * target_speed;
    let current_vel = forces.linear_velocity();
    let current_vel_xz = Vec3::new(current_vel.x, 0.0, current_vel.z);

    // 使用更强的力来控制速度
    let vel_diff = velocity_xz - current_vel_xz;
    forces.apply_linear_impulse(mass.0 * dt * vel_diff * 20.0);

    // 应用自转（转/秒 转换为 弧度/秒）
    let rotation_speed_rad = config.rotation_speed * 2.0 * std::f32::consts::PI;
    chassis.yaw += rotation_speed_rad * dt;

    // 更新底盘的旋转 Transform
    chassis_transform.rotation = Quat::from_euler(EulerRot::YXZ, chassis.yaw, 0.0, 0.0);
}

// 键盘切换自动运动
fn toggle_auto_move(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut auto_move: ResMut<AutoMove>,
    query: Query<&GlobalTransform, (With<InfantryRoot>, With<RemoteInfantry>)>,
) {
    if keyboard.just_pressed(KeyCode::KeyP) {
        auto_move.enabled = !auto_move.enabled;
        if auto_move.enabled {
            // 启用时记录起始位置
            if let Ok(global_transform) = query.single() {
                auto_move.start_position = global_transform.translation();
                auto_move.current_distance = 0.0;
                auto_move.moving_forward = true;
            }
            info!("Auto move enabled for remote vehicle");
        } else {
            info!("Auto move disabled for remote vehicle");
        }
    }
}

/// Calculate shortest angle difference (handling ±π boundary)
fn angle_diff(current: f32, target: f32) -> f32 {
    let diff = target - current;
    let mut normalized = diff % (2.0 * std::f32::consts::PI);
    if normalized > std::f32::consts::PI {
        normalized -= 2.0 * std::f32::consts::PI;
    } else if normalized < -std::f32::consts::PI {
        normalized += 2.0 * std::f32::consts::PI;
    }
    normalized
}

fn gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (With<LocalInfantry>, Without<InfantryChassis>),
    >,
    auto_aim: Res<AutoAimMode>,
    target_euler: Res<TargetEuler>,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    // Choose control mode based on auto-aim status
    if auto_aim.enabled && target_euler.valid {
        // ============== ROS2 Auto-Aim Mode ==============
        let target_yaw = target_euler.yaw;
        let target_pitch = target_euler.pitch;

        // Calculate angle difference (handling ±π boundary)
        let yaw_diff = angle_diff(gimbal_data.local_yaw, target_yaw);
        let pitch_diff = target_pitch - gimbal_data.pitch;

        // Smooth rotation at fixed rate
        let max_delta = GIMBAL_ROTATION_SPEED * dt;

        // Yaw control - gradually approach target
        if yaw_diff.abs() > 0.001 {
            let yaw_step = yaw_diff.clamp(-max_delta, max_delta);
            gimbal_data.local_yaw += yaw_step;
        }

        // Pitch control - gradually approach target
        if pitch_diff.abs() > 0.001 {
            let pitch_step = pitch_diff.clamp(-max_delta, max_delta);
            gimbal_data.pitch += pitch_step;
        }
    } else {
        // ============== Manual Keyboard Control ==============
        if keyboard.pressed(KeyCode::ArrowLeft) {
            gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
        }
        if keyboard.pressed(KeyCode::ArrowRight) {
            gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
        }
        if keyboard.pressed(KeyCode::ArrowUp) {
            gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
        }
        if keyboard.pressed(KeyCode::ArrowDown) {
            gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
        }
    }

    // Clamp pitch to valid range (±45°)
    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    // Apply rotation to transform
    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);
    gimbal_transform.rotation = gimbal_rotation;
}

fn remote_gimbal_controls(
    time: Res<Time>,
    keyboard: Res<ButtonInput<KeyCode>>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal),
        (Without<LocalInfantry>, Without<InfantryChassis>),
    >,
) {
    let dt = time.delta_secs();
    let (mut gimbal_transform, mut gimbal_data) = gimbal.into_inner();

    (gimbal_data.local_yaw, gimbal_data.pitch, _) =
        gimbal_transform.rotation.to_euler(EulerRot::YXZ);

    if keyboard.pressed(KeyCode::KeyC) {
        gimbal_data.local_yaw += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyB) {
        gimbal_data.local_yaw -= GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyF) {
        gimbal_data.pitch += GIMBAL_ROTATION_SPEED * dt;
    }
    if keyboard.pressed(KeyCode::KeyV) {
        gimbal_data.pitch -= GIMBAL_ROTATION_SPEED * dt;
    }

    gimbal_data.pitch = gimbal_data.pitch.clamp(-0.785, 0.785);

    let gimbal_rotation =
        Quat::from_euler(EulerRot::YXZ, gimbal_data.local_yaw, gimbal_data.pitch, 0.0);

    gimbal_transform.rotation = gimbal_rotation;
}

fn following_controls(mut mode: ResMut<CameraMode>, keyboard: Res<ButtonInput<KeyCode>>) {
    if keyboard.just_pressed(KeyCode::F3) {
        mode.0 = match mode.0 {
            FollowingType::Free => FollowingType::Robot,
            FollowingType::Robot => FollowingType::ThirdPerson,
            FollowingType::ThirdPerson => FollowingType::Free,
        };
    }
}

/// Toggle auto-aim mode with T key
fn update_auto_aim_mode(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut auto_aim: ResMut<AutoAimMode>,
) {
    if keyboard.just_pressed(KeyCode::KeyT) {
        auto_aim.enabled = !auto_aim.enabled;
        info!(
            "Auto-aim mode: {}",
            if auto_aim.enabled { "ENABLED" } else { "DISABLED" }
        );
    }
}

fn update_camera_follow(
    camera_query: Single<(&mut Transform, &MainCamera), Without<LocalInfantry>>,
    infantry: Single<&Transform, (With<InfantryRoot>, With<LocalInfantry>)>,
    gimbal: Single<&Transform, (With<LocalInfantry>, With<InfantryGimbal>)>,
    view_offset: Single<&InfantryViewOffset, With<LocalInfantry>>,
    mode: Res<CameraMode>,
) {
    let gimbal_transform = gimbal.into_inner();
    let (mut camera_transform, camera_offset) = camera_query.into_inner();

    match mode.0 {
        FollowingType::Robot => {
            camera_transform.translation = infantry.translation
                + (infantry.rotation * gimbal_transform.rotation) * view_offset.0.translation;
            camera_transform.rotation = infantry.rotation * gimbal_transform.rotation;
        }
        FollowingType::ThirdPerson => {
            let base_transform = infantry.into_inner();
            let offset = base_transform.rotation * camera_offset.follow_offset;
            camera_transform.translation = base_transform.translation + offset;
            camera_transform.look_at(base_transform.translation, Vec3::Y);
        }
        FollowingType::Free => {}
    }
}

fn freecam_controls(
    time: Res<Time>,
    mode: Res<CameraMode>,
    mut mouse_motion_events: MessageReader<MouseMotion>,
    keyboard: Res<ButtonInput<KeyCode>>,
    camera_query: Single<&mut Transform, (With<MainCamera>, Without<InfantryRoot>)>,
) {
    if mode.0 != FollowingType::Free {
        return;
    }

    let delta = time.delta_secs();
    let mut camera_transform = camera_query.into_inner();

    let mut mouse_delta = Vec2::ZERO;
    for event in mouse_motion_events.read() {
        mouse_delta += event.delta;
    }

    if mouse_delta != Vec2::ZERO {
        let (yaw, pitch, roll) = camera_transform.rotation.to_euler(EulerRot::YXZ);

        let new_yaw = yaw - mouse_delta.x * 0.003;
        let new_pitch = (pitch - mouse_delta.y * 0.003).clamp(-1.4, 1.4);

        camera_transform.rotation = Quat::from_euler(EulerRot::YXZ, new_yaw, new_pitch, roll);
    }

    const CAMERA_SPEED: f32 = 8.0;
    let speed = CAMERA_SPEED * delta;
    let forward = camera_transform.forward();
    let right = camera_transform.right();
    let up = camera_transform.up();

    if keyboard.pressed(KeyCode::KeyW) {
        camera_transform.translation += forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyS) {
        camera_transform.translation -= forward * speed;
    }
    if keyboard.pressed(KeyCode::KeyA) {
        camera_transform.translation -= right * speed;
    }
    if keyboard.pressed(KeyCode::KeyD) {
        camera_transform.translation += right * speed;
    }
    if keyboard.pressed(KeyCode::KeyN) {
        camera_transform.translation += up * speed;
    }
    if keyboard.pressed(KeyCode::KeyJ) {
        camera_transform.translation -= up * speed;
    }
}

fn screenshot_on_f2(
    mut commands: Commands,
    input: Res<ButtonInput<KeyCode>>,
    mut counter: Local<u32>,
) {
    if input.just_pressed(KeyCode::F2) {
        let path = format!("./screenshot-{}.png", *counter);
        *counter += 1;
        commands
            .spawn(Screenshot::primary_window())
            .observe(save_to_disk(path));
    }
}

fn screenshot_saving(
    mut commands: Commands,
    screenshot_saving: Query<Entity, With<Capturing>>,
    window: Single<Entity, With<Window>>,
) {
    match screenshot_saving.iter().count() {
        0 => {
            commands.entity(*window).remove::<CursorIcon>();
        }
        x if x > 0 => {
            commands
                .entity(*window)
                .insert(CursorIcon::from(SystemCursorIcon::Progress));
        }
        _ => {}
    }
}
