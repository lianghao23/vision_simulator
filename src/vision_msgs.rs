// Vision custom message definitions using JSON over std_msgs/String
use r2r::std_msgs::msg::String as RosString;
use serde::{Deserialize, Serialize};

// Constants for message types
// pub mod bullet_speed {
//     pub const BULLET_SPEED_NONE: u8 = 0;
//     pub const HERO10: u8 = 10;
//     pub const HERO16: u8 = 16;
//     pub const INFANTRY15: u8 = 15;
//     pub const INFANTRY18: u8 = 18;
//     pub const INFANTRY30: u8 = 30;
// }

// pub mod self_color {
//     pub const COLOR_NONE: u8 = 0;
//     pub const RED: u8 = 1;
//     pub const BLUE: u8 = 2;
// }

// pub mod work_mode {
//     pub const AUTO_SHOOT: u8 = 0;
//     pub const AUTO_SBUFF: u8 = 1;
//     pub const AUTO_BBUFF: u8 = 2;
// }

// pub mod target_state {
//     pub const LOST_TARGET: u8 = 0;
//     pub const CONVERGING: u8 = 1;
//     pub const FIRE: u8 = 2;
// }

// pub mod target_type {
//     pub const NONE: u8 = 0;
//     pub const HERO: u8 = 1;
//     pub const ENGINEER: u8 = 2;
//     pub const INFANTRY3: u8 = 3;
//     pub const INFANTRY4: u8 = 4;
//     pub const INFANTRY5: u8 = 5;
//     pub const OUTPOST: u8 = 6;
//     pub const SENTRY: u8 = 7;
//     pub const BASE: u8 = 8;
// }

// VisionRecvData message - serialized as JSON over std_msgs/String
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
pub struct VisionRecvData {
    pub timestamp: i64,  // nanoseconds
    pub self_color: u8,
    pub work_mode: u8,
    pub bullet_speed: u8,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub vel_yaw: f32,
    pub control_id: f32,
    pub game_progress: f32,
    pub stage_remain_time: f32,
    pub current_hp: f32,
    pub current_enemy_sentry_hp: f32,
    pub current_enemy_base_hp: f32,
    pub current_virtual_shield: f32,
    pub current_base_hp: f32,
}

impl VisionRecvData {
    #[allow(dead_code)]
    pub fn to_ros_string(&self) -> RosString {
        RosString {
            data: serde_json::to_string(self).unwrap_or_default(),
        }
    }
}

// VisionSendData message - serialized as JSON over std_msgs/String
#[derive(Debug, Clone, Serialize, Deserialize)]
#[allow(dead_code)]
pub struct VisionSendData {
    pub timestamp: i64,  // nanoseconds
    pub target_state: u8,
    pub target_type: u8,
    pub pitch: f32,
    pub yaw: f32,
    pub target_distance: f32,
    pub vel_x: f32,
    pub vel_y: f32,
    pub vel_yaw: f32,
    pub control_id: f32,
}

impl VisionSendData {
    #[allow(dead_code)]
    pub fn from_ros_string(msg: &RosString) -> Option<Self> {
        serde_json::from_str(&msg.data).ok()
    }
}
