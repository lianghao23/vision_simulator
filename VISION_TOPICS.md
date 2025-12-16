# Vision ROS2 Topics Integration

本项目实现了通过 ROS2 话题进行视觉系统通信的功能。

## 话题说明

### 发布话题: `/vision_recv_data`

模拟器发布云台当前状态信息，包括：

- **消息格式**: `std_msgs/String` (JSON 格式)
- **频率**: 根据 Update 系统频率
- **内容**:
  ```json
  {
    "timestamp": 1234567890,
    "self_color": 2,           // 1=RED, 2=BLUE
    "work_mode": 0,            // 0=AUTO_SHOOT, 1=AUTO_SBUFF, 2=AUTO_BBUFF
    "bullet_speed": 15,        // 10, 15, 16, 18, 30
    "roll": 0.0,
    "pitch": 0.1,
    "yaw": 0.5,
    "vel_x": 1.0,
    "vel_y": 0.0,
    "vel_yaw": 0.2,
    "control_id": 1.0,
    "game_progress": 0.0,
    "stage_remain_time": 420.0,
    "current_hp": 600.0,
    "current_enemy_sentry_hp": 600.0,
    "current_enemy_base_hp": 5000.0,
    "current_virtual_shield": 100.0,
    "current_base_hp": 5000.0
  }
  ```

### 订阅话题: `/vision_send_data`

模拟器订阅视觉算法发送的云台控制指令：

- **消息格式**: `std_msgs/String` (JSON 格式)
- **内容**:
  ```json
  {
    "timestamp": 1234567890,
    "target_state": 2,         // 0=LOST_TARGET, 1=CONVERGING, 2=FIRE
    "target_type": 3,          // 0=NONE, 1=HERO, 2=ENGINEER, 3-5=INFANTRY, 6=OUTPOST, 7=SENTRY, 8=BASE
    "pitch": 0.1,              // 目标俯仰角 (rad)
    "yaw": 0.5,                // 目标偏航角 (rad)
    "target_distance": 5.0,    // 目标距离 (m)
    "vel_x": 0.0,
    "vel_y": 0.0,
    "vel_yaw": 0.0,
    "control_id": 1.0
  }
  ```

## 配置参数

可以通过修改 `VisionConfig` 资源来配置系统参数（在 `src/ros2/plugin.rs` 中）：

```rust
VisionConfig {
    bullet_speed: 15,      // INFANTRY15
    self_color: 2,         // BLUE
    work_mode: 0,          // AUTO_SHOOT
    game_progress: 0.0,
    stage_remain_time: 420.0,
    current_hp: 600.0,
    // ...
}
```

## 云台控制

当接收到 `/vision_send_data` 消息后，系统会自动平滑插值到目标角度：

- 使用平滑插值避免抖动
- 俯仰角限制在 [-0.785, 0.785] rad (约 ±45°)
- 可通过 `GIMBAL_LERP_SPEED` 调整响应速度

## 使用示例

### Python 发布控制指令

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')
        self.publisher = self.create_publisher(String, '/vision_send_data', 10)
        
    def send_gimbal_command(self, pitch, yaw, distance):
        msg = String()
        data = {
            "timestamp": self.get_clock().now().nanoseconds,
            "target_state": 2,  # FIRE
            "target_type": 3,   # INFANTRY3
            "pitch": pitch,
            "yaw": yaw,
            "target_distance": distance,
            "vel_x": 0.0,
            "vel_y": 0.0,
            "vel_yaw": 0.0,
            "control_id": 1.0
        }
        msg.data = json.dumps(data)
        self.publisher.publish(msg)
```

### Python 订阅状态信息

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VisionReceiver(Node):
    def __init__(self):
        super().__init__('vision_receiver')
        self.subscription = self.create_subscription(
            String,
            '/vision_recv_data',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        data = json.loads(msg.data)
        print(f"Gimbal: pitch={data['pitch']:.3f}, yaw={data['yaw']:.3f}")
```

## 构建与运行

```bash
# 构建并运行模拟器
source /opt/ros/humble/setup.bash
cargo build --release
cargo run --release
```

## 注意事项

1. 消息使用 JSON over `std_msgs/String` 格式，便于跨语言兼容
2. 云台控制采用平滑插值，避免突变
3. 所有角度单位为弧度 (radians)
4. 可通过修改 `VisionConfig` 资源动态调整参数
