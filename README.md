# anti_UAV

基于 ROS 2 Humble 的反无人机视觉跟踪流水线：**相机取流 → YOLO11n 检测 → 单目标跟踪 → 云台系绝对目标角输出**。

当前仓库完成了感知链路（camera / detector / tracker）和串口桥（serial），控制律与发射决策在下位机。串口把 `TrackedTarget` 投影成固定格式的二进制帧下发，并把下位机回传的云台姿态发布为 `GimbalState`；tracker 反过来订阅 `GimbalState`，把光轴误差合成成绝对目标角。

## 系统结构

```
V4L2 相机 (1280x720 MJPG)
        │  裁剪 640x480
        ▼
autoaim_camera ── image_raw (bgr8) ──┐
        └─────── camera_info ────────┤
                                     ▼
                          autoaim_detector (YOLO11n, OpenVINO GPU/CPU 或 Horizon BPU)
                                     │  Detection
                                     ▼
                          autoaim_tracker (α-β 滤波 + 去畸变投影 + 姿态配对)
                                     │  TrackedTarget
                                     ▼
                          autoaim_serial (帧编解码 + 串口)
                                     │  GimbalCommand / UART 命令帧
                                     ▼
                          下位机（控制律、云台、发射决策）
                                     │  UART 反馈帧
                                     ▼
                          autoaim_serial/recv (GimbalState)
                                     │
                                     └──► autoaim_tracker（合成绝对目标角）
```

`autoaim_bringup` 用单个 `component_container_mt` 加载相机/检测/跟踪三个节点，节点间通过进程内通信（intra-process comms）传递图像，避免整帧拷贝。串口节点默认不加载（无硬件时避免拖垮整个容器），用 `use_serial:=true` 加入。

## 包说明

| 包 | 作用 |
| --- | --- |
| `autoaim_camera` | V4L2 取流、裁剪、发布图像与 `CameraInfo`，采集线程独立于 ROS 执行器 |
| `autoaim_detector` | YOLO11n 推理，共享前后处理 + 可切换推理后端（OpenVINO / Horizon） |
| `autoaim_tracker` | 单目标跟踪、状态机、像素→光轴误差，并把误差与观测时刻的云台姿态合成绝对目标角 |
| `autoaim_serial` | 串口桥：订阅 `TrackedTarget` 下发命令帧，发布 `send` 镜像；接收反馈帧发布 `recv` |
| `autoaim_interfaces` | `Detection`、`TrackedTarget`、`GimbalCommand`、`GimbalState` 消息 |
| `autoaim_bringup` | 组合启动（零拷贝） |

## 环境依赖

- ROS 2 Humble
- OpenCV（`libopencv-dev`）、`camera_info_manager`、`cv_bridge`、`image_transport`
- OpenVINO（x86，默认启用）**或** Horizon hbDNN SDK（RDK 板端）
- 工具脚本额外需要：`rosbag2_py`、`rosidl_runtime_py`、`PyYAML`、`numpy`

## 构建

```bash
source /opt/ros/humble/setup.bash
./compile.sh                 # colcon build --symlink-install + compile_commands.json
source install/setup.bash
```

只构建单个包：

```bash
colcon build --packages-select autoaim_detector
```

改动 `autoaim_interfaces` 后，依赖它的包需要一起重建，否则会拿到旧消息定义。

串口协议有单元测试（CRC、帧格式、重同步、NaN 清洗、解析错误上报），tracker 的姿态配对也有单元测试：

```bash
colcon test --packages-select autoaim_tracker autoaim_serial
colcon test-result --verbose
```

后端是纯 ON/OFF 的 CMake 开关，默认 `AUTOAIM_WITH_OPENVINO=ON`、`AUTOAIM_WITH_HORIZON=OFF`。开关为 ON 但探测不到 SDK 时，configure 阶段直接失败：

```bash
# RDK 板端（Horizon BPU）
colcon build --packages-select autoaim_detector \
  --cmake-args -DAUTOAIM_WITH_HORIZON=ON -DAUTOAIM_WITH_OPENVINO=OFF
```

> 模型目录 `src/autoaim_detector/model/` 被 `.gitignore` 忽略，需要自行放入模型文件。

## 运行

组合启动（推荐，进程内零拷贝）：

```bash
source install/setup.bash
ros2 launch autoaim_bringup autoaim.launch.py
# 接上下位机后：
ros2 launch autoaim_bringup autoaim.launch.py use_serial:=true
```

单节点调试：

```bash
ros2 launch autoaim_camera autoaim_camera.launch.py
ros2 launch autoaim_detector autoaim_detector.launch.py
ros2 launch autoaim_tracker autoaim_tracker.launch.py
ros2 launch autoaim_serial autoaim_serial.launch.py
```

串口节点在设备不存在时只打印错误并按 `reconnect_interval_s` 重试，设备出现后自动接上，不会让整个容器退出；波特率不在支持表里属于配置错误，启动仍直接抛错。

单独启动时各节点跨进程通信，图像会走 DDS 序列化，不再是零拷贝。

检查链路：

```bash
ros2 topic hz autoaim_camera/image_raw
ros2 topic echo autoaim_detector/detection
ros2 topic echo autoaim_tracker/target
ros2 topic echo autoaim_serial/send
ros2 topic echo autoaim_serial/recv
ros2 run rqt_image_view rqt_image_view autoaim_camera/image_raw
```

## 话题

| 话题 | 类型 | 发布者 | 说明 |
| --- | --- | --- | --- |
| `autoaim_camera/image_raw` | `sensor_msgs/Image` | camera | bgr8，SensorDataQoS |
| `autoaim_camera/camera_info` | `sensor_msgs/CameraInfo` | camera | 与图像同 stamp，可按帧配对 |
| `autoaim_detector/detection` | `autoaim_interfaces/Detection` | detector | 每帧都发布，无目标时 `detected=false` |
| `autoaim_detector/labeled_image` | `sensor_msgs/Image` | detector | 调试图，`enable_labeled_image` 控制 |
| `autoaim_tracker/target` | `autoaim_interfaces/TrackedTarget` | tracker | 状态 + 滤波框 + 绝对目标角/角速度，每个检测帧都发布（姿态不可用的 TRACKING 帧除外） |
| `autoaim_tracker/debug_image` | `sensor_msgs/Image` | tracker | 调试图，`enable_debug_image` 控制 |
| `autoaim_serial/send` | `autoaim_interfaces/GimbalCommand` | serial | 本帧写入串口的命令镜像，约 90Hz |
| `autoaim_serial/recv` | `autoaim_interfaces/GimbalState` | serial | 下位机回传的云台姿态，每收到一帧有效反馈发布一次 |

### Detection

`header`、`detected`、`score`、框中心 `x/y` 与角点 `x1/y1/x2/y2`，均为原图像素坐标。无目标时 `detected=false`，其余字段为 0。

### TrackedTarget

- `status`：`0=LOST`、`1=TRACKING`、`2=HOLDING`（枚举只在 tracker 内部维护，消息不带常量）
- `x/y/x1..y2`：滤波后的框，原图像素坐标；仅 TRACKING 有值
- `yaw_target` / `pitch_target`：云台坐标系下的绝对目标角（rad）；仅 TRACKING 有值
- `yaw_rate` / `pitch_rate`：目标角速度估计（rad/s），供下游做延迟补偿；仅 TRACKING 有值

HOLDING/LOST 时除 `status` 外所有数值字段为 0（包括框和角度），且每个检测帧都会发布一条消息。

`yaw_target = 观测时刻云台 yaw + 光轴 yaw 误差`。姿态按 `Detection.header.stamp` 在 `GimbalState` 历史里选最近样本，所以下发的是"转到多少度"而不是"还差多少度"：同一帧重复执行是幂等的。**拿不到匹配姿态的 TRACKING 帧不发布**（不伪造 `status=1` + 零值），命令流出现空洞由下位机串口超时保护接管。

**角度符号**：yaw 向左为正，pitch 向下为正。图像 x 向右、y 向下，因此目标在画面右侧时光轴误差为负，合成后的 `yaw_target` 自然就比当前姿态小。符号写死在代码里，装配方向不同请在相机端翻转图像或在下游翻电机方向，不要改消息语义。目标角不做 ±π 展开，下位机需接受连续角。

## 配置

所有运行时参数都没有默认值，必须由对应 YAML 显式提供；缺失时节点启动即抛 `ParameterUninitializedException`。**YAML 顶层 key 必须等于节点名**（`autoaim_camera` / `autoaim_detector` / `autoaim_tracker` / `autoaim_serial`），否则参数文件会被静默忽略。

### 相机 `src/autoaim_camera/config/config.yaml`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/video0` | V4L2 设备 |
| `fourcc` | `MJPG` | 必须是 4 字符 |
| `width` / `height` / `fps` | `1280` / `720` / `60.0` | 采集分辨率与帧率 |
| `crop_x` / `crop_y` | `320` / `120` | 裁剪窗口左上角（采集帧坐标） |
| `crop_width` / `crop_height` | `640` / `480` | 发布尺寸 |
| `buffer_size` | `4` | V4L2 缓冲数；该 USB2.0 相机设为 1 会严重丢帧，勿改 |
| `frame_id` | `camera_optical` | 图像坐标系 |
| `image_topic` / `camera_info_topic` | `autoaim_camera/...` | 发布话题 |
| `camera_info_url` | `package://autoaim_camera/config/camera_info.yaml` | 标定文件 |

`camera_info.yaml` 描述的是**裁剪后**的图像尺寸。当前 `fx=fy=554`、畸变系数全 0 是按 60° FOV 估的占位值，实标后直接替换该文件即可，代码无需改动。

### 检测 `src/autoaim_detector/config/detector.yaml`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `model_path` | `finetuned_p2.onnx` | 相对 `share/autoaim_detector/model/` 解析，不能为空 |
| `enable_gpu` | `true` | OpenVINO 设备选择：true=GPU，false=CPU |
| `enable_bpu` | `false` | Horizon hbDNN 后端，读 `.bin` |
| `confidence_thres` | `0.2` | 置信度阈值 |
| `enable_labeled_image` | `true` | 发布标注图，调试用，正式运行建议关闭 |

`enable_gpu` 与 `enable_bpu` 不能同时为 true。

**模型契约**：输入 float32 `[1,3,640,640]` NCHW；输出为 NHWC 的成对张量——分类头单通道 raw logits（需 sigmoid），框头 64 通道 DFL（`reg_max=16`，需 softmax + 期望）。尺度由 `stride = 640 / height` 动态识别，支持 stride 4/8/16/32（P2/P3/P4/P5），不硬编码 3 个尺度。

**后处理只取全图得分最高的一个 cell**，不做 NMS，也不支持多目标。当前模型在 `model/` 下提供了 `best_p2.onnx`、`best_p345.onnx`（更快、少 P2 小目标头）和 `finetuned_p2.onnx`。

### 跟踪 `src/autoaim_tracker/config/tracker.yaml`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `gimbal_state_topic` | `autoaim_serial/recv` | 云台姿态输入，用于合成绝对目标角 |
| `attitude_timeout_s` | `0.05` | 观测时刻与最近姿态样本的最大允许间隔，超过则该 TRACKING 帧不发布 |
| `filter_alpha` / `filter_beta` | `0.5` / `0.1` | α-β 滤波增益（像素域常速模型） |
| `max_jump_px` | `120.0` | 跳变门限，超过视为误检 |
| `max_speed_px_s` | `4000.0` | 速度估计上限 |
| `hold_frames` | `3` | 连续无有效观测多少帧转 HOLDING |
| `lost_frames` | `10` | 连续无有效观测多少帧转 LOST |
| `enable_debug_image` | `false` | 发布调试图 |

状态完全由**连续无有效观测的推理帧数**决定：`miss_count >= lost_frames` 为 LOST，`>= hold_frames` 为 HOLDING，否则 TRACKING；接受新观测立即回到 TRACKING。跳变门限拒绝的帧同样计一次 miss。`hold_frames` 必须大于 0，`lost_frames` 必须大于 `hold_frames`。

HOLDING/LOST 期间不外推位置，`status` 之外的字段全部为 0；tracker 仍然每个检测帧发布一条消息，下游因此能拿到连续的状态流。相机掉线或 detector 崩溃时 tracker 收不到消息，串口链路随之静默；视觉侧不伪造心跳，时效性由下位机自己的串口超时判断。

角度合成需要与观测时刻同一瞬间的云台姿态：tracker 缓存最近 256 帧 `GimbalState`，按 `Detection.header.stamp` 选时间最近的样本，超过 `attitude_timeout_s` 就认为不可用。用"最新姿态 + 旧误差"会把云台自身的运动正反馈进设定值，必须避免。

单目台架调试（`use_serial:=false`）时没有姿态来源，检测到目标期间 target 话题会停止发布；需要看角度就伪造 `GimbalState`（`tools/serial_pty_test.py` 或 `ros2 topic pub`）。

内参来自 `CameraInfo`（`camera_info_manager` 加载标定文件），未收到内参前不发布有效目标。像素→角度用 `undistortPoints` + `atan`，畸变系数全 0 时退化为无畸变模型。

### 串口 `src/autoaim_serial/config/serial.yaml`

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `device` | `/dev/ttyUSB0` | 建议改用 `/dev/serial/by-id/...`，避免 ttyUSB 编号漂移 |
| `baudrate` | `115200` | 必须与下位机一致；支持到 921600 |
| `target_topic` | `autoaim_tracker/target` | 输入 |
| `send_topic` / `feedback_topic` | `autoaim_serial/send` / `autoaim_serial/recv` | 输出 |
| `frame_id` | `gimbal` | 反馈 header 的坐标系 |
| `feedback_timeout_s` | `0.20` | 反馈超过该时间未到就在统计日志里打印 ERROR，不会伪造帧 |
| `reconnect_interval_s` | `1.0` | 掉线/设备不存在时的重试间隔 |
| `statistics_period_s` | `5.0` | 统计日志周期 |

每条 `TrackedTarget` 发一帧（约 90Hz），HOLDING/LOST 也持续发送。链路静默由下位机超时处理，视觉侧不发心跳。

设备打不开时节点打印 ERROR 并持续重试，设备出现后自动接上。解析失败（扫描不到帧头、长度非法、CRC 不符、反馈值非法）会限流告警并打印出错位置起的原始字节，便于直接看线上数据。

帧格式、CRC 测试向量与下位机 C 参考实现见 [`docs/serial_protocol.md`](docs/serial_protocol.md)。

## 工具

`tools/` 下的辅助脚本：

```bash
# rosbag 图像话题 → MP4
python3 tools/rosbag_to_mp4.py <bag_dir> --topic /autoaim_camera/image_raw --output out.mp4 --fps 30

# 视频 → 逐帧图片
python3 tools/video_to_frames.py input.mp4 --output frames/ --step 1 --ext jpg

# 串口协议 pty 回环：无硬件时模拟下位机（脚本会打印设备路径）
python3 tools/serial_pty_test.py --rate 50
```

## 已知限制

- 仅单目标、单峰 argmax，无 NMS / 多目标跟踪。
- 未建立云台-相机外参，光轴与 yaw/pitch 轴按平行处理；单目框尺寸测距未实现。
- 角度符号硬编码，不支持参数翻转。
- Horizon BPU 后端在本机无 SDK，未实机验证；仓库中没有 `.bin` 模型。
- 串口协议 v1 尚未与实机下位机联调；没有时间同步和端到端延迟测量，下位机只能按 `yaw_rate/pitch_rate` 自行外推。
- 绝对目标角的姿态配对用的是"视觉侧收到反馈帧的时刻"（协议没有板端采样时间戳），误差约为传输延迟加半个反馈周期；反馈频率越低偏差越大。
- yaw/pitch 直接相加忽略两轴耦合，光轴误差超过约 10° 时目标角会有可见偏移。
- 串口节点独占设备，同一个 `/dev/tty*` 不能同时启动两个实例。
- `autoaim_camera/config/camera_info.yaml` 尚未实标。

## 开发约定

- 节点名必须等于 YAML 顶层 key；组件插件名必须为 `CameraNode::CameraNode`、`DetectorNode::DetectorNode`、`TrackerNode::TrackerNode`、`SerialNode::SerialNode`，与 CMake `PLUGIN` 和 `RCLCPP_COMPONENTS_REGISTER_NODE` 保持同步。
- 新增参数一律 `declare_parameter<T>("name")`，不设默认值，并同步更新对应 YAML。
- 格式化使用根目录 `.clang-format`（4 空格、100 列、指针左对齐）；该文件被 `.gitignore` 忽略，新 clone 中可能不存在。
- 进程内零拷贝依赖三点：节点构造函数开启 `use_intra_process_comms(true)`、相机端用 `unique_ptr` 发布、订阅端用 `ConstSharedPtr` 回调 + `toCvShare`。
- 提交信息使用中文，保持简短。
