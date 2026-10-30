# AGENTS.md

ROS 2 Humble (ament_cmake) 工作空间，位于 `/home/walt/ros_ws/anti_UAV`，根目录即 colcon 工作空间。

## 仓库结构

- `src/autoaim_camera/`：V4L2 相机取流（C++ rclcpp + OpenCV），经 rclcpp 原生发布器以 unique_ptr 发布 `autoaim_camera/image_raw`（bgr8，SensorDataQoS），并用 `camera_info_manager` 加载 `config/camera_info.yaml`、随图像每帧发布 `autoaim_camera/camera_info`。发布前按 `crop_x/crop_y/crop_width/crop_height`（采集帧像素坐标）裁剪子图，默认 1280x720 居中取 640x480；`camera_info.yaml` 必须描述裁剪后的图像尺寸。
- `src/autoaim_interfaces/`：纯消息包，`msg/Detection.msg`（检测结果）、`msg/TrackedTarget.msg`（跟踪状态与云台系绝对目标角）、`msg/GimbalCommand.msg`（串口命令的 ROS 镜像）、`msg/GimbalState.msg`（下位机回传姿态），供检测/跟踪/串口复用。
- `src/autoaim_detector/`：YOLO11n 无人机检测。共享 preprocess/postprocess + 可切换推理后端。
- `src/autoaim_tracker/`：单目标跟踪。订阅 `Detection` + `CameraInfo` + `GimbalState`，α-β 滤波后把光轴误差与观测时刻的云台姿态合成绝对目标角，输出 `autoaim_tracker/target`。只做状态估计、误差解算与坐标合成，不含控制律。
- `src/autoaim_serial/`：串口桥。订阅 `TrackedTarget` 下发命令帧，同时发布 `autoaim_serial/send`；读反馈帧发布 `autoaim_serial/recv`。控制律在下位机，这里不做限幅/预测/发射决策。
- `src/autoaim_bringup/`：纯 launch 包。`autoaim.launch.py` 用单个 `component_container_mt` 加载相机+检测+跟踪三个组件（节点构造函数开 `use_intra_process_comms`，进程内零拷贝），`use_serial:=true` 时把串口节点作为第 4 个组件加入。
- `build/`、`install/`、`log/`：构建产物，不要手改。

## 构建与运行

- 先 source ROS，否则 `colcon`/`ros2` 不可用：`source /opt/ros/humble/setup.bash`。
- 全量构建：`./compile.sh`（`--symlink-install` + `compile_commands.json`）。
- 只构建某个包：`colcon build --packages-select autoaim_detector`（改动 `autoaim_interfaces` 后要把它一起构建，否则依赖包拿到旧消息）。
- 运行：`source install/setup.bash && ros2 launch autoaim_bringup autoaim.launch.py`（组合：单容器装相机+检测+跟踪；接串口用 `use_serial:=true`）；单节点调试用 `ros2 launch autoaim_camera autoaim_camera.launch.py`、`ros2 launch autoaim_detector autoaim_detector.launch.py`、`ros2 launch autoaim_tracker autoaim_tracker.launch.py` 或 `ros2 launch autoaim_serial autoaim_serial.launch.py`。
- 协议 codec 有 gtest：`colcon test --packages-select autoaim_serial`（覆盖 CRC、帧字节、重同步、NaN 清洗、解析错误上报）；`autoaim_tracker` 有姿态配对 gtest（`colcon test --packages-select autoaim_tracker`）；其余验证 = 编译通过 + 启动不报参数/插件错误。

## 检测器后端（关键）

- 两个 CMake 开关（纯 ON/OFF），默认 `AUTOAIM_WITH_OPENVINO=ON`、`AUTOAIM_WITH_HORIZON=OFF`；ON 但探测不到 SDK 时 configure 直接 `FATAL_ERROR`。Horizon 用 `-DHORIZON_SDK_ROOT=<sdk>`；RDK 板端：`colcon build --packages-select autoaim_detector --cmake-args -DAUTOAIM_WITH_HORIZON=ON -DAUTOAIM_WITH_OPENVINO=OFF`。
- 运行时参数 `enable_gpu`：OpenVINO 设备 GPU/CPU（读 `.onnx`）；`enable_bpu`：Horizon hbDNN 读 `.bin`（本机无 SDK，未实测；NV12 U8 输入，输出按 `tensorType` 分两支：int32 + SCALE/SHIFT 走反量化，float32 直接读取）。两者同时为 true 直接抛错。`best.bin` 目前不存在，`enable_bpu:=true` 会报错。
- `model_path` 不能为空（为空直接抛错），其值始终相对模型目录 `share/autoaim_detector/model/` 解析（如 `best_p2.onnx`）；`model/` 目录被 `.gitignore` 忽略，CMake 仅在目录存在时安装。
- 模型契约（动预处理/后处理前必读）：输入 float32 `[1,3,640,640]` NCHW；NHWC 原始输出，每个尺度成对（cls 单通道 **raw logits，需 sigmoid**；box 64 通道 DFL，reg_max=16，需 softmax+期望）；尺度按 `stride = kInputSize / height` 动态识别，支持 stride 4/8/16/32（P2/P3/P4/P5），不再硬编码 3 尺度。
- 无目标也发布 `Detection`，`detected=false`、其余字段为 0。

## 跟踪（tracker，关键）

- 输出 `TrackedTarget`：状态 `0=LOST`、`1=TRACKING`、`2=HOLDING`。状态完全由连续无有效观测的**推理帧数**决定：连续 `hold_frames` 帧转 HOLDING，`lost_frames` 帧转 LOST；接受新观测立即回 TRACKING，跳变门限拒绝也算一个丢失帧。**每个检测帧都发布**，但只有 TRACKING 携带数值；HOLDING/LOST 时除 `status` 外所有字段（框、角度、角速度）都是 0。不做时间外推，HOLDING/LOST 期间 `header.stamp` 仍是最后观测时刻。
- **绝对目标角合成（核心）**：tracker 订阅 `GimbalState`（`gimbal_state_topic`，默认 `autoaim_serial/recv`），把最近 256 帧姿态按 `header.stamp` 缓存；TRACKING 时取与 `Detection.header.stamp` 最近的样本，`|Δt| > attitude_timeout_s` 就认为不可用 → 限流告警并**丢弃该帧不发布**（不能发 `status=1` + 零值，那等于命令转到零位）。`yaw_target = 姿态 yaw + 光轴 yaw 误差`，不做 ±π 展开、不做两轴耦合修正。绝不能用"最新姿态 + 旧误差"：那会把云台自身运动正反馈进设定值，延迟振荡只是换了个位置。
- 姿态缺失时（刚启动、串口断开、`use_serial:=false` 台架调试）tracker 不终止，只是停止产出 TRACKING 目标；需要台架看角度就伪造 `GimbalState`。
- 断流（相机掉线、detector 崩溃）时 tracker 收不到消息，也就不会发布，串口链路随之静默；时效性由下游/下位机的超时判断，tracker 内部不做超时。
- 角度符号写死在代码里：**yaw 向左为正，pitch 向下为正**。图像 x 向右、y 向下，因此目标在右侧时光轴误差为负、在下方时为正；绝对目标角与 `GimbalState` 同一零位、同一符号。没有符号参数；装配反向时在相机端翻转图像或在 controller 里翻电机方向，不改变 tracker 的消息语义。
- 内参不走节点参数：相机节点用 `camera_info_manager` 加载标准 ROS 标定文件 `autoaim_camera/config/camera_info.yaml`，tracker 订阅 `CameraInfo` 缓存 K/D；收到之前只发布 LOST，不发布有效目标。像素→角度用 `undistortPoints` + `atan`，D 全 0 时退化为无畸变。
- 未建云台-相机外参（光轴与 yaw/pitch 轴按平行处理）；单目框尺寸测距未做。
- `camera_info.yaml` 中 `fx/fy=554`、畸变系数 0 是按 60° FOV 估的占位值，实标后直接替换该文件（`camera_calibration` 的输出格式），代码不用改。

## 串口（serial，关键）

- `autoaim_serial` 是流水线终点：订阅 `autoaim_tracker/target`，把 status / yaw_target / pitch_target / yaw_rate / pitch_rate 原样投影，写串口并发布镜像 `autoaim_serial/send`；读反馈帧发布 `autoaim_serial/recv`（`GimbalState`），该消息又被 tracker 用于合成绝对目标角。控制律不在 ROS 侧。
- 线协议 v1.1 见 `docs/serial_protocol.md`：`AA 55` + msg_id + seq + len + payload + CRC16-CCITT，小端，无帧尾、无转义。命令帧 24 字节（msg_id `0x01`），反馈帧 15 字节（msg_id `0x02`），角度全部是原生 float32，不做定点量化。**偏移 6..13 是绝对目标角，不是误差**；这是 2026-10-28 的语义变更，布局没变，新旧固件混用不会报错而会算错，必须两端同版本刷写。
- 发送跟随 tracker 消息（约 90Hz），**不设定频定时器，也不在链路静默时伪造 LOST 心跳**：静默与 `status=0` 语义不同，链路故障由下位机串口超时保护。HOLDING/LOST 期间 tracker 仍逐帧发布（数值 0），所以正常时链路不会静默；tracker 拿不到匹配姿态时该帧不发布，链路出现空洞是刻意的。
- 逐字节读写，**禁止 memcpy 结构体上串口**；编码前用 `std::isfinite` 清洗 NaN/Inf（置 0 + 计数 + 告警）。解析器按 `len` 跳过未知 msg_id，CRC 失败逐字节回退重扫，坏长度上限 32 字节；四类解析错误（`kGarbage`/`kBadLength`/`kBadCrc`/`kBadFeedback`）通过 `ParseErrorCallback` 上报，节点限流告警并打印出错位置起的原始字节。
- 串口 fd 单节点独占；读线程 `poll(50ms)`，析构先 join 再关 fd；写/重连共用 `port_mutex_`，写超时 5ms。read 返回 0/EIO/POLLERR 后按 `reconnect_interval_s` 重连。
- 设备打不开**不 fail-fast**：构造时打印 ERROR，`port_ready_=false`，读线程按 `reconnect_interval_s` 持续重试，设备出现即自动接上（让无硬件的台架调试不炸）。只有参数错误（如非法 `baudrate`）仍在启动时抛错。反馈超过 `feedback_timeout_s` 未到只在统计定时器里打印 ERROR，不关节点、不伪造帧。
- bringup 里 `use_serial` 默认 false：没有下位机时不要拖垮相机链路。
- 改协议必须同步 `docs/serial_protocol.md` 与两端实现；字段的偏移/类型不变，含义变更必须像 v1.1 那样在文档版本表登记并强制两端同版本刷写；扩展新消息只加 msg_id。
- 无硬件联调：`python3 tools/serial_pty_test.py` 造 pty 对端，节点用 `-p device:=/dev/pts/N` 指向它。

## 命名不变量

- 运行时节点名必须等于 YAML 顶层 key：相机 `autoaim_camera`，检测 `autoaim_detector`，跟踪 `autoaim_tracker`，串口 `autoaim_serial`（不匹配则整个参数文件被静默忽略）。
- 组件插件名必须与「命名空间::类名」一致：相机 `CameraNode::CameraNode`、检测 `DetectorNode::DetectorNode`、跟踪 `TrackerNode::TrackerNode`、串口 `SerialNode::SerialNode`（CMake `PLUGIN` 与 `RCLCPP_COMPONENTS_REGISTER_NODE` 同步，否则组件加载失败）。可执行名分别为 `camera_node`、`detector_node`、`tracker_node`、`serial_node`。

## 参数约定

- 所有运行时参数都不得有默认值：一律 `declare_parameter<T>("name")`，必须由对应 `config/*.yaml` 显式提供；缺失时节点启动即抛 `ParameterUninitializedException`（“Statically typed parameter ... must be initialized”）。新增/改名参数时同步更新对应 YAML。
- 例外：`camera_info.yaml` 是标准 ROS 标定文件（`image_width`/`camera_matrix`/...，不是 `ros__parameters`），由相机参数 `camera_info_url` 指向（`package://autoaim_camera/config/camera_info.yaml`），不参与参数声明。

## 代码风格与坑

- 用仓库根 `.clang-format` 格式化：4 空格、100 列、指针左对齐、不排序 include；特殊项 `NamespaceIndentation: Inner`（命名空间内容缩进）、`IndentPPDirectives: BeforeHash`（命名空间内的 `#if` 也缩进）。该文件被 `.gitignore` 忽略，全新 clone 里可能不存在。
- Humble 的 `image_transport::create_subscription(node, topic, cb, transport, qos)` 第 4 个参数是 transport 字符串，必须显式传 `"raw"`（不能像 `create_publisher` 那样直接传 QoS）。
- 相机参数 `buffer_size: 4` 是有意为之：该 USB2.0 相机在 `CAP_PROP_BUFFERSIZE=1` 时采集/解码串行严重丢帧（实测 ~45Hz vs 4 时 ~97Hz），不要改回 1。
- 进程内零拷贝三要素，动任何一环都会退化为多一次整帧拷贝或走序列化：① 相机/检测/跟踪节点构造函数开 `NodeOptions.use_intra_process_comms(true)`（节点内 image_transport 创建的 pub/sub 默认继承该开关）；② 相机端必须 `unique_ptr` 发布（Humble 的 rclcpp 只有 unique_ptr 免拷贝、无 shared_ptr 重载，`const&` 会在进入 IPC 缓冲区前多拷一次整帧；Humble 的 image_transport 发布路径固定走 const&，所以相机端不能用）；③ 订阅端 `ConstSharedPtr` 回调 + `toCvShare` 共享数据。
- `ComposableNode` 的 `name=` 必须显式写且等于 YAML 顶层 key：launch_ros 用它匹配参数文件，缺省时参数文件静默不生效。

## 版本控制

- 分支 `master`，远程 `origin` = GitHub。提交信息用中文，保持简短。
