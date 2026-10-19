# AGENTS.md

ROS 2 Humble (ament_cmake) 工作空间，位于 `/home/walt/ros_ws/anti_UAV`，根目录即 colcon 工作空间。

## 仓库结构

- `src/autoaim_camera/`：V4L2 相机取流（C++ rclcpp + OpenCV），经 rclcpp 原生发布器以 unique_ptr 发布 `autoaim_camera/image_raw`（bgr8，SensorDataQoS），并用 `camera_info_manager` 加载 `config/camera_info.yaml`、随图像每帧发布 `autoaim_camera/camera_info`。
- `src/autoaim_interfaces/`：纯消息包，`msg/Detection.msg`（检测结果）、`msg/TrackedTarget.msg`（跟踪状态与角度误差），供检测/跟踪/控制复用。
- `src/autoaim_detector/`：YOLO11n 无人机检测。共享 preprocess/postprocess + 可切换推理后端。
- `src/autoaim_tracker/`：单目标跟踪。订阅 `Detection` + `CameraInfo`，α-β 滤波后输出 `autoaim_tracker/target`。只做状态估计与误差解算，不含控制律。
- `src/autoaim_bringup/`：纯 launch 包。`autoaim.launch.py` 用单个 `component_container_mt` 加载相机+检测+跟踪三个组件（节点构造函数开 `use_intra_process_comms`，进程内零拷贝）。
- `build/`、`install/`、`log/`：构建产物，不要手改。

## 构建与运行

- 先 source ROS，否则 `colcon`/`ros2` 不可用：`source /opt/ros/humble/setup.bash`。
- 全量构建：`./compile.sh`（`--symlink-install` + `compile_commands.json`）。
- 只构建某个包：`colcon build --packages-select autoaim_detector`（改动 `autoaim_interfaces` 后要把它一起构建，否则依赖包拿到旧消息）。
- 运行：`source install/setup.bash && ros2 launch autoaim_bringup autoaim.launch.py`（组合：单容器装相机+检测+跟踪）；单节点调试用 `ros2 launch autoaim_camera autoaim_camera.launch.py`、`ros2 launch autoaim_detector autoaim_detector.launch.py` 或 `ros2 launch autoaim_tracker autoaim_tracker.launch.py`。
- 仓库没有测试、lint 目标；验证 = 编译通过 + 启动不报参数/插件错误。

## 检测器后端（关键）

- 两个 CMake 开关（纯 ON/OFF），默认 `AUTOAIM_WITH_OPENVINO=ON`、`AUTOAIM_WITH_HORIZON=OFF`；ON 但探测不到 SDK 时 configure 直接 `FATAL_ERROR`。Horizon 用 `-DHORIZON_SDK_ROOT=<sdk>`；RDK 板端：`colcon build --packages-select autoaim_detector --cmake-args -DAUTOAIM_WITH_HORIZON=ON -DAUTOAIM_WITH_OPENVINO=OFF`。
- 运行时参数 `enable_gpu`：OpenVINO 设备 GPU/CPU（读 `.onnx`）；`enable_bpu`：Horizon hbDNN 读 `.bin`（本机无 SDK，未实测；按 NV12 U8 输入 + SCALE/SHIFT 量化输出写）。两者同时为 true 直接抛错。`best.bin` 目前不存在，`enable_bpu:=true` 会报错。
- `model_path` 不能为空（为空直接抛错），其值始终相对模型目录 `share/autoaim_detector/model/` 解析（如 `best_p2.onnx`）；`model/` 目录被 `.gitignore` 忽略，CMake 仅在目录存在时安装。
- 模型契约（动预处理/后处理前必读）：输入 float32 `[1,3,640,640]` NCHW；NHWC 原始输出，每个尺度成对（cls 单通道 **raw logits，需 sigmoid**；box 64 通道 DFL，reg_max=16，需 softmax+期望）；尺度按 `stride = kInputSize / height` 动态识别，支持 stride 4/8/16/32（P2/P3/P4/P5），不再硬编码 3 尺度。
- 无目标也发布 `Detection`，`detected=false`、其余字段为 0。

## 跟踪（tracker，关键）

- 输出 `TrackedTarget`：状态 `0=LOST`、`1=TRACKING`、`2=HOLDING`。状态完全由连续无有效观测的**推理帧数**决定：连续 `hold_frames` 帧转 HOLDING，`lost_frames` 帧转 LOST；接受新观测立即回 TRACKING，跳变门限拒绝也算一个丢失帧。不做时间外推，HOLDING/LOST 期间 `header.stamp` 仍是最后观测时刻，下游用 `now - stamp` 判断数据年龄。
- 断流（相机掉线、detector 崩溃）时 tracker 收不到任何消息，帧数不再增长，状态会冻结在最后状态；时效性必须由下游按 `header.stamp` 判断，tracker 内部不做超时。
- 角度符号写死在代码里：**yaw 向左为正，pitch 向下为正**。图像 x 向右、y 向下，因此目标在右侧时 yaw 为负、在下方时 pitch 为正。没有符号参数；装配反向时在相机端翻转图像或在 controller 里翻电机方向，不改变 tracker 的消息语义。
- 内参不走节点参数：相机节点用 `camera_info_manager` 加载标准 ROS 标定文件 `autoaim_camera/config/camera_info.yaml`，tracker 订阅 `CameraInfo` 缓存 K/D；收到之前不发布 target。像素→角度用 `undistortPoints` + `atan`，D 全 0 时退化为无畸变。
- 未建云台-相机外参（光轴与 yaw/pitch 轴按平行处理）；单目框尺寸测距未做。
- `camera_info.yaml` 中 `fx/fy=554`、畸变系数 0 是按 60° FOV 估的占位值，实标后直接替换该文件（`camera_calibration` 的输出格式），代码不用改。

## 命名不变量

- 运行时节点名必须等于 YAML 顶层 key：相机 `autoaim_camera`，检测 `autoaim_detector`，跟踪 `autoaim_tracker`（不匹配则整个参数文件被静默忽略）。
- 组件插件名必须与「命名空间::类名」一致：相机 `CameraNode::CameraNode`、检测 `DetectorNode::DetectorNode`、跟踪 `TrackerNode::TrackerNode`（CMake `PLUGIN` 与 `RCLCPP_COMPONENTS_REGISTER_NODE` 同步，否则组件加载失败）。可执行名分别为 `camera_node`、`detector_node`、`tracker_node`。

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
