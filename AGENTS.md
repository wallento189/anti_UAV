# AGENTS.md

ROS 2 Humble (ament_cmake) 工作空间，位于 `/home/walt/ros_ws/anti_UAV`，根目录即 colcon 工作空间。

## 仓库结构

- `src/autoaim_camera/`：V4L2 相机取流（C++ rclcpp + OpenCV），经 rclcpp 原生发布器以 unique_ptr 发布 `autoaim_camera/image_raw`（bgr8，SensorDataQoS）。
- `src/autoaim_interfaces/`：纯消息包，`msg/Detection.msg`（检测结果，供检测/跟踪/控制复用）。
- `src/autoaim_detector/`：YOLO11n 无人机检测。共享 preprocess/postprocess + 可切换推理后端。
- `src/autoaim_bringup/`：纯 launch 包。`autoaim.launch.py` 用单个 `component_container_mt` 加载相机+检测两个组件（节点构造函数开 `use_intra_process_comms`，进程内零拷贝）。
- `build/`、`install/`、`log/`：构建产物，不要手改。

## 构建与运行

- 先 source ROS，否则 `colcon`/`ros2` 不可用：`source /opt/ros/humble/setup.bash`。
- 全量构建：`./compile.sh`（`--symlink-install` + `compile_commands.json`）。
- 只构建某个包：`colcon build --packages-select autoaim_detector`（改动 `autoaim_interfaces` 后要把它一起构建，否则依赖包拿到旧消息）。
- 运行：`source install/setup.bash && ros2 launch autoaim_bringup autoaim.launch.py`（组合：单容器装相机+检测）；单节点调试用 `ros2 launch autoaim_camera autoaim_camera.launch.py` 或 `ros2 launch autoaim_detector autoaim_detector.launch.py`。
- 仓库没有测试、lint 目标；验证 = 编译通过 + 启动不报参数/插件错误。

## 检测器后端（关键）

- 两个 CMake 开关（纯 ON/OFF），默认 `AUTOAIM_WITH_OPENVINO=ON`、`AUTOAIM_WITH_HORIZON=OFF`；ON 但探测不到 SDK 时 configure 直接 `FATAL_ERROR`。Horizon 用 `-DHORIZON_SDK_ROOT=<sdk>`；RDK 板端：`colcon build --packages-select autoaim_detector --cmake-args -DAUTOAIM_WITH_HORIZON=ON -DAUTOAIM_WITH_OPENVINO=OFF`。
- 运行时参数 `enable_gpu`：OpenVINO 设备 GPU/CPU（读 `.onnx`）；`enable_bpu`：Horizon hbDNN 读 `.bin`（本机无 SDK，未实测；按 NV12 U8 输入 + SCALE/SHIFT 量化输出写）。两者同时为 true 直接抛错。`best.bin` 目前不存在，`enable_bpu:=true` 会报错。
- `model_path` 不能为空（为空直接抛错），其值始终相对模型目录 `share/autoaim_detector/model/` 解析（如 `best_p2.onnx`）；`model/` 目录被 `.gitignore` 忽略，CMake 仅在目录存在时安装。
- 模型契约（动预处理/后处理前必读）：输入 float32 `[1,3,640,640]` NCHW；NHWC 原始输出，每个尺度成对（cls 单通道 **raw logits，需 sigmoid**；box 64 通道 DFL，reg_max=16，需 softmax+期望）；尺度按 `stride = kInputSize / height` 动态识别，支持 stride 4/8/16/32（P2/P3/P4/P5），不再硬编码 3 尺度。
- 无目标也发布 `Detection`，`detected=false`、其余字段为 0。

## 命名不变量

- 运行时节点名必须等于 YAML 顶层 key：相机 `autoaim_camera`，检测 `autoaim_detector`（不匹配则整个参数文件被静默忽略）。
- 组件插件名必须与「命名空间::类名」一致：相机 `CameraNode::CameraNode`、检测 `DetectorNode::DetectorNode`（CMake `PLUGIN` 与 `RCLCPP_COMPONENTS_REGISTER_NODE` 同步，否则组件加载失败）。可执行名分别为 `camera_node`、`detector_node`。

## 参数约定

- 所有运行时参数都不得有默认值：一律 `declare_parameter<T>("name")`，必须由对应 `config/*.yaml` 显式提供；缺失时节点启动即抛 `ParameterUninitializedException`（“Statically typed parameter ... must be initialized”）。新增/改名参数时同步更新对应 YAML。

## 代码风格与坑

- 用仓库根 `.clang-format` 格式化：4 空格、100 列、指针左对齐、不排序 include；特殊项 `NamespaceIndentation: Inner`（命名空间内容缩进）、`IndentPPDirectives: BeforeHash`（命名空间内的 `#if` 也缩进）。该文件被 `.gitignore` 忽略，全新 clone 里可能不存在。
- Humble 的 `image_transport::create_subscription(node, topic, cb, transport, qos)` 第 4 个参数是 transport 字符串，必须显式传 `"raw"`（不能像 `create_publisher` 那样直接传 QoS）。
- 相机参数 `buffer_size: 4` 是有意为之：该 USB2.0 相机在 `CAP_PROP_BUFFERSIZE=1` 时采集/解码串行严重丢帧（实测 ~45Hz vs 4 时 ~97Hz），不要改回 1。
- 进程内零拷贝三要素，动任何一环都会退化为多一次整帧拷贝或走序列化：① 两节点构造函数开 `NodeOptions.use_intra_process_comms(true)`（节点内 image_transport 创建的 pub/sub 默认继承该开关）；② 相机端必须 `unique_ptr` 发布（Humble 的 rclcpp 只有 unique_ptr 免拷贝、无 shared_ptr 重载，`const&` 会在进入 IPC 缓冲区前多拷一次整帧；Humble 的 image_transport 发布路径固定走 const&，所以相机端不能用）；③ 检测端 `ConstSharedPtr` 回调 + `toCvShare` 共享数据。
- `ComposableNode` 的 `name=` 必须显式写且等于 YAML 顶层 key：launch_ros 用它匹配参数文件，缺省时参数文件静默不生效。

## 版本控制

- 分支 `master`，远程 `origin` = GitHub。提交信息用中文，保持简短。
