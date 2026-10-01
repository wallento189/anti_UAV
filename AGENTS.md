# AGENTS.md

ROS 2 Humble (ament_cmake) 工作空间，位于 `/home/walt/ros_ws/anti_UAV`，根目录即 colcon 工作空间。

## 仓库结构

- `src/autoaim_camera/`：唯一有实现的包（C++ rclcpp + OpenCV V4L2 相机取流，经 `image_transport` 发布图像）。
- `src/autoaim_detector/`、`src/model/`：空目录占位，没有 `package.xml`，colcon 会直接忽略。
- `build/`、`install/`、`log/`：构建产物，不要手改。

## 构建与运行

- 先 source ROS，否则 `colcon`/`ros2` 不可用：`source /opt/ros/humble/setup.bash`。
- 全量构建（含 `--symlink-install` 与 `compile_commands.json`）：`./compile.sh`。
- 只构建相机包：`colcon build --packages-select autoaim_camera`。
- 运行：`source install/setup.bash && ros2 launch autoaim_camera autoaim_camera.launch.py`。
- 仓库没有测试、lint 目标；验证方式 = 编译通过 + 启动不报参数/插件错误。

## 命名不变量

- 运行时节点名固定为 `autoaim_camera`（`camera_node.cpp` 构造 `Node("autoaim_camera", options)`）。
- C++ 命名空间为 `CameraNode`，与类名 `CameraNode` 同名，完整类型写作 `CameraNode::CameraNode`；`RCLCPP_COMPONENTS_REGISTER_NODE` 与 `CMakeLists.txt` 的 `PLUGIN` 必须同步为 `CameraNode::CameraNode`，否则组件加载失败。

## 代码风格

- 用仓库根 `.clang-format` 格式化（4 空格缩进、100 列、指针左对齐、命名空间内缩进、不排序 include）。注意该文件被 `.gitignore` 忽略，全新 clone 里可能不存在。
- 相机参数 `buffer_size: 4` 是有意为之：该 USB2.0 相机在 `CAP_PROP_BUFFERSIZE=1` 时采集/解码串行会严重丢帧（实测 ~45Hz vs 4 时 ~97Hz），不要改回 1。

## 版本控制

- 分支 `master`，远程 `origin` = GitHub。提交信息用中文，保持简短。
