# 打符自瞄模块说明（auto_buff）

## 一、项目整体架构中的打符位置

本仓库在整体上是一个“视觉框架”，打符模块是其中的一个功能组，对应目录 `tasks/auto_buff`，与自瞄模块 `tasks/auto_aim` 并列。

- `tools` 工具层  
  - 数学与几何：`tools/math_tools.hpp`，包含坐标系变换、角度归一化、球坐标/直角坐标互转等。  
  - 滤波与拟合：`tools/extended_kalman_filter.hpp`（EKF）、`tools/ransac_sine_fitter.hpp`（正弦拟合大符转速）。  
  - 弹道解算：`tools/trajectory.hpp`，给定初速、水平距离、高差，解出子弹飞行时间和俯仰角。  
  - 日志与可视化：`tools/logger.hpp`、`tools/plotter.hpp`、`tools/recorder.hpp`，用于调试、画轨迹、录视频。  
  - 其他：线程池、线程安全队列、CRC 等通用组件。

- `io` 硬件抽象层  
  - 相机：`io/camera.hpp` 统一了 `CameraBase::read(img, timestamp)` 接口，具体封装 Hikrobot、MindVision、USB 相机等。  
  - 云台/下位机：  
    - 步兵/英雄等用 `io::CBoard`（`io/cboard.hpp`）管理 CAN 通信、IMU 四元数插值、当前模式等。  
    - 部分调试/哨兵用 `io::Gimbal`（`io/gimbal`），直接控制云台位置/速度/加速度。  
  - ROS2：`io/ros2/*` 给哨兵预留与导航通信的接口（追击用）。

- `tasks` 功能层  
  - `tasks/auto_aim`：装甲板自瞄（传统 + YOLO 检测、PnP 解算、目标 EKF、决策 Aimer、轨迹规划器 MPC 等）。  
  - `tasks/auto_buff`：打符（能量机关）自瞄（本文件重点）。  
  - `tasks/omniperception`：哨兵全向感知与目标切换（参考 `src/sentry.cpp`）。

- `src` 应用层（各兵种主程序）  
  - 每个 `src/*.cpp` 是一个完整的“功能组合”：  
    - `src/standard.cpp`：步兵自瞄主程序。  
    - `src/auto_buff_debug.cpp`：打符调试主程序。  
  - 模式选择通过下位机的 `io::Mode` 实现（`io/cboard.hpp`），理论上可以在一个 main 里根据模式切换“自瞄 / 打符 / 前哨站”等功能组。

- 其他关键目录  
  - `configs/`：每台车的 YAML 参数（相机内参、手眼外参、控制参数、阈值等）。  
  - `calibration/`：相机标定、手眼标定、机器人-世界标定程序。  
  - `tests/`：各模块独立测试程序，例如 `tests/auto_buff_test.cpp` 是打符视频回放测试。

通用数据流（自瞄/打符类似）：

> 相机线程输出 `(图像, 时间戳)` → 通过下位机/云台读取对应时刻 IMU 四元数 →  
> 识别模块做目标检测 → 解算模块把像素点变成 3D 位姿 →  
> 目标滤波/预测 → 决策器算出瞄准点和开火时机 → 下位机控制云台和发射机构。

---

## 二、打符自瞄整体流程（从图像到开火）

打符实时调试程序入口在 `src/auto_buff_debug.cpp`，主循环大致如下：

1. 从命令行读取配置路径（包含标定、模型路径和各种参数）。  
2. 初始化组件：  
   - `io::CBoard`：与电控板通信，读 IMU 四元数、子弹速度、当前模式。  
   - `io::Camera`：工业相机，负责采集图像与时间戳。  
   - 打符功能组：  
     - `auto_buff::Buff_Detector`：检测能量机关扇叶与 R 标。  
     - `auto_buff::Solver`：像素坐标 → 3D 位姿解算。  
     - `auto_buff::SmallTarget` / `BigTarget`：小符/大符的目标状态估计。  
     - `auto_buff::Aimer`：根据目标状态 + 弹道，输出瞄准角 & 开火指令。

在主循环 `while (!exiter.exit())` 中，每一帧执行：

1. `camera.read(img, t)`：获取当前图像和时间戳 `t`。  
2. `q = cboard.imu_at(t)`：按时间戳插值获取该时刻云台姿态四元数。  
3. 打符核心逻辑：  
   - `solver.set_R_gimbal2world(q)` 更新云台→世界旋转矩阵；  
   - `power_runes = detector.detect(img)`：检测扇叶及 R 标像素；  
   - `solver.solve(power_runes)`：解算 buff 在世界坐标系的 3D 位姿；  
   - `target.get_target(power_runes, t)`：EKF 更新能量机关状态；  
   - `command = aimer.aim(target_copy, t, cboard.bullet_speed, true)`：预测 + 弹道 + 决策；  
   - `cboard.send(command)`：下发到电控板，由下位机执行云台控制和开火。

下面分别展开每个模块。

---

## 三、检测模块：YOLO11_BUFF + Buff_Detector

### 3.1 YOLO11_BUFF：从图像到关键点

`YOLO11_BUFF` 位于 `tasks/auto_buff/yolo11_buff.cpp`，负责用 OpenVINO 运行打符检测模型。

- 构造函数：  
  - 从 YAML 中读取模型路径 `model`；  
  - `core.read_model(model_path)` → `compile_model("CPU")` → `create_infer_request()`；  
  - 预先创建好输入 `Tensor`。

- `get_onecandidatebox / get_multicandidateboxes`：  
  - 预处理：将输入图缩放/letterbox 到 `640×640`，写入 `ov::Tensor`；  
  - 执行 `infer_request.infer()`；  
  - 输出张量形状约为 `[17, 8400]`，每列一个候选框，格式类似：  
    `[cx, cy, w, h, score, kpts...]`；  
  - 对每个候选框：  
    - 若 `score > ConfidenceThreshold`（例如 0.7），通过缩放因子恢复到原图坐标；  
    - 提取目标矩形框 `rect`；  
    - 提取 6 个关键点 `kpt`（4 个角点 + 扇叶中心 + R 标中心）。  

输出：一个或多个 `Object`，内部已包含扇叶角点、扇叶中心与 R 标中心的像素坐标。

### 3.2 Buff_Detector：寻找旋转中心并构造 PowerRune

`Buff_Detector` 位于 `tasks/auto_buff/buff_detector.cpp`，负责将 YOLO 输出进一步结构化为 `PowerRune`。

`Buff_Detector::detect` 核心步骤：

1. 调用 YOLO：`MODE_.get_onecandidatebox(bgr_img)` 获得置信度最高的候选框。  
2. 将 `Object.kpt` 转为 `FanBlade`：  
   - 扇叶 4 个角点 + 两个关键点（扇叶中心、R 标附近中心）封装为 `FanBlade`。  
3. 进一步精确 R 标中心：`get_r_center(fanblades, bgr_img)`：  
   - 彩色图 → 灰度 → 二值化 → 膨胀；  
   - 利用扇叶关键点估算出大概的 R 中心 `r_center_t`；  
   - 在 `r_center_t` 附近画圆形 mask，只保留该区域；  
   - 调 `cv::findContours` 找轮廓，用外接旋转矩形的长宽比 + 与 `r_center_t` 距离来打分，选出最符合的 R 标中心。  
4. 构造 `PowerRune`：`PowerRune powerrune(fanblades, r_center, last_powerrune_)`：  
   - 根据当前扇叶数量和与上一帧扇叶的空间距离，判断哪片扇叶是“当前点亮的 target”；  
   - 将 target 扇叶移到 `fanblades[0]`，其余按顺时针排序，保证扇叶顺序稳定；  
   - 记录扇叶数量 `light_num` 与中心点 `r_center`。

此时 `PowerRune` 中已经有了“哪个扇叶是当前目标 + 所有扇叶的像素坐标”，但还没有 3D 位姿，这交给下一步的 Solver 处理。

---

## 四、位姿解算模块：Solver（像素 → 世界坐标）

`auto_buff::Solver` 位于 `tasks/auto_buff/buff_solver.cpp`，负责完成从像素坐标到世界坐标的转换。

### 4.1 构造与标定数据加载

在构造函数中，`Solver` 从 YAML 里读取：

- 相机内参 `camera_matrix_` 与畸变系数 `distort_coeffs_`；  
- 手眼标定外参：`R_camera2gimbal_`、`t_camera2gimbal_`；  
- IMU 与云台之间的外参：`R_gimbal2imubody_`。

这些参数共同决定了图像坐标系、相机坐标系、云台坐标系与世界坐标系之间的几何关系。

### 4.2 云台姿态更新：set_R_gimbal2world

运行过程中，`set_R_gimbal2world(q)` 会被主循环每帧调用：

- `q` 为从下位机或云台读到的 IMU 四元数；  
- 先转成旋转矩阵 `R_imubody2imuabs`；  
- 再通过 `R_gimbal2imubody_` 转换成云台坐标系到世界坐标系的旋转矩阵 `R_gimbal2world_`。

### 4.3 solve：求 buff 与扇叶的世界系位姿

`Solver::solve(std::optional<PowerRune> & ps)` 主流程：

1. 若当前没有 `PowerRune`，直接返回。  
2. 从 `ps.value()` 里取出当前目标扇叶的四个角点 `p.target().points`。  
3. 取物理世界中对应的四个 3D 点（`OBJECT_POINTS` 的前四个元素），单位为米。  
4. 用 `cv::solvePnP` 估计 buff 坐标系到相机坐标系的旋转 `R_buff2camera` 与平移 `t_buff2camera`。  
5. 构造 buff 坐标系中的某个固定点（例如 `(0, 0, 0.7)` 一般对应扇叶中心），依次通过：  
   - buff → camera（用 `R_buff2camera` 与 `t_buff2camera`）；  
   - camera → gimbal（用 `R_camera2gimbal_` 与 `t_camera2gimbal_`）；  
   - gimbal → world（用 `R_gimbal2world_`）；  
   得到 buff 中心和扇叶中心在世界坐标系下的三维位置。  
6. 写回 `PowerRune`：  
   - `xyz_in_world`：buff 中心的世界坐标；  
   - `ypd_in_world`：其球坐标（yaw, pitch, distance）；  
   - `blade_xyz_in_world / blade_ypd_in_world`：目标扇叶中心的三维坐标与球坐标；  
   - `ypr_in_world`：buff 坐标系相对世界坐标系的欧拉角（yaw/pitch/roll）。

到此为止，能量机关模型已经从“图像上的扇叶角点”转化成了“世界坐标系下的 buff 中心和扇叶中心位置与姿态”。

---

## 五、目标建模与状态估计：SmallTarget / BigTarget + EKF

`Target` 系列类位于 `tasks/auto_buff/buff_target.cpp/.hpp`，负责对能量机关的运动状态进行滤波与预测。

### 5.1 基类 Target

基类 `Target` 定义了统一接口：

- `get_target(const std::optional<PowerRune>& p, timestamp)`：根据当前观测更新状态（纯虚函数，由子类实现）。  
- `predict(double dt)`：在时间上预测 `dt` 秒后的状态（纯虚函数）。  
- `point_buff2world(point_in_buff)`：给定 buff 坐标系下的一点（例如 `(0,0,0.7)`），根据当前 EKF 状态，转换到世界坐标系。  
- `ekf_x()`：返回当前 EKF 状态向量；`is_unsolve()` 表示当前状态是否可靠。

内部维护：

- `tools::ExtendedKalmanFilter ekf_` 及其所需的矩阵 `A_, Q_, H_, R_`；  
- `Voter` 用于投票判断旋转方向（顺时针还是逆时针）。

### 5.2 SmallTarget：小符（近似匀速角速度）

`SmallTarget` 的状态向量为 7 维：

> `[ R_yaw, R_v_yaw, R_pitch, R_dis, yaw, roll, spd ]`

- `R_yaw, R_pitch, R_dis`：buff 中心相对于世界系的球坐标。  
- `R_v_yaw`：buff 中心 yaw 方向角速度。  
- `yaw, roll`：buff 自身姿态中的 yaw 和 roll。  
- `spd`：扇叶的角速度，理论上接近常数 `SMALL_W = π/3`。

`get_target` 的主要逻辑：

1. 若当前无观测：设置 `unsolvable_ = true`，并累积丢失计数。  
2. 第一次有观测：用 `PowerRune` 中的 `ypd_in_world` 和 `ypr_in_world` 初始化 EKF。  
3. 若连续多帧丢失（`lost_cn > 6`）：认为目标丢失，重新初始化。  
4. 正常更新：  
   - 处理扇叶角度的“跳变”：扇叶角度是以 `2π/5` 周期重复的，需要根据上次角度给当前角度加减 `2π/5` 使得变化连续。  
   - 用 `Voter` 对连续多帧角度变化投票，判断顺/逆时针，若 `spd` 与方向不一致则翻转 `spd` 符号。  
   - 调用 `predict(nowtime - lasttime_)` 做时间预测，再用当前观测更新 EKF。  
   - 若 `spd` 明显偏离期望（比如超出 `SMALL_W ± 10°/s`）：认为滤波发散，重新初始化。

`predict(double dt)` 中，A 矩阵实现的是“匀速角运动 + 位置保持”，并在预测时对各个角度做 `limit_rad` 归一化，保证角度在合理范围内。

### 5.3 BigTarget：大符（角速度为正弦函数）

`BigTarget` 在 `SmallTarget` 的基础上，增加了 3 个参数，用来拟合扇叶角速度：

> `spd(t) = a * sin(w * t + fi) + 2.09 - a`

- 状态维度扩展到 10 维，增加了 `a, w, fi` 等参数。  
- 使用 `tools::RansacSineFitter` 对 `ekf_.x[6]`（观测到的速度）做正弦拟合，得到更平滑的 `fit_spd_`。  
- 预测和更新函数中引入了这个正弦模型，使得大符在速度周期变化情况下依然可预测。

整体结构与 `SmallTarget` 相似，只是模型更复杂，适用于大符的速度起伏情况。

---

## 六、瞄准与弹道解算：Aimer

瞄准器 `Aimer` 位于 `tasks/auto_buff/buff_aimer.cpp`，对外提供两个主要接口：

- `io::Command aim(Target& target, timestamp, bullet_speed, bool to_now)`  
- `auto_aim::Plan mpc_aim(Target& target, timestamp, io::GimbalState gs, bool to_now)`

这里重点介绍普通 `aim` 和内部核心函数 `get_send_angle`。

### 6.1 参数初始化

在构造函数中，`Aimer` 从 YAML 读取：

- `yaw_offset_`, `pitch_offset_`：枪口/相机安装误差补偿（由度转换为弧度）。  
- `fire_gap_time_`：两次开火之间的最小时间间隔（限制射频）。  
- `predict_time_`：估计整条链路的时间延迟（图像传输、网络推理、CAN 通信、云台响应等的总和）。

### 6.2 get_send_angle：预测 + 双弹道解算 + 时间一致性检查

`get_send_angle` 是打符瞄准的核心：

1. 预测到“控制指令生效时刻”：  
   - 调用 `target.predict(predict_time)`，把目标状态从“图像时间”推进到“现在 + 延迟”的估计时刻。  
2. 计算世界系中的瞄准点：  
   - 使用 `target.point_buff2world(Eigen::Vector3d(0, 0, 0.7))` 将 buff 坐标系中的固定点（例如扇叶打击点）转到世界系；  
   - 计算平面距离 `d = sqrt(x² + y²)` 和高度 `h = z`。  
3. 第一次弹道解算：  
   - 构造 `tools::Trajectory trajectory0(bullet_speed, d, h)`；  
   - 若解不出来（无解）则返回失败。  
4. 预测到“子弹命中时刻”：  
   - 调用 `target.predict(trajectory0.fly_time)` 再向前预测 `fly_time` 秒；  
   - 重新计算瞄准点位置，得到新的 `(d, h)`，再构造 `trajectory1`。  
5. 时间一致性检查：  
   - 若 `trajectory1.fly_time - trajectory0.fly_time` 的绝对值 > 0.01s，则认为弹道解算不收敛（目标在飞行过程中变化太快），返回失败。  
6. 生成输出角度：  
   - `yaw = atan2(y, x) + yaw_offset_`；  
   - `pitch = trajectory1.pitch + pitch_offset_`；  
   - 返回 `true`，表示当前帧可以给出稳定的瞄准角。

### 6.3 Aimer::aim：考虑延迟补偿、扇叶切换与开火节奏

`Aimer::aim` 完整封装了“一次打符决策”：

1. 若 `target.is_unsolve()` 为真，直接返回默认 `io::Command{false, false, 0, 0}`。  
2. 使用 `now - timestamp` 计算从图像采集到当前的延迟 `detect_now_gap`，并计算未来预测时间  
   `future = detect_now_gap + predict_time_` 作为 `get_send_angle` 的预测量。  
3. 若 `get_send_angle` 成功：  
   - 写入 `command.yaw = yaw`，`command.pitch = -pitch`（内部约定 pitch 向上为负）。  
   - 通过对比 `last_yaw_` / `last_pitch_` 与当前角度：  
     - 若连续多次变化超过约 5°，认为扇叶切换或识别抖动，标记 `switch_fanblade_ = true` 并将 `command.control = false`，暂时不让云台追随跳变；  
     - 若角度变化平滑，则 `switch_fanblade_ = false`，清空错误计数并设置 `command.control = true`。  
4. 开火逻辑：  
   - 若 `switch_fanblade_ = true`：说明刚换扇叶或状态不稳定，此时禁止开火并刷新 `last_fire_t_`；  
   - 若 `switch_fanblade_ = false` 且距离上一发射时间超过 `fire_gap_time_`：设置 `command.shoot = true`，允许开火，并更新 `last_fire_t_`。

MPC 版本 `mpc_aim` 在上述逻辑基础上，会额外根据当前云台状态 `io::GimbalState` 估计 yaw/pitch 的速度与加速度，生成包含位置、速度、加速度的 `Plan`，用于云台端的模型预测控制。

---

## 七、指令下发与执行闭环

在 `src/auto_buff_debug.cpp` 中，`Aimer::aim` 输出的 `io::Command` 最终通过：

```cpp
cboard.send(command);
```

发送给电控板，由下位机完成：

- 云台控制：根据 `command.control` 与 `command.yaw / command.pitch`，通过 PID/MPC 等控制律驱动电机，将云台跟随到期望角度；  
- 开火控制：根据 `command.shoot` 控制摩擦轮与拨弹轮，执行实际发射。

到此，一个完整的“打符自瞄”闭环在单帧上的流程是：

> 相机图像 → YOLO 检测扇叶 → 形态学精细化 R 中心 →  
> PnP + 手眼标定解算 3D 位姿 → EKF 跟踪旋转状态 →  
> 预测到“指令生效 + 子弹飞行”时间 → 双弹道解算稳定 yaw/pitch →  
> 扇叶切换判定 + 开火节奏控制 → 通过 CAN 下发至下位机执行。

---

## 八、打符自瞄的层次化理解

从工程实现上，可以将打符自瞄划分为以下层次，便于记忆与调试：

1. **检测层**：`YOLO11_BUFF + Buff_Detector`  
   - 从图像中识别出扇叶关键点与 R 标中心，形成结构化的 `PowerRune`。  
2. **几何层**：`Solver`  
   - 利用相机内参、手眼标定与 IMU 姿态，将 buff 与扇叶从像素坐标系映射到世界坐标系。  
3. **状态层**：`SmallTarget / BigTarget`  
   - 用 EKF 建模小符/大符的旋转运动，估计角度、角速度及旋转方向，并支持时间预测。  
4. **决策层**：`Aimer`  
   - 综合考虑传输延迟、图像处理耗时、云台响应与子弹飞行时间，通过双弹道解算找到稳定的瞄准角；  
   - 监控扇叶切换与角度抖动，仅在状态稳定时按节奏开火。  
5. **执行层**：`io::CBoard / io::Gimbal`  
   - 执行云台控制与发射机构控制，构成真正的物理闭环。

理解这五层之间的关系，有助于在调试时快速定位问题：  
- 若扇叶框漂移或识别不到，优先看检测层；  
- 若 3D 重投影错位，优先看几何层与标定参数；  
- 若预测角度抖动或发散，优先看 EKF 模型与测量噪声；  
- 若打得准但打不穿或打不死，多半是决策层（延迟估计、开火策略）与下位机控制层的配合问题。

