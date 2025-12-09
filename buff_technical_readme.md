# auto_buff 技术说明：Detector / Solver / Target / Aimer

本说明面向需要维护打符自瞄的队员，重点从“工程视角”解释 `auto_buff` 四个核心模块的职责、输入输出和相互关系，帮助你快速接管并定位问题。

推荐配套文件：

- 源码目录：`tasks/auto_buff/`
- 运行入口（真相机调试）：`src/auto_buff_debug.cpp`
- 离线整体测试：`tests/auto_buff_test.cpp`
- 分阶段测试：`tests/test_buff/` 下的若干 `*_test.cpp`

---

## 1. 总体数据流（从图像到开火）

整体链路可以抽象为：

```text
相机帧 (BGR 图像, timestamp)
  └─ Buff_Detector：像素系下的扇叶 / R 标几何
      └─ Solver：像素系 → 世界系 3D 位姿
          └─ Target：EKF 估计 buff 运动（角度 / 角速度 / 周期参数）
              └─ Aimer：弹道解算 + 决策（yaw/pitch + shoot）
                  └─ 下位机：云台伺服 + 发射机构
```

在 `src/auto_buff_debug.cpp` 中，一帧的核心逻辑是：

```cpp
camera.read(img, t);                   // 图像 + 时间戳
auto q = cboard.imu_at(t);             // 插值得到该时刻云台四元数

solver.set_R_gimbal2world(q);          // 更新云台 → 世界旋转
auto power_runes = detector.detect(img);
solver.solve(power_runes);             // 填充 3D 位姿
target.get_target(power_runes, t);     // EKF 更新/预测
auto target_copy = target;
auto command = aimer.aim(target_copy, t, cboard.bullet_speed, true);

cboard.send(command);                  // 云台控制 + 开火
```

下面按模块依次说明。

---

## 2. Buff_Detector（检测器）

**位置**

- 接口与实现：`tasks/auto_buff/buff_detector.hpp/.cpp`
- YOLO 包装：`tasks/auto_buff/yolo11_buff.hpp/.cpp`
- 类型定义：`tasks/auto_buff/buff_type.hpp`

**职责**

从单帧 BGR 图像中检测出能量机关的像素几何信息：

- 每片扇叶的角点、中心与类型（_target/_light/_unlight）；
- R 标旋转中心 `r_center`；
- 一圈扇叶的顺序（从目标扇叶开始顺时针）。

结果封装在 `auto_buff::PowerRune` 中。

### 2.1 构造与输入

```cpp
auto_buff::Buff_Detector detector(config_path);
```

`config_path` 对应 YAML（如 `configs/standard4.yaml`），至少需要：

```yaml
#####-----buff_detector参数-----#####
model: "assets/yolo11_buff_int8.xml"
```

构造时会创建 `YOLO11_BUFF MODE_`，加载 OpenVINO 模型。

检测接口：

```cpp
std::optional<PowerRune> detect(cv::Mat& bgr_img);
```

- 输入：一帧 `cv::Mat` BGR 图；
- 输出：`std::optional<PowerRune>`：
  - `nullopt`：当前帧没有可靠目标；
  - 否则：一帧的 buff 检测结果。

### 2.2 PowerRune 内容（输出）

`buff_type.hpp` 中：

```cpp
class PowerRune {
public:
  cv::Point2f r_center;                // 旋转中心（像素）
  std::vector<FanBlade> fanblades;     // 从 target 开始顺时针
  int light_num;                       // 点亮扇叶数量

  // 下面这些字段由 Solver 填充：
  Eigen::Vector3d xyz_in_world;        // R 标在世界系下的 xyz（m）
  Eigen::Vector3d ypr_in_world;        // buff 姿态 yaw/pitch/roll（rad）
  Eigen::Vector3d ypd_in_world;        // yaw/pitch/distance（球坐标）
  Eigen::Vector3d blade_xyz_in_world;  // 目标扇叶中心 xyz（m）
  Eigen::Vector3d blade_ypd_in_world;  // 目标扇叶中心球坐标

  FanBlade& target();                  // fanblades[0]
  bool is_unsolve() const;
};
```

Detector 负责：

- 用 YOLO11 检出扇叶关键点 → 构造 `FanBlade`；
- 通过灰度化 + 二值化 + 轮廓分析，估计 R 标中心 `r_center`；
- 根据上一帧 `last_powerrune_` 确定当前 `_target` 扇叶和顺序；
- 将这些像素信息打包为 `PowerRune` 返回。

Solver 会在这些像素信息的基础上，填上世界系的三维信息。

### 2.3 当前 YOLO 模型的输出数据格式

当前项目实际使用的打符检测模型是一个 **“bbox + 6 点关键点”** 的 YOLO 模型，对应的解析代码在 `tasks/auto_buff/yolo11_buff.cpp` 中。

以主流程使用的 `get_onecandidatebox` 为例：

- 模型输出张量形状为 `[17, 8400]`：
  - 17 行表示每个候选框的 17 个数，8400 列表示最多 8400 个候选；
  - 每一列的布局为：
    - row 0: `cx`（锚框中心 x，归一化坐标）  
    - row 1: `cy`（锚框中心 y，归一化坐标）  
    - row 2: `ow`（锚框宽度 w，归一化）  
    - row 3: `oh`（锚框高度 h，归一化）  
    - row 4: `score`（置信度）  
    - row 5 ~ 16: 6 个关键点，每个关键点 2 个数 `(x_i, y_i)`（归一化）
  - 总计：`4（bbox） + 1（score） + 6*2（关键点） = 17`。

在代码里，关键点数量通过 `NUM_POINTS = 6` 固定：

```cpp
const int NUM_POINTS = 6;
cv::Mat kpts = det_output.col(best_index).rowRange(5, 5 + NUM_POINTS * 2);
for (int i = 0; i < NUM_POINTS; ++i) {
  const float x = kpts.at<float>(i * 2 + 0, 0) * factor;
  const float y = kpts.at<float>(i * 2 + 1, 0) * factor;
  obj.kpt.push_back(cv::Point2f(x, y));
}
```

**这 6 个关键点在本项目中的约定语义是：**

- `kpt[0..3]`：扇叶的四个角点（在 `FanBlade` 中被视为“从左上角开始逆时针”的四点轮廓）；  
- `kpt[4]`：扇叶中心（在 `Buff_Detector` 中直接作为 `FanBlade.center`）；  
- `kpt[5]`：位于扇叶中心沿半径方向的一点（向 R 中心方向延伸），用于几何上粗略外推 R 中心：

  ```cpp
  auto point5 = fanblade.points[4];  // 扇叶中心
  auto point6 = fanblade.points[5];  // 半径方向辅助点
  r_center_t += (point6 - point5) * 1.4 + point5; // 粗略 R 中心估计
  ```

之后再结合二值化 + 轮廓筛选对 R 中心进行细化定位。

> 小结：当前项目假定模型输出为 “bbox + 6 个关键点”，关键点语义是 **4 个扇叶角点 + 扇叶中心 + 半径方向辅助点**，R 中心通过几何与传统图像处理从这些点间接求出。后续如需支持其它关键点格式，可在 quickstart 文档的 Future Work 中参考相应改造建议。 

---

## 3. Solver（坐标变换 / 3D 解算）

**位置**

- 接口与实现：`tasks/auto_buff/buff_solver.hpp/.cpp`

**职责**

将：

- Detector 输出的像素级扇叶几何；
- 相机标定（内参与畸变）；
- 手眼外参（相机 ↔ 云台 ↔ IMU）；
- 当前云台姿态四元数；

转换为“世界坐标系下 buff 的 3D 位置和姿态”，并写回 `PowerRune`。

### 3.1 构造与输入

```cpp
auto_buff::Solver solver(config_path);
```

从 YAML 中读取：

- `camera_matrix`, `distort_coeffs`
- `R_gimbal2imubody`
- `R_camera2gimbal`, `t_camera2gimbal`

在主循环中，每帧需先设置云台姿态：

```cpp
solver.set_R_gimbal2world(q); // q = Eigen::Quaterniond(w,x,y,z)
```

然后调用：

```cpp
solver.solve(power_runes);
```

其中 `power_runes` 是 `std::optional<PowerRune>`，可以为空。

### 3.2 solve 的输出含义

对于有值的 `PowerRune p`，Solver 会补齐：

- `p.xyz_in_world`：
  - R 标中心在世界坐标系下的三维坐标，单位：米；
  - 大致表示 buff 距云台多远、偏左/偏右、偏上/偏下。

- `p.ypd_in_world = (yaw, pitch, distance)`：
  - yaw：R 标在云台前方的水平角度（rad）；
  - pitch：垂直角（rad）；
  - distance：距离（m）；
  - 这是后续 EKF 和 Aimer 最关心的量。

- `p.ypr_in_world = (yaw, pitch, roll)`：
  - buff 坐标系相对世界系的欧拉角；
  - 特别是 roll 分量，用于表示扇叶转动角度（Target 中的 `angle`）。

- `p.blade_xyz_in_world` / `p.blade_ypd_in_world`：
  - 目标扇叶中心在世界系下的 3D 坐标 / 球坐标；
  - Target 用它作为第二组观测（补充 R 标的信息）。

此外，Solver 提供调试接口：

- `point_buff2pixel(x)`：给定 buff 内一点，按当前姿态投影到像素；
- `reproject_buff(xyz_in_world, yaw, roll)`：给定 buff 中心世界坐标 + 姿态，回投影出 buff 几何轮廓（7 个像素点），用于在图像上画出绿色框验证解算质量。

---

## 4. Target（估计器 / 运动模型）

**位置**

- 接口与实现：`tasks/auto_buff/buff_target.hpp/.cpp`
- 工具依赖：`tools/extended_kalman_filter.hpp`, `tools/ransac_sine_fitter.hpp`

**职责**

在 Solver 给出的每帧 3D 观测基础上，使用 EKF 建模并估计 buff 的运动状态，主要包括：

- R 标相对于云台的 yaw / pitch / distance（平滑后的）；
- buff 自身 yaw / 扇叶角度 angle；
- 小符：近似常数的角速度 `spd`；
- 大符：周期性角速度的参数 `a, w, fi`。

并提供：

- `predict(dt)`：将状态时间推进 `dt` 秒；
- `point_buff2world(point_in_buff)`：把 buff 坐标系下某点转换到世界系，用于 Aimer 预测瞄准点。

### 4.1 基类接口

```cpp
class Target {
public:
  virtual void get_target(
    const std::optional<PowerRune>& p,
    std::chrono::steady_clock::time_point& timestamp) = 0;

  virtual void predict(double dt) = 0;

  Eigen::Vector3d point_buff2world(const Eigen::Vector3d& point_in_buff) const;
  Eigen::VectorXd ekf_x() const;
  bool is_unsolve() const;
};
```

`is_unsolve() == true` 表示当前估计不可靠，上层 Aimer 应跳过本帧。

### 4.2 SmallTarget：小符

状态向量（7 维）：

> `[ R_yaw, v_R_yaw, R_pitch, R_dis, yaw, angle(=roll), spd ]`

- `R_yaw, R_pitch, R_dis`：R 标的球坐标（来自 Solver）；
- `v_R_yaw`：R_yaw 的角速度；
- `yaw`：buff 的 yaw；
- `angle`：扇叶转动角（roll），经过 2π/5 周期对齐处理，保证连续；
- `spd`：扇叶角速度，理论值约等于 `SMALL_W = π/3`。

`get_target` 的逻辑概要：

1. 若本帧 `PowerRune` 为空 → 累计丢失计数，`unsolvable_ = true`；
2. 初次有观测 → 用当前帧 `ypd_in_world` / `ypr_in_world` 初始化 EKF；
3. 连续丢失超过阈值 → 打印 `[Target] 丢失buff`，重置滤波；
4. 正常更新：
   - 对扇叶角做周期对齐，避免 2π/5 跳变；
   - 通过 `Voter` 统计旋转方向，保证 `spd` 符号与方向一致；
   - 调用 `predict(dt)` 再做观测更新；
   - 若 `spd` 偏离 `SMALL_W` 过多（发散），打印 debug 并重置。

### 4.3 BigTarget：大符

状态向量（10 维）：

> `[ R_yaw, v_R_yaw, R_pitch, R_dis, yaw, angle, spd, a, w, fi ]`

除前 7 项与 SmallTarget 类似外，额外三项描述大符角速度的正弦模型：

```text
spd(t) ≈ a * sin(w * t + fi) + 2.09 - a
```

BigTarget 在更新时：

- 对 `a, w` 做经验范围检查，不在合理区间时打印：
  - `[Target] 大符角度发散a: .. b: ..`，并重置滤波；
- 使用 `tools::RansacSineFitter` 对 `spd` 做拟合，得到更稳的 `fit_spd_`，用于预测；
- 将 `fit_spd_` 作为下一步预测的速度基础。

**Target 的输出供 Aimer 使用：**

- `ekf_x()`：提供当前估计的姿态与扇叶状态；
- `point_buff2world`：给定 buff 坐标系点（例如打击点 `(0,0,0.7)`），得到未来某时刻在世界系的坐标。

---

## 5. Aimer（决策器 / 弹道解算）

**位置**

- 接口与实现：`tasks/auto_buff/buff_aimer.hpp/.cpp`
- 依赖：`tools/trajectory.hpp`, `tools/math_tools.hpp`

**职责**

在 Target 提供的“buff 运动预测”基础上：

- 考虑图像处理延迟、固定预测时间和子弹飞行时间；
- 计算当前应输出的云台 yaw/pitch；
- 决定是否在当前帧触发开火；
- MPC 模式下还可输出期望角速度/角加速度。

### 5.1 构造与配置

```cpp
auto_buff::Aimer aimer(config_path);
```

YAML 中相关参数：

```yaml
#####-----aimer参数-----#####
yaw_offset:   -2   # 安装误差（deg）
pitch_offset:  0

#####-----buff_aimer参数-----#####
fire_gap_time: 0.520   # 两次开火的间隔（s）
predict_time:  0.100   # 额外预测时间（s）
```

### 5.2 输入 / 输出接口

普通模式：

```cpp
io::Command aim(
  Target& target,
  std::chrono::steady_clock::time_point& timestamp,
  double bullet_speed,
  bool to_now = true);
```

返回：

```cpp
struct Command {
  bool control;  // 是否控制云台跟随 yaw/pitch
  bool shoot;    // 是否触发开火
  double yaw;    // 期望 yaw（rad）
  double pitch;  // 期望 pitch（rad）
};
```

MPC 版本 `mpc_aim` 在此基础上再输出角速度和角加速度。

### 5.3 核心逻辑概要

1. **状态检查与时间估计**
   - 若 `target.is_unsolve()` → 返回空命令；
   - 将 `bullet_speed < 10` 的情况修正为 24 m/s；
   - 计算 `detect_now_gap = now - timestamp`；
   - `future = detect_now_gap + predict_time_`（当 `to_now == true`）。

2. **get_send_angle：预测 + 双弹道解算**

   - 调用 `target.predict(predict_time_)` 将 buff 状态预推一段时间；
   - 使用 `target.point_buff2world(0,0,0.7)` 得到打击点在世界系的 `(x,y,z)`；
   - 构造 `tools::Trajectory(bullet_speed, d, h)` 做第一次弹道解算，得到飞行时间 `t0`；
   - 再调用 `target.predict(t0)`，重新求 `(d,h)` 并解弹道得到 `t1`；
   - 若 `|t1 - t0| > 0.01`，打印 `[Aimer] Large time error`，认为不稳定，返回失败；
   - 否则：
     - `yaw = atan2(y,x) + yaw_offset_`；
     - `pitch = trajectory1.pitch + pitch_offset_`。

3. **切扇叶与控制逻辑**

   - 对比当前 `yaw/pitch` 与上一帧 `last_yaw_/last_pitch_` 的差值：
     - 若跳变 > 5°，认为可能换扇叶，增加 `mistake_count_` 并设置 `switch_fanblade_`；
   - 若 `mistake_count_` 超过阈值，则暂时关闭开火，仅调整云台。

4. **开火节奏控制**

   - 若 `switch_fanblade_ == true`：`command.shoot = false`，并重置 `last_fire_t_`；
   - 若 `switch_fanblade_ == false` 且距离 `last_fire_t_` 的时间 > `fire_gap_time_`：
     - `command.shoot = true`，并更新 `last_fire_t_`；
   - 效果：只在稳定追踪某片扇叶的窗口期内按固定节奏开火。

---

## 6. 上手建议

1. 先阅读根目录的 `buff_readme.md` / `buffer_test_readme.md`，了解整体架构和测试方法；
2. 对照本技术说明，理解四个模块的职责与输入/输出；
3. 按顺序运行分阶段测试（位于 `tests/test_buff/`）：
   - `buff_detector_test` → `buff_solver_test` → `buff_target_test` → `buff_aimer_test`；
4. 利用 Plotter 查看：
   - `buff_R_yaw/pitch/dis`、`angle`、`spd` 曲线的平滑性；
   - `cmd_yaw/pitch/shoot` 是否与 buff 运动和扇叶位置匹配；
5. 最后在 `src/auto_buff_debug.cpp` 上联调真机，将离线经验迁移到实车参数配置中。

通过理解 Detector / Solver / Target / Aimer 四层的输入、输出及关系，你可以更有针对性地：

- 替换/微调检测模型；
- 调整 EKF 模型和噪声参数；
- 调整弹道和开火策略；
- 快速定位“识别 / 解算 / 估计 / 决策”哪一环出了问题。+
