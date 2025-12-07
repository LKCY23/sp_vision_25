# Buffer / 打符 测试说明

本文件回答你现在关心的 3 个问题，并给出一个从“识别器 → 估计器 → 坐标变换 → 决策器”的分阶段测试方案，方便你用内录的能量机关视频做验证。

---

## 1. 相机部分是否支持“传入视频测试”？

需要区分两类接口：

- **实时相机接口（硬件）**
  - 统一入口是 `io::Camera`（`io/camera.hpp:16`），构造时传入 YAML 配置路径：
    - `io::Camera camera(config_path);`
    - `camera.read(img, timestamp);`
  - 底层根据 `configs/*.yaml` 里的 `camera_name` 字段选择具体实现：
    - `"mindvision"` → `io::MindVision`
    - `"hikrobot"` → `io::HikRobot`
  - 这一层只面向真实工业相机，不直接支持“读取某个 .avi 文件”。

- **离线视频测试接口（通过 OpenCV，而不是 io::Camera）**
  - 项目中所有“用视频回放进行算法测试”的入口都在 `tests/` 目录里，直接用 `cv::VideoCapture` 读取视频文件，而不是走 `io::Camera`：
    - 自瞄整体链路离线测试：`tests/auto_aim_test.cpp:39`  
      `cv::VideoCapture video(video_path);`
    - 自瞄检测器 + YOLO 离线测试：`tests/detector_video_test.cpp:31`
    - 打符整体链路离线测试：`tests/auto_buff_test.cpp:38`
  - 例如你刚才运行的 `./build/auto_aim_test`，默认使用：
    - 视频：`assets/demo/demo.avi`
    - 云台姿态数据：`assets/demo/demo.txt`
    - 默认参数：在 `tests/auto_aim_test.cpp` 的 `keys` 里已经写死：
      - `config-path` 默认是 `configs/demo.yaml`
      - `@input-path` 默认是 `assets/demo/demo`

**结论：**

- 相机抽象层 `io::Camera` 仅支持真实相机，不带“从文件读视频”的模式。
- 要用视频离线测试，自瞄/打符模块都已经在 `tests/*.cpp` 里提供了示例（`auto_aim_test`、`auto_buff_test`、`detector_video_test` 等），都是直接用 `cv::VideoCapture`。

---

## 2. 是否有现成的 auto_buff（打符）测试程序？

有两类：

- **实时打符调试（真相机 + 真云台）**
  - 主程序：`src/auto_buff_debug.cpp`
    - 入口：`int main(...)`，使用 `io::CBoard` + `io::Camera`。
    - 主循环核心链路：
      - `camera.read(img, t);`
      - `q = cboard.imu_at(t);`
      - `solver.set_R_gimbal2world(q);`
      - `power_runes = detector.detect(img);`
      - `solver.solve(power_runes);`
      - `target.get_target(power_runes, t);`
      - `command = aimer.aim(target_copy, t, cboard.bullet_speed, true);`
  - MPC 版本：`src/auto_buff_debug_mpc.cpp`
    - 与上面类似，只是用 `io::Gimbal` + `Aimer::mpc_aim(...)` 走 MPC 轨迹规划。

- **离线视频回放打符测试**
  - 程序入口：`tests/auto_buff_test.cpp`
  - 对应可执行文件：`build/auto_buff_test`
  - 输入参数（见 `tests/auto_buff_test.cpp:25`）：
    - `@input-path`：**不带扩展名的基路径**，程序内部会拼成：
      - `fmt::format("{}.avi", input_path);`
      - `fmt::format("{}.txt", input_path);`
    - `-c/--config-path`：YAML 配置路径，默认 `configs/sentry.yaml`。
    - `-s/--start-index`、`-e/--end-index`：起止帧号。
  - 典型使用方式：
    ```bash
    # 假设你有 my_buff.avi 和 my_buff.txt，放在 assets/buff/ 里
    ./build/auto_buff_test \
      -c configs/sentry.yaml \
      assets/buff/my_buff
    ```
    其中：
    - `assets/buff/my_buff.avi`：内录的能量机关视频；
    - `assets/buff/my_buff.txt`：按行存的时间戳和云台四元数：
      `t w x y z`（格式同 `assets/demo/demo.txt`）。

**结论：**

- 项目已经自带了一个完整的 auto_buff 视频回放测试程序：`auto_buff_test`。
- 如果你有成对的 `.avi + .txt` 数据，只要按上述方式组织路径，就可以直接离线测试打符链路。

---

## 3. 如何分模块测试 buffer（打符）链路？

你的目标是：  
> 识别器 → 估计器 → 坐标变换 → 决策器，分阶段验证。

结合现有代码，可以这样拆：

### 3.1 模块对应关系

- **识别器（Detector）**  
  - 代码：`tasks/auto_buff/buff_detector.[ch]pp`  
  - 类：`auto_buff::Buff_Detector`  
  - 核心接口：
    - `std::optional<PowerRune> detect(cv::Mat& bgr_img);`
    - 返回值里包含：扇叶关键点、R 标、r_center 等像素坐标信息。

- **坐标变换（Solver）**  
  - 代码：`tasks/auto_buff/buff_solver.[ch]pp`  
  - 类：`auto_buff::Solver`  
  - 核心接口：
    - `set_R_gimbal2world(const Eigen::Quaterniond& q);`
    - `solve(std::optional<PowerRune>& ps) const;`
    - `reproject_buff(...)`：用于调试回投影。
  - 功能：结合标定参数和云台四元数，把像素点转成世界坐标系下的 buff 位姿。

- **估计器（Target / EKF）**  
  - 代码：`tasks/auto_buff/buff_target.[ch]pp`  
  - 类：
    - 抽象基类：`auto_buff::Target`
    - 小符：`auto_buff::SmallTarget`
    - 大符：`auto_buff::BigTarget`
  - 核心接口：
    - `get_target(const std::optional<PowerRune>& p, timestamp);`
    - `predict(double dt);`
    - `point_buff2world(const Eigen::Vector3d& point_in_buff);`
  - 功能：使用 EKF 对 buff 的中心位置、扇叶角度/角速度等进行滤波和预测。

- **决策器（Aimer）**
  - 代码：`tasks/auto_buff/buff_aimer.[ch]pp`
  - 类：`auto_buff::Aimer`
  - 核心接口：
    - `io::Command aim(Target& target, timestamp, double bullet_speed, bool to_now = true);`
    - `auto_aim::Plan mpc_aim(Target& target, timestamp, io::GimbalState gs, bool to_now = true);`
  - 功能：根据目标状态 + 弹道模型，输出瞄准角（yaw/pitch）与开火信号。

### 3.2 阶段一：只测识别器（Detector）

**目标：** 确认 YOLO11_BUFF + Buff_Detector 能在你的视频中稳定识别能量机关扇叶和 R 标，先不管 EKF 和瞄准。

**建议做法：**

- 参考 `tests/detector_video_test.cpp` 的写法，写一个“打符版本”的测试（或直接在现有代码基础上快速改一个私有测试）：
  - 用 `cv::VideoCapture` 打开你的 buff 视频；
  - 每帧：
    - 调用 `power_rune = detector.detect(img);`
    - 若有值：
      - 用 `PowerRune::target().points[i]`、`PowerRune::center`、`PowerRune::r_center` 在图像上画点/连线；
    - `cv::imshow("buff_detector", img);` + `cv::waitKey(33);`
- 也可以临时在 `tests/auto_buff_test.cpp` 里注释掉 `Solver/Target/Aimer` 部分，只保留 `detector.detect(img)` 和可视化逻辑，作为纯识别测试。

**验证要点：**

- 扇叶轮廓和 R 标在所有关键帧都有被标出来；
- r_center 位置基本稳定，抖动和漏检在可接受范围内。

### 3.3 阶段二：识别器 + 坐标变换（Detector + Solver）

**目标：** 确认从“像素坐标 → 世界系三维坐标”的变换正确，外参/内参配置合理。

**现有参考代码：**

- `tests/auto_buff_test.cpp` 中已经包含完整链路：
  - `power_runes = detector.detect(img);`
  - `solver.set_R_gimbal2world({w, x, y, z});`
  - `solver.solve(power_runes);`
  - 随后用 `solver.reproject_buff(...)` 把 3D 点回投影到图像上并画出来。

**建议做法：**

- 仍然使用 `auto_buff_test` 的结构，但暂时忽略 `Target` 和 `Aimer` 的输出，只观察回投影结果：
  - 如果 `power_runes` 有值：
    - 查看 `solver.solve(power_runes);` 后的 `power_runes.value().ypr_in_world`、`ypd_in_world` 是否连续合理；
    - 使用 `solver.reproject_buff(...)` 的结果在图像上画出绿色/红色框，对比真实扇叶位置。

**验证要点：**

- 回投影框与视频中实际扇叶轮廓重合度高；
- 云台 yaw/pitch（由 `.txt` 中四元数解出）变化与画面中的运动方向一致。

### 3.4 阶段三：识别器 + 坐标变换 + 估计器（Detector + Solver + Target）

**目标：** 检查 EKF 对 buff 运动的拟合是否稳定，例如：
  - 小符：角速度 spd 是否稳定在预期范围；
  - 大符：正弦拟合后的速度曲线是否接近真实。

**现有参考代码：**

- `tests/auto_buff_test.cpp` 中：
  - `auto_buff::SmallTarget target;` 或 `BigTarget target;`
  - `target.get_target(power_runes, timestamp);`
  - 使用 `target.ekf_x()` 输出各种内部状态到 `plotter`。

**建议做法：**

- 运行 `auto_buff_test`，重点观察：
  - `buff_R_yaw` / `buff_R_pitch` / `buff_R_dis` 曲线是否平滑；
  - `x[5]`（扇叶角度）、`x[6]`（角速度）在时间上是否连续；
  - 大符时，拟合出的 `a, w, fi` 是否合理（速度在一个周期内上下波动，但总体不会发散）。

**验证要点：**

- 少量短时间丢失观测时，EKF 能平滑补上，不会瞬间乱跳；
- 角度、角速度曲线与视频中扇叶的真实转速感受一致。

### 3.5 阶段四：完整链路 + 决策器（Detector + Solver + Target + Aimer）

**目标：** 验证从视频到“瞄准角 + 开火指令”的全链路，在给定子弹初速和延迟估计下能稳定输出合理指令。

**现有参考代码：**

- 离线版本：`tests/auto_buff_test.cpp`
  - `command = aimer.aim(target_copy, timestamp, bullet_speed, false);`
- 实时版本：`src/auto_buff_debug.cpp`
  - `command = aimer.aim(target_copy, t, cboard.bullet_speed, true);`

**建议做法：**

- 离线测试时，可以先固定一个子弹速度（比如 22 m/s）：
  - 在 `auto_buff_test.cpp` 中已经是这样调用的：`aimer.aim(target_copy, timestamp, 22, false);`
- 重点关注：
  - 输出的 `command.yaw` / `command.pitch` 是否与 buff 位置变化同步；
  - `command.shoot` 信号是否只在合适位置附近拉高，而不是一直打或完全不打；
  - 在 `buff_readme.md` 里提到的“预测时间、fire_gap_time”等参数调整是否对轨迹有预期影响。

**验证要点：**

- 在视频中你能够直观感觉：准心（回投影的瞄准点）始终提前追到扇叶将要到的位置，而不是落后很多；
- 开火逻辑有明显的“窗口”，而不是在大角度范围内乱打。

---

## 4. 建议的整体测试顺序（结合你的目标）

基于你说的“先识别器、再估计器、再坐标变换、最后决策器”，在当前项目中对应的推荐顺序是：

1. **纯识别测试（Buff_Detector + 视频）**
   - 只关心 `detector.detect(img)` 的输出是否稳定；
   - 可参照 `tests/detector_video_test.cpp` 自行写一个 buff 版测试。
2. **识别 + 坐标变换（Detector + Solver）**
   - 使用 `auto_buff_test` 结构，重点看回投影和世界坐标是否合理；
   - 相当于验证标定 + IMU 四元数 + 解算逻辑。
3. **识别 + 坐标变换 + 估计器（Detector + Solver + Target）**
   - 在 `auto_buff_test` 里观察 EKF 内部状态曲线，确认不会发散；
   - 小符用 `SmallTarget`，大符用 `BigTarget`。
4. **完整链路（Detector + Solver + Target + Aimer）**
   - 离线用 `auto_buff_test`，在线用 `auto_buff_debug`/`auto_buff_debug_mpc`；
   - 在 Plotter 曲线 + 图像可视化上综合判断效果。

如果你希望，我也可以根据上面的思路，帮你在 `tests/` 下面加一个专门的 `buff_detector_video_test` 或 `buff_pipeline_test`，专门为你的内录视频做调参和可视化。你可以先按这个 README 尝试把 `auto_buff_test` 跑起来，如果有具体报错或想要更细粒度的测试入口，我们再往下拆。 

