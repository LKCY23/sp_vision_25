# auto_buff 使用与测试快速手册

这份文档只讲怎么做，方便新同学从 0 到能跑通打符测试。详细原理见 `buff_technical_readme.md` 和 `buff_readme.md`。

---

## 1. 安装依赖

在一台 Ubuntu/WSL 上，建议按下面顺序准备环境：

1. OpenVINO（用于神经网络推理）
   - Linux 安装参考官方文档：  
     https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html

2. Ceres Solver（打符标定与部分优化会用到）
   - 官方安装说明：  
     http://ceres-solver.org/installation.html

3. 其余依赖（APT 安装）

   ```bash
   sudo apt update
   sudo apt install -y \
       git \
       g++ \
       cmake \
       can-utils \
       libopencv-dev \
       libfmt-dev \
       libeigen3-dev \
       libspdlog-dev \
       libyaml-cpp-dev \
       libusb-1.0-0-dev \
       nlohmann-json3-dev \
       openssh-server \
       screen
   ```

---

## 2. 克隆与构建项目

```bash
git clone https://github.com/LKCY23/sp_vision_25.git
cd sp_vision_25

# 生成构建文件
cmake -S . -B build

# 编译（含所有核心模块与测试）
cmake --build build -j4
```

若只改了某个测试，可单独构建对应 target，例如：

```bash
cmake --build build --target buff_detector_test -j8
```
请根据设备的核数自行决定并行编译的核数。
---

## 3. 构建打符测试数据（视频 + 姿态 txt）

打符离线测试需要“视频 + 同步姿态文本”这一对数据。我们提供了一个脚本生成“固定云台姿态”的 txt，适合离线算法调试。

1. 将你录制的 buff 视频放到 `assets/test_buff/` 下，例如：

```bash
ls assets/test_buff
# 示例：
#   buff1.mp4  buff1.txt  gen_buff_txt.py
#   buff2.mp4  buff2.txt  ...
```

2. 在该目录下运行生成姿态脚本：

```bash
cd assets/test_buff

# 为新视频 buff_new.mp4 生成对应的 buff_new.txt
python gen_buff_txt.py buff_new.mp4
```

脚本会：

- 读取视频帧率；
- 生成同名的 `.txt` 文件（如 `buff_new.txt`）；
- 每帧写入一行：`t 1 0 0 0`（云台始终保持单位四元数，适合离线调试）。

之后回到项目根目录继续运行各种测试：

```bash
cd ../../
```

---

## 4. 分阶段打符测试（test_buff）

假设你已经在根目录，并执行过一次构建：

```bash
cmake -S . -B build
cmake --build build -j4
```

### 3.1 阶段一：检测器（Detector）

程序：`tests/test_buff/buff_detector_test.cpp`  
可执行：`build/buff_detector_test`

命令示例：

```bash
cmake --build build --target buff_detector_test -j4
./build/buff_detector_test -c configs/standard4.yaml assets/test_buff/buff1.mp4
```

说明：

- 只测 YOLO + 几何中心求解，不做 3D 解算；
- 窗口中会显示：
  - 扇叶轮廓与中心；
  - 旋转中心 `r_center`；
- 按 `q` 退出。

### 3.2 阶段二：检测 + 解算（Detector + Solver）

程序：`tests/test_buff/buff_solver_test.cpp`  
可执行：`build/buff_solver_test`

命令示例（推荐用“前缀”，自动匹配 mp4/txt）：

```bash
cmake --build build --target buff_solver_test -j4
./build/buff_solver_test -c configs/standard4.yaml assets/test_buff/buff1
```

说明：

- 程序会自动组合：
  - 视频：`assets/test_buff/buff1.avi` 或 `buff1.mp4`；
  - 姿态：`assets/test_buff/buff1.txt`；
- 在图上叠加：
  - 原始识别结果；
  - Solver 回投影出的绿色 buff 轮廓；
- 通过 log 看 `xyz_in_world` / `ypr_in_world` 是否平滑合理。

也支持直接传完整视频路径（自动改后缀找 txt）：

```bash
./build/buff_solver_test -c configs/standard4.yaml assets/test_buff/buff2.mp4
```

### 3.3 阶段三：检测 + 解算 + 估计（Detector + Solver + Target）

程序：`tests/test_buff/buff_target_test.cpp`  
可执行：`build/buff_target_test`

命令示例：

```bash
cmake --build build --target buff_target_test -j4
./build/buff_target_test -c configs/standard4.yaml assets/test_buff/buff1
```

说明：

- 在阶段二基础上加入 EKF（SmallTarget/BigTarget，当前默认大符 BigTarget）；
- Plotter 中可以看到：
  - `buff_R_yaw/pitch/dis`；
  - `angle`（扇叶角度）、`spd`（角速度）；
- 图像中绿色框为 EKF 当前估计回投影。

若需要测试小符，可在 `buff_target_test.cpp` 中将：

```cpp
// auto_buff::SmallTarget target;
auto_buff::BigTarget target;
```

改成使用 `SmallTarget`，重新构建后再运行。

### 3.4 阶段四：完整链路（Detector + Solver + Target + Aimer）

程序：`tests/test_buff/buff_aimer_test.cpp`  
可执行：`build/buff_aimer_test`

命令示例：

```bash
cmake --build build --target buff_aimer_test -j4
./build/buff_aimer_test -c configs/standard4.yaml assets/test_buff/buff1
```

说明：

- 完整跑通：Detector → Solver → Target → Aimer；
- 图像中：
  - 绿色框：当前 EKF 估计的 buff；
  - 红色框：Aimer 预测瞄准位置；
- Plotter 中：
  - buff 曲线（`buff_R_yaw/pitch/dis`、`angle`、`spd` 等）；
  - 云台角度（由姿态 txt 推出）；
  - `cmd_yaw/cmd_pitch/cmd_shoot`。

如果想直接用老的整体测试程序，也可以用：

```bash
cmake --build build --target auto_buff_test -j4
./build/auto_buff_test -c configs/standard4.yaml assets/test_buff/buff1
```

该程序默认读取 `assets/test_buff/buff1.avi` 和对应 `.txt`。

---

## 5. 在 macOS 上通过 SSH + XQuartz 使用 WSL 图形界面

打符测试使用 OpenCV 的窗口，需要 X11 显示。若你在 **Mac 上通过 SSH 连接运行 WSL**，可以用 XQuartz 作为 X 服务器。

### 4.1 在 macOS 上安装和配置 XQuartz

1. 从官网下载并安装 XQuartz：https://www.xquartz.org
2. 启动 XQuartz；
3. 在 XQuartz 菜单中打开终端（`Applications → Terminal`），执行：

   ```bash
   xhost +localhost
   ```

   如需允许局域网其他机器连接，使用 `xhost +`（有安全风险，仅在可信网络下使用）。

### 4.2 在 WSL 中启用 X11 转发（通过 SSH）

下面是一种常见做法：在 **WSL 中启动 sshd**，从 Mac 通过 `ssh -Y` 进入 WSL，这样 `DISPLAY` 会自动配置。

1. 在 WSL 中安装 sshd 和 xauth（只需一次）：

   ```bash
   sudo apt update
   sudo apt install openssh-server xauth -y
   ```

2. 编辑 `/etc/ssh/sshd_config`，确保：

   ```text
   X11Forwarding yes
   X11UseLocalhost yes
   ```

3. 启动 sshd：

   ```bash
   sudo service ssh --full-restart
   # 或
   sudo service ssh start
   ```

4. 在 Mac 上，用 XQuartz 自带的终端或系统终端连接到 WSL：

   ```bash
   ssh -Y <wsl-user>@<wsl-ip>
   ```

   - `<wsl-ip>` 可在 Windows 终端中用 `ip addr` 或 `ipconfig` 查到；
   - 连接成功后，`echo $DISPLAY` 应该显示一个非空值（如 `localhost:10.0`）。

5. 在这个 SSH 会话里运行打符测试，例如：

   ```bash
   cd ~/sp_vision_25
   ./build/buff_detector_test -c configs/standard4.yaml assets/test_buff/buff1.mp4
   ```

   OpenCV 窗口应该会通过 XQuartz 显示在 Mac 上。

---

## 6. 小结

日常开发/调试推荐流程：

1. 拉代码 + `cmake -S . -B build && cmake --build build -j4`；
2. 将自己的 buff 视频放到 `assets/test_buff/` 并用 `gen_buff_txt.py` 生成 txt；
3. 按阶段 1–4 依次运行各测试，观察：
   - 检测框是否稳定；
   - 回投影是否与实际扇叶贴合；
   - EKF 曲线是否平滑；
   - Aimer 的 yaw/pitch/shoot 是否与扇叶运动匹配；
4. 调通后再切换到 `src/auto_buff_debug.cpp` 在真机上联调。

遇到任何问题，先看对应阶段的测试，再结合 `buff_technical_readme.md` 定位是 Detector / Solver / Target / Aimer 中的哪一层出了问题。+

---

## 7. Future Work / 后续工作建议

这里列出一些后续可以推进的方向，便于接手同学规划：

1. **GPU 推理打符模型**
   - 当前打符使用的 OpenVINO 模型（`YOLO11_BUFF`）在代码中写死为 `"CPU"` 推理：
     - `tasks/auto_buff/yolo11_buff.cpp` 中：`compiled_model = core.compile_model(model, "CPU");`
   - 后续可以改为从配置读取设备（如 `CPU` / `GPU`），并测试在 GPU 上的性能与稳定性：
     - 修改构造函数从 YAML 里读 `device` 字段；
     - 对比 CPU/GPU 在相同视频上的 FPS 和检测稳定性。

2. **实车联调测试**
   - 目标：把目前的离线链路迁移到真机环境上，重点确认两件事：
     1. **从视频流切到真实相机数据** 后，Detector 能正常工作：
        - 替换所有 `cv::VideoCapture` 为 `io::Camera`；
        - 检查在实时相机图像上，打符检测是否仍然稳定（帧率、漏检率、抖动）。
     2. **决策器输出能否正常送到下位机**：
        - 在 `src/auto_buff_debug.cpp` 中，确认 `cboard.send(command)` 或对应串口/CAN 通道工作正常；
        - 使用示波/日志观察云台是否按预期跟随 `yaw/pitch`；
        - 验证串口/ CAN 通信稳定，无明显丢包和延迟异常。

3. **大符模型训练与替换（现有 6 点模型 vs 你们的 5 点模型）**
   - 目前项目内置的打符检测模型是 **bbox + 6 个关键点** 的形式（详见 `buff_technical_readme.md`）：
     - bbox：`cx, cy, w, h`（归一化中心点 + 宽高）；
     - 关键点（6 个）：
       - 4 个扇叶角点：左上、左下、右下、右上；
       - 1 个扇叶中心点；
       - 1 个沿扇叶中心 → R 中心方向的辅助点，用于几何外推 R 中心。
     - R 中心：并未直接输出，而是由“扇叶中心 + 辅助点 + 传统图像处理”间接估计。
   - 你们自有的“大符 5 点模型”数据形式大致是：
     - 第 0 位：大符类别（class id）；
     - 第 1–4 位：bbox `(x, y, w, h)`（归一化中心 + 宽高）；
     - 接下来 5 个关键点（归一化）：
       - 左上：`x1, y1`；左下：`x2, y2`；右下：`x3, y3`；右上：`x4, y4`；  
       - R 中心：`x5, y5`（显式给出能量机关中心）。
   - 如果将来想用你们的模型替换当前模型，大体有两条路线：
     1. **保持当前项目的 6 点协议，重标 / 重训**
        - 数据按“4 角 + 扇叶中心 + 辅助点”的格式重新标注；
        - 按现有输出格式训练新的模型，生成与现在相同维度的输出；
        - 训练好后直接替换 `assets/yolo11_buff_int8.xml`（或更新 YAML 中 `model` 路径），Detector / Solver / Target / Aimer 基本不用改，只需微调阈值和参数。
     2. **保留 5 点格式，在 Detector 层做适配**
        - 在 `YOLO11_BUFF` 里按“4 角 + R 点”的方式解析关键点；
        - 在 `Buff_Detector` 中：
          - 用四个角点几何求出扇叶中心（比如四点平均或更精细的几何方法）；
          - 将 R 点直接作为粗略 R 中心，或基于 R 点和扇叶中心构造辅助点，再走现有的 `get_r_center` 细化流程；
        - 下游 Solver / Target / Aimer 无需大改，只要 `PowerRune` 里的 `r_center` 和扇叶轮廓稳定可靠即可。

接手该模块的同学可以优先把现有 CPU + 离线流程跑熟，然后按上述三个方向逐步推进。***
