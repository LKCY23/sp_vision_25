#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

// 命令行参数：
//  -c/--config-path  : buff 相关模块使用的 yaml 配置，默认 configs/standard4.yaml
//  -s/--start-index  : 视频起始帧下标
//  -e/--end-index    : 视频结束帧下标（0 表示到视频结束）
//  @input-path       : 视频和姿态 txt 的前缀路径（不带扩展名），默认 assets/test_buff/buff1
//                      程序会优先查找 <input-path>.avi，否则回退到 <input-path>.mp4，
//                      姿态文件固定为 <input-path>.txt，格式为: t w x y z
const std::string keys =
  "{help h usage ? |                                      | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml               | yaml配置文件的路径}"
  "{start-index s  | 0                                    | 视频起始帧下标    }"
  "{end-index e    | 0                                    | 视频结束帧下标    }"
  "{@input-path    | assets/test_buff/buff1               | 视频和姿态文件前缀}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Plotter plotter;
  tools::Exiter exiter;

  namespace fs = std::filesystem;
  fs::path base_path(input_path);
  fs::path video_path;
  fs::path text_path;

  if (base_path.has_extension()) {
    video_path = base_path;
    text_path = base_path;
    text_path.replace_extension(".txt");
  } else {
    fs::path avi_path = base_path;
    avi_path += ".avi";
    fs::path mp4_path = base_path;
    mp4_path += ".mp4";

    if (fs::exists(avi_path)) {
      video_path = avi_path;
    } else {
      video_path = mp4_path;
    }

    text_path = base_path;
    text_path += ".txt";
  }

  if (!fs::exists(video_path)) {
    tools::logger()->error("Failed to find video file: {}", video_path.string());
    return -1;
  }
  if (!fs::exists(text_path)) {
    tools::logger()->error("Failed to find pose file: {}", text_path.string());
    return -1;
  }

  tools::logger()->info("video file: {}", video_path.string());
  tools::logger()->info("pose  file: {}", text_path.string());

  cv::VideoCapture video(video_path.string());
  if (!video.isOpened()) {
    tools::logger()->error("Failed to open video: {}", video_path.string());
    return -1;
  }

  std::ifstream text(text_path);
  if (!text.is_open()) {
    tools::logger()->error("Failed to open pose file: {}", text_path.string());
    return -1;
  }

  try {
    auto_buff::Buff_Detector detector(config_path);
    auto_buff::Solver solver(config_path);
    // auto_buff::SmallTarget target;
    auto_buff::BigTarget target;
    auto_buff::Aimer aimer(config_path);

    cv::Mat img;
    auto t0 = std::chrono::steady_clock::now();

    video.set(cv::CAP_PROP_POS_FRAMES, start_index);

    // 跳过前面的姿态数据，使其与 start_index 对齐
    double t, w, x, y, z;
    for (int i = 0; i < start_index; i++) {
      if (!(text >> t >> w >> x >> y >> z)) {
        tools::logger()->warn("Pose file ended before start_index, i = {}", i);
        return -1;
      }
    }

    for (int frame_count = start_index; !exiter.exit(); frame_count++) {
      if (end_index > 0 && frame_count > end_index) break;

      video.read(img);
      if (img.empty()) break;

      if (!(text >> t >> w >> x >> y >> z)) {
        tools::logger()->warn("Pose file ended at frame {}", frame_count);
        break;
      }
      auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

      /// 自瞄核心逻辑：Detector + Solver + Target + Aimer

      solver.set_R_gimbal2world({w, x, y, z});

      auto power_runes = detector.detect(img);

      solver.solve(power_runes);

      target.get_target(power_runes, timestamp);

      auto target_copy = target;

      auto command = aimer.aim(target_copy, timestamp, 22, false);

      // -------------- 调试输出 --------------

      nlohmann::json data;

      // buff 原始观测数据
      if (power_runes.has_value()) {
        const auto & p = power_runes.value();
        data["buff_R_yaw"] = p.ypd_in_world[0];
        data["buff_R_pitch"] = p.ypd_in_world[1];
        data["buff_R_dis"] = p.ypd_in_world[2];
        data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
        data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
        data["buff_roll"] = p.ypr_in_world[2] * 57.3;

        // 绘制识别结果
        tools::draw_point(img, p.r_center, {255, 255, 0}, 4);
        for (const auto & blade : p.fanblades) {
          cv::Scalar color =
            (blade.type == auto_buff::_target) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
          if (!blade.points.empty()) {
            tools::draw_points(img, blade.points, color, 2);
            tools::draw_point(img, blade.center, color, 3);
          }
        }
      }

      if (!target.is_unsolve() && power_runes.has_value()) {
        auto & p = power_runes.value();

        // 当前帧 target 更新后的 buff 回投影（绿色）
        auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
        auto image_points_now =
          solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
        if (image_points_now.size() >= 4) {
          tools::draw_points(
            img, std::vector<cv::Point2f>(image_points_now.begin(), image_points_now.begin() + 4),
            {0, 255, 0});
          tools::draw_points(
            img,
            std::vector<cv::Point2f>(image_points_now.begin() + 4, image_points_now.end()),
            {0, 255, 0});
        }

        // buff 瞄准位置（预测，红色）
        auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
        auto image_points_pre = solver.reproject_buff(
          Rxyz_in_world_pre, target_copy.ekf_x()[4], target_copy.ekf_x()[5]);
        if (image_points_pre.size() >= 4) {
          tools::draw_points(
            img, std::vector<cv::Point2f>(image_points_pre.begin(), image_points_pre.begin() + 4),
            {255, 0, 0});
          tools::draw_points(
            img,
            std::vector<cv::Point2f>(image_points_pre.begin() + 4, image_points_pre.end()),
            {255, 0, 0});
        }

        // EKF 内部状态
        Eigen::VectorXd x = target.ekf_x();
        data["R_yaw"] = x[0];
        data["R_V_yaw"] = x[1];
        data["R_pitch"] = x[2];
        data["R_dis"] = x[3];
        data["yaw"] = x[4] * 57.3;
        data["angle"] = x[5] * 57.3;
        data["spd"] = x[6] * 57.3;
        if (x.size() >= 10) {
          data["spd"] = x[6];
          data["a"] = x[7];
          data["w"] = x[8];
          data["fi"] = x[9];
          data["spd0"] = target.spd;
        }
      }

      // 云台响应情况
      Eigen::Vector3d ypr_gimbal = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      data["gimbal_yaw"] = ypr_gimbal[0] * 57.3;
      data["gimbal_pitch"] = -ypr_gimbal[1] * 57.3;

      // Aimer 输出
      if (command.control) {
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
      }
      data["cmd_shoot"] = command.shoot ? 1 : 0;

      if (!data.empty()) {
        plotter.plot(data);
      }

      cv::imshow("buff_aimer_test", img);

      int key = cv::waitKey(1);
      if (key == 'q') break;
      if (key == ' ') {
        // 空格暂停，任意键继续，q 退出
        while (true) {
          int k = cv::waitKey(30);
          if (k == 'q') {
            return 0;
          }
          if (k >= 0) break;
        }
      }
    }
  } catch (const YAML::Exception & e) {
    tools::logger()->error(
      "Failed to create Buff modules with config '{}': {}", config_path, e.what());
    tools::logger()->error(
      "请确认该 yaml 中包含 buff_detector 的 'model' 字段以及标定参数 "
      "(camera_matrix, distort_coeffs, R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal)，"
      "以及 buff_aimer 的 fire_gap_time, predict_time 等参数。");
    return -1;
  }

  return 0;
}

