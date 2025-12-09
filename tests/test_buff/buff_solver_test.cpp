#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 命令行参数：
//  -c/--config-path  : buff_detector 和 buff_solver 使用的 yaml 配置，默认 configs/standard4.yaml
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
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

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

    video.set(cv::CAP_PROP_POS_FRAMES, start_index);

    // 跳过前面的姿态数据，使其与 start_index 对齐
    double t, w, x, y, z;
    for (int i = 0; i < start_index; i++) {
      if (!(text >> t >> w >> x >> y >> z)) {
        tools::logger()->warn("Pose file ended before start_index, i = {}", i);
        return -1;
      }
    }

    cv::Mat img;

    for (int frame_count = start_index; !exiter.exit(); frame_count++) {
      if (end_index > 0 && frame_count > end_index) break;

      video.read(img);
      if (img.empty()) break;

      if (!(text >> t >> w >> x >> y >> z)) {
        tools::logger()->warn("Pose file ended at frame {}", frame_count);
        break;
      }

      solver.set_R_gimbal2world({w, x, y, z});

      auto t0 = std::chrono::steady_clock::now();
      auto power_rune = detector.detect(img);
      auto t1 = std::chrono::steady_clock::now();

      double dt = tools::delta_time(t1, t0);

      if (power_rune.has_value()) {
        solver.solve(power_rune);
        auto & p = power_rune.value();

        tools::logger()->info(
          "[{}] detect+solve: {:.1f} ms, xyz=({:.3f}, {:.3f}, {:.3f}) m, "
          "ypr=({:.1f}, {:.1f}, {:.1f}) deg",
          frame_count, dt * 1e3, p.xyz_in_world[0], p.xyz_in_world[1], p.xyz_in_world[2],
          p.ypr_in_world[0] * 57.3, p.ypr_in_world[1] * 57.3, p.ypr_in_world[2] * 57.3);

        // 画原始识别结果
        tools::draw_point(img, p.r_center, {255, 255, 0}, 4);
        for (const auto & blade : p.fanblades) {
          cv::Scalar color =
            (blade.type == auto_buff::_target) ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0);
          if (!blade.points.empty()) {
            tools::draw_points(img, blade.points, color, 2);
            tools::draw_point(img, blade.center, color, 3);
          }
        }

        // 使用 Solver 回投影 buff 几何体，用于验证外参与解算是否正确
        auto image_points =
          solver.reproject_buff(p.xyz_in_world, p.ypr_in_world[0], p.ypr_in_world[2]);
        if (image_points.size() >= 4) {
          std::vector<cv::Point2f> outer(image_points.begin(), image_points.begin() + 4);
          std::vector<cv::Point2f> inner(image_points.begin() + 4, image_points.end());
          // 回投影轮廓：绿色
          tools::draw_points(img, outer, {0, 255, 0}, 2);
          tools::draw_points(img, inner, {0, 255, 0}, 2);
        }

        // 显示球坐标信息（yaw/pitch/distance）
        tools::draw_text(
          img,
          fmt::format(
            "R_yaw={:.1f}deg R_pitch={:.1f}deg R_dis={:.2f}m", p.ypd_in_world[0] * 57.3,
            p.ypd_in_world[1] * 57.3, p.ypd_in_world[2]),
          {10, 30}, {0, 255, 255}, 0.7, 2);
      } else {
        tools::logger()->info(
          "[{}] detect+solve: {:.1f} ms, found: false", frame_count, dt * 1e3);
      }

      // 输出云台自身的 yaw/pitch 角度，便于对比
      Eigen::Vector3d ypr_gimbal = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      tools::logger()->info(
        "gimbal y/p (deg): ({:.1f}, {:.1f})", ypr_gimbal[0] * 57.3, -ypr_gimbal[1] * 57.3);

      cv::imshow("buff_solver_test", img);
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
      "Failed to create Buff_Detector / Solver with config '{}': {}", config_path, e.what());
    tools::logger()->error(
      "请确认该 yaml 中包含 buff_detector 的 'model' 字段以及标定参数 "
      "(camera_matrix, distort_coeffs, R_gimbal2imubody, R_camera2gimbal, t_camera2gimbal)。");
    return -1;
  }

  return 0;
}

