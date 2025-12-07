#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_buff/buff_detector.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

// 命令行参数：
//  -c/--config-path  : buff_detector 使用的 yaml 配置，默认 configs/standard4.yaml
//  -s/--start-index  : 视频起始帧下标
//  -e/--end-index    : 视频结束帧下标（0 表示到视频结束）
//  @video-path       : 视频文件路径，默认 assets/test_buff/buff1.mp4
const std::string keys =
  "{help h usage ? |                                      | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml               | yaml配置文件的路径}"
  "{start-index s  | 0                                    | 视频起始帧下标    }"
  "{end-index e    | 0                                    | 视频结束帧下标    }"
  "{@video-path    | assets/test_buff/buff1.mp4           | 视频文件路径      }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Exiter exiter;

  // 打开视频
  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    tools::logger()->error("Failed to open video: {}", video_path);
    return -1;
  }

  // 初始化识别器（只测识别器，不走解算/估计/瞄准）
  auto_buff::Buff_Detector * detector_ptr = nullptr;
  try {
    detector_ptr = new auto_buff::Buff_Detector(config_path);
  } catch (const YAML::Exception & e) {
    tools::logger()->error(
      "Failed to create Buff_Detector with config '{}': {}", config_path, e.what());
    tools::logger()->error(
      "请确认该 yaml 中包含 buff_detector 的 'model' 字段，例如:\n"
      "  #####-----buff_detector参数-----#####\n"
      "  model: \"assets/yolo11_buff_int8.xml\"");
    return -1;
  }

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    auto t0 = std::chrono::steady_clock::now();
    auto power_rune = detector_ptr->detect(img);
    auto t1 = std::chrono::steady_clock::now();

    double dt = tools::delta_time(t1, t0);
    tools::logger()->info(
      "[{}] detect: {:.1f} ms, found: {}", frame_count, dt * 1e3,
      power_rune.has_value() ? "true" : "false");

    // 可视化识别结果
    if (power_rune.has_value()) {
      const auto & p = power_rune.value();

      // 画 r_center
      tools::draw_point(img, p.r_center, {255, 255, 0}, 4);

      // 画 target 扇叶及所有扇叶轮廓
      for (const auto & blade : p.fanblades) {
        cv::Scalar color = (blade.type == auto_buff::_target) ? cv::Scalar(0, 255, 0)
                                                               : cv::Scalar(255, 0, 0);
        if (!blade.points.empty()) {
          tools::draw_points(img, blade.points, color, 2);
          tools::draw_point(img, blade.center, color, 3);
        }
      }

      tools::draw_text(
        img, fmt::format("lights: {}", p.light_num), {10, 30}, {0, 255, 255}, 0.8, 2);
    }

    cv::imshow("buff_detector_test", img);
    int key = cv::waitKey(30);
    if (key == 'q') break;
  }

  delete detector_ptr;
  return 0;
}
