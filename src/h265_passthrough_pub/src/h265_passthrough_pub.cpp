#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>


#include <libavutil/avutil.h>


#include <libavcodec/bsf.h>


}





#include <string>


#include <thread>


#include <chrono>


#include <vector>


#include <limits>


#include <cstring>


#include <cmath>


#include <atomic>





using namespace std::chrono_literals;





class H26xPassthroughPub : public rclcpp::Node {


public:


  // 新增：允许外部传 NodeOptions（带参数覆盖）和自定义节点名
  explicit H26xPassthroughPub(const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                              const std::string& name = "h26x_passthrough_pub")
  : Node(name, options) {
    // ===== 参数 =====


    input_       = declare_parameter<std::string>("input", "");


    topic_       = declare_parameter<std::string>("topic", "/h26x/compressed");


    frame_id_    = declare_parameter<std::string>("frame_id", "camera");


    loop_        = declare_parameter<bool>("loop", false);  // ★ 默认不循环


    stamp_now_   = declare_parameter<bool>("stamp_now", true);


    reliable_    = declare_parameter<bool>("reliable", true);


    aggregate_by_pts_    = declare_parameter<bool>("aggregate_by_pts", true);


    wait_idr_on_start_   = declare_parameter<bool>("wait_idr_on_start", true);


    pace_to_media_time_  = declare_parameter<bool>("pace_to_media_time", true);


    playback_speed_      = declare_parameter<double>("playback_speed", 1.0);


    start_on_subscriber_ = declare_parameter<bool>("start_on_subscriber", true); // ★ 有订阅者才开始


    exit_on_finish_      = declare_parameter<bool>("exit_on_finish", true);      // ★ 发完是否退出





    if (input_.empty()) {


      RCLCPP_FATAL(get_logger(), "请设置输入：--ros-args -p input:=/path/to/file.mp4 或 rtsp://...");


      throw std::runtime_error("missing input");


    }


    if (playback_speed_ <= 0.0) playback_speed_ = 1.0;





    // ===== Publisher QoS =====


    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile();


    qos = reliable_ ? qos.reliable() : qos.best_effort();


    pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(topic_, qos);





    // 不立刻打开输入：等真的开始发送时再 open_input()


    expected_period_s_ = 1.0 / 30.0;  // 先给个默认





    // 状态


    started_  = false;


    finished_ = false;


    running_  = true;





    // 读线程


    worker_ = std::thread(&H26xPassthroughPub::reader_loop, this);





    RCLCPP_INFO(get_logger(),


      "等待订阅者后开始发送: %s, 发送完后%s；QoS=%s, 聚帧=%s, 按媒体时间节拍=%s x%.2f",


      start_on_subscriber_ ? "true" : "false",


      exit_on_finish_ ? "将退出" : "不退出(仅停止发送)",


      reliable_ ? "RELIABLE" : "BEST_EFFORT",


      aggregate_by_pts_ ? "true" : "false",


      pace_to_media_time_ ? "true" : "false", playback_speed_);


  }





  ~H26xPassthroughPub() override {


    running_ = false;


    if (worker_.joinable()) worker_.join();


    flush_current_au(/*force_publish=*/false);


    close_all();


  }





private:


  // ===== 打开输入并准备 bitstream filter（mp4/flv → Annex-B）=====


  bool open_input() {


    close_all();


    if (avformat_open_input(&fmt_, input_.c_str(), nullptr, nullptr) < 0) {


      RCLCPP_ERROR(get_logger(), "avformat_open_input 失败: %s", input_.c_str());


      return false;


    }


    if (avformat_find_stream_info(fmt_, nullptr) < 0) {


      RCLCPP_ERROR(get_logger(), "avformat_find_stream_info 失败");


      return false;


    }





    video_stream_index_ = av_find_best_stream(fmt_, AVMEDIA_TYPE_VIDEO, -1, -1, nullptr, 0);


    if (video_stream_index_ < 0) {


      RCLCPP_ERROR(get_logger(), "找不到视频流");


      return false;


    }


    vs_ = fmt_->streams[video_stream_index_];





    const AVCodecParameters* par = vs_->codecpar;


    if (par->codec_id == AV_CODEC_ID_HEVC || par->codec_id == AV_CODEC_ID_H265) {


      bsf_name_   = "hevc_mp4toannexb"; codec_name_ = "h265";


    } else if (par->codec_id == AV_CODEC_ID_H264) {


      bsf_name_   = "h264_mp4toannexb"; codec_name_ = "h264";


    } else {


      RCLCPP_ERROR(get_logger(), "不支持的编码: codec_id=%d（仅 H.264/H.265）", par->codec_id);


      return false;


    }





    const AVBitStreamFilter* bsf = av_bsf_get_by_name(bsf_name_.c_str());


    if (!bsf) { RCLCPP_ERROR(get_logger(), "av_bsf_get_by_name 失败: %s", bsf_name_.c_str()); return false; }


    if (av_bsf_alloc(bsf, &bsf_ctx_) < 0) { RCLCPP_ERROR(get_logger(), "av_bsf_alloc 失败"); return false; }


    if (avcodec_parameters_copy(bsf_ctx_->par_in, par) < 0) {


      RCLCPP_ERROR(get_logger(), "avcodec_parameters_copy(par_in) 失败");


      return false;


    }


    bsf_ctx_->time_base_in = vs_->time_base;


    if (av_bsf_init(bsf_ctx_) < 0) { RCLCPP_ERROR(get_logger(), "av_bsf_init 失败"); return false; }





    // AU 缓存/节拍复位


    au_buf_.clear(); au_pts_ = AV_NOPTS_VALUE; au_is_key_ = false;


    media_t0_s_ = std::numeric_limits<double>::quiet_NaN();


    wall_t0_    = std::chrono::steady_clock::time_point{};


    expected_period_s_ = guess_period_from_stream();





    return true;


  }





  void close_all() {


    if (bsf_ctx_) { av_bsf_free(&bsf_ctx_); bsf_ctx_ = nullptr; }


    if (fmt_)      { avformat_close_input(&fmt_); fmt_ = nullptr; }


    vs_ = nullptr; video_stream_index_ = -1;


  }





  // 估计帧周期


  double guess_period_from_stream() const {


    auto frac_to_double = [](AVRational q)->double {


      if (q.num <= 0 || q.den <= 0) return NAN;


      return static_cast<double>(q.den) / static_cast<double>(q.num);


    };


    if (!vs_) return 1.0/30.0;


    double p = frac_to_double(vs_->avg_frame_rate);


    if (!std::isfinite(p)) p = frac_to_double(vs_->r_frame_rate);


    if (!std::isfinite(p) || p <= 0) p = 1.0/30.0;


    return p;


  }





  // 工具


  static inline bool is_valid_packet(const AVPacket* pkt) {


    return pkt && pkt->size > 4;


  }


  static inline int64_t pkt_ts_raw(const AVPacket* pkt) {


    if (!pkt) return AV_NOPTS_VALUE;


    if (pkt->pts != AV_NOPTS_VALUE) return pkt->pts;


    if (pkt->dts != AV_NOPTS_VALUE) return pkt->dts;


    return AV_NOPTS_VALUE;


  }


  inline double ts_to_seconds(int64_t ts) const {


    if (ts == AV_NOPTS_VALUE || !vs_) return NAN;


    return ts * av_q2d(vs_->time_base);


  }


  static inline bool is_keypkt(const AVPacket* pkt) {


    return pkt && (pkt->flags & AV_PKT_FLAG_KEY);


  }





  // 按媒体时间节拍


  void pace_by_pts(int64_t ts) {


    if (!pace_to_media_time_) return;


    double t_s = ts_to_seconds(ts);


    auto now = std::chrono::steady_clock::now();





    if (!std::isfinite(t_s)) {


      if (expected_period_s_ > 0) {


        auto dt = std::chrono::duration<double>(expected_period_s_ / playback_speed_);


        std::this_thread::sleep_for(dt);


      }


      return;


    }





    if (!std::isfinite(media_t0_s_)) {


      media_t0_s_ = t_s; wall_t0_ = now; return; // 第一帧不等待


    }





    auto target = wall_t0_ + std::chrono::duration<double>((t_s - media_t0_s_) / playback_speed_);


    if (target > now) std::this_thread::sleep_until(target);


  }





  // 聚帧完成后发布/丢弃


  void flush_current_au(bool force_publish) {


    if (au_buf_.empty()) return;





    if (force_publish || !waiting_for_idr_ || au_is_key_) {


      pace_by_pts(au_pts_);





      sensor_msgs::msg::CompressedImage msg;


      if (stamp_now_) {


        msg.header.stamp = now();


      } else {


        uint64_t ts = (au_pts_ == AV_NOPTS_VALUE) ? 0 : static_cast<uint64_t>(au_pts_);


        msg.header.stamp = rclcpp::Time(ts, 0, RCL_ROS_TIME);


      }


      msg.header.frame_id = frame_id_;


      msg.format = codec_name_;


      msg.data.swap(au_buf_);


      pub_->publish(std::move(msg));





      if (waiting_for_idr_ && au_is_key_) {


        waiting_for_idr_ = false;


        RCLCPP_INFO(get_logger(), "检测到 IDR，开始按媒体时间节拍发布。");


      }


    } else {


      au_buf_.clear(); // 等 IDR：丢弃非关键帧


    }





    au_buf_.clear(); au_pts_ = AV_NOPTS_VALUE; au_is_key_ = false;


  }





  // 主循环


  void reader_loop() {


    av_log_set_level(AV_LOG_ERROR);





    while (running_ && rclcpp::ok()) {


      // 1) 等待订阅者


      if (!started_) {


        if (start_on_subscriber_) {


          if (pub_->get_subscription_count() == 0) {


            std::this_thread::sleep_for(50ms);


            continue;


          }


        }


        // 有订阅者或不需要等待 → 从头开始发送


        if (!open_input()) {


          RCLCPP_ERROR(get_logger(), "open_input 失败，退出");


          break;


        }


        waiting_for_idr_ = wait_idr_on_start_;


        started_ = true;


        RCLCPP_INFO(get_logger(), "检测到订阅者，开始发送文件：%s", input_.c_str());


      }





      // 2) 已经开始发送但已完成


      if (finished_) {


        std::this_thread::sleep_for(100ms);


        continue;


      }





      // 3) 正常读取与发布


      if (!fmt_) { std::this_thread::sleep_for(20ms); continue; }





      AVPacket* in = av_packet_alloc();


      int ret = av_read_frame(fmt_, in);


      if (ret == AVERROR_EOF) {


        av_packet_free(&in);


        flush_current_au(/*force_publish=*/false);





        if (loop_) {


          RCLCPP_INFO(get_logger(), "到达文件尾（loop=true），从头重播。");


          // 重新从头


          if (!open_input()) { RCLCPP_ERROR(get_logger(), "重播时 open_input 失败"); break; }


          waiting_for_idr_ = wait_idr_on_start_;


          continue;


        } else {


          // 不循环：标记完成并按需退出


          finished_ = true;


          RCLCPP_INFO(get_logger(), "已完成文件发送：%s", input_.c_str());


          if (exit_on_finish_) {


            RCLCPP_INFO(get_logger(), "exit_on_finish=true：退出节点。");


            running_ = false;


            rclcpp::shutdown();


          }


          break;


        }


      } else if (ret < 0) {


        av_packet_free(&in);


        std::this_thread::sleep_for(5ms);


        continue;


      }





      if (in->stream_index != video_stream_index_) {


        av_packet_unref(in); av_packet_free(&in); continue;


      }





      if (av_bsf_send_packet(bsf_ctx_, in) < 0) { av_packet_free(&in); continue; }


      av_packet_free(&in);





      // 取 bsf 输出


      while (running_) {


        AVPacket* out = av_packet_alloc();


        int rr = av_bsf_receive_packet(bsf_ctx_, out);


        if (rr == AVERROR(EAGAIN) || rr == AVERROR_EOF) { av_packet_free(&out); break; }


        if (rr < 0) { av_packet_free(&out); break; }


        if (!is_valid_packet(out)) { av_packet_unref(out); av_packet_free(&out); continue; }





        if (aggregate_by_pts_) {


          int64_t ts = pkt_ts_raw(out);


          if (au_pts_ == AV_NOPTS_VALUE) {


            au_pts_ = (ts == AV_NOPTS_VALUE) ? 0 : ts;


            au_is_key_ = is_keypkt(out);


          } else if (ts != AV_NOPTS_VALUE && ts != au_pts_) {


            // 新帧：先发上一帧


            flush_current_au(/*force_publish=*/false);


            au_pts_ = ts;


            au_is_key_ = is_keypkt(out);


          } else {


            au_is_key_ = au_is_key_ || is_keypkt(out);


          }





          size_t old = au_buf_.size();


          au_buf_.resize(old + out->size);


          std::memcpy(au_buf_.data() + old, out->data, out->size);


        } else {


          // （不建议）一包一消息


          bool key = is_keypkt(out);


          if (!waiting_for_idr_ || key) {


            pace_by_pts(pkt_ts_raw(out));


            sensor_msgs::msg::CompressedImage msg;


            int64_t ts = pkt_ts_raw(out);


            msg.header.stamp = stamp_now_ ? now()


                                          : rclcpp::Time(static_cast<uint64_t>(


                                              (ts == AV_NOPTS_VALUE ? 0 : ts)), 0, RCL_ROS_TIME);


            msg.header.frame_id = frame_id_;


            msg.format = codec_name_;


            msg.data.assign(out->data, out->data + out->size);


            pub_->publish(std::move(msg));


            if (waiting_for_idr_ && key) {


              waiting_for_idr_ = false;


              RCLCPP_INFO(get_logger(), "检测到 IDR（非聚帧），开始按媒体时间节拍发布。");


            }


          }


        }





        av_packet_unref(out); av_packet_free(&out);


      }


    }


    flush_current_au(/*force_publish=*/false);


  }





private:


  // 参数


  std::string input_;


  std::string topic_;


  std::string frame_id_;


  bool  loop_{false};


  bool  stamp_now_{true};


  bool  reliable_{true};


  bool  aggregate_by_pts_{true};


  bool  wait_idr_on_start_{true};


  bool  pace_to_media_time_{true};


  double playback_speed_{1.0};


  bool  start_on_subscriber_{true};


  bool  exit_on_finish_{true};





  // 发布


  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;





  // FFmpeg


  AVFormatContext* fmt_ = nullptr;


  AVStream*        vs_  = nullptr;


  int              video_stream_index_ = -1;





  // Bitstream Filter


  AVBSFContext*    bsf_ctx_ = nullptr;


  std::string      bsf_name_;


  std::string      codec_name_;  // "h264" / "h265"





  // 当前帧（AU）缓存


  std::vector<uint8_t> au_buf_;


  int64_t au_pts_{AV_NOPTS_VALUE};


  bool au_is_key_{false};





  // 节拍


  double expected_period_s_{1.0/30.0};


  double media_t0_s_{std::numeric_limits<double>::quiet_NaN()};


  std::chrono::steady_clock::time_point wall_t0_{};





  // 线程/状态


  std::atomic<bool> running_{false};


  std::thread       worker_;


  bool started_{false};


  bool finished_{false};


  bool waiting_for_idr_{true};


};





int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 先做一个“管理节点”，只用于取四路输入路径等公共参数
  auto mgr = std::make_shared<rclcpp::Node>("h26x_multi_mgr");

  // 四路输入（若有一路为空则跳过那一路）
  std::string in_front = mgr->declare_parameter<std::string>("front_input", "");
  std::string in_left  = mgr->declare_parameter<std::string>("left_input",  "");
  std::string in_right = mgr->declare_parameter<std::string>("right_input", "");
  std::string in_back  = mgr->declare_parameter<std::string>("back_input",  "");

  // 公共开关。仍然可以被 CLI 覆盖，这里只是给默认值；每路我们都会再覆盖一遍。
  bool loop               = mgr->declare_parameter<bool>("loop", false);
  bool stamp_now          = mgr->declare_parameter<bool>("stamp_now", true);
  bool reliable           = mgr->declare_parameter<bool>("reliable", true);
  bool aggregate_by_pts   = mgr->declare_parameter<bool>("aggregate_by_pts", true);
  bool wait_idr_on_start  = mgr->declare_parameter<bool>("wait_idr_on_start", true);
  bool pace_to_media_time = mgr->declare_parameter<bool>("pace_to_media_time", true);
  double playback_speed   = mgr->declare_parameter<double>("playback_speed", 1.0);
  bool start_on_subscriber= mgr->declare_parameter<bool>("start_on_subscriber", true);
  bool exit_on_finish     = mgr->declare_parameter<bool>("exit_on_finish", true);
  std::string frame_id    = mgr->declare_parameter<std::string>("frame_id", "camera");

  // 一个小工具：按路生成一个实例（带参数覆盖）
  auto make_node = [&](const std::string& node_name,
                       const std::string& input_path,
                       const std::string& topic,
                       const std::string& frame_id_override) -> std::shared_ptr<H26xPassthroughPub> {
    if (input_path.empty()) return nullptr;

    rclcpp::NodeOptions opts;
    // 覆盖你原类里 declare_parameter 的那些参数
    opts.append_parameter_override("input", input_path);
    opts.append_parameter_override("topic", topic);
    opts.append_parameter_override("frame_id", frame_id_override);
    opts.append_parameter_override("loop", loop);
    opts.append_parameter_override("stamp_now", stamp_now);
    opts.append_parameter_override("reliable", reliable);
    opts.append_parameter_override("aggregate_by_pts", aggregate_by_pts);
    opts.append_parameter_override("wait_idr_on_start", wait_idr_on_start);
    opts.append_parameter_override("pace_to_media_time", pace_to_media_time);
    opts.append_parameter_override("playback_speed", playback_speed);
    opts.append_parameter_override("start_on_subscriber", start_on_subscriber);
    opts.append_parameter_override("exit_on_finish", exit_on_finish);

    // 关键：用新增的构造函数把参数传进去，并给每个实例不同的节点名
    return std::make_shared<H26xPassthroughPub>(opts, node_name);
  };

  // 逐一路创建
  std::vector<rclcpp::Node::SharedPtr> nodes;
  if (auto n = make_node("h26x_pub_front", in_front, "/surrounding_front/h26x/compressed", frame_id + "_front")) nodes.push_back(n);
  if (auto n = make_node("h26x_pub_left",  in_left,  "/surrounding_left/h26x/compressed",  frame_id + "_left"))  nodes.push_back(n);
  if (auto n = make_node("h26x_pub_right", in_right, "/surrounding_right/h26x/compressed", frame_id + "_right")) nodes.push_back(n);
  if (auto n = make_node("h26x_pub_back",  in_back,  "/surrounding_back/h26x/compressed",  frame_id + "_back"))  nodes.push_back(n);

  if (nodes.empty()) {
    RCLCPP_FATAL(mgr->get_logger(), "四路均未提供输入（front/left/right/back），请至少设置一路。");
    rclcpp::shutdown();
    return 1;
  }

  // 用多线程执行器同时 spin 这几个发布者
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(mgr); // 管理节点一起挂上，便于动态改参数（可选）
  for (auto& n : nodes) exec.add_node(n);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}

