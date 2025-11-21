#include <memory>
#include <string>
#include <sstream>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

using std::placeholders::_1;


class Streamer : public rclcpp::Node
{
  public:
    Streamer()
    : Node("g_streamer")
    {
      // Stream settings
      width_ = 640;
      height_ = 480;
      framerate = 30;
      host_ = "127.0.0.1";
      port = 25000;

      // Initialize GStreamer
      gst_init(nullptr, nullptr);
      // Create pipeline String with string stream :(
      std::ostringstream oss;
      oss << "appsrc name=mysrc is-live=true block=true format=time ! "
          << "image/jpeg,width=" << width_ << ",height=" << height_ << ",framerate=" << framerate << "/1 ! "
          << "jpegparse ! rtpjpegpay ! "
          << "udpsink host=" << host_ << " port=" << port << " sync=false";
      std::string pipeline_str = oss.str();

      GError* error = nullptr;
      stream_pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
      if(!stream_pipeline || error) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create pipeline: %s", error->message);
        return;
      }

      appsrc = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(stream_pipeline), "mysrc"));
      gst_element_set_state(stream_pipeline, GST_STATE_PLAYING);

      // ROS subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "awareness/image_raw", 10, std::bind(&Streamer::image_callback, this, _1));
    }

    ~Streamer()
    {
      gst_element_set_state(stream_pipeline, GST_STATE_NULL);
      gst_object_unref(stream_pipeline);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    GstElement* stream_pipeline;
    GstAppSrc* appsrc;

    int width_, height_;
    int framerate;
    std::string host_;
    int port;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Streamer>());
  rclcpp::shutdown();
  return 0;
}