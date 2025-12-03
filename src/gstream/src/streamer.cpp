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
      RCLCPP_INFO(this->get_logger(), "Pipeline created");

      appsrc = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(stream_pipeline), "mysrc"));
      GstCaps *caps = gst_caps_new_simple(
        "image/jpeg",
        "width", G_TYPE_INT, width_,
        "height", G_TYPE_INT, height_,
        "framerate", GST_TYPE_FRACTION, framerate, 1,
        NULL);
      
      gst_app_src_set_caps(appsrc, caps);
      gst_caps_unref(caps);    
      g_object_set(appsrc,
        "stream-type", GST_APP_STREAM_TYPE_STREAM,
        "is-live", TRUE,
        "do-timestamp", TRUE,
        NULL);

      gst_element_set_state(stream_pipeline, GST_STATE_PLAYING);

      // ROS subscriber
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "awareness/image_raw", 10, std::bind(&Streamer::image_callback, this, _1));
    }

    ~Streamer()
    {
      gst_app_src_end_of_stream(appsrc);
      gst_element_set_state(stream_pipeline, GST_STATE_NULL);
      gst_object_unref(stream_pipeline);
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
      RCLCPP_DEBUG(this->get_logger(), "got image")
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        return;
      }

      cv::Mat frame;
      cv::resize(cv_ptr->image, frame, cv::Size(width_, height_));

      // Encode to jpeg
      std::vector<uchar> jpeg_data;
      if (!cv::imencode(".jpg", frame, jpeg_data)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to encode jpeg");
        return;
      }

      // Create gstream buffer
      GstBuffer *buffer = gst_buffer_new_allocate(nullptr, jpeg_data.size(), nullptr);
      gst_buffer_fill(buffer, 0, jpeg_data.data(), jpeg_data.size());

      // Push buffer into appsrc
      GstFlowReturn ret = gst_app_src_push_buffer(appsrc, buffer);
      if (ret != GST_FLOW_OK) {
        RCLCPP_WARN(this->get_logger(), "GStreamer push buffer returned: %d", ret);
      }
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