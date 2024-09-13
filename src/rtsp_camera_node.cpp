#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

class RTSPCameraNode : public rclcpp::Node {
public:
    RTSPCameraNode()
    : Node("rtsp_camera_node")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("camera_name", "camera");
        this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.144.25:8554/main.264");
        this->declare_parameter<int>("width", 1280);
        this->declare_parameter<int>("height", 720);

        this->get_parameter("camera_name", camera_name_);
        this->get_parameter("rtsp_url", rtsp_url_);
        this->get_parameter("width", width_);
        this->get_parameter("height", height_);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_name_ + "/image_raw", 10);

        RCLCPP_INFO(this->get_logger(), "Starting RTSP camera node with URL: %s", rtsp_url_.c_str());

        // Initialize GStreamer
        gst_init(nullptr, nullptr);

        // Start the GStreamer pipeline
        start_pipeline();
    }

    ~RTSPCameraNode()
    {
        // Clean up
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(bus_);
        gst_object_unref(pipeline_);
    }

private:
    void start_pipeline()
    {
        // Create GStreamer pipeline
        pipeline_ = gst_pipeline_new("rtsp-pipeline");
        src_ = gst_element_factory_make("rtspsrc", "source");
        depay_ = gst_element_factory_make("rtph264depay", "depay");
        h264parse_ = gst_element_factory_make("h264parse", "h264parse");
        decoder_ = gst_element_factory_make("avdec_h264", "decoder");
        videoconvert_ = gst_element_factory_make("videoconvert", "videoconvert");
        appsink_ = gst_element_factory_make("appsink", "appsink");

        if (!pipeline_ || !src_ || !depay_ || !h264parse_ || !decoder_ || !videoconvert_ || !appsink_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create GStreamer elements.");
            return;
        }

        // Set RTSP source properties
        g_object_set(G_OBJECT(src_), "location", rtsp_url_.c_str(), NULL);
        g_object_set(G_OBJECT(src_), "latency", 0, NULL);  // Set latency to 0 for minimal delay
        g_object_set(G_OBJECT(src_), "buffer-mode", 4, NULL);  // Live stream mode

        // Configure appsink to request a higher framerate
        g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, "sync", FALSE, NULL);
        GstCaps* caps = gst_caps_from_string("video/x-raw, format=(string)BGR, framerate=(fraction)30/1");
        gst_app_sink_set_caps(GST_APP_SINK(appsink_), caps);
        gst_caps_unref(caps);

        // Add elements to the pipeline
        gst_bin_add_many(GST_BIN(pipeline_), src_, depay_, h264parse_, decoder_, videoconvert_, appsink_, NULL);

        // Link elements together
        if (!gst_element_link_many(depay_, h264parse_, decoder_, videoconvert_, appsink_, NULL)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to link elements.");
            gst_object_unref(pipeline_);
            return;
        }

        // Connect the pad-added signal
        g_signal_connect(src_, "pad-added", G_CALLBACK(+[](GstElement* src, GstPad* new_pad, GstElement* depay) {
            GstPad* sink_pad = gst_element_get_static_pad(depay, "sink");
            if (!gst_pad_is_linked(sink_pad)) {
                gst_pad_link(new_pad, sink_pad);
            }
            gst_object_unref(sink_pad);
        }), depay_);

        // Set up bus to handle messages
        bus_ = gst_pipeline_get_bus(GST_PIPELINE(pipeline_));

        // Start the pipeline
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        // Start the frame processing loop
        processing_thread_ = std::thread(&RTSPCameraNode::process_frames, this);
    }

    void process_frames()
    {
        gboolean quit = FALSE;

        while (rclcpp::ok() && !quit) {
            GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), GST_SECOND / 10);
            if (sample) {
                GstBuffer* buffer = gst_sample_get_buffer(sample);
                GstCaps* caps = gst_sample_get_caps(sample);
                GstStructure* s = gst_caps_get_structure(caps, 0);

                // Get width and height
                int width, height;
                gst_structure_get_int(s, "width", &width);
                gst_structure_get_int(s, "height", &height);

                GstMapInfo map;
                if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                    // Convert to OpenCV Mat
                    cv::Mat frame(cv::Size(width, height), CV_8UC3, (char*)map.data, cv::Mat::AUTO_STEP);

                    // Resize the frame if necessary
                    if (width != width_ || height != height_) {
                        cv::resize(frame, frame, cv::Size(width_, height_));
                    }

                    // Convert to ROS Image message
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    msg->header.stamp = this->get_clock()->now();
                    msg->header.frame_id = camera_name_;

                    // Publish the image
                    image_pub_->publish(*msg);

                    gst_buffer_unmap(buffer, &map);
                }

                gst_sample_unref(sample);
            } else {
                // No sample available, sleep briefly
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Check for messages on the bus
            while (gst_bus_have_pending(bus_)) {
                GstMessage* msg = gst_bus_pop(bus_);
                if (msg != NULL) {
                    GError* err;
                    gchar* debug_info;

                    switch (GST_MESSAGE_TYPE(msg)) {
                        case GST_MESSAGE_ERROR:
                            gst_message_parse_error(msg, &err, &debug_info);
                            RCLCPP_ERROR(this->get_logger(), "Error received from element %s: %s",
                                         GST_OBJECT_NAME(msg->src), err->message);
                            RCLCPP_ERROR(this->get_logger(), "Debugging information: %s", debug_info ? debug_info : "none");
                            g_clear_error(&err);
                            g_free(debug_info);
                            quit = TRUE;
                            break;
                        case GST_MESSAGE_EOS:
                            RCLCPP_INFO(this->get_logger(), "End-Of-Stream reached.");
                            quit = TRUE;
                            break;
                        default:
                            // For other messages, do nothing
                            break;
                    }
                    gst_message_unref(msg);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "Stopping pipeline.");
        gst_element_set_state(pipeline_, GST_STATE_NULL);
    }

    // ROS parameters
    std::string camera_name_;
    std::string rtsp_url_;
    int width_;
    int height_;

    // ROS publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // GStreamer elements
    GstElement* pipeline_;
    GstElement* src_;
    GstElement* depay_;
    GstElement* h264parse_;
    GstElement* decoder_;
    GstElement* videoconvert_;
    GstElement* appsink_;
    GstBus* bus_;

    // Thread for processing frames
    std::thread processing_thread_;
};

int main(int argc, char* argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<RTSPCameraNode>();

    // Spin the node (process callbacks)
    rclcpp::spin(node);

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}
