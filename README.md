
# RTSP Camera ROS 2 Package

## Overview

The **`rtsp_camera`** package is a ROS 2 node that captures an RTSP (Real Time Streaming Protocol) video stream with minimal latency using GStreamer and publishes the frames as ROS 2 image messages. It allows for configurable stream parameters via ROS 2 node parameters, enabling easy integration into your robotic applications.

## Features

- **Low-Latency RTSP Streaming**: Utilizes GStreamer to efficiently receive RTSP streams with minimal latency.
- **Configurable Parameters**: Stream URL, image size, and camera name can be controlled via ROS 2 node parameters.
- **Image Publishing**: Publishes the video stream as ROS 2 image messages on a configurable topic.
- **ROS 2 Logging**: Provides informative logging for connection status and errors.
- **OpenCV Integration**: Uses OpenCV for image processing, allowing for easy extension and customization.

## Dependencies

- **ROS 2**: Tested with ROS 2 humble.
- **GStreamer 1.0**: Multimedia framework for handling RTSP streams.
- **OpenCV**: For image manipulation and conversion.
- **cv_bridge**: ROS 2 package that provides an interface between ROS 2 image messages and OpenCV images.

## Installation

### Prerequisites

Ensure that you have the following dependencies installed on your system:

#### GStreamer and Plugins

```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

#### OpenCV Development Libraries

```bash
sudo apt-get install libopencv-dev
```

#### ROS 2 Packages

```bash
sudo apt-get install ros-humble-rclcpp ros-humble-sensor-msgs ros-humble-image-transport ros-humble-cv-bridge
```

*Replace `humble` with your ROS 2 distribution name if different.*

## Building the Package

1. **Clone the Repository**

   Navigate to your ROS 2 workspace source directory and clone the package:

   ```bash
   cd ~/ros2_ws/src  # Replace with your workspace path
   git clone https://github.com/yourusername/rtsp_camera.git
   ```

2. **Build the Package**

   From your workspace root directory, build the package using `colcon`:

   ```bash
   cd ~/ros2_ws  # Replace with your workspace path
   colcon build --packages-select rtsp_camera
   ```

3. **Source the Workspace**

   After building, source your workspace:

   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Node

You can run the node using the `ros2 run` command and specify parameters as needed.

```bash
ros2 run rtsp_camera rtsp_camera_node --ros-args \
-p rtsp_url:=<your_rtsp_url> \
-p camera_name:=<your_camera_name> \
-p width:=<desired_width> \
-p height:=<desired_height>
```

#### Example

```bash
ros2 run rtsp_camera rtsp_camera_node --ros-args \
-p rtsp_url:=rtsp://192.168.144.25:8554/main.264 \
-p camera_name:=camera1 \
-p width:=640 \
-p height:=480
```

### Parameters

- `rtsp_url` (string): **[Required]** The RTSP stream URL.
- `camera_name` (string): Name of the camera, used in the image topic name. Default is `"camera"`.
- `width` (integer): Desired width of the published image. Default is `1280`.
- `height` (integer): Desired height of the published image. Default is `720`.

### Using a YAML Parameters File

You can also use a YAML file to specify parameters.

**Create a `params.yaml` file:**

```yaml
rtsp_camera_node:
  ros__parameters:
    camera_name: "camera1"
    rtsp_url: "rtsp://192.168.144.25:8554/main.264"
    width: 640
    height: 480
```

**Run the node with the parameters file:**

```bash
ros2 run rtsp_camera rtsp_camera_node --ros-args --params-file params.yaml
```

### Visualizing the Image Stream

Use `rqt_image_view` or `rviz2` to visualize the published image topic.

#### Using `rqt_image_view`

```bash
ros2 run rqt_image_view rqt_image_view
```

- In the GUI, select the topic `/camera1/image_raw` (replace `camera1` with your `camera_name` parameter).

#### Using `rviz2`

```bash
ros2 run rviz2 rviz2
```

- Add an **Image** display.
- Set the **Image Topic** to `/camera1/image_raw`.

## Node Details

### Published Topics

- `/<camera_name>/image_raw` (`sensor_msgs/msg/Image`): Publishes the raw image stream from the RTSP source.

### Logging

The node uses ROS 2 logging to provide information and error messages:

- **Info Messages**:
  - Starting the node and pipeline.
  - End-of-stream notifications.
- **Error Messages**:
  - Issues with GStreamer elements.
  - Errors received from the GStreamer pipeline.

## Customization

### Hardware-Accelerated Decoding

If you have hardware-accelerated decoding available (e.g., NVIDIA GPUs), you can modify the node to use a hardware decoder.

**Example: Replace `avdec_h264` with `nvh264dec`**

In `src/rtsp_camera_node.cpp`, find:

```cpp
decoder_ = gst_element_factory_make("avdec_h264", "decoder");
```

Replace with:

```cpp
decoder_ = gst_element_factory_make("nvh264dec", "decoder");  // For NVIDIA GPUs
```

Ensure you have the necessary GStreamer plugins installed:

```bash
sudo apt-get install gstreamer1.0-vaapi  # For Intel GPUs
sudo apt-get install gstreamer1.0-nvidia  # For NVIDIA GPUs (if available)
```

### Adjusting Latency

You can adjust the `latency` property of the `rtspsrc` element if needed.

**Example:**

```cpp
g_object_set(G_OBJECT(src_), "latency", 100, NULL);  // Set latency to 100 ms
```

## Troubleshooting

- **No Image Published**: Ensure that the RTSP URL is correct and accessible. Check network connectivity.
- **High Latency**: Adjust the `latency` parameter or consider hardware-accelerated decoding.
- **Error Messages**: Monitor the node's logs for error messages to diagnose issues.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Based on GStreamer and OpenCV integration with ROS 2.
- Inspired by community efforts to integrate RTSP streaming into robotic applications.

## Contribution

Contributions are welcome! Please open issues or pull requests for improvements, bug fixes, or new features.

## Contact

For questions or support, please contact [Your Name](mailto:your.email@example.com).
