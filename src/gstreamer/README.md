# GStreamer Node
This package contains the node we're using to stream video to the UI.
The node subscribes to a rostopic that publishes images from the webcam,
and feeds those images into a webrtc stream.\
GStreamer 1.22 or later is required.

## Available Nodes

### `webrtc_stream`
How to run:\
`ros2 run gstreamer webrtc_stream`
#### Parameters
| Name | Type | Description |
| ---- | ---- | ----------- |
| `signaling_url` | `string` | The url of the signaling server (example: `"ws://172.26.38.226:6767"`) |
| `video_topic` | `string` | The name of the topic the node subscribes to for video frames (default: `/camera/image_raw`) |
| `bitrate` | `int` | The bitrate of the video stream (default: `1800000`) |
| `stream_height` | `int` | The height of the output video stream (default: `480`) |
| `stream_width` | `int` | The width of the output video stream (default: `640`) |
#### Topics
| Name | Message Type | Behavior |
| ---- | ------------ | -------- |
| `video_topic` parameter | `sensor_msgs/Image` | The node subscribes to this topic to get the frames for the video stream |
