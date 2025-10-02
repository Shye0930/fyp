# stereo_image_compressor

ROS 2 Python package for compressing and decompressing camera streams:

- Stereo: stitch left and right images side-by-side and publish a single JPEG `sensor_msgs/CompressedImage` for bandwidth-efficient transport; decompress back into two `sensor_msgs/Image` topics.
- RGB‑D (RealSense-friendly): compress RGB to JPEG and Depth (16UC1) to the standard `compressedDepth` PNG format; decompress back to raw images.

This is useful when streaming over constrained links (e.g., Wi‑Fi) or recording compact bag files while keeping a simple, ROS-native workflow.


## What’s included

Executables (console scripts):

- Stereo
	- `compressor` → `stereo_image_compressor/compressor_node.py`
	- `decompressor` → `stereo_image_compressor/decompressor_node.py`
- RGB‑D
	- `rgbd_compressor` → `stereo_image_compressor/rgbd_compressor_node.py`
	- `rgbd_decompressor` → `stereo_image_compressor/rgbd_decompressor_node.py`

Launch files (in `launch/`):

- `compressor.launch.py` (stereo stitched JPEG)
- `decompressor.launch.py` (stereo stitched JPEG)
- `rgbd_compress.launch.py` (RGB‑D JPEG + compressedDepth)
- `rgbd_decompress.launch.py` (RGB‑D decompression)


## Features

- Approximate time synchronization for stereo (`message_filters::ApproximateTimeSynchronizer`, slop 0.05s)
- Adjustable JPEG quality for compression speed vs. fidelity
- Optional resize to a fixed width/height before stitching
- Reliable QoS profile (keep last 10)
- Depth compression uses the standardized `compressed_depth_image_transport` header: 12‑byte header followed by PNG data (format code 0: PNG_RAW)


## Dependencies

ROS2 Foxy

Runtime (ROS 2):

- `rclpy`, `sensor_msgs`, `cv_bridge`, `message_filters`

Python/system:

- `numpy` (>= 1.24), OpenCV (Python bindings)

On Ubuntu/ROS 2, typical packages include:

```bash
sudo apt install ros-${ROS_DISTRO}-cv-bridge python3-opencv python3-numpy
```


## Build

Inside your ROS 2 workspace root (here, `ros_ws/`):

```bash
# Within the ROS workspace
colcon build --packages-select stereo_image_compressor
source install/setup.bash
```


## Usage

### Stereo stitched JPEG

Launch (recommended):

```bash
ros2 launch stereo_image_compressor compressor.launch.py 
```

Decompress back to two raw image topics:

```bash
ros2 launch stereo_image_compressor decompressor.launch.py
```

Run as nodes directly:

```bash
# Compressor
ros2 run stereo_image_compressor compressor 

# Decompressor
ros2 run stereo_image_compressor decompressor 
```

Topics and parameters (stereo):

- Compressor subscribes:
	- `left_topic` (Image, default `/camera/camera/infra1/image_rect_raw`)
	- `right_topic` (Image, default `/camera/camera/infra2/image_rect_raw`)
- Compressor publishes:
	- `compressed_topic` (CompressedImage, format `jpeg`, default `/camera/stereo/compressed`)
- Decompressor subscribes:
	- `input_topic` (CompressedImage `jpeg`, default `/camera/stereo/compressed`)
- Decompressor publishes:
	- `left_output_topic` (Image, default `/camera/stereo/left_decompressed`)
	- `right_output_topic` (Image, default `/camera/stereo/right_decompressed`)
- Shared parameters:
	- `image_width` (int, default 640) and `image_height` (int, default 480) — size of each individual view
	- `jpeg_quality` (int 0–100, default 80 for node, 100 in launch file example)

Important: The decompressor uses `image_width` to split the JPEG into left/right halves; ensure the width/height match the compressor settings.


### RGB‑D (JPEG + compressedDepth)

Launch (recommended):

```bash
ros2 launch stereo_image_compressor rgbd_compress.launch.py 
```

Decompress:

```bash
ros2 launch stereo_image_compressor rgbd_decompress.launch.py 
```


Topics and parameters (RGB‑D):

- Compressor subscribes:
	- `rgb_topic` (Image `bgr8`, default `/camera/camera/color/image_raw`)
	- `depth_topic` (Image `16UC1`/`mono16`, default `/camera/camera/aligned_depth_to_color/image_raw`)
- Compressor publishes:
	- `rgb_compressed_topic` (CompressedImage format `jpeg`, default `/camera/color/image_raw/compressed`)
	- `depth_compressed_topic` (CompressedImage format `16UC1; compressedDepth`, default `/camera/depth/image_rect_raw/compressedDepth`)
- Decompressor subscribes:
	- `compressed_rgb_topic` (CompressedImage `jpeg`)
	- `compressed_depth_topic` (CompressedImage `compressedDepth`)
- Decompressor publishes:
	- `decompressed_rgb_topic` (Image `bgr8`)
	- `decompressed_depth_topic` (Image `16UC1`)
- Compression controls:
	- `jpeg_quality` (0–100, higher = better quality, larger size)
	- `png_level` (0–9, higher = smaller size, slower)


## Notes and caveats

- Stereo decompression assumes a horizontal stitch and a fixed `image_width`; mismatches will cause left/right images to be misaligned.
- RGB‑D depth compression uses the same on-wire format as `compressed_depth_image_transport` (format code 0 = PNG_RAW). Custom depth scaling parameters in the header are set to 0.0 and are currently unused by this package.
- QoS is RELIABLE with a depth of 10; adjust in source if needed for high-rate streams.
- Launch defaults use topic names under `/camera/camera/...` to match some RealSense configurations; change them to suit your setup.
- Known quirk: `launch/compressor.launch.py` currently points to the `rgbd_compressor` executable. If you want the stereo stitched compressor via launch, either run the node directly with `ros2 run ... compressor` or update that launch file to use the `compressor` executable.


## Verifying it works

Optional checks while running the nodes:

```bash
# List topics
ros2 topic list

# Inspect compressed stream
ros2 topic echo /camera/stereo/compressed --no-arr

# Check rates
ros2 topic hz /camera/stereo/compressed

# Show images (requires image_tools or your preferred viewer)
ros2 run image_tools showimage --ros-args -r image:=/decompressed_rgb
```


## Troubleshooting

- No images published:
	- Ensure input topics exist and encodings are correct (`bgr8` for RGB, `16UC1` for depth).
	- Check QoS compatibility between publishers and subscribers.
	- For stereo, confirm the left/right topics have timestamps close enough for ApproximateTimeSynchronizer (`slop=0.05`).
- Decompression looks wrong or split is offset:
	- Verify `image_width`/`image_height` match the compressor configuration.
- Depth decompression fails:
	- Ensure the message format string contains `compressedDepth` and the first 12 bytes header is present; this package only supports format code 0 (PNG_RAW).

