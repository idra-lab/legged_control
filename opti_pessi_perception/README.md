# opti_pessi_perception

Perception front end of the Opti-Pessi controller. The node runs **YOLO26** on the RGB image of an RGB-D camera,
measures the distance of every detection from the depth image registered to color, and publishes the persons and
forklifts it sees as `legged_controllers/ObstacleArray` in `odom` on `/opti_pessi/obstacles`, the topic
`OptiPessiController` takes its obstacles from.

It is the C++ port of `yolo/yolo26_rgbd_detect.py`, with the same node name, parameters and topics. Inference runs
through [YOLOs-CPP](https://github.com/Geekgineer/YOLOs-CPP) (header-only) on ONNX Runtime instead of
Ultralytics/PyTorch, so the model is an **ONNX export**.

## Layout

```
include/opti_pessi_perception/Yolo26RGBD.h   the node: parameters, subscriptions, publishers
src/Yolo26RGBD.cpp                           detection, depth, 3D point, obstacles, drawing
src/yolo26_rgbd_detect.cpp                   executable: spins the node, OpenCV viewer window
weights/                                     models (*.onnx) and class name files, installed to share/opti_pessi_perception/weights
```

## What happens on every frame

1. **Pairing.** RGB and depth are matched by `message_filters` ApproximateTime (stamps at most 0.05 s apart). Depth in
   `16UC1`/`mono16` is millimeters and is converted to meters, `32FC1` is used as it is. A depth image whose size
   differs from the RGB is rejected: it is not registered to color, and resizing it would give wrong ranges.
2. **Detection.** YOLO26 runs on the RGB image with the `conf` threshold. YOLO26 is end-to-end (no NMS). Only the
   classes listed in `classes` are kept.
3. **Distance.** `z` is the median of the valid depth (finite, > 0) over the central half of the box (25 % cut from
   each side), so the background around the object does not count.
4. **3D point.** The box centre `(u, v)` is back-projected with the intrinsics `K` from `camera_info`:
   `x = (u − cx) z / fx`, `y = (v − cy) z / fy`, in the camera optical frame. Until the first `camera_info` arrives
   there is no 3D point, so no `Detection3D` and no obstacle.
5. **`~/detections`.** One `Detection3D` per box with a 3D point. The box size is the metric width and height at `z`;
   its depth size is 0, since one view does not show the thickness.
6. **Obstacles.** `person` becomes `Obstacle.HUMAN`, `forklift` becomes `Obstacle.CAR`. The measured depth is the
   visible front surface, so the point is moved `obstacle_center_offset` (0.15 m) further along the camera ray to
   approximate the centre of the body. It is then transformed to `obstacles_frame` (`odom`) with the TF at the
   image stamp (waiting up to 0.1 s).
   * The whole set goes out in one message on every frame, **an empty set included**: the controller replaces its
     set with each message, so an empty one clears the obstacles that left the view.
   * Without the TF nothing is published (a warning every 5 s), and the controller keeps the last set it received.
7. **`~/detections_image`.** The RGB with boxes, class, confidence, distance and `xyz`, under a status bar with the
   detection count, the inference time and the input rate (from the message stamps, so it follows sim time).
   With `blur_persons`, persons are blurred before drawing.

The keep-out radius and the assumed maximum speed of each type are not set here: they come from `obstacleTypes` in
`opti_pessi_interface/config/task.info`. The controller ignores `position.z` and any frame other than `odom`.

## Dependencies

| dependency | where it comes from |
| --- | --- |
| `rclcpp`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`, `cv_bridge`, `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `ament_index_cpp` | ROS 2 Jazzy |
| OpenCV 4 | system (`libopencv-dev`) |
| `legged_controllers` | this workspace, only for `Obstacle.msg` and `ObstacleArray.msg` |
| YOLOs-CPP | `src/YOLOs-CPP`, CMake variable `YOLOS_CPP_DIR`. The checkout carries `COLCON_IGNORE`: its own CMakeLists only builds demos |
| ONNX Runtime 1.20.1, GPU build | `src/YOLOs-CPP/onnxruntime-linux-x64-gpu-1.20.1`, CMake variable `ONNXRUNTIME_DIR`. Found at run time through the RPATH |

`find_package(legged_controllers)` also loads everything that package exports (OCS2, Pinocchio, …), which is why
the first configure needs `pinocchio_DIR`. Moving the two messages into a small message package would remove that.

## Build

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select opti_pessi_perception --cmake-args -DCMAKE_BUILD_TYPE=Release -Dpinocchio_DIR=/opt/openrobots/lib/cmake/pinocchio
```

`pinocchio_DIR` is only needed the first time: CMake keeps it in its cache.

## Models

Export a `.pt` to ONNX with the Ultralytics venv (it needs `onnx`: `.venv-yolo/bin/pip install onnx onnxslim`), put
the `.onnx` in `weights/` and rebuild, since the build copies the files into `install/`:

```bash
.venv-yolo/bin/yolo export model=yolo/yolo26n.pt format=onnx     # writes yolo/yolo26n.onnx
cp yolo/yolo26n.onnx src/legged_control/opti_pessi_perception/weights/
colcon build --packages-select opti_pessi_perception
```

* **Where `model` is looked up.** An absolute path is used as it is. A relative one is taken from
  `share/opti_pessi_perception/weights/` when the file is there, otherwise from the current directory. Only `*.onnx`,
  `*.names` and `*.txt` are installed; the `.pt` weights stay in the source folder.
* **Class names.** Read from the ONNX metadata, which Ultralytics writes on export. For a model without it, pass
  `labels:=<file>` with one name per line in class-id order; it is looked up like `model`.
* **Forklifts.** The default model is trained on COCO, which has no forklift class. To publish forklifts as `CAR`
  obstacles, use a model trained on both classes (the runs in `yolo/runs/detect/`, on the `forklift`/`person`
  datasets of `yolo/personforklift.yaml` or `yolo/forklift_person.yaml`), export it and ask for both:

  ```bash
  ros2 run opti_pessi_perception yolo26_rgbd_detect --ros-args -p model:=best.onnx -p "classes:=[person, forklift]"
  ```

## Run

```bash
ros2 run opti_pessi_perception yolo26_rgbd_detect
ros2 run opti_pessi_perception yolo26_rgbd_detect --ros-args -p use_sim_time:=true -p show:=false
```

The depth must be registered to the color image (RealSense: `align_depth.enable:=true`).

**GPU.** The CUDA execution provider needs the CUDA 12 runtime, cuBLAS, cuFFT, cuRAND, NVRTC and cuDNN 9 on the
loader path. They are not installed system-wide here; the PyTorch wheels in `.venv-yolo` provide all of them:

```bash
export LD_LIBRARY_PATH=$(ls -d ~/RaNAV/.venv-yolo/lib/python3.12/site-packages/nvidia/*/lib | tr '\n' ':')$LD_LIBRARY_PATH
```

The startup log says `model … on gpu` or `on cpu`. When the CUDA provider cannot load, the node warns and falls
back to the CPU.

**Viewer** (`show:=true`): the annotated RGB. `q` or `Esc` quits (and stops the node), `s` saves a PNG snapshot in
the current directory. Closing the window also stops the node.

## Topics

| direction | topic | type | notes |
| --- | --- | --- | --- |
| in | `rgb_topic` | `sensor_msgs/Image` | sensor-data QoS, converted to `bgr8` |
| in | `depth_topic` | `sensor_msgs/Image` | registered to color, `16UC1`/`mono16` (mm) or `32FC1` (m) |
| in | `info_topic` | `sensor_msgs/CameraInfo` | intrinsics `K` |
| in | `/tf`, `/tf_static` | | camera optical frame to `odom` |
| out | `/yolo26_rgbd_detect/detections` | `vision_msgs/Detection3DArray` | camera optical frame, meters |
| out | `/yolo26_rgbd_detect/detections_image` | `sensor_msgs/Image` | annotated RGB |
| out | `obstacles_topic` | `legged_controllers/ObstacleArray` | `obstacles_frame`, one message per frame |

## Parameters

| parameter | default | meaning |
| --- | --- | --- |
| `rgb_topic` | `/camera/color/image_raw` | color image |
| `depth_topic` | `/camera/aligned_depth_to_color/image_raw` | depth registered to color |
| `info_topic` | `/camera/color/camera_info` | color camera intrinsics |
| `model` | `yolo26n.onnx` | ONNX model, looked up as described in [Models](#models) |
| `labels` | `""` | class names file; empty: the names in the ONNX metadata |
| `use_gpu` | `true` | CUDA through ONNX Runtime, CPU when it cannot load |
| `conf` | `0.4` | confidence threshold |
| `classes` | `[person]` | class names to keep; names not in the model are reported at startup |
| `show` | `true` | viewer window |
| `max_depth` | `10.0` | range of the depth colormap [m] |
| `blur_persons` | `true` | blur persons in the annotated image |
| `obstacles_topic` | `/opti_pessi/obstacles` | where the obstacles go |
| `obstacles_frame` | `odom` | frame of the obstacles; the controller ignores any other |
| `obstacle_center_offset` | `0.15` | distance from the visible surface to the obstacle centre along the ray [m] |
