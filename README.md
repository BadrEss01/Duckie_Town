# Duckietown robotics

> **Selected project · academic** · ROS 1 and camera-based line-following experiments
>
> [Selected projects](https://github.com/BadrEss01/BadrEss01#selected-projects) · [Coursework](https://github.com/BadrEss01/BadrEss01/blob/main/COURSEWORK.md)

Python experiments collected during robotics coursework, using a Duckietown ROS template. The repository contains a camera-based line follower, object-detection integration code, landmark simulation helpers and a CNN training script.

**Status:** historical coursework under documentation and reproducibility review. This is not a complete, validated autonomous-driving stack.

## Runnable ROS package (2026 extension)

The new `packages/duckie_lane_following/` catkin package wraps the image-to-steering
pipeline in a configurable ROS 1 node. It includes package.xml, CMake/setup files,
YAML parameters, launch file, headless processing, bounded steering, line-loss stop
and a received-frame timeout. The original scripts below remain as historical evidence.
This maintenance extension is AI-assisted and is not claimed as the original 2021 implementation.

### Run on ROS Noetic

From the repository root, with ROS Noetic and its dependencies installed:

```bash
mkdir -p /tmp/duckie_ws/src
cp -r packages/duckie_lane_following /tmp/duckie_ws/src/
cd /tmp/duckie_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
roslaunch duckie_lane_following lane_following.launch image_topic:=/mybot/camera1/image_raw cmd_topic:=/cmd_vel
```

The root Dockerfile provides a Noetic build with dependencies. It replaces the
unconfigured legacy Duckietown template container. On Linux, with a compatible ROS
master/camera already available, `docker build -t duckie-lane .` builds the image.
Configure ROS_MASTER_URI and ROS_IP for your network before connecting a physical robot.

Input is raw `sensor_msgs/Image`; output is `geometry_msgs/Twist`. A physical
Duckiebot using compressed images or `Twist2DStamped` requires an adapter and correct
topic mapping; this package is not a tested plug-and-play Duckiebot driver.
Tune HSV bounds and speed in `config/lane.yaml`. The watchdog measures time since
callback receipt, not camera capture age. It cannot protect against a crashed process
or paused ROS clock: a separate motor-controller timeout is required on hardware.
No obstacle avoidance or complete SLAM is provided by this package.

Tests: pure control tests run without ROS; the ROS integration workflow builds catkin
and exercises actual image/Twist transport, steering, camera timeout and line loss.
Physical driving and Gazebo end-to-end navigation remain unvalidated.

## Historical repository contents

| File | What is present | Current limitation |
| --- | --- | --- |
| `packages/LineFollower.py` | ROS 1 image subscriber, OpenCV HSV masking, image-moment centroid and proportional steering published as `Twist` | Adapted/included upstream example; hardware and simulator execution have not been revalidated |
| `packages/featur_det.py` | ROS wrapper using `dodo_detector`, with TensorFlow/keypoint detector options and optional point-cloud input | External detector packages, models and configuration are required; integration is not verified |
| `packages/SLAM.py` | Landmark-world visualization and simulated measurement/motion generation | Imports `robot_class`, which is absent; this file is not a complete SLAM estimator |
| `packages/model.py` | Keras binary CNN training script with image augmentation | Referenced dataset is absent; no verified training result or robotics integration is supplied |
| `Dockerfile`, `launchers/` | Original Duckietown template infrastructure | Root container and launcher replaced by the new Noetic package; historical configs remain |

## Line-following pipeline

A ROS image is converted to BGR with `cv_bridge`, converted to HSV and thresholded. A horizontal strip near the lower portion of the image is retained. Its mask centroid determines horizontal error relative to the image center, and a proportional controller publishes forward and angular velocity.

Current interfaces:
- Camera input: `/mybot/camera1/image_raw` (`sensor_msgs/Image`).
- Velocity output: `/cmd_vel` (`geometry_msgs/Twist`).
- ROS API: `rospy` (ROS 1, not ROS 2).
- Debug display: OpenCV GUI windows.

These topic names follow the upstream simulated robot example; they are not a verified Duckiebot hardware interface.

## Dependencies and execution status

The line follower requires a compatible ROS 1 environment with `rospy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, NumPy and OpenCV. Python, ROS and cv_bridge versions must be compatible. The original working environment has not been recovered.

Once ROS is configured and a compatible simulated robot supplies the camera and velocity interfaces, the script entry point is:

```bash
python3 packages/LineFollower.py
```

This is an entry point, not a verified end-to-end installation recipe. The root Dockerfile now launches the packaged node described above.

For local image-processing checks without ROS:

```bash
python3 -m pip install numpy opencv-python-headless
python3 -m unittest discover -s tests -v
```

The tests use real NumPy/OpenCV on synthetic images and stub ROS transport and GUI calls. They check centroid-based steering, cropping and stopping after line loss; they do not validate ROS communication, simulation, hardware or real-world perception.

## Limitations

- The inherited HSV thresholds are broad and need calibration; the code does not establish robust yellow-lane detection.
- A frame with no detected line now publishes a zero-velocity command, tested after a moving frame. The new package adds a received-frame watchdog and invalid-frame stop; the historical script is unchanged.
- The repository does not establish obstacle avoidance, integrated mapping, a completed SLAM pipeline or measured navigation performance.
- Dataset identities, trained model results and historical personal modifications require additional evidence.

## Attribution and project context

Repository maintained by Badr Essefiany as a record of robotics coursework.

The line follower is derived from [Arjun S Kumar's Line-Follower--ROS](https://github.com/arjunskumar/Line-Follower--ROS). Its image-processing and steering logic closely match that example. Upstream authorship is retained; inclusion here does not imply original authorship of that algorithm.

The repository infrastructure comes from [Duckietown's ROS template](https://github.com/duckietown/template-ros). The detector wrapper imports `dodo_detector` and `dodo_detector_ros`; the provenance and extent of modifications to the remaining experiments still need review. Existing license material is retained.

The current maintenance pass fixes Python syntax, slice indexing, callback initialization order and stopping after line loss and adds documentation and focused checks. It does not establish which historical components were independently implemented by the repository owner.

