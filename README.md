# MPL Dataset Toolbox

[VECtor Benchmark](https://star-datasets.github.io/vector/) is the first complete set of benchmark datasets captured with a multi-sensor setup containing an event-based stereo camera, a regular stereo camera, multiple depth sensors, and an inertial measurement unit. The setup is fully hardware-synchronized and underwent accurate extrinsic calibration. All sequences come with ground truth data captured by highly accurate external reference devices such as a motion capture system. Individual sequences include both small and large-scale environments, and cover the specific challenges targeted by dynamic vision sensors.

This toolbox is a ROS workspace integrating with a set of easy-to-use dataset functions, including:

- [Event Visualizer](https://github.com/mgaoling/mpl_dataset_toolbox#event-visualizer): convert raw event stream into accumulated event frames.
- [Data Validator](https://github.com/mgaoling/mpl_dataset_toolbox#data-validator): validate the downloaded rosbag or the merged rosbag for its completeness.
- [Bag Merger](https://github.com/mgaoling/mpl_dataset_toolbox#bag-merger): merge multiple, single-topic rosbags chronologically into one. 
- [Bag Splitter](https://github.com/mgaoling/mpl_dataset_toolbox#bag-splitter-optional): split one multi-topic rosbag into a few compressed, single-topic rosbags.

# License and Citation

This toolbox, together with the [MPL Calibration Toolbox](https://github.com/mgaoling/mpl_calibration_toolbox), is available as open-source under the terms of the [BSD-3-Clause-Clear License](https://github.com/mgaoling/mpl_dataset_toolbox/blob/main/LICENSE.txt). If you use this toolbox in an academic context, please cite the [publication](https://star-datasets.github.io/vector/assets/pdf/vector.pdf) as follows:

```bibtex
@Article{gao2022vector,
  author  = {Gao, Ling and Liang, Yuxuan and Yang, Jiaqi and Wu, Shaoxun and Wang, Chenyu and Chen, Jiaben, and Kneip, Laurent},
  title   = {{VECtor}: A Versatile Event-Centric Benchmark for Multi-Sensor SLAM},
  journal = {IEEE Robotics and Automation Letters"},
  year    = {2022}
}
```

# Getting Started

## Requirement and Dependency

- [Ubuntu 20.04 or 18.04](https://ubuntu.com/download/desktop)
- [ROS Melodic or Noetic](http://wiki.ros.org/ROS/Installation)
- [OpenCV 4 or 3](https://opencv.org/releases/)
- [Prophesee ROS Wrapper](https://github.com/prophesee-ai/prophesee_ros_wrapper)

## Compile

```
cd ~/catkin_ws/src
git clone https://github.com/mgaoling/mpl_dataset_toolbox.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Usage

## Event Visualizer

The event visualizer converts raw event stream into accumulated event frames. It can handle at most two different event streams.

- For monocular setup, check and modify the parameters in the `config/event_visualizer.yaml`, then launch the visualization by:

```
roslaunch mpl_dataset_toolbox event_visualizer.launch
```

- For stereo setup, check and modify the parameters in the `config/event_stereo_visualizer.yaml`, then launch the visualization by:

```
roslaunch mpl_dataset_toolbox event_stereo_visualizer.launch
```

- To visualize the rosbag, run `rqt` or `rqt_image_view` in another terminal to display the accumulated event frames, along with regular image frames.

## Data Validator

The data validator helps to check the correctness and completeness of a downloaded, single-topic rosbag or a merged rosbag. It also calculates the Mean Event Rate if the event stream exists.

- Launch the validation in one terminal by:

```
roslaunch mpl_dataset_toolbox data_validation.launch
```

- Play the rosbag in another terminal by:

```
rosbag play [bag_path]
```

- Terminate the validation once the playback is over.

**Note** Dropped frames on Kinect stream are to be expected, owing to the singularity of the depth calculation during recording. We have deliberately left a blank on the depth stream during the timeline reconstruction, to maintain the overall correctness.

## Bag Merger

Bag merger helps to merge different single-topic bags chronologically. Note that data sequences with different prefixes can be safely stored under the same directory. **DO NOT MODIFY THE FILENAME!**

- Put all single-topic rosbags under one folder, then launch the bag merger by:

```
roslaunch mpl_dataset_toolbox bag_merger.launch directory_path:=[path_to_directory]
```

## Bag Splitter (optional)

Bag Splitter helps to split a multi-topic bag into a few single-topic rosbags, with a compression option.

- Launch the bag splitter by:

```
roslaunch mpl_dataset_toolbox bag_splitter.launch bag_path:=[path_to_bag] need_compression:=[true_or_false]
```
