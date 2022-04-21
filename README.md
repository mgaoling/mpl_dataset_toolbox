# MPL Dataset Toolbox

[VECtor Benchmark](https://star-datasets.github.io/vector/) is the first complete set of benchmark datasets captured with a multi-sensor setup containing an event-based stereo camera, a regular stereo camera, multiple depth sensors, and an inertial measurement unit. The setup is fully hardware-synchronized and underwent accurate extrinsic calibration. All sequences come with ground truth data captured by highly accurate external reference devices such as a motion capture system. Individual sequences include both small and large-scale environments, and cover the specific challenges targeted by dynamic vision sensors.

This toolbox is a ROS workspace integrating with a set of easy-to-use dataset functions, including:

- [Event Visualizer](https://github.com/mgaoling/mpl_dataset_toolbox#event-visualizer): convert raw event stream into accumulated event frames.
- [Data Validator](https://github.com/mgaoling/mpl_dataset_toolbox#data-validator): validate the downloaded rosbag or the merged rosbag for its completeness.
- [Bag Merger](https://github.com/mgaoling/mpl_dataset_toolbox#bag-merger): merge multiple, single-topic rosbags chronologically into one complete rosbag.
- [Bag Splitter](https://github.com/mgaoling/mpl_dataset_toolbox#bag-splitter): split one multi-topic rosbag into a few compressed, single-topic rosbags.

# Getting Started

The following instructions are tested on [Ubuntu 20.04](https://ubuntu.com/download/desktop) with [ROS Noetic](http://wiki.ros.org/ROS/Installation), a ROS **desktop-full installation** is therefore required. On top of that, the [OpenCV Library](https://opencv.org/releases/) has to be installed:

```
sudo apt-get install libopencv-dev
```

After that, enter your catkin workspace and the build can be triggered with the following command:

```
cd ~/catkin_ws/src
git clone https://github.com/mgaoling/mpl_dataset_toolbox.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.bash
```

# Event Visualizer

The event visualizer converts raw event stream into accumulated event frames. It can handle at most two different event streams.

- For monocular setup, check and modify the parameters in the `config/event_visualizer.yaml`, then launch the visualization by:

```
roslaunch mpl_dataset_toolbox event_visualization.launch
```

- For stereo setup, check and modify the parameters in the `config/event_stereo_visualizer.yaml`, then launch the visualization by:

```
roslaunch mpl_dataset_toolbox event_stereo_visualization.launch
```

- To visualize the rosbag, run `rqt` or `rqt_image_view` in another terminal to display the accumulated event frames.

# Data Validator

The data validator helps to check the correctness and completeness of a downloaded, single-topic rosbag or a merged rosbag. It also calculates the Mean Event Rate if the event stream exists.

- Launch the validation in one terminal by:

```
roslaunch mpl_dataset_toolbox data_validation.launch
```

- Play the rosbag in another terminal by:

```
rosbag play [data.bag]
```

- Terminate the validation once the playback is over.

**Note:** Dropped frames on Kinect stream are to be expected, owing to the singularity of the depth calculation during recording. We have deliberately left a blank on the depth stream during the timeline reconstruction, to maintain the overall correctness.

# Bag Merger

Bag merger helps to merge different single-topic rosbags chronologically. Note that data sequences with different prefixes can be safely stored under the same directory. **DO NOT MODIFY THE FILENAME!**

- Put all single-topic rosbags under one folder, then launch the bag merger by:

```
roslaunch mpl_dataset_toolbox bag_merger.launch directory_path:=[path_to_directory]
```

**Note:** Here, only an absolute path without `~`, or a relative path that is relative to this toolbox is allowed for the directory path.

# Bag Splitter

Bag Splitter helps to split a multi-topic rosbag into a few single-topic rosbags, with a compression option.

- Launch the bag splitter by:

```
roslaunch mpl_dataset_toolbox bag_splitter.launch bag_path:=[data.bag] need_compression:=[true_or_false]
```

**Note:** Here, only an absolute path without `~`, or a relative path that is relative to this toolbox is allowed for the rosbag path.
