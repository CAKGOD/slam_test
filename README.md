# SLAM_EVALUTION
(WIP)

**This is repository about SLAM evalution. There are some [codes](https://github.com/CAKGOD/slam_test/tree/master/codes) and [reports](https://github.com/CAKGOD/slam_test/tree/master/reports).**

## Evaluation method

We consider the working principle and application scenarios of SLAM. Three dimensions are used to evaluate SLAM more comprehensively.

- Visual mapping effect

Since we have not found a good method to evaluate the mapping effort, we can only view the 3D maps with naked eyes.

- Simultaneous localization ability

To evalute the simultaneous localization ability, the 3D slam dataset with groudtruth can be used. We can record the topics contain trajecroty data and then compare them with the standard data provided by groundtruth. Some tools like [evo tool](https://github.com/MichaelGrupp/evo) can help us to achieve this function.

- Hardware resource usage

To evalute the hardware resource usage of SLAM, the usages of hardwares including cpu and memory need to be considered. 

## SLAM frameworks  

We have test some SLAM frameworks based on ROS1 or ROS2. They are shown below. We will keep looking for new SLAM frameworks and test them.

- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)

- [LEGO_LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)

- [lidarslam_ros2](https://github.com/rsasaki0109/lidarslam_ros2)

- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM)

- [li-slam-ros2](https://github.com/rsasaki0109/li_slam_ros2)

## Evalution codes

Data used for slam evalution contains hardware resource usages and trajectory offsets. To get these data, we use "top" command to get the cpu & memory usages and "rosbag record" command to get the rosbag file contains trajectory offset values. We wrote a automatic evalution script in Python which you can find it [here](https://github.com/CAKGOD/slam_test/tree/master/codes).

## Related package

- [evo tool](https://github.com/MichaelGrupp/evo)

## Evalution reports

We have test some frameworks and write some evalution reports about them.

- [evalution reports]((https://github.com/CAKGOD/slam_test/tree/master/reports))

## TODO

There are still some things to do. 

- Improve the codes and tell you how to use it.

- Integrate the reports in a complete report.

- Make a comparison table to help you make a clear choice of SLAM frameworks.
