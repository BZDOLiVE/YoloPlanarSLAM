# YOLO-Planar-SLAM
This is a RGBD SLAM system developed based on [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2). I tried to develop a SLAM system that can work in both dynamic environment and low-texture environment. This system use [YOLO-fastest](https://github.com/dog-qiuqiu/Yolo-Fastest) to remove moving human in the environment and use [CAPE](https://github.com/pedropro/CAPE) plane detection to extract plane features. This system do not need GPU, I use a desktop computer with i76800k CPU to test my system, FPS of this system is about 24.

Test video on TUM dataset: https://youtu.be/nfauXS6AWnk

**Authors:** BZD

<!-- ## License

``` -->

## 1. Prerequisites

This system has been tested on **ubuntu 16.04** and **ubuntu 20.04**.

### C++11 or C++0x Compiler

### [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)
I use ORB-SLAM2 to develop my system, so first please follow the installation guidance from [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2), make sure ORB-SLAM2 can work well on your computer.

### [YOLO-fastest](https://github.com/dog-qiuqiu/Yolo-Fastest)
This work use NCNN to deploy YOLO-fastest model in our system. Please follow the guidance from [YOLO-fastest](https://github.com/dog-qiuqiu/Yolo-Fastest), and [NCNN](https://github.com/Tencent/ncnn), you need to make the ncnn sample in YOLO-fastest work well on your computer. You do not need GPU support in NCNN, this system do not use GPU. In YOLO-fastest they have already provide some YOLO models for you, this system use yolo-fastest-1.1_body that only detect bbox of human from input RGB image.

### [NCNN](https://github.com/Tencent/ncnn)
After you install ncnn in your computer, you will get some head files in "/home/yourUsername/ncnn/build/install/include" and a "libncnn.a" file in "/home/yourUsername/ncnn/build/install/lib". This system needs these files to compile. You can put these file in the folder of this system, our you can adjust the include file path in CMakeLists.txt

### [PCL](http://www.pointclouds.org/)
This system use PCL to manage plane features. I use PCL 1.9 on my computer, and I have tested that PCL 1.10 can not work.



## 2. Test the system

This system has been tested on [TUM RGB-D](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) dataset

### Test the system locally

1. Download sequences in TUM RGB-D dataset and associate RGB-D pairs based on [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools) provided by the dataset.

   ```
   python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
   ```

2. Compile the system

  ```
  ./build.sh
  ```


3.  Run the system

An example command for run this system with rgbd_dataset_freiburg3_walking_xyz sequence
```
./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM3.yaml /home/yourUsername/rgbd_dataset_freiburg3_walking_xyz /home/yourUsername/rgbd_dataset_freiburg3_walking_xyz/associations.txt
```

4.  Run with ROS and Realsense D435i

Install ros in your computer.

Follow the guidence in ORBSLAM2 to run ORBSLAM2 with ros

Go to Intel homepage and install realsense ros library

```
roscore

rosrun ORB_SLAM2 RGBD /home/yourusername/YoloPlanarSLAM/Vocabulary/ORBvoc.txt /home/yourusername/YoloPlanarSLAM/Examples/ROS/ORB_SLAM2/RealsenseD435i.yaml

roslaunch realsense2_camera rs_rgbd.launch
```

if you can't run
```
roslaunch realsense2_camera rs_rgbd.launch
```

in ubuntu 16 you can try
```
sudo apt install ros-kinetic-rgbd-launch
```


## Acknowledgement

Except ORB_SLAM2 this system has used code from [DS-SLAM](https://github.com/ivipsourcecode/DS-SLAM) and [SP-SLAM](https://github.com/fishmarch/SP-SLAM)

