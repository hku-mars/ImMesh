# ImMesh: An **Im**mediate LiDAR Localization and **Mesh**ing Framework


## News (Jan, 2024): Release of Code

The codes of **ImMesh** (pronounced as "***I-am-Mesh***") are now available! You can following our guidelines in this page to setup, run and evaluate our work. 

The source code of this package is released under [**GPLv2**](http://www.gnu.org/licenses/) license. We only allow it free for personal and academic usage. For commercial use, please contact me <ziv.lin.ljrATgmail.com> and Dr. Fu Zhang <fuzhangAThku.hk> to negotiate a different license.

I hope you will love and enjoy our work, ImMesh. If you feel like ImMesh has indeed helped in your current research or work, I would greatly appreciate it if you could give a star to this repository or cite our paper in your academic research paper :)

## **1. Introduction**
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/cover_v4.jpg" alt="video" width="100%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/island_appendix.jpg" alt="video" width="100%" />
</div>

**ImMesh** (pronounced as "***I-am-Mesh***") is a novel LiDAR(-inertial) odometry and meshing framework, which takes advantage of input of LiDAR data, achieving the goal of **simultaneous localization and meshing** in real-time. ImMesh comprises four tightly-coupled modules: receiver, localization, meshing, and broadcaster. The localization module utilizes the prepossessed sensor data from the receiver, estimates the sensor pose online by registering LiDAR scans to maps, and dynamically grows the map. Then, our meshing module takes the registered LiDAR scan for **incrementally reconstructing the triangle mesh on the fly**. Finally, the real-time odometry, map, and mesh are published via our broadcaster.

<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/overview.jpg" alt="video" width="100%" />
</div>

### 1.1 Our paper
Our research has been accepted for publication in ***IEEE Transactions on Robotics (T-RO)*** ([**avaible here**](https://ieeexplore.ieee.org/document/10304337)). 

Our paper can be [accessed from T-RO](https://ieeexplore.ieee.org/document/10304337) or be downloaded [here](https://github.com/hku-mars/ImMesh/raw/main/paper/ImMesh_tro.pdf).

### 1.2 Our accompanying videos
Our **accompanying videos** are now available on **YouTube** (click below images to open) and **Bilibili**<sup>[1]( https://www.bilibili.com/video/BV1AG4y1177z), [2](https://www.bilibili.com/video/BV1Xd4y1j7on/), [3](https://www.bilibili.com/video/BV1W8411N7D2)</sup>.

<div align="center">
<a href="https://youtu.be/pzT2fMwz428" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=4" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_contents.jpg" alt="video" width="48%" /></a>
</div>

## **2. What can ImMesh do?**
### 2.1 Simultaneous LiDAR localization and mesh reconstruction on the fly
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/hku_seq_campus.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/hku_seq_main_building.gif"  width="48%" />
</div>

### 2.2 ImMesh for LiDAR point cloud reinforement
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_1_fov.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_1_res.gif"  width="48%" />
</div>

### 2.3 ImMesh for rapid, lossless texture reconstruction
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_2_trial_1-0.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_2_trial_1-1.gif"  width="48%" />
</div>

## 3. Prerequisites
### 3.1 **ROS**
Following this [ROS Installation](http://wiki.ros.org/ROS/Installation) to install ROS and its additional pacakge:<br>
```
sudo apt-get install ros-XXX-cv-bridge ros-XXX-tf ros-XXX-message-filters ros-XXX-image-transport ros-XXX-image-transport*
```
**NOTICE:** remember to replace "XXX" on above command as your ROS distributions, for example, if your use ROS-kinetic, the command should be:<br>
```
sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-tf ros-kinetic-message-filters ros-kinetic-image-transport*
```
### 3.2. **livox_ros_driver**
Follow this [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

### 3.3 **CGAL** and **OpenGL**
```
sudo apt-get install -y libcgal-dev pcl-tools
sudo apt-get install -y libgl-dev libglm-dev libglfw3-dev libglew-dev libglw1-mesa-dev 
sudo apt-get install -y libcgal-dev libxkbcommon-x11-dev
```
## 4. Build ImMesh on ROS:
Clone this repository and catkin_make:
```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/ImMesh.git
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 5. Run our examples
### 5.1 Solid-state LiDAR (e.g., Livox LiDARs)
ImMesh is capable run with solid-state LiDAR (e.g., Livox-avia form ([r3live_dataset](https://github.com/ziv-lin/r3live_dataset))). 

Our example rosbag files for evaluation can be download from our [Google drive](https://drive.google.com/drive/folders/15i-TRa0EA8BCbNdARVqPMDsU9JOlagVF?usp=sharing) or [Baidu-NetDisk [百度网盘]](https://pan.baidu.com/s/1zmVxkcwOSul8oTBwaHfuFg) (code提取码: wwxw). 

After you have downloaded our bag files, you can now run our example ^_^
```
roslaunch ImMesh mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
``` 

### 5.2 Mechanical spinning LiDAR (e.g., Velodyne and Ouster LiDARs)
ImMesh is also capable to mechanical spinning LiDAR (e.g., Velodyne LiDAR form ([KITTI-dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php), [NCLT-dataset](https://robots.engin.umich.edu/nclt), and [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/) ). 

We also provide some example rosbag files for evaluation (thanks for [KITTI-dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) and  [NCLT-dataset](https://robots.engin.umich.edu/nclt)) , which can be download from our [OneDrive](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zivlin_connect_hku_hk/EkoRwaDJskVOpeZTetFDQNYBdKwfRYvvHJyncLTM78t13w?e=Tte1Lh).

After you have preparation of rosbag files, you can now run our example with:

For [KITTI-dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php):
```
roslaunch ImMesh mapping_velody64.launch
rosbag play YOUR_DOWNLOADED.bag
``` 

For [NCLT-dataset](https://www.cvlibs.net/datasets/kitti/eval_odometry.php):
```
roslaunch ImMesh mapping_nclt.launch
rosbag play YOUR_DOWNLOADED.bag
``` 

For [NTU-VIRAL](https://ntu-aris.github.io/ntu_viral_dataset):
```
roslaunch ImMesh mapping_ntu_viral.launch
rosbag play YOUR_DOWNLOADED.bag
``` 

### 5.2 Fast offline mesh reconstruction with given pointcloud file

ImMesh can also perform the fast offline mesh reconstruction with given point cloud file (*.pcd). Modified the pointcloud file name in mapping_pointcloud.launch:

```
# In mapping_pointcloud.launch file:
<param name="pc_name" type="string" value="YOUR_POINTCLOUD.pcd" />
```

Then, perform the mesh reconstruction with:
```
roslaunch ImMesh mapping_pointcloud.launch
```

## 6. Result and tips.
If everything is correct, you will get the result same as the screenshot shown in below:

<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/ImMesh_gui.jpg" alt="video" width="100%" />
</div>

We provide some functions/features/demonstrations in our GUI (implemented based on [ImGUI](https://github.com/ocornut/imgui) ), for example, as listed as below (press "*F1*" key for help):

### 6.1 LiDAR pointcloud reinforcement

Click "*enable*" checkbox (under the treenode of "*LiDAR pointcloud reinforcement*") to toggle realtime real-time depth rasterization and LiDAR pointcloud reinforcement. 

Tips: You can click the "*Dynamic configuration*" checkbox to online dynamic reconfigure the rasterization parameter.

### 6.2 Export the reconstructed mesh result to disk
ImMesh allow you to save the online reconstructed mesh you build at anytime you wanted. 

Click the "*Save Mesh to PLY file*" button (on the Main windows panel) to export the mesh to PLY file (default save in directory: ${HOME}/ImMesh_output).

### 6.3 Other features
Due to the space limitation, we are unable to list out all our features on this page. We recommend you to explore our work through GUI interaction by yourself.

I hope you will love and enjoy our work, ImMesh. If you feel like ImMesh has indeed helped in your current research or work, I would greatly appreciate it if you could give a star to this repository or cite our paper in your academic research paper.:)

## Related works

ImMesh is building upon the foundations of our previous SLAM works including  [R3LIVE](https://github.com/hku-mars/r3live), [VoxelMap](https://github.com/hku-mars/VoxelMap), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [R2LIVE](https://github.com/hku-mars/r2live), and [ikd-Tree](https://github.com/hku-mars/ikd-Tree). These works are all available on our GitHub, as listed below:

1. [R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package](https://github.com/hku-mars/r3live)
2. [VoxelMap: An efficient and probabilistic adaptive(coarse-to-fine) voxel mapping method for 3D LiDAR](https://github.com/hku-mars/VoxelMap)
3. [FAST-LIO: A computationally efficient and robust LiDAR-inertial odometry (LIO) package](https://github.com/hku-mars/FAST_LIO)
4. [R2LIVE: a robust, real-time tightly-coupled multi-sensor fusion framework](https://github.com/hku-mars/r2live)
5. [ikd-Tree: an incremental k-d tree designed for robotic applications](https://github.com/hku-mars/ikd-Tree) 

## **Contact us**

We know our packages might not totally stable in this stage, and we are keep working on improving the performance and reliability of our codes. So, if you have met any bug or problem, please feel free to draw an issue or contact me via email <ziv.lin.ljrATgmail.com> and I will respond ASAP.

If you have any other question about this work, please feel free to contact me via [**www.jiaronglin.com**](https://jiaronglin.com/) and Dr. Fu Zhang <fuzhangAThku.hk> via email.
