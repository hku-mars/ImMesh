# ImMesh: An **Im**mediate LiDAR Localization and **Mesh**ing Framework


## News (Nov, 2023): ImMesh is accepted to [T-RO](https://ieeexplore.ieee.org/document/10304337)
Our research has been accepted for publication in ***IEEE Transactions on Robotics (T-RO)*** ([**avaible here**](https://ieeexplore.ieee.org/document/10304337)). We plan to release our work codes as we have promised. Before that, we still need to improve our codes by following the suggestions of reviewers, and need to refactor our code to make it clearer and cleaner.

Due to the busy timeline near and after my graduation, **I anticipate releasing my codes no later than the 2024 Spring Festival (i.e., February 10, 2024).**

### Date of code release:
 No later than **<u>Feb 10, 2024</u> (Chinese New Year 2024).**


## **1. Introduction**
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/cover_v4.jpg" alt="video" width="100%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/island_appendix.jpg" alt="video" width="100%" />
</div>
**ImMesh** is a novel LiDAR(-inertial) odometry and meshing framework, which takes advantage of input of LiDAR data, achieving the goal of **simultaneous localization and meshing** in real-time. ImMesh comprises four tightly-coupled modules: receiver, localization, meshing, and broadcaster. The localization module utilizes the prepossessed sensor data from the receiver, estimates the sensor pose online by registering LiDAR scans to maps, and dynamically grows the map. Then, our meshing module takes the registered LiDAR scan for **incrementally reconstructing the triangle mesh on the fly**. Finally, the real-time odometry, map, and mesh are published via our broadcaster.

<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/overview.jpg" alt="video" width="100%" />
</div>

### 1.1 Our paper
Our research paper can be [accessed from T-RO](https://ieeexplore.ieee.org/document/10304337) or be downloaded [here](https://github.com/hku-mars/ImMesh/raw/main/paper/ImMesh_tro.pdf).

### 1.2 Our accompanying videos
Our **accompanying videos** are now available on **YouTube** (click below images to open) and **Bilibili**<sup>[1]( https://www.bilibili.com/video/BV1AG4y1177z), [2](https://www.bilibili.com/video/BV1Xd4y1j7on/), [3](https://www.bilibili.com/video/BV1W8411N7D2)</sup>.

<div align="center">
<a href="https://youtu.be/pzT2fMwz428" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=4" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_contents.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=10" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_1.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=191" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_2.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=321" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_3.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=499" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_4.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=622" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_5.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=892" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/video_cover_6.jpg" alt="video" width="48%" /></a>
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
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_2_trial_2-0.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/raw/main/pics/gifs/application_2_trial_2-1.gif"  width="48%" />
</div>

## Related works

ImMesh is building upon the foundations of our previous SLAM works including  [R3LIVE](https://github.com/hku-mars/r3live), [VoxelMap](https://github.com/hku-mars/VoxelMap), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [R2LIVE](https://github.com/hku-mars/r2live), and [ikd-Tree](https://github.com/hku-mars/ikd-Tree). These works are all available on our GitHub, as listed below:

1. [R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package](https://github.com/hku-mars/r3live)
2. [VoxelMap: An efficient and probabilistic adaptive(coarse-to-fine) voxel mapping method for 3D LiDAR](https://github.com/hku-mars/VoxelMap)
3. [FAST-LIO: A computationally efficient and robust LiDAR-inertial odometry (LIO) package](https://github.com/hku-mars/FAST_LIO)
4. [R2LIVE: a robust, real-time tightly-coupled multi-sensor fusion framework](https://github.com/hku-mars/r2live)
5.  [ikd-Tree: an incremental k-d tree designed for robotic applications](https://github.com/hku-mars/ikd-Tree) 

## **3. Contact us**
If you have any questions about this work, please feel free to contact me via [**www.jiaronglin.com**](https://jiaronglin.com/) and Dr. Fu Zhang <fuzhangAThku.hk> via email.

