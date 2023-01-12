# ImMesh
## ImMesh: An **Im**mediate LiDAR Localization and **Mesh**ing Framework
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/cover_v4.jpg" alt="video" width="100%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/island_appendix.jpg" alt="video" width="100%" />
</div>

## 1. Introduction
**ImMesh** is a novel LiDAR(-inertial) odometry and meshing framework, which takes advantage of input of LiDAR data, achieving the goal of **simultaneous localization and meshing** in real-time. ImMesh comprises four tightly-coupled modules: receiver, localization, meshing, and broadcaster. The localization module utilizes the prepossessed sensor data from the receiver, estimates the sensor pose online by registering LiDAR scans to maps, and dynamically grows the map. Then, our meshing module takes the registered LiDAR scan for **incrementally reconstructing the triangle mesh on the fly**. Finally, the real-time odometry, map, and mesh are published via our broadcaster.
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/overview_v7.jpg" alt="video" width="100%" />
</div>

## **Date of code release**
Our paper is currently under review, and **the code of ImMesh will be released as our work is accepted**. However, our previous SLAM works [R3LIVE](https://github.com/hku-mars/r3live), [VoxelMap](https://github.com/hku-mars/VoxelMap), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [R2LIVE](https://github.com/hku-mars/r2live), and [ikd-Tree](https://github.com/hku-mars/ikd-Tree) are now avaiable on our github.

1. [R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package](https://github.com/hku-mars/r3live)
2. [VoxelMap: An efficient and probabilistic adaptive(coarse-to-fine) voxel mapping method for 3D LiDAR](https://github.com/hku-mars/VoxelMap)
3. [FAST-LIO: A computationally efficient and robust LiDAR-inertial odometry (LIO) package](https://github.com/hku-mars/FAST_LIO)
4. [R2LIVE: a robust, real-time tightly-coupled multi-sensor fusion framework](https://github.com/hku-mars/r2live)
5.  [ikd-Tree: an incremental k-d tree designed for robotic applications](https://github.com/hku-mars/ikd-Tree) 


### 1.1 Our paper
Our preprint paper is now can be downloaded [here](https://github.com/hku-mars/ImMesh/blob/main/paper/ImMesh_v0.pdf).

### 1.2 Our accompanying videos
Our **accompanying videos** are now available on **YouTube** (click below images to open) and **Bilibili**<sup>[1]( https://www.bilibili.com/video/BV1AG4y1177z), [2](https://www.bilibili.com/video/BV1Xd4y1j7on/), [3](https://www.bilibili.com/video/BV1W8411N7D2)</sup>.

<div align="center">
<a href="https://youtu.be/pzT2fMwz428" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=4" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_contents.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=10" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_1.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=191" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_2.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=321" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_3.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=499" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_4.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=622" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_5.jpg" alt="video" width="48%" /></a>
<a href="https://youtu.be/pzT2fMwz428?t=892" target="_blank"><img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/video_cover_6.jpg" alt="video" width="48%" /></a>
</div>

## 2. What can ImMesh do?
### 2.1 Simultaneous LiDAR localization and mesh reconstruction on the fly
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/hku_seq_campus.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/hku_seq_main_building.gif"  width="48%" />
</div>

### 2.2 ImMesh for LiDAR point cloud reinforement
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_1_fov.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_1_res.gif"  width="48%" />
</div>

### 2.3 ImMesh for rapid, lossless texture reconstruction
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_2_trial_1-0.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_2_trial_1-1.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_2_trial_2-0.gif"  width="48%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/gifs/application_2_trial_2-1.gif"  width="48%" />
</div>

## 3. Contact us
If you have any questions about this work, please feel free to contact me <ziv.lin.ljrATgmail.com> and Dr. Fu Zhang <fuzhangAThku.hk> via email.