# ImMesh
## ImMesh: An **Im**mediate LiDAR Localization and **Mesh**ing Framework
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/cover_v4.jpg" alt="video" width="100%" />
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/island_appendix.jpg" alt="video" width="100%" />
</div>

## **Date of code release**
Our paper is currently under review, and the code of ImMesh will be released as our work is accepted.

## 1. Introduction
**ImMesh** is a novel LiDAR(-inertial) odometry and meshing framework, which takes advantage of input of LiDAR data, achieving the goal of **simultaneous localization and meshing** in real-time. ImMesh comprises four tightly-coupled modules: receiver, localization, meshing, and broadcaster. The localization module utilizes the prepossessed sensor data from the receiver, estimates the sensor pose online by registering LiDAR scans to maps, and dynamically grows the map. Then, our meshing module takes the registered LiDAR scan for **incrementally reconstructing the triangle mesh on the fly**. Finally, the real-time odometry, map, and mesh are published via our broadcaster.
<div align="center">
<img src="https://github.com/ziv-lin/ImMesh_release/blob/main/pics/overview_v7.jpg" alt="video" width="100%" />
</div>

### 1.1 Our paper
Will be released in recent days...


### 1.2 Our accompanying videos
Our **accompanying videos** are now available on **YouTube** (click below images to open) and **Bilibili**<sup>[1](https://youtu.be/pzT2fMwz428), [2](https://youtu.be/pzT2fMwz428), [3](https://youtu.be/pzT2fMwz428)</sup>.

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