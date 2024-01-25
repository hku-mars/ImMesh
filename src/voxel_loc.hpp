/* 
This code is the implementation of our paper "ImMesh: An Immediate LiDAR Localization and Meshing Framework".

The source code of this package is released under GPLv2 license. We only allow it free for personal and academic usage. 

If you use any code of this repo in your academic research, please cite at least one of our papers:
[1] Lin, Jiarong, et al. "Immesh: An immediate lidar localization and meshing framework." IEEE Transactions on Robotics
   (T-RO 2023)
[2] Yuan, Chongjian, et al. "Efficient and probabilistic adaptive voxel mapping for accurate online lidar odometry."
    IEEE Robotics and Automation Letters (RA-L 2022)
[3] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled
    state Estimation and mapping package." IEEE International Conference on Robotics and Automation (ICRA 2022)

For commercial use, please contact me <ziv.lin.ljr@gmail.com> and Dr. Fu Zhang <fuzhang@hku.hk> to negotiate a 
different license.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once

#include "common_lib.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <csignal>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <so3_math.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// int layer_size[3] = {8, 5, 10};
// float eigen_value_array[3] = {0.0025, 0.0025, 0.0005};
typedef struct ptpl
{
    Eigen::Vector3d               point;
    Eigen::Vector3d               normal;
    Eigen::Vector3d               center;
    Eigen::Matrix< double, 6, 6 > plane_var;
    int                           layer;
    double                        d;
    double                        eigen_value;
    bool                          is_valid;
} ptpl;

typedef struct Point_with_var
{
    Eigen::Vector3d m_point;
    Eigen::Vector3d m_point_world;
    Eigen::Matrix3d m_var;
} Point_with_var;

struct M_POINT
{
    float xyz[ 3 ];
    float intensity;
    int   count = 0;
};

typedef struct Plane
{
    pcl::PointXYZINormal          m_p_center;
    Eigen::Vector3d               m_center;
    Eigen::Vector3d               m_normal;
    Eigen::Matrix3d               m_covariance;
    Eigen::Matrix< double, 6, 6 > m_plane_var;
    float                         m_radius = 0;
    float                         m_min_eigen_value = 1;
    float                         m_d = 0;
    int                           m_points_size = 0;
    bool                          m_is_plane = false;
    bool                          m_is_init = false;
    int                           m_id;
    bool                          m_is_update = false;
} Plane;

class VOXEL_LOC
{
  public:
    int64_t x, y, z;
    VOXEL_LOC( int64_t vx = 0, int64_t vy = 0, int64_t vz = 0 ) : x( vx ), y( vy ), z( vz ) {}
    bool operator==( const VOXEL_LOC &other ) const { return ( x == other.x && y == other.y && z == other.z ); }
};

// Hash value
namespace std
{
template <>
struct hash< VOXEL_LOC >
{
    int64_t operator()( const VOXEL_LOC &s ) const
    {
        using std::hash;
        using std::size_t;
        return ( ( ( ( s.z ) * HASH_P ) % MAX_N + ( s.y ) ) * HASH_P ) % MAX_N + ( s.x );
    }
};
} // namespace std

class OctoTree
{
  public:
    std::vector< Point_with_var > m_temp_points_;
    Plane *                       m_plane_ptr_;
    int                           m_layer_;
    int                           m_max_layer_;
    int                           m_octo_state_; // 0 is end of tree, 1 is not
    OctoTree *                    m_leaves_[ 8 ];
    double                        m_voxel_center_[ 3 ]; // x, y, z
    std::vector< int >            m_layer_init_num_;
    float                         m_quater_length_;
    float                         m_planer_threshold_;
    int                           m_update_size_threshold_;
    int                           m_octo_init_size_;
    int                           m_max_points_size_;
    int                           m_new_points_;
    bool                          m_init_octo_;
    bool                          m_update_enable_;

    OctoTree( int max_layer, int layer, std::vector< int > layer_init_num, int max_point_size, float planer_threshold )
        : m_max_layer_( max_layer ), m_layer_( layer ), m_layer_init_num_( layer_init_num ), m_max_points_size_( max_point_size ),
          m_planer_threshold_( planer_threshold )
    {
        m_temp_points_.clear();
        m_octo_state_ = 0;
        m_new_points_ = 0;
        // when new points num > 5, do a update
        m_update_size_threshold_ = 5;
        m_init_octo_ = false;
        m_update_enable_ = true;
        m_octo_init_size_ = m_layer_init_num_[ m_layer_ ];
        for ( int i = 0; i < 8; i++ )
        {
            m_leaves_[ i ] = nullptr;
        }
        m_plane_ptr_ = new Plane;
    }

    void init_plane( const std::vector< Point_with_var > &points, Plane *plane );

    void init_octo_tree();

    void cut_octo_tree();

    void UpdateOctoTree( const Point_with_var &pv );

    void updatePlane();
};
