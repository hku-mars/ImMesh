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
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID( a ) ( ( abs( a ) > 1e8 ) ? true : false )

// enum LID_TYPE{AVIA = 1, VELO16, OUST64, L515}; //{1, 2, 3}
enum Feature
{
    Nor,
    Poss_Plane,
    Real_Plane,
    Edge_Jump,
    Edge_Plane,
    Wire,
    ZeroPoint
};
enum Surround
{
    Prev,
    Next
};
enum E_jump
{
    Nr_nor,
    Nr_zero,
    Nr_180,
    Nr_inf,
    Nr_blind
};

struct orgtype
{
    double  range;
    double  dista;
    double  angle[ 2 ];
    double  intersect;
    E_jump  edj[ 2 ];
    Feature ftype;
    orgtype()
    {
        range = 0;
        edj[ Prev ] = Nr_nor;
        edj[ Next ] = Nr_nor;
        ftype = Nor;
        intersect = 2;
    }
};

/*** Velodyne ***/
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float    intensity;
    float    time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT( velodyne_ros::Point,
                                   ( float, x, x )( float, y, y )( float, z, z )( float, intensity, intensity )( float, time, time )( uint16_t, ring,
                                                                                                                                      ring ) )
/****************/

/*** Ouster ***/
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float    intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT( ouster_ros::Point, ( float, x, x )( float, y, y )( float, z, z )( float, intensity, intensity )
                                   // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                   ( std::uint32_t, t, t )( std::uint16_t, reflectivity,
                                                            reflectivity )( std::uint8_t, ring, ring )( std::uint16_t, ambient,
                                                                                                        ambient )( std::uint32_t, range, range ) )
/****************/

/*** Hesai_XT32 ***/
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
    PCL_ADD_POINT4D;
    float    intensity;
    double   timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT( xt32_ros::Point,
                                   ( float, x, x )( float, y, y )( float, z, z )( float, intensity, intensity )( double, timestamp,
                                                                                                                 timestamp )( uint16_t, ring, ring ) )
/*****************/

class Preprocess
{
  public:
    //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();
    ~Preprocess();

    void process( const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out );
    void process( const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out );
    void set( bool feat_en, int lid_type, double bld, int pfilt_num );

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
    PointCloudXYZI    pl_full, pl_corn, pl_surf;
    PointCloudXYZI    pl_buff[ 128 ]; // maximum 128 line lidar
    vector< orgtype > typess[ 128 ];  // maximum 128 line lidar
    int               lidar_type, point_filter_num, N_SCANS, time_unit;
    double            blind, blind_sqr;
    double            time_unit_scale;
    bool              feature_enabled, given_offset_time, calib_laser;
    ros::Publisher    pub_full, pub_surf, pub_corn;

  private:
    void avia_handler( const livox_ros_driver::CustomMsg::ConstPtr &msg );
    void oust64_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
    void velodyne_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
    void velodyne32_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
    void xt32_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
    void l515_handler( const sensor_msgs::PointCloud2::ConstPtr &msg );
    void give_feature( PointCloudXYZI &pl, vector< orgtype > &types );
    void pub_func( PointCloudXYZI &pl, const ros::Time &ct );
    int  plane_judge( const PointCloudXYZI &pl, vector< orgtype > &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct );
    bool small_plane( const PointCloudXYZI &pl, vector< orgtype > &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct );
    bool edge_jump_judge( const PointCloudXYZI &pl, vector< orgtype > &types, uint i, Surround nor_dir );

    int    group_size;
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};
