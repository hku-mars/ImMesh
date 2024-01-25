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
#include <condition_variable>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "IMU_Processing.h"
#include "preprocess.h"
#include "ikd-Tree/ikd_Tree.h"
#include "voxel_loc.hpp"

#define INIT_TIME ( 0.0 )
#define MAXN ( 360000 )
#define PUBFRAME_PERIOD ( 20 )
#define NUM_POINTS 2000

using KDtree_pt = ikdTree_PointType;
using KDtree_pt_vector = KD_TREE< KDtree_pt >::PointVector;


#define time_debug
#define HASH_P 116101
#define MAX_N 10000000000

const bool intensity_contrast( PointType &x, PointType &y );

const bool var_contrast( Point_with_var &x, Point_with_var &y );

float calc_dist( PointType p1, PointType p2 );

void mapJet( double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b );

void buildUnorderMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const Eigen::Vector3d &layer_size,
                      const float planer_threshold, std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map );

void buildVoxelMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const int max_layer,
                    const std::vector< int > &layer_init_num, const int max_points_size, const float planer_threshold,
                    std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map );

V3F RGBFromVoxel( const V3D &input_point, const float voxel_size, const Eigen::Vector3d &layer_size, const float planer_threshold,
                  std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map );

void updateUnorderMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const Eigen::Vector3d &layer_size,
                       const float planer_threshold, std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map );

void updateVoxelMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const int max_layer,
                     const std::vector< int > &layer_init_num, const int max_points_size, const float planer_threshold,
                     std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map );
void build_single_residual( const Point_with_var &pv, const OctoTree *current_octo, const int current_layer, const int max_layer,
                            const double sigma_num, bool &is_sucess, double &prob, ptpl &single_ptpl );

void transformLidar( const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                     pcl::PointCloud< pcl::PointXYZI >::Ptr &trans_cloud );

void BuildPtplList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const float match_eigen_value, const int layer, const float voxel_size,
                    const float match_constraint, const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr input_cloud,
                    std::vector< ptpl > &ptpl_list );

void BuildResidualListOMP( const unordered_map< VOXEL_LOC, OctoTree * > &voxel_map, const double voxel_size, const double sigma_num,
                           const int max_layer, const std::vector< Point_with_var > &pv_list, std::vector< ptpl > &ptpl_list,
                           std::vector< Eigen::Vector3d > &non_match );

void BuildResidualList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const double voxel_size, const double sigma_num,
                        const Point_with_var &pv, const M3D &rot, const V3D &t, const M3D &ext_R, const V3D &ext_T, ptpl &ptpl_list );

void BuildOptiList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const float voxel_size, const Eigen::Matrix3d rot,
                    const Eigen::Vector3d t, const PointCloudXYZI::Ptr input_cloud, const float sigma_num,
                    const std::vector< Eigen::Matrix3d > var_list, std::vector< ptpl > &ptpl_list );

void CalcQuation( const Eigen::Vector3d &vec, const int axis, geometry_msgs::Quaternion &q );

void NormalToQuaternion( const Eigen::Vector3d &normal_vec, geometry_msgs::Quaternion &q );

void pubNormal( visualization_msgs::MarkerArray &normal_pub, const std::string normal_ns, const int normal_id, const pcl::PointXYZINormal normal_p,
                const float alpha, const Eigen::Vector3d rgb );

void pubPlane( visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const int plane_id, const pcl::PointXYZINormal normal_p,
               const float radius, const float min_eigen_value, const float alpha, const Eigen::Vector3d rgb );

void pubPlaneMap( const std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const ros::Publisher &plane_map_pub, const V3D &position );

// Similar with PCL voxelgrid filter
void down_sampling_voxel( pcl::PointCloud< pcl::PointXYZI > &pl_feat, double voxel_size );

void calcBodyVar( Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var );

void reconstruct_mesh_from_pointcloud( pcl::PointCloud< pcl::PointXYZI >::Ptr frame_pts, double minimum_pts_distance = 0.01 );
class Voxel_mapping
{
  public:
    std::shared_ptr< ros::NodeHandle > m_ros_node_ptr = nullptr;
    float                              DETECTION_RANGE = 300.0f;
    const float                        MOV_THRESHOLD = 1.5f;
    mutex                              m_mutex_buffer;
    condition_variable                 m_sig_buffer;

    // mutex mtx_buffer_pointcloud;

    string m_root_dir = ROOT_DIR;
    string m_map_file_path, m_lid_topic, m_imu_topic, m_hilti_seq_name, m_img_topic, m_config_file;
    M3D    Eye3d = M3D::Identity();
    M3F    Eye3f = M3F::Identity();
    V3D    Zero3d = V3D::Zero();
    V3F    Zero3f = V3F::Zero();
    V3D    m_extT = Zero3d;
    M3D    m_extR = Eye3d;
    M3D    _gravity_correct_rotM = M3D::Identity();

    int    NUM_MAX_ITERATIONS = 0;
    int    MIN_IMG_COUNT = 0;
    double HALF_FOV_COS = 0, FOV_DEG = 0;
    bool   USE_NED = true;
    int    m_iterCount = 0, m_feats_down_size = 0, m_laserCloudValidNum = 0, m_effct_feat_num = 0, m_time_log_counter = 0, m_publish_count = 0;

    double m_res_mean_last = 0.05, m_last_lidar_processed_time = -1.0;
    double m_gyr_cov = 0, m_acc_cov = 0;
    double m_last_timestamp_lidar = -1.0, m_last_timestamp_imu = -1.0, m_last_timestamp_img = -1.0;
    double m_filter_size_corner_min = 0, m_filter_size_surf_min = 0, m_filter_size_map_min = 0, m_fov_deg = 0;
    double m_cube_len = 0, m_total_distance = 0, m_lidar_end_time = 0, m_first_lidar_time = 0.0;
    double m_first_img_time = -1.0;
    double m_kdtree_incremental_time = 0, m_kdtree_search_time = 0, m_kdtree_delete_time = 0.0;
    int    m_kdtree_search_counter = 0, m_kdtree_size_st = 0, m_kdtree_size_end = 0, m_add_point_size = 0, m_kdtree_delete_counter = 0;

    double m_copy_time = 0, m_readd_time = 0, m_fov_check_time = 0, m_readd_box_time = 0, m_delete_box_time = 0;
    double m_T1[ MAXN ], m_T2[ MAXN ], m_s_plot[ MAXN ], m_s_plot2[ MAXN ], m_s_plot3[ MAXN ], m_s_plot4[ MAXN ], m_s_plot5[ MAXN ],
        m_s_plot6[ MAXN ], m_s_plot7[ MAXN ];
    double m_keyf_rotd = 0.05, m_keyf_posd = 0.15;
    double m_match_time = 0, m_solve_time = 0, m_solve_const_H_time = 0;

    /*** For voxel map ***/

    double m_max_voxel_size, m_min_eigen_value = 0.003, m_match_s = 0.90, m_sigma_num = 2.0, m_match_eigen_value = 0.0025;
    double m_beam_err = 0.03, m_dept_err = 0.05;
    int    m_pub_map = 0, m_voxel_layer = 1;
    int    m_last_match_num = 0;
    bool   m_init_map = false, m_use_new_map = true, m_is_pub_plane_map = false, m_pcd_save_en = false, m_img_save_en = false,
         m_effect_point_pub = false, m_hilti_en = false;
    int         m_min_points_size = 30, m_pcd_save_type = 0, m_pcd_save_interval = -1, m_img_save_interval = 1, m_pcd_index = 0, m_pub_point_skip = 1;
    std::time_t m_startTime, m_endTime;
    std::unordered_map< VOXEL_LOC, OctoTree * > m_feat_map;
    V3D                                         m_layer_size = V3D( 20, 10, 10 );
    std::vector< M3D >                          m_cross_mat_list;
    std::vector< M3D >                          m_body_cov_list;

    /*********************/
    int                m_max_points_size;
    int                m_max_layer;
    std::vector< int > m_layer_init_size;

    bool   m_lidar_pushed, m_imu_en, m_flg_reset, m_flg_exit = false;
    int    m_dense_map_en = 1;
    int    m_img_en = 1, m_imu_int_frame = 3;
    int    m_lidar_en = 1;
    int    m_GUI_font_size = 14;
    int    m_debug = 0;
    bool   m_is_first_frame = false;
    int    m_grid_size, m_patch_size;
    double m_outlier_threshold;

    vector< BoxPointType > m_cub_need_rm;
    vector< BoxPointType > m_cub_need_add;
    // deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
    deque< PointCloudXYZI::Ptr >        m_lidar_buffer;
    deque< double >                     m_time_buffer;
    deque< sensor_msgs::Imu::ConstPtr > m_imu_buffer;
    deque< cv::Mat >                    m_img_buffer;
    deque< double >                     m_img_time_buffer;
    vector< bool >                      m_point_selected_surf;
    vector< vector< int > >             m_pointSearchInd_surf;
    vector< PointVector >               m_Nearest_Points;
    vector< double >                    m_res_last;
    double                              m_total_residual;
    double                              LASER_POINT_COV, IMG_POINT_COV, cam_fx, cam_fy, cam_cx, cam_cy;
    bool                                m_flg_EKF_inited, m_flg_EKF_converged, m_EKF_stop_flg = 0;

    // surf feature in map
    PointCloudXYZI::Ptr m_featsFromMap = nullptr;
    PointCloudXYZI::Ptr m_cube_points_add = nullptr;
    PointCloudXYZI::Ptr m_map_cur_frame_point = nullptr;
    PointCloudXYZI::Ptr m_sub_map_cur_frame_point = nullptr;

    PointCloudXYZI::Ptr m_feats_undistort = nullptr;
    PointCloudXYZI::Ptr m_feats_down_body = nullptr;
    PointCloudXYZI::Ptr m_feats_down_world = nullptr;
    PointCloudXYZI::Ptr m_normvec = nullptr;
    PointCloudXYZI::Ptr m_laserCloudOri = nullptr;
    PointCloudXYZI::Ptr m_corr_normvect = nullptr;

    ofstream m_fout_pre, m_fout_out, m_fout_dbg, m_fout_pcd_pos, m_fout_img_pos;

    pcl::VoxelGrid< PointType > m_downSizeFilterSurf;
    // pcl::VoxelGrid<PointType> downSizeFilterMap;

    PointCloudXYZI::Ptr      m_pcl_wait_pub = nullptr;
    PointCloudXYZI::Ptr      m_pcl_wait_save = nullptr;
    shared_ptr< Preprocess > m_p_pre = nullptr;

    PointCloudXYZI::Ptr m_pcl_visual_wait_pub = nullptr;
    PointCloudXYZI::Ptr m_sub_pcl_visual_wait_pub = nullptr;

    vector< double > m_extrin_T;
    vector< double > m_extrin_R;
    vector< double > m_camera_extrin_T;
    vector< double > m_camera_extrin_R;

    // KD_TREE m_ikdtree;
    KD_TREE< PointType > m_ikdtree;

    V3F             m_XAxis_Point_body = V3F( LIDAR_SP_LEN, 0.0, 0.0 );
    V3F             m_XAxis_Point_world = V3F( LIDAR_SP_LEN, 0.0, 0.0 );
    V3D             m_euler_cur;
    V3D             m_position_last = Zero3d;
    Eigen::Matrix3d m_Rcl;
    Eigen::Vector3d m_Pcl;

    // estimator inputs and output;
    LidarMeasureGroup m_Lidar_Measures;
    // SparseMap sparse_map;

    StatesGroup state;

    nav_msgs::Path             m_pub_path;
    nav_msgs::Odometry         m_odom_aft_mapped;
    geometry_msgs::Quaternion  m_geo_Quat;
    geometry_msgs::PoseStamped m_msg_body_pose;

    BoxPointType m_LocalMap_Points;
    bool         m_localmap_Initialized = false;
    int          m_points_cache_size = 0;
    StatesGroup  m_last_state;
    double       m_img_lid_time_diff = 0;
    int          m_scanIdx = 0;

    // Configuration for meshing
    double      m_meshing_distance_scale = 1.0;
    double      m_meshing_points_minimum_scale = 0.1;
    double      m_meshing_voxel_resolution = 0.4;
    double      m_meshing_region_size = 10.0;
    int         m_if_enable_mesh_rec = 1;
    int         m_if_draw_mesh = 1;
    int         m_meshing_maximum_thread_for_rec_mesh = 12;
    int         m_meshing_number_of_pts_append_to_map = 5000;
    std::string m_pointcloud_file_name = std::string( " " );
    // PointCloudXYZRGB::Ptr pcl_wait_pub_RGB(new PointCloudXYZRGB(500000, 1));

    Voxel_mapping()
    {
        m_extrin_T = std::vector< double >( 3, 0.0 );
        m_extrin_R = std::vector< double >( 9, 0.0 );
        m_camera_extrin_T = std::vector< double >( 3, 0.0 );
        m_camera_extrin_R = std::vector< double >( 9, 0.0 );

        m_featsFromMap = PointCloudXYZI().makeShared();
        m_cube_points_add = PointCloudXYZI().makeShared();
        m_map_cur_frame_point = PointCloudXYZI().makeShared();
        m_sub_map_cur_frame_point = PointCloudXYZI().makeShared();

        m_feats_undistort = PointCloudXYZI().makeShared();
        m_feats_down_body = PointCloudXYZI().makeShared();
        m_feats_down_world = PointCloudXYZI().makeShared();

        m_normvec = PointCloudXYZI( 100000, 1 ).makeShared();
        m_laserCloudOri = PointCloudXYZI( 100000, 1 ).makeShared();
        m_corr_normvect = PointCloudXYZI( 100000, 1 ).makeShared();

        m_pcl_wait_pub = PointCloudXYZI( 500000, 1 ).makeShared();
        m_pcl_wait_save = PointCloudXYZI().makeShared();
        m_p_pre = std::make_shared< Preprocess >();

        m_pcl_visual_wait_pub = PointCloudXYZI( 500000, 1 ).makeShared();
        m_sub_pcl_visual_wait_pub = PointCloudXYZI( 500000, 1 ).makeShared();
    }

    void kitti_log( FILE *fp );

    void SigHandle( int sig );

    void dump_lio_state_to_log( FILE *fp );

    void pointBodyToWorld( const PointType &pi, PointType &po );

    template < typename T >
    void pointBodyToWorld( const Matrix< T, 3, 1 > &pi, Matrix< T, 3, 1 > &po )
    {
        V3D p_body( pi[ 0 ], pi[ 1 ], pi[ 2 ] );
        V3D p_global( state.rot_end * ( m_extR * p_body + m_extT ) + state.pos_end );
        po[ 0 ] = p_global( 0 );
        po[ 1 ] = p_global( 1 );
        po[ 2 ] = p_global( 2 );
    }

    template < typename T >
    Matrix< T, 3, 1 > pointBodyToWorld( const Matrix< T, 3, 1 > &pi )
    {
        V3D p( pi[ 0 ], pi[ 1 ], pi[ 2 ] );
        p = ( state.rot_end * ( m_extR * p + m_extT ) + state.pos_end );
        Matrix< T, 3, 1 > po( p[ 0 ], p[ 1 ], p[ 2 ] );
        return po;
    }

    void frameBodyToWorld( const PointCloudXYZI::Ptr &pi, PointCloudXYZI::Ptr &po );

    void get_NED_transform();

    void RGBpointBodyToWorld( PointType const *const pi, PointType *const po );

    void RGBpointBodyLidarToIMU( PointType const *const pi, PointType *const po );

    void points_cache_collect();

    void laser_map_fov_segment();

    void standard_pcl_cbk( const sensor_msgs::PointCloud2::ConstPtr &msg );

    void livox_pcl_cbk( const livox_ros_driver::CustomMsg::ConstPtr &msg );

    void imu_cbk( const sensor_msgs::Imu::ConstPtr &msg_in );

    bool sync_packages( LidarMeasureGroup &meas );

    void map_incremental_grow();

    void publish_voxel_point( const ros::Publisher &pubLaserCloudVoxel, const PointCloudXYZI::Ptr &pcl_wait_pub );

    void publish_visual_world_map( const ros::Publisher &pubVisualCloud );

    void publish_visual_world_sub_map( const ros::Publisher &pubSubVisualCloud );

    void publish_effect_world( const ros::Publisher &pubLaserCloudEffect );

    void publish_map( const ros::Publisher &pubLaserCloudMap );

    template < typename T >
    void set_pose_timestamp( T &out )
    {
#ifdef USE_IKFOM
        // state_ikfom stamp_state = kf.get_x();
        out.position.x = state_point.pos( 0 );
        out.position.y = state_point.pos( 1 );
        out.position.z = state_point.pos( 2 );
#else
        out.position.x = state.pos_end( 0 );
        out.position.y = state.pos_end( 1 );
        out.position.z = state.pos_end( 2 );
#endif
        out.orientation.x = m_geo_Quat.x;
        out.orientation.y = m_geo_Quat.y;
        out.orientation.z = m_geo_Quat.z;
        out.orientation.w = m_geo_Quat.w;
    }

    void publish_odometry( const ros::Publisher &pubOdomAftMapped );

    void publish_mavros( const ros::Publisher &mavros_pose_publisher );

    void publish_frame_world( const ros::Publisher &pubLaserCloudFullRes, const int point_skip );

    void publish_path( const ros::Publisher pubPath );

    void read_ros_parameters( ros::NodeHandle &nh );

    void transformLidar( const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                         pcl::PointCloud< pcl::PointXYZI >::Ptr &trans_cloud );

    bool voxel_map_init();

    /*** EKF update ***/
    void lio_state_estimation( StatesGroup &state_propagat );
    void init_ros_node();
    int  service_LiDAR_update();
};
