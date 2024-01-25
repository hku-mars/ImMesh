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
#include "voxel_mapping.hpp"
double g_LiDAR_frame_start_time = 0;

V3D Lidar_T_wrt_IMU = V3D::Zero();
M3D Lidar_R_wrt_IMU = M3D::Identity();

const bool intensity_contrast( PointType &x, PointType &y ) { return ( x.intensity > y.intensity ); };

const bool var_contrast( Point_with_var &x, Point_with_var &y ) { return ( x.m_var.diagonal().norm() < y.m_var.diagonal().norm() ); };

float calc_dist( PointType p1, PointType p2 )
{
    float d = ( p1.x - p2.x ) * ( p1.x - p2.x ) + ( p1.y - p2.y ) * ( p1.y - p2.y ) + ( p1.z - p2.z ) * ( p1.z - p2.z );
    return d;
}

void mapJet( double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b )
{
    r = 255;
    g = 255;
    b = 255;

    if ( v < vmin )
    {
        v = vmin;
    }

    if ( v > vmax )
    {
        v = vmax;
    }

    double dr, dg, db;

    if ( v < 0.1242 )
    {
        db = 0.504 + ( ( 1. - 0.504 ) / 0.1242 ) * v;
        dg = dr = 0.;
    }
    else if ( v < 0.3747 )
    {
        db = 1.;
        dr = 0.;
        dg = ( v - 0.1242 ) * ( 1. / ( 0.3747 - 0.1242 ) );
    }
    else if ( v < 0.6253 )
    {
        db = ( 0.6253 - v ) * ( 1. / ( 0.6253 - 0.3747 ) );
        dg = 1.;
        dr = ( v - 0.3747 ) * ( 1. / ( 0.6253 - 0.3747 ) );
    }
    else if ( v < 0.8758 )
    {
        db = 0.;
        dr = 1.;
        dg = ( 0.8758 - v ) * ( 1. / ( 0.8758 - 0.6253 ) );
    }
    else
    {
        db = 0.;
        dg = 0.;
        dr = 1. - ( v - 0.8758 ) * ( ( 1. - 0.504 ) / ( 1. - 0.8758 ) );
    }

    r = ( uint8_t )( 255 * dr );
    g = ( uint8_t )( 255 * dg );
    b = ( uint8_t )( 255 * db );
}

void buildVoxelMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const int max_layer,
                    const std::vector< int > &layer_init_num, const int max_points_size, const float planer_threshold,
                    std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map )
{
    uint plsize = input_points.size();
    for ( uint i = 0; i < plsize; i++ )
    {
        const Point_with_var p_v = input_points[ i ];
        float                loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = p_v.m_point[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }
        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = feat_map.find( position );
        if ( iter != feat_map.end() )
        {
            feat_map[ position ]->m_temp_points_.push_back( p_v );
            feat_map[ position ]->m_new_points_++;
        }
        else
        {
            OctoTree *octo_tree = new OctoTree( max_layer, 0, layer_init_num, max_points_size, planer_threshold );
            feat_map[ position ] = octo_tree;
            feat_map[ position ]->m_quater_length_ = voxel_size / 4;
            feat_map[ position ]->m_voxel_center_[ 0 ] = ( 0.5 + position.x ) * voxel_size;
            feat_map[ position ]->m_voxel_center_[ 1 ] = ( 0.5 + position.y ) * voxel_size;
            feat_map[ position ]->m_voxel_center_[ 2 ] = ( 0.5 + position.z ) * voxel_size;
            feat_map[ position ]->m_temp_points_.push_back( p_v );
            feat_map[ position ]->m_new_points_++;
            feat_map[ position ]->m_layer_init_num_ = layer_init_num;
        }
    }
    for ( auto iter = feat_map.begin(); iter != feat_map.end(); ++iter )
    {
        iter->second->init_octo_tree();
    }
}

void BuildResidualListOMP( const unordered_map< VOXEL_LOC, OctoTree * > &voxel_map, const double voxel_size, const double sigma_num,
                           const int max_layer, const std::vector< Point_with_var > &pv_list, std::vector< ptpl > &ptpl_list,
                           std::vector< Eigen::Vector3d > &non_match )
{
    std::mutex mylock;
    ptpl_list.clear();
    std::vector< ptpl >   all_ptpl_list( pv_list.size() );
    std::vector< bool >   useful_ptpl( pv_list.size() );
    std::vector< size_t > index( pv_list.size() );
    for ( size_t i = 0; i < index.size(); ++i )
    {
        index[ i ] = i;
        useful_ptpl[ i ] = false;
    }
    omp_set_num_threads( MP_PROC_NUM );
#pragma omp parallel for
    for ( int i = 0; i < index.size(); i++ )
    {
        Point_with_var pv = pv_list[ i ];
        float          loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = pv.m_point_world[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }
        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = voxel_map.find( position );
        if ( iter != voxel_map.end() )
        {
            OctoTree *current_octo = iter->second;
            ptpl      single_ptpl;
            bool      is_sucess = false;
            double    prob = 0;
            build_single_residual( pv, current_octo, 0, max_layer, sigma_num, is_sucess, prob, single_ptpl );
            if ( !is_sucess )
            {
                VOXEL_LOC near_position = position;
                if ( loc_xyz[ 0 ] > ( current_octo->m_voxel_center_[ 0 ] + current_octo->m_quater_length_ ) )
                {
                    near_position.x = near_position.x + 1;
                }
                else if ( loc_xyz[ 0 ] < ( current_octo->m_voxel_center_[ 0 ] - current_octo->m_quater_length_ ) )
                {
                    near_position.x = near_position.x - 1;
                }
                if ( loc_xyz[ 1 ] > ( current_octo->m_voxel_center_[ 1 ] + current_octo->m_quater_length_ ) )
                {
                    near_position.y = near_position.y + 1;
                }
                else if ( loc_xyz[ 1 ] < ( current_octo->m_voxel_center_[ 1 ] - current_octo->m_quater_length_ ) )
                {
                    near_position.y = near_position.y - 1;
                }
                if ( loc_xyz[ 2 ] > ( current_octo->m_voxel_center_[ 2 ] + current_octo->m_quater_length_ ) )
                {
                    near_position.z = near_position.z + 1;
                }
                else if ( loc_xyz[ 2 ] < ( current_octo->m_voxel_center_[ 2 ] - current_octo->m_quater_length_ ) )
                {
                    near_position.z = near_position.z - 1;
                }
                auto iter_near = voxel_map.find( near_position );
                if ( iter_near != voxel_map.end() )
                {
                    build_single_residual( pv, iter_near->second, 0, max_layer, sigma_num, is_sucess, prob, single_ptpl );
                }
            }
            if ( is_sucess )
            {
                mylock.lock();
                useful_ptpl[ i ] = true;
                all_ptpl_list[ i ] = single_ptpl;
                mylock.unlock();
            }
            else
            {
                mylock.lock();
                useful_ptpl[ i ] = false;
                mylock.unlock();
            }
        }
    }
    for ( size_t i = 0; i < useful_ptpl.size(); i++ )
    {
        if ( useful_ptpl[ i ] )
        {
            ptpl_list.push_back( all_ptpl_list[ i ] );
        }
    }
}

void build_single_residual( const Point_with_var &pv, const OctoTree *current_octo, const int current_layer, const int max_layer,
                            const double sigma_num, bool &is_sucess, double &prob, ptpl &single_ptpl )
{
    double          radius_k = 3;
    Eigen::Vector3d p_w = pv.m_point_world;
    if ( current_octo->m_plane_ptr_->m_is_plane )
    {
        Plane &         plane = *current_octo->m_plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.m_center;
        float dis_to_plane = fabs( plane.m_normal( 0 ) * p_w( 0 ) + plane.m_normal( 1 ) * p_w( 1 ) + plane.m_normal( 2 ) * p_w( 2 ) + plane.m_d );
        float dis_to_center = ( plane.m_center( 0 ) - p_w( 0 ) ) * ( plane.m_center( 0 ) - p_w( 0 ) ) +
                              ( plane.m_center( 1 ) - p_w( 1 ) ) * ( plane.m_center( 1 ) - p_w( 1 ) ) +
                              ( plane.m_center( 2 ) - p_w( 2 ) ) * ( plane.m_center( 2 ) - p_w( 2 ) );
        float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );

        if ( range_dis <= radius_k * plane.m_radius )
        {
            Eigen::Matrix< double, 1, 6 > J_nq;
            J_nq.block< 1, 3 >( 0, 0 ) = p_w - plane.m_center;
            J_nq.block< 1, 3 >( 0, 3 ) = -plane.m_normal;
            double sigma_l = J_nq * plane.m_plane_var * J_nq.transpose();
            sigma_l += plane.m_normal.transpose() * pv.m_var * plane.m_normal;
            if ( dis_to_plane < sigma_num * sqrt( sigma_l ) )
            {
                is_sucess = true;
                double this_prob = 1.0 / ( sqrt( sigma_l ) ) * exp( -0.5 * dis_to_plane * dis_to_plane / sigma_l );
                if ( this_prob > prob )
                {
                    prob = this_prob;
                    single_ptpl.point = pv.m_point;
                    single_ptpl.plane_var = plane.m_plane_var;
                    single_ptpl.normal = plane.m_normal;
                    single_ptpl.center = plane.m_center;
                    single_ptpl.d = plane.m_d;
                    single_ptpl.layer = current_layer;
                }
                return;
            }
            else
            {
                // is_sucess = false;
                return;
            }
        }
        else
        {
            // is_sucess = false;
            return;
        }
    }
    else
    {
        if ( current_layer < max_layer )
        {
            for ( size_t leafnum = 0; leafnum < 8; leafnum++ )
            {
                if ( current_octo->m_leaves_[ leafnum ] != nullptr )
                {

                    OctoTree *leaf_octo = current_octo->m_leaves_[ leafnum ];
                    build_single_residual( pv, leaf_octo, current_layer + 1, max_layer, sigma_num, is_sucess, prob, single_ptpl );
                }
            }
            return;
        }
        else
        {
            // is_sucess = false;
            return;
        }
    }
}

void updateVoxelMap( const std::vector< Point_with_var > &input_points, const float voxel_size, const int max_layer,
                     const std::vector< int > &layer_init_num, const int max_points_size, const float planer_threshold,
                     std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map )
{
    uint plsize = input_points.size();
    for ( uint i = 0; i < plsize; i++ )
    {
        const Point_with_var p_v = input_points[ i ];
        float                loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = p_v.m_point[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }
        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = feat_map.find( position );
        if ( iter != feat_map.end() )
        {
            feat_map[ position ]->UpdateOctoTree( p_v );
        }
        else
        {
            OctoTree *octo_tree = new OctoTree( max_layer, 0, layer_init_num, max_points_size, planer_threshold );
            feat_map[ position ] = octo_tree;
            feat_map[ position ]->m_quater_length_ = voxel_size / 4;
            feat_map[ position ]->m_voxel_center_[ 0 ] = ( 0.5 + position.x ) * voxel_size;
            feat_map[ position ]->m_voxel_center_[ 1 ] = ( 0.5 + position.y ) * voxel_size;
            feat_map[ position ]->m_voxel_center_[ 2 ] = ( 0.5 + position.z ) * voxel_size;
            feat_map[ position ]->UpdateOctoTree( p_v );
        }
    }
}

V3F RGBFromVoxel( const V3D &input_point, const float voxel_size, const Eigen::Vector3d &layer_size, const float planer_threshold,
                  std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map )
{
    int64_t loc_xyz[ 3 ];
    for ( int j = 0; j < 3; j++ )
    {
        loc_xyz[ j ] = floor( input_point[ j ] / voxel_size );
        // if (loc_xyz[j] < 0)
        // {
        //   loc_xyz[j] -= 1;
        // }
    }

    VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
    int64_t   ind = loc_xyz[ 0 ] + loc_xyz[ 1 ] + loc_xyz[ 2 ];
    uint      k( ( ind + 100000 ) % 3 );
    V3F       RGB( ( k == 0 ) * 255.0, ( k == 1 ) * 255.0, ( k == 2 ) * 255.0 );
    // cout<<"RGB: "<<RGB.transpose()<<endl;
    return RGB;
}

void transformLidar( const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                     pcl::PointCloud< pcl::PointXYZI >::Ptr &trans_cloud );

void BuildPtplList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const float match_eigen_value, const int layer, const float voxel_size,
                    const float match_constraint, const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr input_cloud,
                    std::vector< ptpl > &ptpl_list )
{
    // debug time
    double t_sum = 0;
    int    match_plane_size = 0;
    int    match_sub_plane_size = 0;
    for ( size_t i = 0; i < input_cloud->points.size(); i++ )
    {
        pcl::PointXYZI p_c;
        p_c.x = input_cloud->points[ i ].x;
        p_c.y = input_cloud->points[ i ].y;
        p_c.z = input_cloud->points[ i ].z;
        p_c.intensity = input_cloud->points[ i ].intensity;

        pcl::PointXYZI  p_w;
        Eigen::Vector3d p_cv( p_c.x, p_c.y, p_c.z );
        Eigen::Vector3d p_wv( p_c.x, p_c.y, p_c.z );
        p_wv = rot * ( p_cv ) + t;
        p_w.x = p_wv( 0 );
        p_w.y = p_wv( 1 );
        p_w.z = p_wv( 2 );
        p_w.intensity = p_c.intensity;
        float loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = p_w.data[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }
        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = feat_map.find( position );
        bool      match_flag = false;
        float     radius_k1 = 1.25;
        float     radius_k2 = 1.25;
        if ( iter != feat_map.end() )
        {
            if ( iter->second->m_octo_state_ == 0 && iter->second->m_plane_ptr_->m_is_plane &&
                 iter->second->m_plane_ptr_->m_min_eigen_value < match_eigen_value )
            {
                // use new dis_threshold;
                Plane *plane = iter->second->m_plane_ptr_;
                float  dis_to_plane = ( plane->m_normal( 0 ) * p_w.x + plane->m_normal( 1 ) * p_w.y + plane->m_normal( 2 ) * p_w.z + plane->m_d );
                float  dis_to_center = ( plane->m_center( 0 ) - p_w.x ) * ( plane->m_center( 0 ) - p_w.x ) +
                                      ( plane->m_center( 1 ) - p_w.y ) * ( plane->m_center( 1 ) - p_w.y ) +
                                      ( plane->m_center( 2 ) - p_w.z ) * ( plane->m_center( 2 ) - p_w.z );
                float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                float s = 1 - 0.9 * fabs( dis_to_plane ) / sqrt( sqrt( p_c.x * p_c.x + p_c.y * p_c.y + p_c.z * p_c.z ) );
                float dis_threshold = 3 * sqrt( iter->second->m_plane_ptr_->m_min_eigen_value );

                // if (fabs(dis_to_plane) < dis_to_plane_threshold) {
                if ( s > match_constraint && range_dis < radius_k1 * iter->second->m_plane_ptr_->m_radius )
                {
                    // std::cout << "range dis: " << range_dis
                    //           << " plane radius: " << iter->second->plane_ptr_->radius
                    //           << std::endl;
                    // if (fabs(dis_to_plane) < dis_threshold) {
                    ptpl single_ptpl;
                    single_ptpl.point << p_c.x, p_c.y, p_c.z;
                    single_ptpl.normal << plane->m_normal( 0 ), plane->m_normal( 1 ), plane->m_normal( 2 );
                    single_ptpl.center = plane->m_center;
                    single_ptpl.plane_var = plane->m_plane_var;
                    single_ptpl.d = plane->m_d;
                    single_ptpl.eigen_value = plane->m_min_eigen_value;
                    ptpl_list.push_back( single_ptpl );
                    match_plane_size++;
                    //}
                    // std::cout << "3_sigma:" << dis_threshold
                    //           << " distance:" << dis_to_plane << std::endl;
                }
            }
            else
            {
                if ( layer >= 1 )
                {
                    float min_dis = 100;
                    ptpl  single_ptpl;
                    for ( int j = 0; j < 8; j++ )
                    {
                        if ( iter->second->m_leaves_[ j ] != nullptr )
                        {
                            if ( iter->second->m_leaves_[ j ]->m_plane_ptr_->m_is_plane &&
                                 iter->second->m_leaves_[ j ]->m_plane_ptr_->m_min_eigen_value < match_eigen_value )
                            {
                                Plane *plane = iter->second->m_leaves_[ j ]->m_plane_ptr_;
                                float  dis_to_plane =
                                    ( plane->m_normal( 0 ) * p_w.x + plane->m_normal( 1 ) * p_w.y + plane->m_normal( 2 ) * p_w.z + plane->m_d );
                                float dis_to_center = ( plane->m_center( 0 ) - p_w.x ) * ( plane->m_center( 0 ) - p_w.x ) +
                                                      ( plane->m_center( 1 ) - p_w.y ) * ( plane->m_center( 1 ) - p_w.y ) +
                                                      ( plane->m_center( 2 ) - p_w.z ) * ( plane->m_center( 2 ) - p_w.z );
                                float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                                float dis_threshold = 3 * sqrt( iter->second->m_leaves_[ j ]->m_plane_ptr_->m_min_eigen_value );
                                float s = 1 - 0.9 * fabs( dis_to_plane ) / sqrt( sqrt( p_c.x * p_c.x + p_c.y * p_c.y + p_c.z * p_c.z ) );
                                if ( s > match_constraint && range_dis < radius_k2 * iter->second->m_leaves_[ j ]->m_plane_ptr_->m_radius )
                                {
                                    // std::cout << "3_sigma:" << dis_threshold
                                    //           << " distance:" << dis_to_plane << std::endl;
                                    // if (min_dis > dis_to_plane) {
                                    // if (fabs(dis_to_plane > dis_threshold)) {
                                    min_dis = dis_to_plane;
                                    single_ptpl.point << p_c.x, p_c.y, p_c.z;
                                    single_ptpl.normal << plane->m_normal( 0 ), plane->m_normal( 1 ), plane->m_normal( 2 );
                                    single_ptpl.plane_var = plane->m_plane_var;
                                    single_ptpl.center = plane->m_center;
                                    single_ptpl.d = plane->m_d;
                                    single_ptpl.eigen_value = plane->m_min_eigen_value;
                                    // ptpl_list.push_back(single_ptpl);
                                    // match_sub_plane_size++;
                                    //}
                                    //}
                                }
                            }
                        }
                    }
                    if ( min_dis != 100 )
                    {
                        ptpl_list.push_back( single_ptpl );
                        match_sub_plane_size++;
                    }
                }
            }
        }
    }
    // std::cout << "[ Matching ]: match plane size: " << match_plane_size << " , match sub plane size: " << match_sub_plane_size << std::endl;
}

void BuildResidualList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const double voxel_size, const double sigma_num,
                        const Point_with_var &pv, const M3D &rot, const V3D &t, const M3D &ext_R, const V3D &ext_T, ptpl &ptpl_list )
{
    float radius_k = 3;
    int   match_index = -1;
    int   match_plane = 0;
    int   match_sub_plane = 0;
    int   match_sub_sub_plane = 0;
    int   check_plane = 0;
    int   check_sub_plane = 0;
    int   check_sub_sub_plane = 0;
    // ptpl_list.clear();
    ptpl_list.is_valid = false;

    V3D p_w = rot * ( ext_R * pv.m_point + ext_T ) + t;

    if ( pv.m_point.norm() < 0 )
    {
        return;
    }

    float loc_xyz[ 3 ];
    for ( int j = 0; j < 3; j++ )
    {
        loc_xyz[ j ] = p_w[ j ] / voxel_size;
        if ( loc_xyz[ j ] < 0 )
        {
            loc_xyz[ j ] -= 1.0;
        }
    }
    VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
    auto      iter = feat_map.find( position );
    double    current_layer = 0;

    if ( iter != feat_map.end() )
    {
        OctoTree *current_octo = iter->second;
        ptpl      single_ptpl;
        double    max_prob = 0;
        if ( current_octo->m_plane_ptr_->m_is_plane )
        {
            Plane &plane = *current_octo->m_plane_ptr_;
            float dis_to_plane = fabs( plane.m_normal( 0 ) * p_w( 0 ) + plane.m_normal( 1 ) * p_w( 1 ) + plane.m_normal( 2 ) * p_w( 2 ) + plane.m_d );
            float dis_to_center = ( plane.m_center( 0 ) - p_w( 0 ) ) * ( plane.m_center( 0 ) - p_w( 0 ) ) +
                                  ( plane.m_center( 1 ) - p_w( 1 ) ) * ( plane.m_center( 1 ) - p_w( 1 ) ) +
                                  ( plane.m_center( 2 ) - p_w( 2 ) ) * ( plane.m_center( 2 ) - p_w( 2 ) );
            float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
            if ( range_dis <= radius_k * plane.m_radius )
            {
                Eigen::Matrix< double, 1, 6 > J_nq;
                J_nq.block< 1, 3 >( 0, 0 ) = p_w - plane.m_center;
                J_nq.block< 1, 3 >( 0, 3 ) = -plane.m_normal;
                double sigma_l = J_nq * plane.m_plane_var * J_nq.transpose();
                sigma_l += plane.m_normal.transpose() * pv.m_var * plane.m_normal;
                if ( dis_to_plane < sigma_num * sqrt( sigma_l ) )
                {
                    check_plane++;
                    float prob = 1 / ( sqrt( 2 * M_PI ) * sqrt( sigma_l ) ) * exp( -0.5 * dis_to_plane * dis_to_plane / sigma_l );
                    if ( prob > max_prob )
                    {
                        max_prob = prob;
                        single_ptpl.point = pv.m_point;
                        single_ptpl.plane_var = plane.m_plane_var;
                        single_ptpl.normal = plane.m_normal;
                        single_ptpl.center = plane.m_center;
                        single_ptpl.d = plane.m_d;
                        single_ptpl.eigen_value = plane.m_min_eigen_value;
                        match_index = 0;
                    }
                }
            }
        }
        else
        {
            for ( size_t leaf_i = 0; leaf_i < 8; leaf_i++ )
            {
                // continue;
                OctoTree *leaf_octo = current_octo->m_leaves_[ leaf_i ];
                if ( leaf_octo != nullptr )
                {
                    if ( leaf_octo->m_plane_ptr_->m_is_plane )
                    {
                        Plane *plane = leaf_octo->m_plane_ptr_;
                        float  dis_to_plane =
                            fabs( plane->m_normal( 0 ) * p_w( 0 ) + plane->m_normal( 1 ) * p_w( 1 ) + plane->m_normal( 2 ) * p_w( 2 ) + plane->m_d );
                        float dis_to_center = ( plane->m_center( 0 ) - p_w( 0 ) ) * ( plane->m_center( 0 ) - p_w( 0 ) ) +
                                              ( plane->m_center( 1 ) - p_w( 1 ) ) * ( plane->m_center( 1 ) - p_w( 1 ) ) +
                                              ( plane->m_center( 2 ) - p_w( 2 ) ) * ( plane->m_center( 2 ) - p_w( 2 ) );
                        float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                        if ( range_dis <= radius_k * plane->m_radius )
                        {
                            Eigen::Matrix< double, 1, 6 > J_nq;
                            J_nq.block< 1, 3 >( 0, 0 ) = p_w - plane->m_center;
                            J_nq.block< 1, 3 >( 0, 3 ) = -plane->m_normal;
                            double sigma_l = J_nq * plane->m_plane_var * J_nq.transpose();
                            sigma_l += plane->m_normal.transpose() * pv.m_var * plane->m_normal;
                            if ( dis_to_plane < sigma_num * sqrt( sigma_l ) )
                            {
                                check_sub_plane++;
                                float prob = 1 / ( sqrt( 2 * M_PI ) * sqrt( sigma_l ) ) * exp( -0.5 * dis_to_plane * dis_to_plane / sigma_l );
                                if ( prob > max_prob )
                                {
                                    max_prob = prob;
                                    single_ptpl.point = pv.m_point;
                                    single_ptpl.plane_var = plane->m_plane_var;
                                    single_ptpl.normal = plane->m_normal;
                                    single_ptpl.center = plane->m_center;
                                    single_ptpl.d = plane->m_d;
                                    single_ptpl.eigen_value = plane->m_min_eigen_value;
                                    match_index = 1;
                                }
                            }
                        }
                    }
                    else
                    {
                        for ( size_t leaf_j = 0; leaf_j < 8; leaf_j++ )
                        {
                            OctoTree *leaf_leaf_octo = leaf_octo->m_leaves_[ leaf_j ];
                            if ( leaf_leaf_octo != nullptr )
                            {
                                if ( leaf_leaf_octo->m_plane_ptr_->m_is_plane )
                                {
                                    Plane *plane = leaf_leaf_octo->m_plane_ptr_;
                                    float  dis_to_plane = fabs( plane->m_normal( 0 ) * p_w( 0 ) + plane->m_normal( 1 ) * p_w( 1 ) +
                                                               plane->m_normal( 2 ) * p_w( 2 ) + plane->m_d );
                                    float  dis_to_center = ( plane->m_center( 0 ) - p_w( 0 ) ) * ( plane->m_center( 0 ) - p_w( 0 ) ) +
                                                          ( plane->m_center( 1 ) - p_w( 1 ) ) * ( plane->m_center( 1 ) - p_w( 1 ) ) +
                                                          ( plane->m_center( 2 ) - p_w( 2 ) ) * ( plane->m_center( 2 ) - p_w( 2 ) );
                                    float range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                                    if ( range_dis <= radius_k * plane->m_radius )
                                    {
                                        Eigen::Matrix< double, 1, 6 > J_nq;
                                        J_nq.block< 1, 3 >( 0, 0 ) = p_w - plane->m_center;
                                        J_nq.block< 1, 3 >( 0, 3 ) = -plane->m_normal;
                                        double sigma_l = J_nq * plane->m_plane_var * J_nq.transpose();

                                        sigma_l += plane->m_normal.transpose() * pv.m_var * plane->m_normal;
                                        if ( dis_to_plane < sigma_num * sqrt( sigma_l ) )
                                        {
                                            check_sub_sub_plane++;
                                            float prob =
                                                1 / ( sqrt( 2 * M_PI ) * sqrt( sigma_l ) ) * exp( -0.5 * dis_to_plane * dis_to_plane / sigma_l );
                                            if ( prob > max_prob )
                                            {
                                                max_prob = prob;
                                                single_ptpl.point = pv.m_point;
                                                single_ptpl.plane_var = plane->m_plane_var;
                                                single_ptpl.normal = plane->m_normal;
                                                single_ptpl.center = plane->m_center;
                                                single_ptpl.d = plane->m_d;
                                                single_ptpl.eigen_value = plane->m_min_eigen_value;
                                                match_index = 2;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        if ( max_prob > 0 )
        {
            ptpl_list = ( single_ptpl );
            ptpl_list.is_valid = true;
            if ( match_index == 0 )
            {
                match_plane++;
            }
            else if ( match_index == 1 )
            {
                match_sub_plane++;
            }
            else if ( match_index == 2 )
            {
                match_sub_sub_plane++;
            }
        }
        else
        {
            ptpl_list.is_valid = false;
        }
    }
}

void BuildOptiList( const unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const float voxel_size, const Eigen::Matrix3d rot,
                    const Eigen::Vector3d t, const PointCloudXYZI::Ptr input_cloud, const float sigma_num,
                    const std::vector< Eigen::Matrix3d > var_list, std::vector< ptpl > &ptpl_list )
{
    int match_plane = 0;
    int match_sub_plane = 0;
    int match_sub_sub_plane = 0;
    ptpl_list.clear();
    // old 1.25
    float                                  radius_k1 = 1.5;
    float                                  radius_k2 = 1.5;
    float                                  num_sigma = sigma_num * sigma_num;
    pcl::PointCloud< pcl::PointXYZI >::Ptr world_cloud( new pcl::PointCloud< pcl::PointXYZI > );
    for ( size_t i = 0; i < input_cloud->points.size(); i++ )
    {
        Eigen::Vector3d p_c( input_cloud->points[ i ].x, input_cloud->points[ i ].y, input_cloud->points[ i ].z );
        Eigen::Vector3d p_w = rot * ( p_c + Lidar_T_wrt_IMU ) + t;
        float           loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = p_w[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }
        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = feat_map.find( position );
        if ( iter != feat_map.end() )
        {
            if ( iter->second->m_octo_state_ == 0 && iter->second->m_plane_ptr_->m_is_plane )
            {
                Plane *plane = iter->second->m_plane_ptr_;
                float  dis_to_plane =
                    fabs( plane->m_normal( 0 ) * p_w( 0 ) + plane->m_normal( 1 ) * p_w( 1 ) + plane->m_normal( 2 ) * p_w( 2 ) + plane->m_d );
                float dis_to_center = ( plane->m_center( 0 ) - p_w( 0 ) ) * ( plane->m_center( 0 ) - p_w( 0 ) ) +
                                      ( plane->m_center( 1 ) - p_w( 1 ) ) * ( plane->m_center( 1 ) - p_w( 1 ) ) +
                                      ( plane->m_center( 2 ) - p_w( 2 ) ) * ( plane->m_center( 2 ) - p_w( 2 ) );
                float           range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                Eigen::Vector3d pq = ( p_w - plane->m_center );
                pq = pq * dis_to_plane / pq.norm();
                if ( pq[ 0 ] * pq[ 0 ] < num_sigma * var_list[ i ]( 0, 0 ) && pq[ 1 ] * pq[ 1 ] < num_sigma * var_list[ i ]( 1, 1 ) &&
                     pq[ 2 ] * pq[ 2 ] < num_sigma * var_list[ i ]( 2, 2 ) && range_dis < radius_k1 * plane->m_radius )
                {
                    ptpl single_ptpl;
                    single_ptpl.point = p_c;
                    single_ptpl.normal = plane->m_normal;
                    single_ptpl.d = plane->m_d;
                    single_ptpl.eigen_value = plane->m_min_eigen_value;
                    single_ptpl.center = plane->m_center;
                    single_ptpl.plane_var = plane->m_plane_var;
                    ptpl_list.push_back( single_ptpl );
                    match_plane++;
                }
            }
            else
            {
                for ( int j = 0; j < 8; j++ )
                {
                    if ( iter->second->m_leaves_[ j ] != nullptr )
                    {
                        if ( iter->second->m_leaves_[ j ]->m_plane_ptr_->m_is_plane )
                        {
                            Plane *plane = iter->second->m_leaves_[ j ]->m_plane_ptr_;
                            float  dis_to_plane = fabs( plane->m_normal( 0 ) * p_w( 0 ) + plane->m_normal( 1 ) * p_w( 1 ) +
                                                       plane->m_normal( 2 ) * p_w( 2 ) + plane->m_d );
                            float  dis_to_center = ( plane->m_center( 0 ) - p_w( 0 ) ) * ( plane->m_center( 0 ) - p_w( 0 ) ) +
                                                  ( plane->m_center( 1 ) - p_w( 1 ) ) * ( plane->m_center( 1 ) - p_w( 1 ) ) +
                                                  ( plane->m_center( 2 ) - p_w( 2 ) ) * ( plane->m_center( 2 ) - p_w( 2 ) );
                            float           range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                            Eigen::Vector3d pq = ( p_w - plane->m_center );
                            // pq = pq * (dis_to_plane - sqrt(plane->min_eigen_value) / 2) /
                            //      pq.norm();
                            pq = pq * dis_to_plane / pq.norm();
                            if ( pq[ 0 ] * pq[ 0 ] < num_sigma * var_list[ i ]( 0, 0 ) && pq[ 1 ] * pq[ 1 ] < num_sigma * var_list[ i ]( 1, 1 ) &&
                                 pq[ 2 ] * pq[ 2 ] < num_sigma * var_list[ i ]( 2, 2 ) && range_dis < radius_k2 * plane->m_radius )
                            {
                                ptpl single_ptpl;
                                single_ptpl.point = p_c;
                                single_ptpl.normal = plane->m_normal;
                                single_ptpl.center = plane->m_center;
                                single_ptpl.d = plane->m_d;
                                single_ptpl.eigen_value = plane->m_min_eigen_value;
                                single_ptpl.plane_var = plane->m_plane_var;
                                ptpl_list.push_back( single_ptpl );
                                match_sub_plane++;
                                break;
                            }
                        }
                        else
                        {
                            for ( int k = 0; k < 8; k++ )
                            {
                                if ( iter->second->m_leaves_[ j ]->m_leaves_[ k ] != nullptr &&
                                     iter->second->m_leaves_[ j ]->m_leaves_[ k ]->m_plane_ptr_->m_is_plane )
                                {
                                    Plane *plane = iter->second->m_leaves_[ j ]->m_leaves_[ k ]->m_plane_ptr_;
                                    float  dis_to_plane = fabs( plane->m_normal( 0 ) * p_w( 0 ) + plane->m_normal( 1 ) * p_w( 1 ) +
                                                               plane->m_normal( 2 ) * p_w( 2 ) + plane->m_d );
                                    float  dis_to_center = ( plane->m_center( 0 ) - p_w( 0 ) ) * ( plane->m_center( 0 ) - p_w( 0 ) ) +
                                                          ( plane->m_center( 1 ) - p_w( 1 ) ) * ( plane->m_center( 1 ) - p_w( 1 ) ) +
                                                          ( plane->m_center( 2 ) - p_w( 2 ) ) * ( plane->m_center( 2 ) - p_w( 2 ) );
                                    float           range_dis = sqrt( dis_to_center - dis_to_plane * dis_to_plane );
                                    Eigen::Vector3d pq = ( p_w - plane->m_center );
                                    // pq = pq * (dis_to_plane - sqrt(plane->min_eigen_value) / 2)
                                    // /
                                    //      pq.norm();
                                    pq = pq * dis_to_plane / pq.norm();
                                    if ( pq[ 0 ] * pq[ 0 ] < num_sigma * var_list[ i ]( 0, 0 ) &&
                                         pq[ 1 ] * pq[ 1 ] < num_sigma * var_list[ i ]( 1, 1 ) &&
                                         pq[ 2 ] * pq[ 2 ] < num_sigma * var_list[ i ]( 2, 2 ) && range_dis < radius_k2 * plane->m_radius )
                                    {
                                        ptpl single_ptpl;
                                        single_ptpl.point = p_c;
                                        single_ptpl.normal = plane->m_normal;
                                        single_ptpl.center = plane->m_center;
                                        single_ptpl.d = plane->m_d;
                                        single_ptpl.eigen_value = plane->m_min_eigen_value;
                                        single_ptpl.plane_var = plane->m_plane_var;
                                        ptpl_list.push_back( single_ptpl );
                                        match_sub_sub_plane++;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // std::cout << "[ Matching ]: match plane size: " << match_plane << " , match sub plane size: " << match_sub_plane
    //           << " ,match sub sub plane size:" << match_sub_sub_plane << std::endl;
}

void CalcQuation( const Eigen::Vector3d &vec, const int axis, geometry_msgs::Quaternion &q )
{
    Eigen::Vector3d x_body = vec;
    Eigen::Vector3d y_body( 1, 1, 0 );
    if ( x_body( 2 ) != 0 )
    {
        y_body( 2 ) = -( y_body( 0 ) * x_body( 0 ) + y_body( 1 ) * x_body( 1 ) ) / x_body( 2 );
    }
    else
    {
        if ( x_body( 1 ) != 0 )
        {
            y_body( 1 ) = -( y_body( 0 ) * x_body( 0 ) ) / x_body( 1 );
        }
        else
        {
            y_body( 0 ) = 0;
        }
    }
    y_body.normalize();
    Eigen::Vector3d z_body = x_body.cross( y_body );
    Eigen::Matrix3d rot;

    rot << x_body( 0 ), x_body( 1 ), x_body( 2 ), y_body( 0 ), y_body( 1 ), y_body( 2 ), z_body( 0 ), z_body( 1 ), z_body( 2 );
    Eigen::Matrix3d rotation = rot.transpose();
    if ( axis == 2 )
    {
        Eigen::Matrix3d rot_inc;
        rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
        rotation = rotation * rot_inc;
    }
    Eigen::Quaterniond eq( rotation );
    q.w = eq.w();
    q.x = eq.x();
    q.y = eq.y();
    q.z = eq.z();
}

void NormalToQuaternion( const Eigen::Vector3d &normal_vec, geometry_msgs::Quaternion &q )
{
    float CosY = normal_vec( 2 ) / sqrt( normal_vec( 0 ) * normal_vec( 0 ) + normal_vec( 1 ) * normal_vec( 1 ) );
    float CosYDiv2 = sqrt( ( CosY + 1 ) / 2 );
    if ( normal_vec( 0 ) < 0 )
        CosYDiv2 = -CosYDiv2;
    float SinYDiv2 = sqrt( ( 1 - CosY ) / 2 );
    float CosX = sqrt( ( normal_vec( 0 ) * normal_vec( 0 ) + normal_vec( 2 ) * normal_vec( 2 ) ) /
                       ( normal_vec( 0 ) * normal_vec( 0 ) + normal_vec( 1 ) * normal_vec( 1 ) + normal_vec( 2 ) * normal_vec( 2 ) ) );
    if ( normal_vec( 2 ) < 0 )
        CosX = -CosX;
    float CosXDiv2 = sqrt( ( CosX + 1 ) / 2 );
    if ( normal_vec( 1 ) < 0 )
        CosXDiv2 = -CosXDiv2;
    float SinXDiv2 = sqrt( ( 1 - CosX ) / 2 );
    q.w = CosXDiv2 * CosYDiv2;
    q.x = SinXDiv2 * CosYDiv2;
    q.y = CosXDiv2 * SinYDiv2;
    q.z = -SinXDiv2 * SinYDiv2;
}

void pubNormal( visualization_msgs::MarkerArray &normal_pub, const std::string normal_ns, const int normal_id, const pcl::PointXYZINormal normal_p,
                const float alpha, const Eigen::Vector3d rgb )
{
    visualization_msgs::Marker normal;
    normal.header.frame_id = "camera_init";
    normal.header.stamp = ros::Time();
    normal.ns = normal_ns;
    normal.id = normal_id;
    normal.type = visualization_msgs::Marker::ARROW;
    normal.action = visualization_msgs::Marker::ADD;
    normal.pose.position.x = normal_p.x;
    normal.pose.position.y = normal_p.y;
    normal.pose.position.z = normal_p.z;
    geometry_msgs::Quaternion q;
    Eigen::Vector3d           normal_vec( normal_p.normal_x, normal_p.normal_y, normal_p.normal_z );
    CalcQuation( normal_vec, 0, q );
    normal.pose.orientation = q;
    normal.scale.x = 0.4;
    normal.scale.y = 0.05;
    normal.scale.z = 0.05;
    normal.color.a = alpha; // Don't forget to set the alpha!
    normal.color.r = rgb( 0 );
    normal.color.g = rgb( 1 );
    normal.color.b = rgb( 2 );
    normal.lifetime = ros::Duration();
    normal_pub.markers.push_back( normal ); // normal_pub.publish(normal);
}

void pubPlane( visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const int plane_id, const pcl::PointXYZINormal normal_p,
               const float radius, const float min_eigen_value, const float alpha, const Eigen::Vector3d rgb )
{
    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = plane_ns;
    plane.id = plane_id;
    plane.type = visualization_msgs::Marker::CYLINDER;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = normal_p.x;
    plane.pose.position.y = normal_p.y;
    plane.pose.position.z = normal_p.z;
    geometry_msgs::Quaternion q;
    Eigen::Vector3d           normal_vec( normal_p.normal_x, normal_p.normal_y, normal_p.normal_z );
    CalcQuation( normal_vec, 2, q );
    plane.pose.orientation = q;
    plane.scale.x = 2 * radius;
    plane.scale.y = 2 * radius;
    plane.scale.z = sqrt( min_eigen_value );
    plane.color.a = alpha;
    plane.color.r = rgb( 0 );
    plane.color.g = rgb( 1 );
    plane.color.b = rgb( 2 );
    plane.lifetime = ros::Duration();
    plane_pub.markers.push_back( plane ); // plane_pub.publish(plane);
}

void pubPlaneMap( const std::unordered_map< VOXEL_LOC, OctoTree * > &feat_map, const ros::Publisher &plane_map_pub, const V3D &position )
{
    int       normal_id = 0;
    int       plane_count = 0;
    int       sub_plane_count = 0;
    int       sub_sub_plane_count = 0;
    OctoTree *current_octo = nullptr;
    float     plane_threshold = 0.0025;
    double    max_trace = 0.25;
    double    pow_num = 0.2;
    double    dis_threshold = 100;
    ros::Rate loop( 500 );
    float     use_alpha = 1.0;
    int       update_count = 0;
    int       id = 0;

    visualization_msgs::MarkerArray voxel_plane;
    // visualization_msgs::MarkerArray voxel_norm;
    voxel_plane.markers.reserve( 1000000 );

    for ( auto iter = feat_map.begin(); iter != feat_map.end(); iter++ )
    {
        if ( iter->second->m_plane_ptr_->m_is_update )
        {
            // V3D position_inc = position - iter->second->plane_ptr_->center;
            // if (position_inc.norm() > dis_threshold) {
            //   continue;
            // }
            Eigen::Vector3d normal_rgb( 0.0, 1.0, 1.0 );

            V3D    plane_var = iter->second->m_plane_ptr_->m_plane_var.block< 3, 3 >( 0, 0 ).diagonal();
            double trace = plane_var.sum();
            // outfile << trace << "," << iter->second->plane_ptr_->points_size << ","
            //         << 0 << std::endl;
            if ( trace >= max_trace )
            {
                trace = max_trace;
            }
            trace = trace * ( 1.0 / max_trace );
            // trace = (max_trace - trace) / max_trace;
            trace = pow( trace, pow_num );
            uint8_t r, g, b;
            mapJet( trace, 0, 1, r, g, b );
            Eigen::Vector3d plane_rgb( r / 256.0, g / 256.0, b / 256.0 );
            // if (plane_var.norm() < 0.001) {
            //   plane_rgb << 1, 0, 0;
            // } else if (plane_var.norm() < 0.005) {
            //   plane_rgb << 0, 1, 0;
            // } else {
            //   plane_rgb << 0, 0, 1;
            // }
            // plane_rgb << plane_var[0] / 0.0001, plane_var[1] / 0.0001,
            //     plane_var[2] / 0.0001;
            // plane_rgb << fabs(iter->second->plane_ptr_->normal[0]),
            //     fabs(iter->second->plane_ptr_->normal[1]),
            //     fabs(iter->second->plane_ptr_->normal[2]);
            float alpha = 0.0;
            if ( iter->second->m_plane_ptr_->m_is_plane )
            {
                alpha = use_alpha;
            }
            else
            {
                std::cout << "delete plane" << std::endl;
            }
            // pubNormal(plane_map_pub, "normal", iter->second->plane_ptr_->id,
            //           iter->second->plane_ptr_->p_center, alpha, normal_rgb);
            // loop.sleep();
            pubPlane( voxel_plane, "plane", iter->second->m_plane_ptr_->m_id, iter->second->m_plane_ptr_->m_p_center,
                      1.25 * iter->second->m_plane_ptr_->m_radius, iter->second->m_plane_ptr_->m_min_eigen_value, alpha, plane_rgb );
            // loop.sleep();
            iter->second->m_plane_ptr_->m_is_update = false;
            update_count++;
            plane_count++;
        }
        else
        {
            for ( uint i = 0; i < 8; i++ )
            {
                if ( iter->second->m_leaves_[ i ] != nullptr )
                {
                    if ( iter->second->m_leaves_[ i ]->m_plane_ptr_->m_is_update )
                    {
                        // std::cout << "plane var:"
                        //           << iter->second->leaves_[i]
                        //                  ->plane_ptr_->plane_var.diagonal()
                        //                  .transpose()
                        //           << std::endl;
                        V3D position_inc = position - iter->second->m_leaves_[ i ]->m_plane_ptr_->m_center;
                        // if (position_inc.norm() > dis_threshold) {
                        //   continue;
                        // }
                        Eigen::Vector3d normal_rgb( 112.0 / 255, 173.0 / 255, 71.0 / 255 );

                        V3D    plane_var = iter->second->m_leaves_[ i ]->m_plane_ptr_->m_plane_var.block< 3, 3 >( 0, 0 ).diagonal();
                        double trace = plane_var.sum();
                        // outfile << trace << ","
                        //         << iter->second->leaves_[i]->plane_ptr_->points_size <<
                        //         ",1"
                        //         << std::endl;
                        if ( trace >= max_trace )
                        {
                            trace = max_trace;
                        }
                        trace = trace * ( 1.0 / max_trace );
                        // trace = (max_trace - trace) / max_trace;
                        trace = pow( trace, pow_num );
                        uint8_t r, g, b;
                        mapJet( trace, 0, 1, r, g, b );
                        Eigen::Vector3d plane_rgb( r / 256.0, g / 256.0, b / 256.0 );
                        // plane_rgb <<
                        // fabs(iter->second->leaves_[i]->plane_ptr_->normal[0]),
                        //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[1]),
                        //     fabs(iter->second->leaves_[i]->plane_ptr_->normal[2]);
                        float alpha = 0.0;
                        if ( iter->second->m_leaves_[ i ]->m_plane_ptr_->m_is_plane )
                        {
                            alpha = use_alpha;
                        }
                        else
                        {
                            std::cout << "delete plane" << std::endl;
                        }
                        // pubNormal(plane_map_pub, "normal",
                        //           iter->second->leaves_[i]->plane_ptr_->id,
                        //           iter->second->leaves_[i]->plane_ptr_->p_center, alpha,
                        //           normal_rgb);
                        // loop.sleep();
                        pubPlane( voxel_plane, "plane", iter->second->m_leaves_[ i ]->m_plane_ptr_->m_id,
                                  iter->second->m_leaves_[ i ]->m_plane_ptr_->m_p_center, 1.25 * iter->second->m_leaves_[ i ]->m_plane_ptr_->m_radius,
                                  iter->second->m_leaves_[ i ]->m_plane_ptr_->m_min_eigen_value, alpha, plane_rgb );
                        // loop.sleep();
                        iter->second->m_leaves_[ i ]->m_plane_ptr_->m_is_update = false;
                        update_count++;

                        sub_plane_count++;
                        normal_id++;
                        // loop.sleep();
                    }
                    else
                    {
                        OctoTree *temp_octo_tree = iter->second->m_leaves_[ i ];
                        for ( uint j = 0; j < 8; j++ )
                        {
                            if ( temp_octo_tree->m_leaves_[ j ] != nullptr )
                            {
                                if ( temp_octo_tree->m_leaves_[ j ]->m_octo_state_ == 0 && temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_is_update )
                                {
                                    // std::cout << "plane var:"
                                    //           << temp_octo_tree->leaves_[j]
                                    //                  ->plane_ptr_->plane_var.diagonal()
                                    //                  .transpose()
                                    //           << std::endl;
                                    V3D position_inc = position - temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_center;
                                    // if (position_inc.norm() > dis_threshold) {
                                    //   continue;
                                    // }
                                    if ( temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_is_plane )
                                    {
                                        // std::cout << "subsubplane" << std::endl;
                                        Eigen::Vector3d normal_rgb( 1.0, 0.0, 0.0 );
                                        V3D    plane_var = temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_plane_var.block< 3, 3 >( 0, 0 ).diagonal();
                                        double trace = plane_var.sum();
                                        // outfile
                                        //     << trace << ","
                                        //     <<
                                        //     temp_octo_tree->leaves_[j]->plane_ptr_->points_size
                                        //     << ",2" << std::endl;
                                        if ( trace >= max_trace )
                                        {
                                            trace = max_trace;
                                        }
                                        trace = trace * ( 1.0 / max_trace );
                                        // trace = (max_trace - trace) / max_trace;
                                        trace = pow( trace, pow_num );
                                        uint8_t r, g, b;
                                        mapJet( trace, 0, 1, r, g, b );
                                        Eigen::Vector3d plane_rgb( r / 256.0, g / 256.0, b / 256.0 );
                                        float           alpha = 0.0;
                                        if ( temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_is_plane )
                                        {
                                            alpha = use_alpha;
                                        }
                                        // pubNormal(plane_map_pub, "normal",
                                        //           iter->second->leaves_[i]->plane_ptr_->id,
                                        //           temp_octo_tree->leaves_[j]->plane_ptr_->p_center,
                                        //           alpha, normal_rgb);
                                        pubPlane( voxel_plane, "plane", temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_id,
                                                  temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_p_center,
                                                  temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_radius,
                                                  temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_min_eigen_value, alpha, plane_rgb );
                                        // loop.sleep();
                                        temp_octo_tree->m_leaves_[ j ]->m_plane_ptr_->m_is_update = false;
                                        update_count++;
                                    }
                                    sub_sub_plane_count++;
                                    // loop.sleep();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    plane_map_pub.publish( voxel_plane );
    // plane_map_pub.publish(voxel_norm);
    loop.sleep();
    cout << "[Map Info] Plane counts:" << plane_count << " Sub Plane counts:" << sub_plane_count << " Sub Sub Plane counts:" << sub_sub_plane_count
         << endl;
    cout << "[Map Info] Update plane counts:" << update_count << "total size: " << feat_map.size() << endl;
}

// Similar with PCL voxelgrid filter
void down_sampling_voxel( pcl::PointCloud< pcl::PointXYZI > &pl_feat, double voxel_size )
{
    int intensity = rand() % 255;
    if ( voxel_size < 0.01 )
    {
        return;
    }
    std::unordered_map< VOXEL_LOC, M_POINT > feat_map;
    uint                                     plsize = pl_feat.size();

    for ( uint i = 0; i < plsize; i++ )
    {
        pcl::PointXYZI &p_c = pl_feat[ i ];
        float           loc_xyz[ 3 ];
        for ( int j = 0; j < 3; j++ )
        {
            loc_xyz[ j ] = p_c.data[ j ] / voxel_size;
            if ( loc_xyz[ j ] < 0 )
            {
                loc_xyz[ j ] -= 1.0;
            }
        }

        VOXEL_LOC position( ( int64_t ) loc_xyz[ 0 ], ( int64_t ) loc_xyz[ 1 ], ( int64_t ) loc_xyz[ 2 ] );
        auto      iter = feat_map.find( position );
        if ( iter != feat_map.end() )
        {
            iter->second.xyz[ 0 ] += p_c.x;
            iter->second.xyz[ 1 ] += p_c.y;
            iter->second.xyz[ 2 ] += p_c.z;
            iter->second.intensity += p_c.intensity;
            iter->second.count++;
        }
        else
        {
            M_POINT anp;
            anp.xyz[ 0 ] = p_c.x;
            anp.xyz[ 1 ] = p_c.y;
            anp.xyz[ 2 ] = p_c.z;
            anp.intensity = p_c.intensity;
            anp.count = 1;
            feat_map[ position ] = anp;
        }
    }
    plsize = feat_map.size();
    pl_feat.clear();
    pl_feat.resize( plsize );

    uint i = 0;
    for ( auto iter = feat_map.begin(); iter != feat_map.end(); ++iter )
    {
        pl_feat[ i ].x = iter->second.xyz[ 0 ] / iter->second.count;
        pl_feat[ i ].y = iter->second.xyz[ 1 ] / iter->second.count;
        pl_feat[ i ].z = iter->second.xyz[ 2 ] / iter->second.count;
        pl_feat[ i ].intensity = iter->second.intensity / iter->second.count;
        i++;
    }
}

void calcBodyVar( Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var )
{
    if ( pb[ 2 ] == 0 )
        pb[ 2 ] = 0.0001;
    float           range = sqrt( pb[ 0 ] * pb[ 0 ] + pb[ 1 ] * pb[ 1 ] + pb[ 2 ] * pb[ 2 ] );
    float           range_var = range_inc * range_inc;
    Eigen::Matrix2d direction_var;
    direction_var << pow( sin( DEG2RAD( degree_inc ) ), 2 ), 0, 0, pow( sin( DEG2RAD( degree_inc ) ), 2 );
    Eigen::Vector3d direction( pb );
    direction.normalize();
    Eigen::Matrix3d direction_hat;
    direction_hat << 0, -direction( 2 ), direction( 1 ), direction( 2 ), 0, -direction( 0 ), -direction( 1 ), direction( 0 ), 0;
    Eigen::Vector3d base_vector1( 1, 1, -( direction( 0 ) + direction( 1 ) ) / direction( 2 ) );
    base_vector1.normalize();
    Eigen::Vector3d base_vector2 = base_vector1.cross( direction );
    base_vector2.normalize();
    Eigen::Matrix< double, 3, 2 > N;
    N << base_vector1( 0 ), base_vector2( 0 ), base_vector1( 1 ), base_vector2( 1 ), base_vector1( 2 ), base_vector2( 2 );
    Eigen::Matrix< double, 3, 2 > A = range * direction_hat * N;
    var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

bool   Voxel_mapping::voxel_map_init()
{
    pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
    Eigen::Quaterniond                     q( state.rot_end );

    // std::cout << "Begin build unorder map" << std::endl;
    transformLidar( state.rot_end, state.pos_end, m_feats_undistort, world_lidar );
    std::vector< Point_with_var > pv_list;
    pv_list.reserve( world_lidar->size() );
    for ( size_t i = 0; i < world_lidar->size(); i++ )
    {
        Point_with_var pv;
        pv.m_point << world_lidar->points[ i ].x, world_lidar->points[ i ].y, world_lidar->points[ i ].z;
        V3D point_this( m_feats_undistort->points[ i ].x, m_feats_undistort->points[ i ].y, m_feats_undistort->points[ i ].z );
        M3D var;
        calcBodyVar( point_this, m_dept_err, m_beam_err, var );

        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX( point_this );
        var = state.rot_end * var * state.rot_end.transpose() +
              ( -point_crossmat ) * state.cov.block< 3, 3 >( 0, 0 ) * ( -point_crossmat ).transpose() + state.cov.block< 3, 3 >( 3, 3 );
        pv.m_var = var;
        pv_list.push_back( pv );

        // if ( i % 300 == 3 )
        // {
        //     printf( "pw: %lf %lf %lf, var: %lf %lf %lf \n", world_lidar->points[ i ].x, world_lidar->points[ i ].y,
        //             world_lidar->points[ i ].z, var( 0, 0 ), var( 1, 1 ), var( 2, 2 ) );
        // }
    }

    buildVoxelMap( pv_list, m_max_voxel_size, m_max_layer, m_layer_init_size, m_max_points_size, m_min_eigen_value, m_feat_map );
    // std::cout << "Build unorderMap,init points size:" << world_lidar->size() << std::endl;
    // std::cout << "params: " << pv_list.size() << " " << m_max_voxel_size << " [" << m_layer_size.transpose() << "] " << m_min_eigen_value << " "
    //           << m_feat_map.size() << endl;
    // std::cout << "state: " << state.rot_end << " " << state.pos_end << endl;

    return true;
}

/*** EKF update ***/
void Voxel_mapping::lio_state_estimation( StatesGroup &state_propagat )
{
    m_point_selected_surf.resize( m_feats_down_size, true );
    m_pointSearchInd_surf.resize( m_feats_down_size );
    m_Nearest_Points.resize( m_feats_down_size );
    m_cross_mat_list.clear();
    m_cross_mat_list.reserve( m_feats_down_size );
    m_body_cov_list.clear();
    m_body_cov_list.reserve( m_feats_down_size );

    if ( m_use_new_map )
    {
        if ( m_feat_map.empty() )
        {
            printf( "feat_map.empty!!" );
            return;
        }

        for ( size_t i = 0; i < m_feats_down_body->size(); i++ )
        {
            V3D point_this( m_feats_down_body->points[ i ].x, m_feats_down_body->points[ i ].y, m_feats_down_body->points[ i ].z );
            if ( point_this[ 2 ] == 0 )
            {
                point_this[ 2 ] = 0.001;
            }
            M3D var;
            calcBodyVar( point_this, m_dept_err, m_beam_err, var );
            m_body_cov_list.push_back( var );
            point_this = m_extR * point_this + m_extT;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX( point_this );
            m_cross_mat_list.push_back( point_crossmat );
        }
    }

    if ( !m_flg_EKF_inited )
        return;

    m_point_selected_surf.resize( m_feats_down_size, true );
    m_Nearest_Points.resize( m_feats_down_size );
    int  rematch_num = 0;
    bool nearest_search_en = true; //
    MD( DIM_STATE, DIM_STATE ) G, H_T_H, I_STATE;
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();

    for ( int iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++ )
    {
        m_laserCloudOri->clear(); //
        m_corr_normvect->clear(); //
        m_total_residual = 0.0;   //

        std::vector< double > r_list;
        std::vector< ptpl >   ptpl_list;
        // ptpl_list.reserve( m_feats_down_size );
        if ( m_use_new_map )
        {
            vector< Point_with_var >               pv_list;
            pcl::PointCloud< pcl::PointXYZI >::Ptr world_lidar( new pcl::PointCloud< pcl::PointXYZI > );
            transformLidar( state.rot_end, state.pos_end, m_feats_down_body, world_lidar );

            M3D rot_var = state.cov.block< 3, 3 >( 0, 0 );
            M3D t_var = state.cov.block< 3, 3 >( 3, 3 );
            for ( size_t i = 0; i < m_feats_down_body->size(); i++ )
            {
                Point_with_var pv;
                pv.m_point << m_feats_down_body->points[ i ].x, m_feats_down_body->points[ i ].y, m_feats_down_body->points[ i ].z;
                pv.m_point_world << world_lidar->points[ i ].x, world_lidar->points[ i ].y, world_lidar->points[ i ].z;
                M3D cov = m_body_cov_list[ i ];
                M3D point_crossmat = m_cross_mat_list[ i ];

                cov = state.rot_end * cov * state.rot_end.transpose() + ( -point_crossmat ) * rot_var * ( -point_crossmat.transpose() ) + t_var;
                pv.m_var = cov;
                pv_list.push_back( pv );
            }
            // std::cout << "rot var:" << rot_var << std::endl;
            // std::cout << "t var:" << t_var << std::endl;
            auto               scan_match_time_start = std::chrono::high_resolution_clock::now();
            std::vector< V3D > non_match_list;
            // int                m_max_layer = 4;
            BuildResidualListOMP( m_feat_map, m_max_voxel_size, 3.0, m_max_layer, pv_list, ptpl_list, non_match_list );
            m_effct_feat_num = 0;
            int layer_match[ 4 ] = { 0 };
            // int layer0 = 0;
            // int layer1 = 0;
            // int layer2 = 0;
            // int layer3 = 0;
            for ( int i = 0; i < ptpl_list.size(); i++ )
            {
                PointType pi_body;
                PointType pi_world;
                PointType pl;
                pi_body.x = ptpl_list[ i ].point( 0 );
                pi_body.y = ptpl_list[ i ].point( 1 );
                pi_body.z = ptpl_list[ i ].point( 2 );
                Eigen::Vector3d point_world;
                pointBodyToWorld( ptpl_list[ i ].point, point_world );
                pl.x = ptpl_list[ i ].normal( 0 );
                pl.y = ptpl_list[ i ].normal( 1 );
                pl.z = ptpl_list[ i ].normal( 2 );
                m_effct_feat_num++;
                float dis = ( point_world[ 0 ] * pl.x + point_world[ 1 ] * pl.y + point_world[ 2 ] * pl.z + ptpl_list[ i ].d );
                pl.intensity = dis;
                m_laserCloudOri->push_back( pi_body );
                m_corr_normvect->push_back( pl );
                m_total_residual += fabs( dis );
                layer_match[ ptpl_list[ i ].layer ]++;
            }
            auto scan_match_time_end = std::chrono::high_resolution_clock::now();
            m_res_mean_last = m_total_residual / m_effct_feat_num;
            // for ( int k = 0; k < 4; k++ )
            // {
            //     std::cout << "layer " << k << ": " << layer_match[ k ] << std::endl;
            // }
        }
        else
        {
/** Old map ICP **/
#ifdef MP_EN
            omp_set_num_threads( 10 );
#pragma omp parallel for
#endif
            for ( int i = 0; i < m_feats_down_size; i++ )
            {
                PointType &point_body = m_feats_down_body->points[ i ];
                PointType &point_world = m_feats_down_world->points[ i ];
                V3D        p_body( point_body.x, point_body.y, point_body.z );
                /* transform to world frame */
                pointBodyToWorld( point_body, point_world );
                vector< float > pointSearchSqDis( NUM_MATCH_POINTS );
#ifdef USE_ikdtree
                auto &points_near = m_Nearest_Points[ i ];
#else
                auto &points_near = pointSearchInd_surf[ i ];
#endif
                uint8_t search_flag = 0;
                double  search_start = omp_get_wtime();
                if ( nearest_search_en )
                {
/** Find the closest surfaces in the map **/
#ifdef USE_ikdtree
#ifdef USE_ikdforest
                    search_flag = ikdforest.Nearest_Search( point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5 );
#else
                    m_ikdtree.Nearest_Search( point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis );
#endif
#else
                    kdtreeSurfFromMap->nearestKSearch( point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis );
#endif

                    m_point_selected_surf[ i ] = pointSearchSqDis[ NUM_MATCH_POINTS - 1 ] > 5 ? false : true;

#ifdef USE_ikdforest
                    point_selected_surf[ i ] = point_selected_surf[ i ] && ( search_flag == 0 );
#endif
                    m_kdtree_search_time += omp_get_wtime() - search_start;
                    m_kdtree_search_counter++;
                }

                if ( !m_point_selected_surf[ i ] || points_near.size() < NUM_MATCH_POINTS )
                    continue;

                VF( 4 ) pabcd;
                m_point_selected_surf[ i ] = false;
                if ( esti_plane( pabcd, points_near, 0.05f ) ) //(planeValid)
                {
                    float pd2 = pabcd( 0 ) * point_world.x + pabcd( 1 ) * point_world.y + pabcd( 2 ) * point_world.z + pabcd( 3 );
                    float s = 1 - 0.9 * fabs( pd2 ) / sqrt( p_body.norm() );

                    if ( s > 0.9 )
                    {
                        m_point_selected_surf[ i ] = true;
                        m_normvec->points[ i ].x = pabcd( 0 );
                        m_normvec->points[ i ].y = pabcd( 1 );
                        m_normvec->points[ i ].z = pabcd( 2 );
                        m_normvec->points[ i ].intensity = pd2;
                        m_res_last[ i ] = abs( pd2 );
                    }
                }
            }
            // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
            m_effct_feat_num = 0;

            for ( int i = 0; i < m_feats_down_size; i++ )
            {
                if ( m_point_selected_surf[ i ] && ( m_res_last[ i ] <= 2.0 ) )
                {
                    m_laserCloudOri->points[ m_effct_feat_num ] = m_feats_down_body->points[ i ];
                    m_corr_normvect->points[ m_effct_feat_num ] = m_normvec->points[ i ];
                    m_total_residual += m_res_last[ i ];
                    m_effct_feat_num++;
                }
            }

            m_res_mean_last = m_total_residual / m_effct_feat_num;
        }
        // std::cout << "pose:" << state.pos_end.transpose() << std::endl;
        // cout << "[ mapping ]: Effective feature num: " << m_effct_feat_num << ", ds size:" << m_feats_down_size
        //      << " , raw size:" << m_feats_undistort->size() << " res_mean_last " << m_res_mean_last << endl;
        double solve_start = omp_get_wtime();
        /*** Computation of Measuremnt Jacobian matrix H and measurents covarience
         * ***/
        MatrixXd Hsub( m_effct_feat_num, 6 );
        MatrixXd Hsub_T_R_inv( 6, m_effct_feat_num );
        VectorXd R_inv( m_effct_feat_num );
        VectorXd meas_vec( m_effct_feat_num );
        meas_vec.setZero();

        for ( int i = 0; i < m_effct_feat_num; i++ )
        {
            const PointType &laser_p = m_laserCloudOri->points[ i ];
            V3D              point_this( laser_p.x, laser_p.y, laser_p.z );
            V3D              point_body( laser_p.x, laser_p.y, laser_p.z );
            point_this = m_extR * point_this + m_extT;
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX( point_this );

            /*** get the normal vector of closest surface/corner ***/
            PointType &norm_p = m_corr_normvect->points[ i ];
            V3D        norm_vec( norm_p.x, norm_p.y, norm_p.z );

            if ( m_use_new_map )
            {
                V3D point_world = state.rot_end * point_this + state.pos_end;
                // /*** get the normal vector of closest surface/corner ***/

                M3D var;

                if ( m_p_pre->calib_laser )
                {
                    calcBodyVar( point_this, m_dept_err, CALIB_ANGLE_COV, var );
                }
                else
                {
                    calcBodyVar( point_this, m_dept_err, m_beam_err, var );
                }
                var = state.rot_end * m_extR * var * ( state.rot_end * m_extR ).transpose();
                // bug exist, to be fixed
                Eigen::Matrix< double, 1, 6 > J_nq;
                J_nq.block< 1, 3 >( 0, 0 ) = point_world - ptpl_list[ i ].center;
                J_nq.block< 1, 3 >( 0, 3 ) = -ptpl_list[ i ].normal;
                double sigma_l = J_nq * ptpl_list[ i ].plane_var * J_nq.transpose();
                R_inv( i ) = 1.0 / ( sigma_l + norm_vec.transpose() * var * norm_vec );
                // if (fabs(ptpl_list[i].normal[2]) > 0.7) {
                //   R_inv(i) =
                //       1.0 / (0.0009 + sigma_l + norm_vec.transpose() * var *
                //       norm_vec);
                // } else {
                //
                // }

                // R_inv(i) = 1000;0.0004 +
                // cout<<"sigma_l: "<<sigma_l<<" "<<var(0,0)<<" "<<var(1,1)<<"
                // "<<var(2,2)<<" "<<norm_vec.transpose()<<endl;

                // if ( fabs( reflec_ang_cos ) < 0.15 && point_this.norm() < 8 )
                // {
                //     // R_inv(i) = 1;
                //     // norm_p.intensity = 0;
                //     // cout<<" angle: "<<acosf(reflec_ang_cos)*57.3<<" point_this:
                //     // "<<point_world[2]<<endl;
                // }
                // else
                // {
                //     // depth_err = 0.0;
                // }

                // printf("R_inv(i): %.6lf index: %d \n", R_inv(i), i);
            }
            else
            {
                R_inv( i ) = 1 / LASER_POINT_COV;
            }
            m_laserCloudOri->points[ i ].intensity = sqrt( R_inv( i ) );
            //            R_inv(i) = 1.0;

            /*** calculate the Measuremnt Jacobian matrix H ***/
            V3D A( point_crossmat * state.rot_end.transpose() * norm_vec );
            Hsub.row( i ) << VEC_FROM_ARRAY( A ), norm_p.x, norm_p.y, norm_p.z;
            Hsub_T_R_inv.col( i ) << A[ 0 ] * R_inv( i ), A[ 1 ] * R_inv( i ), A[ 2 ] * R_inv( i ), norm_p.x * R_inv( i ), norm_p.y * R_inv( i ),
                norm_p.z * R_inv( i );
            // cout<<"Hsub.row(i): "<<Hsub.row(i)<<" "<<norm_vec.transpose()<<endl;
            // cout<<"Hsub_T_R_inv.row(i): "<<Hsub_T_R_inv.col(i).transpose()<<endl;
            /*** Measuremnt: distance to the closest surface/corner ***/
            meas_vec( i ) = -norm_p.intensity;
            // if (iterCount == 0) fout_dbg<<setprecision(6)<<i<<" "<<meas_vec[i]<<"
            // "<<Hsub.row(i)<<endl; if (fabs(meas_vec(i)) > 0.07)
            // {
            //     printf("meas_vec(i): %.6lf index: %d \n", meas_vec(i), i);
            // }
        }
        m_solve_const_H_time += omp_get_wtime() - solve_start;

        m_EKF_stop_flg = false;
        m_flg_EKF_converged = false;

        MatrixXd K( DIM_STATE, m_effct_feat_num );

        /*** Iterative Kalman Filter Update ***/
        // auto &&Hsub_T = Hsub.transpose();
        auto &&HTz = Hsub_T_R_inv * meas_vec;
        H_T_H.block< 6, 6 >( 0, 0 ) = Hsub_T_R_inv * Hsub;
        // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
        MD( DIM_STATE, DIM_STATE ) &&K_1 = ( H_T_H + state.cov.inverse() ).inverse();
        G.block< DIM_STATE, 6 >( 0, 0 ) = K_1.block< DIM_STATE, 6 >( 0, 0 ) * H_T_H.block< 6, 6 >( 0, 0 );
        auto vec = state_propagat - state;
        VD( DIM_STATE )
        solution = K_1.block< DIM_STATE, 6 >( 0, 0 ) * HTz + vec - G.block< DIM_STATE, 6 >( 0, 0 ) * vec.block< 6, 1 >( 0, 0 );

        // K = K_1.block<DIM_STATE,6>(0,0) * Hsub_T_R_inv;
        // solution = K * meas_vec + vec - K * Hsub * vec.block<6,1>(0,0);

        int minRow, minCol;
        if ( 0 ) // if(V.minCoeff(&minRow, &minCol) < 1.0f)
        {
            VD( 6 ) V = H_T_H.block< 6, 6 >( 0, 0 ).eigenvalues().real();
            cout << "!!!!!! Degeneration Happend, eigen values: " << V.transpose() << endl;
            m_EKF_stop_flg = true;
            solution.block< 6, 1 >( 9, 0 ).setZero();
        }

        state += solution;

        auto rot_add = solution.block< 3, 1 >( 0, 0 );
        auto t_add = solution.block< 3, 1 >( 3, 0 );

        // cout<<"solution: "<<solution.transpose()<<endl;
        // cout<<"HTz: "<<HTz.transpose()<<endl;
        // cout<<"H_T_H sub: "<<H_T_H.block<6,6>(0,0)<<endl;
        // cout<<"cov: "<<endl;
        // cout<<state.cov.diagonal().transpose()<<endl;
        // cout<<"state up: "<<state.pos_end.transpose()<<"
        // "<<state.vel_end.transpose()<<endl; cout<<"vec: "<<vec.transpose()<<endl;

        if ( ( rot_add.norm() * 57.3 < 0.01 ) && ( t_add.norm() * 100 < 0.015 ) )
        {
            m_flg_EKF_converged = true;
        }

        m_euler_cur = RotMtoEuler( state.rot_end );

        /*** Rematch Judgement ***/
        nearest_search_en = false;
        if ( m_flg_EKF_converged || ( ( rematch_num == 0 ) && ( iterCount == ( NUM_MAX_ITERATIONS - 2 ) ) ) )
        {
            nearest_search_en = true;
            rematch_num++;
        }

        /*** Convergence Judgements and Covariance Update ***/
        if ( !m_EKF_stop_flg && ( rematch_num >= 2 || ( iterCount == NUM_MAX_ITERATIONS - 1 ) ) )
        {
            /*** Covariance Update ***/
            state.cov = ( I_STATE - G ) * state.cov;
            m_total_distance += ( state.pos_end - m_position_last ).norm();
            m_position_last = state.pos_end;
            m_geo_Quat = tf::createQuaternionMsgFromRollPitchYaw( m_euler_cur( 0 ), m_euler_cur( 1 ), m_euler_cur( 2 ) );

            // VD(DIM_STATE) K_sum  = K.rowwise().sum();
            // VD(DIM_STATE) P_diag = state.cov.diagonal();
            m_EKF_stop_flg = true;
        }
        m_solve_time += omp_get_wtime() - solve_start;

        if ( m_EKF_stop_flg )
            break;
    }
}

void Voxel_mapping::init_ros_node()
{
    m_ros_node_ptr = std::make_shared< ros::NodeHandle >();
    read_ros_parameters( *m_ros_node_ptr );
}

int Voxel_mapping::service_LiDAR_update()
{
    cout << "debug:" << m_debug << " MIN_IMG_COUNT: " << MIN_IMG_COUNT << endl;
    m_pcl_wait_pub->clear();
    // if (m_lidar_en)
    // {
    ros::Subscriber sub_pcl;
    if ( m_p_pre->lidar_type == AVIA )
    {
        sub_pcl = m_ros_node_ptr->subscribe( m_lid_topic, 200000, &Voxel_mapping::livox_pcl_cbk, this );
    }
    else
    {
        sub_pcl = m_ros_node_ptr->subscribe( m_lid_topic, 200000, &Voxel_mapping::standard_pcl_cbk, this );
    }
    // }
    ros::Subscriber sub_imu = m_ros_node_ptr->subscribe( m_imu_topic, 200000, &Voxel_mapping::imu_cbk, this );
    // ros::Subscriber sub_img = m_ros_node_ptr->subscribe(m_img_topic, 200000, img_cbk);
    ros::Publisher pubLaserCloudFullRes = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/cloud_registered", 100 );
    ros::Publisher pubLaserCloudVoxel = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/cloud_voxel", 100 );
    ros::Publisher pubVisualCloud = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/cloud_visual_map", 100 );
    ros::Publisher pubSubVisualCloud = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/cloud_visual_sub_map", 100 );
    ros::Publisher pubLaserCloudEffect = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/cloud_effected", 100 );
    ros::Publisher pubLaserCloudMap = m_ros_node_ptr->advertise< sensor_msgs::PointCloud2 >( "/Laser_map", 100 );
    ros::Publisher pubOdomAftMapped = m_ros_node_ptr->advertise< nav_msgs::Odometry >( "/aft_mapped_to_init", 10 );
    ros::Publisher pubPath = m_ros_node_ptr->advertise< nav_msgs::Path >( "/path", 10 );
    ros::Publisher plane_pub = m_ros_node_ptr->advertise< visualization_msgs::Marker >( "/planner_normal", 1 );
    ros::Publisher voxel_pub = m_ros_node_ptr->advertise< visualization_msgs::MarkerArray >( "/voxels", 1 );

    // #ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = m_ros_node_ptr->advertise< geometry_msgs::PoseStamped >( "/mavros/vision_pose/pose", 10 );
    // #endif

    m_pub_path.header.stamp = ros::Time::now();
    m_pub_path.header.frame_id = "camera_init";
    FILE *fp_kitti;
    // string kitti_log_dir = "/home/zivlin/temp/FAST-LIO-Voxelmap//Log/"
    //                        "kitti_log_voxelmap.txt";
    string kitti_log_dir = std::string( ROOT_DIR ).append( "/Log/kitti_log_voxelmap.txt" );
    fp_kitti = fopen( kitti_log_dir.c_str(), "w" );
/*** variables definition ***/
#ifndef USE_IKFOM
    VD( DIM_STATE ) solution;
    MD( DIM_STATE, DIM_STATE ) G, H_T_H, I_STATE;
    V3D         rot_add, t_add;
    StatesGroup state_propagat;
    PointType   pointOri, pointSel, coeff;
#endif
    // PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
    int    effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_solve = 0, aver_time_const_H_time = 0;

    FOV_DEG = ( m_fov_deg + 10.0 ) > 179.9 ? 179.9 : ( m_fov_deg + 10.0 );
    HALF_FOV_COS = cos( ( FOV_DEG ) *0.5 * PI_M / 180.0 );

    m_downSizeFilterSurf.setLeafSize( m_filter_size_surf_min, m_filter_size_surf_min, m_filter_size_surf_min );
// downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
// filter_size_map_min);
#ifdef USE_ikdforest
    ikdforest.Set_balance_criterion_param( 0.6 );
    ikdforest.Set_delete_criterion_param( 0.5 );
    ikdforest.Set_environment( laserCloudDepth, laserCloudWidth, laserCloudHeight, cube_len );
    ikdforest.Set_downsample_param( filter_size_map_min );
#endif

    shared_ptr< ImuProcess > p_imu( new ImuProcess() );
    // p_imu->set_extrinsic(V3D(0.04165, 0.02326, -0.0284));   //avia
    // p_imu->set_extrinsic(V3D(0.05512, 0.02226, -0.0297));   //horizon
    m_extT << VEC_FROM_ARRAY( m_extrin_T );
    m_extR << MAT_FROM_ARRAY( m_extrin_R );

    p_imu->set_extrinsic( m_extT, m_extR );
    cout << "IMU set extrinsic_R:\r\n " << m_extR << endl;
    cout << "IMU set extrinsic_t:\r\n " << vec_3( m_extrin_T[ 0 ], m_extrin_T[ 1 ], m_extrin_T[ 2 ] ).transpose() << endl;
    p_imu->set_gyr_cov_scale( V3D( m_gyr_cov, m_gyr_cov, m_gyr_cov ) );
    p_imu->set_acc_cov_scale( V3D( m_acc_cov, m_acc_cov, m_acc_cov ) );
    p_imu->set_gyr_bias_cov( V3D( 0.0001, 0.0001, 0.0001 ) );
    p_imu->set_acc_bias_cov( V3D( 0.0001, 0.0001, 0.0001 ) );
    p_imu->set_imu_init_frame_num( m_imu_int_frame );

    if ( !m_imu_en )
        p_imu->disable_imu();

#ifndef USE_IKFOM
    G.setZero();
    H_T_H.setZero();
    I_STATE.setIdentity();
#endif

#ifdef USE_IKFOM
    double epsi[ 23 ] = { 0.001 };
    fill( epsi, epsi + 23, 0.001 );
    kf.init_dyn_share( get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi );
#endif
    /*** debug record ***/
    FILE * fp;
    string pos_log_dir = m_root_dir + "/Log/pos_log.txt";
    fp = fopen( pos_log_dir.c_str(), "w" );
    m_fout_img_pos.open( string( string( ROOT_DIR ) + "PCD/img_pos.json" ), ios::out );
    m_fout_pcd_pos.open( string( string( ROOT_DIR ) + "PCD/scans_pos.json" ), ios::out );
    m_fout_pre.open( DEBUG_FILE_DIR( "mat_pre.txt" ), ios::out );
    m_fout_out.open( DEBUG_FILE_DIR( "mat_out.txt" ), ios::out );
    m_fout_dbg.open( DEBUG_FILE_DIR( "dbg.txt" ), ios::out );
    // if (fout_pre && fout_out)
    //     cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    // else
    //     cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

#ifdef USE_ikdforest
    ikdforest.Set_balance_criterion_param( 0.6 );
    ikdforest.Set_delete_criterion_param( 0.5 );
#endif
    //------------------------------------------------------------------------------------------------------

    // signal( SIGINT, SigHandle );
    ros::Rate rate( 5000 );
    bool      status = ros::ok();
    while ( ( status = ros::ok() ) )
    {
        if ( m_flg_exit )
            break;
        ros::spinOnce();
        if ( !sync_packages( m_Lidar_Measures ) )
        {
            status = ros::ok();
            // cv::waitKey(1);
            rate.sleep();
            continue;
        }

        /*** Packaged got ***/
        if ( m_flg_reset )
        {
            ROS_WARN( "reset when rosbag play back" );
            p_imu->Reset();
            m_flg_reset = false;
            continue;
        }

        if ( !m_is_first_frame )
        {
            m_first_lidar_time = m_Lidar_Measures.lidar_beg_time;
            p_imu->first_lidar_time = m_first_lidar_time;
            m_is_first_frame = true;
            cout << "FIRST LIDAR FRAME!" << endl;
        }
        // std::cout << "ScanIdx:" << frame_num << std::endl;
        // double t0,t1,t2,t3,t4,t5,match_start, match_time, solve_start,
        // solve_time, svd_time;
        double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

        m_match_time = m_kdtree_search_time = m_kdtree_search_counter = m_solve_time = m_solve_const_H_time = svd_time = 0;
        t0 = omp_get_wtime();
        g_LiDAR_frame_start_time = t0;
        auto t_all_begin = std::chrono::high_resolution_clock::now();
#ifdef USE_IKFOM
        p_imu->Process( LidarMeasures, kf, feats_undistort );
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
#else
        p_imu->Process2( m_Lidar_Measures, state, m_feats_undistort );

        // ANCHOR: Remove point that nearing 10 meter
        if ( 0 )
        {
            int feats_undistort_available = 0;
            for ( int i = 0; i < m_feats_undistort->size(); i++ )
            {
                // vec_3 pt_vec = vec_3( m_feats_undistort->points[ i ].x, m_feats_undistort->points[ i ].y, m_feats_undistort->points[ i ].z );
                vec_3 pt_vec = vec_3( m_feats_undistort->points[ i ].x, m_feats_undistort->points[ i ].y, m_feats_undistort->points[ i ].z );
                if ( pt_vec.norm() <= 10 )
                {
                    continue;
                }
                feats_undistort_available++;
                m_feats_undistort->points[ feats_undistort_available ] = m_feats_undistort->points[ i ];
            }
            m_feats_undistort->points.resize( feats_undistort_available );
            cout << ANSI_COLOR_BLUE_BOLD << "Number of pts = " << feats_undistort_available << ANSI_COLOR_RESET << endl;
        }

        state_propagat = state;
#endif
        // if(p_imu)
        if ( m_p_pre->calib_laser )
        {
            for ( size_t i = 0; i < m_feats_undistort->size(); i++ )
            {
                PointType pi = m_feats_undistort->points[ i ];
                double    range = sqrt( pi.x * pi.x + pi.y * pi.y + pi.z * pi.z );
                double    calib_vertical_angle = deg2rad( 0.15 );
                double    vertical_angle = asin( pi.z / range ) + calib_vertical_angle;
                double    horizon_angle = atan2( pi.y, pi.x );
                pi.z = range * sin( vertical_angle );
                double project_len = range * cos( vertical_angle );
                pi.x = project_len * cos( horizon_angle );
                pi.y = project_len * sin( horizon_angle );
                m_feats_undistort->points[ i ] = pi;
            }
        }

        if ( m_lidar_en )
        {
            m_euler_cur = RotMtoEuler( state.rot_end );
#ifdef USE_IKFOM
            // state_ikfom fout_state = kf.get_x();
            fout_pre << setw( 20 ) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
                     << state_point.pos.transpose() << " " << state_point.vel.transpose() << " " << state_point.bg.transpose() << " "
                     << state_point.ba.transpose() << " " << state_point.grav << endl;
#else
            m_fout_pre << setw( 20 ) << m_Lidar_Measures.last_update_time - m_first_lidar_time << " " << m_euler_cur.transpose() * 57.3 << " "
                       << state.pos_end.transpose() << " " << state.vel_end.transpose() << " " << state.bias_g.transpose() << " "
                       << state.bias_a.transpose() << " " << state.gravity.transpose() << endl;
#endif
        }

        if ( m_feats_undistort->empty() || ( m_feats_undistort == nullptr ) )
        {
            cout << " No point!!!" << endl;
            continue;
        }

        m_flg_EKF_inited = ( m_Lidar_Measures.lidar_beg_time - m_first_lidar_time ) < INIT_TIME ? false : true;

        if ( !m_use_new_map )
            laser_map_fov_segment();

        /*** downsample the feature points in a scan ***/
        m_downSizeFilterSurf.setInputCloud( m_feats_undistort );
        m_downSizeFilterSurf.filter( *m_feats_down_body );
        // std::cout << "feats size" << m_feats_undistort->size() << ", down size: " << m_feats_down_body->size() << std::endl;
        m_feats_down_size = m_feats_down_body->points.size();

        /*** initialize the map ***/
        if ( m_use_new_map )
        {
            if ( !m_init_map )
            {
                m_init_map = voxel_map_init();
                if ( m_is_pub_plane_map )
                    pubPlaneMap( m_feat_map, voxel_pub, state.pos_end );

                frame_num++;
                continue;
            };
        }
        else if ( m_ikdtree.Root_Node == nullptr )
        {
            if ( m_feats_down_body->points.size() > 5 )
            {
                m_ikdtree.set_downsample_param( m_filter_size_map_min );
                frameBodyToWorld( m_feats_down_body, m_feats_down_world );
                m_ikdtree.Build( m_feats_down_world->points );
            }
            continue;
        }
        int featsFromMapNum = m_ikdtree.size();

        // cout<<"[ LIO ]: Raw feature num: "<<feats_undistort->points.size()<<"
        // downsamp num "<<feats_down_size<<" Map num: "<<featsFromMapNum<<endl;
        // cout<<"LidarMeasures.lidar_beg_time: "<<LidarMeasures.lidar_beg_time -
        // 1634087477.0<<endl;

        /*** ICP and iterated Kalman filter update ***/
        m_normvec->resize( m_feats_down_size );
        m_feats_down_world->clear();
        m_feats_down_world->resize( m_feats_down_size );
        // vector<double> res_last(feats_down_size, 1000.0); // initial //
        m_res_last.resize( m_feats_down_size, 1000.0 );
        m_point_selected_surf.resize( m_feats_down_size, true );
        m_pointSearchInd_surf.resize( m_feats_down_size );
        m_Nearest_Points.resize( m_feats_down_size );

        t1 = omp_get_wtime();

        t2 = omp_get_wtime();

/*** iterated state estimation ***/
#ifdef MP_EN
// cout<<"Using multi-processor, used core number: "<<MP_PROC_NUM<<endl;
#endif
        double t_update_start = omp_get_wtime();
#ifdef USE_IKFOM
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified( LASER_POINT_COV, solve_H_time );
        // state_ikfom updated_state = kf.get_x();
        state_point = kf.get_x();
        // euler_cur = RotMtoEuler(state_point.rot.toRotationMatrix());
        euler_cur = SO3ToEuler( state_point.rot );
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        // cout<<"position: "<<pos_lid.transpose()<<endl;
        geoQuat.x = state_point.rot.coeffs()[ 0 ];
        geoQuat.y = state_point.rot.coeffs()[ 1 ];
        geoQuat.z = state_point.rot.coeffs()[ 2 ];
        geoQuat.w = state_point.rot.coeffs()[ 3 ];
#else

        if ( m_lidar_en )
        {
            lio_state_estimation( state_propagat );
        }
#endif
        double t_update_end = omp_get_wtime();
        /******* Publish odometry *******/
        m_euler_cur = RotMtoEuler( state.rot_end );
        m_geo_Quat = tf::createQuaternionMsgFromRollPitchYaw( m_euler_cur( 0 ), m_euler_cur( 1 ), m_euler_cur( 2 ) );
        publish_odometry( pubOdomAftMapped );
        kitti_log( fp_kitti );
        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        // cout << "Frame time consumption:" << (t3 - t0)*1000.0 << " ms" << endl;

        if ( m_lidar_en )
            map_incremental_grow();

        if ( m_is_pub_plane_map )
            pubPlaneMap( m_feat_map, voxel_pub, state.pos_end );
        auto t_all_end = std::chrono::high_resolution_clock::now();
        auto all_time = std::chrono::duration_cast< std::chrono::duration< double > >( t_all_end - t_all_begin ).count() * 1000;
        // std::cout << "[Time]: all time:" << all_time << " ms" << std::endl;
        t5 = omp_get_wtime();
        m_kdtree_incremental_time = t5 - t3 + m_readd_time;
        /******* Publish points *******/
        PointCloudXYZI::Ptr laserCloudFullRes( m_dense_map_en ? m_feats_undistort : m_feats_down_body );
        int                 size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI( size, 1 ) );

        // for ( int i = 0; i < size; i++ )
        // {
        //     RGBpointBodyToWorld( &laserCloudFullRes->points[ i ], &laserCloudWorld->points[ i ] );
        // }
        // *m_pcl_wait_pub = *laserCloudWorld;
        publish_frame_world( pubLaserCloudFullRes, m_pub_point_skip );

        if ( m_effect_point_pub )
            publish_effect_world( pubLaserCloudEffect );
        if ( m_use_new_map )
            publish_voxel_point( pubLaserCloudVoxel, m_pcl_wait_pub );
        // publish_map(pubLaserCloudMap);
        publish_path( pubPath );
        // #ifdef DEPLOY
        publish_mavros( mavros_pose_publisher );
        // #endif

        /*** Debug variables ***/
        frame_num++;
        aver_time_consu = aver_time_consu * ( frame_num - 1 ) / frame_num + ( t5 - t0 ) / frame_num;
        aver_time_icp = aver_time_icp * ( frame_num - 1 ) / frame_num + ( t_update_end - t_update_start ) / frame_num;
        aver_time_match = aver_time_match * ( frame_num - 1 ) / frame_num + ( m_match_time ) / frame_num;
#ifdef USE_IKFOM
        aver_time_solve = aver_time_solve * ( frame_num - 1 ) / frame_num + ( solve_time + solve_H_time ) / frame_num;
        aver_time_const_H_time = aver_time_const_H_time * ( frame_num - 1 ) / frame_num + solve_time / frame_num;
#else
        aver_time_solve = aver_time_solve * ( frame_num - 1 ) / frame_num + ( m_solve_time ) / frame_num;
        aver_time_const_H_time = aver_time_const_H_time * ( frame_num - 1 ) / frame_num + m_solve_const_H_time / frame_num;
// cout << "construct H:" << aver_time_const_H_time << std::endl;
#endif
        // aver_time_consu = aver_time_consu * 0.9 + (t5 - t0) * 0.1;
        m_T1[ m_time_log_counter ] = m_Lidar_Measures.lidar_beg_time;
        m_s_plot[ m_time_log_counter ] = aver_time_consu;
        m_s_plot2[ m_time_log_counter ] = m_kdtree_incremental_time;
        m_s_plot3[ m_time_log_counter ] = m_kdtree_search_time / m_kdtree_search_counter;
        m_s_plot4[ m_time_log_counter ] = featsFromMapNum;
        m_s_plot5[ m_time_log_counter ] = t5 - t0;

        m_time_log_counter++;
        // printf( "[ mapping ]: time: fov_check %0.6f fov_check and readd: %0.6f "
        //         "match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: "
        //         "%0.6f icp: %0.6f construct H: %0.6f \n",
        //         m_fov_check_time, t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp,
        //         aver_time_const_H_time );
        if ( m_lidar_en )
        {
            m_euler_cur = RotMtoEuler( state.rot_end );
            // #ifdef USE_IKFOM
            //             fout_out << setw( 20 ) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            //                      << state_point.pos.transpose() << " " << state_point.vel.transpose() << " " << state_point.bg.transpose() << " "
            //                      << state_point.ba.transpose() << " " << state_point.grav << " " << feats_undistort->points.size() << endl;
            // #else
            //             m_fout_out << setw( 20 ) << m_Lidar_Measures.last_update_time - m_first_lidar_time << " " << m_euler_cur.transpose() * 57.3
            //             << " "
            //                        << state.pos_end.transpose() << " " << state.vel_end.transpose() << " " << state.bias_g.transpose() << " "
            //                        << state.bias_a.transpose() << " " << state.gravity.transpose() << " " << m_feats_undistort->points.size() <<
            //                        endl;
            // #endif
        }
    }

    // #endif
    return 0;
}
