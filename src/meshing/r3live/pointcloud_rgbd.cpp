/*
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored,
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored,
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package."
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping."
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

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

#include "pointcloud_rgbd.hpp"
#include "../optical_flow/lkpyramid.hpp"
extern Common_tools::Cost_time_logger              g_cost_time_logger;
extern std::shared_ptr< Common_tools::ThreadPool > m_thread_pool_ptr;
cv::RNG                                            g_rng = cv::RNG( 0 );
// std::atomic<long> g_pts_index(0);
double                     g_voxel_resolution = 0.1;
double                     g_global_map_minimum_dis = 0.01;
std::vector< RGB_pt_ptr >* g_rgb_pts_vec;
class Triangle;
void RGB_pts::set_pos( const vec_3& pos )
{
    for ( size_t i = 0; i < 3; i++ )
    {
        m_pos[ i ] = pos( i );
        m_pos_aft_smooth[ i ] = pos( i );
    }
}

void RGB_pts::set_smooth_pos( const vec_3& pos )
{
    for ( size_t i = 0; i < 3; i++ )
    {
        m_pos_aft_smooth[ i ] = pos( i );
    }
    m_smoothed = true;
}

vec_3 RGB_pts::get_pos( bool get_smooth )
{
    if ( get_smooth )
    {
        return vec_3( m_pos_aft_smooth[ 0 ], m_pos_aft_smooth[ 1 ], m_pos_aft_smooth[ 2 ] );
    }
    else
    {
        return vec_3( m_pos[ 0 ], m_pos[ 1 ], m_pos[ 2 ] );
    }
}

mat_3_3 RGB_pts::get_rgb_cov()
{
    mat_3_3 cov_mat = mat_3_3::Zero();
    for ( int i = 0; i < 3; i++ )
    {
        cov_mat( i, i ) = m_cov_rgb[ i ];
    }
    return cov_mat;
}

vec_3 RGB_pts::get_rgb()
{
    return vec_3( m_rgb[ 0 ], m_rgb[ 1 ], m_rgb[ 2 ] ) / m_first_obs_exposure_time;
}

vec_3 RGB_pts::get_radiance()
{
    return vec_3( m_rgb[ 0 ], m_rgb[ 1 ], m_rgb[ 2 ] );
}

pcl::PointXYZI RGB_pts::get_pt()
{
    pcl::PointXYZI pt;
    pt.x = m_pos[ 0 ];
    pt.y = m_pos[ 1 ];
    pt.z = m_pos[ 2 ];
    return pt;
}

const double image_obs_cov = 1.5;
// const double process_noise_sigma = 1.5;
const double process_noise_sigma = 0.15;
// const double process_noise_sigma = 0.000;
// const double process_noise_sigma = 150000;
// const double process_noise_sigma = 0.0;
const int THRESHOLD_OVEREXPOSURE = 255;
int RGB_pts::update_rgb( const vec_3& rgb, const double obs_dis, const vec_3 obs_sigma, const double obs_time, const double current_exposure_time )
{
    if ( rgb.norm() == 0 ) // avoid less-exposure
    {
        return 0;
    }

    if ( rgb( 0 ) > THRESHOLD_OVEREXPOSURE && rgb( 1 ) > THRESHOLD_OVEREXPOSURE && rgb( 2 ) > THRESHOLD_OVEREXPOSURE ) // avoid the over-exposure
    {
        return 0;
    }

    if ( m_obs_dis != 0 && ( ( obs_dis > m_obs_dis * 1.1 ) ) )
    {
        return 0;
    }

    if ( m_N_rgb == 0 )
    {
        // For first time of observation.
        m_last_obs_time = obs_time;
        m_obs_dis = obs_dis;
        m_first_obs_exposure_time = current_exposure_time;
        for ( int i = 0; i < 3; i++ )
        {
            m_rgb[ i ] = rgb( i ) * current_exposure_time;
            m_cov_rgb[ i ] = obs_sigma( i );
        }
        m_N_rgb = 1;
        return 0;
    }

    // State estimation for robotics, section 2.2.6, page 37-38
    for ( int i = 0; i < 3; i++ )
    {
        m_cov_rgb[ i ] = ( m_cov_rgb[ i ] + process_noise_sigma * ( obs_time - m_last_obs_time ) ); // Add process noise
        double old_sigma = m_cov_rgb[ i ];
        m_cov_rgb[ i ] = sqrt( 1.0 / ( 1.0 / m_cov_rgb[ i ] / m_cov_rgb[ i ] + 1.0 / obs_sigma( i ) / obs_sigma( i ) ) );
        m_rgb[ i ] = m_cov_rgb[ i ] * m_cov_rgb[ i ] *
                     ( m_rgb[ i ] / old_sigma / old_sigma + rgb( i ) * current_exposure_time / obs_sigma( i ) / obs_sigma( i ) );
    }

    vec_3  res_rgb_vec = vec_3( m_rgb[ 0 ], m_rgb[ 1 ], m_rgb[ 2 ] ) / m_first_obs_exposure_time;
    double max_rgb = res_rgb_vec.maxCoeff(); // Avoid overexposure.
    if ( max_rgb > 255 )
    {
        for ( int i = 0; i < 3; i++ )
        {
            m_rgb[ i ] = m_rgb[ i ] * 254.999 / max_rgb;
        }
    }

    // if(m_first_obs_exposure_time > 1.0 / g_camera_exp_tim_lower_bound)
    // {
    //     m_first_obs_exposure_time = 1.0 / g_camera_exp_tim_lower_bound;
    // }

    if ( obs_dis < m_obs_dis )
    {
        m_obs_dis = obs_dis;
    }
    m_last_obs_time = obs_time;
    m_N_rgb++;

    //  if ( m_first_obs_exposure_time <= current_exposure_time )
    // {
    //     m_first_obs_exposure_time = current_exposure_time;
    // }
    m_first_obs_exposure_time = ( m_first_obs_exposure_time * ( m_N_rgb ) + current_exposure_time ) / ( m_N_rgb + 1 );
    return 1;
}

// void RGB_Voxel::refresh_triangles()
// {

//     int pts_size = m_pts_in_grid.size();
//     m_2d_pts_vec.resize( m_triangle_list_in_voxel.size() );
//     // for(int i = 0; i < pts_size; i++ )
//     int tri_idx = 0;
//     for ( Triangle_set::iterator it = m_triangle_list_in_voxel.begin(); it != m_triangle_list_in_voxel.end(); it++ )
//     {
//         vec_3 pt_3d_a = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 0 ] ]->get_pos();
//         vec_3 pt_3d_b = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 1 ] ]->get_pos();
//         vec_3 pt_3d_c = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 2 ] ]->get_pos();
//         m_2d_pts_vec[ tri_idx ] = Common_tools::Triangle_2( Common_tools::Point_2( pt_3d_a.dot( m_long_axis ), pt_3d_a.dot( m_mid_axis ) ),
//                                                             Common_tools::Point_2( pt_3d_b.dot( m_long_axis ), pt_3d_b.dot( m_mid_axis ) ),
//                                                             Common_tools::Point_2( pt_3d_c.dot( m_long_axis ), pt_3d_c.dot( m_mid_axis ) ) );
//         tri_idx++;
//     }
// }

// int RGB_Voxel::insert_triangle( long& id_0, long& id_1, long& id_2 )
// {
//     vec_3                    pt_3d_a = g_rgb_pts_vec->data()[ id_0 ]->get_pos() - m_center;
//     vec_3                    pt_3d_b = g_rgb_pts_vec->data()[ id_1 ]->get_pos() - m_center;
//     vec_3                    pt_3d_c = g_rgb_pts_vec->data()[ id_2 ]->get_pos() - m_center;
//     Common_tools::Triangle_2 tar_triangle =
//         Common_tools::Triangle_2( Common_tools::Point_2( pt_3d_a.dot( m_long_axis ), pt_3d_a.dot( m_mid_axis ) ),
//                                   Common_tools::Point_2( pt_3d_b.dot( m_long_axis ), pt_3d_b.dot( m_mid_axis ) ),
//                                   Common_tools::Point_2( pt_3d_c.dot( m_long_axis ), pt_3d_c.dot( m_mid_axis ) ) );
//     int res_intersection = 0;
//     for ( Triangle_set::iterator it = m_triangle_list_in_voxel.begin(); it != m_triangle_list_in_voxel.end(); it++ )
//     {
//         vec_3                    pt_3d_a = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 0 ] ]->get_pos() - m_center;
//         vec_3                    pt_3d_b = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 1 ] ]->get_pos() - m_center;
//         vec_3                    pt_3d_c = g_rgb_pts_vec->data()[ ( *it )->m_tri_pts_id[ 2 ] ]->get_pos() - m_center;
//         Common_tools::Triangle_2 test_triangle =
//             Common_tools::Triangle_2( Common_tools::Point_2( pt_3d_a.dot( m_long_axis ), pt_3d_a.dot( m_mid_axis ) ),
//                                       Common_tools::Point_2( pt_3d_b.dot( m_long_axis ), pt_3d_b.dot( m_mid_axis ) ),
//                                       Common_tools::Point_2( pt_3d_c.dot( m_long_axis ), pt_3d_c.dot( m_mid_axis ) ) );
//         // Common_tools::printf_triangle_pair(tar_triangle, test_triangle);

//         if ( ( res_intersection = Common_tools::triangle_intersect_triangle( tar_triangle, test_triangle, 0.01 ) ) >= 4 )
//         {
//             // cout << "Axis = " << m_long_axis.transpose() << ", " << m_mid_axis.transpose() << ", " << endl;
//             scope_color( ANSI_COLOR_RED_BOLD );
//             Common_tools::printf_triangle_pair( tar_triangle, test_triangle );
//             cout << " is conflict! type = " << res_intersection - 4 << endl;
//             return 1;
//         }
//     }
//     Triangle_ptr triangle_ptr = std::make_shared< Triangle >( id_0, id_1, id_2 );
//     m_triangle_list_in_voxel.insert( triangle_ptr );
//     return 0;
// }

void Global_map::clear()
{
    m_rgb_pts_vec.clear();
}

void Global_map::set_minimum_dis( double minimum_dis )
{
    m_hashmap_3d_pts.clear();
    m_minimum_pts_size = minimum_dis;
    g_global_map_minimum_dis = minimum_dis;
}

void Global_map::set_voxel_resolution( double resolution )
{
    m_voxel_resolution = resolution;
    g_voxel_resolution = resolution;
}

Global_map::Global_map( int if_start_service )
{
    m_mutex_pts_vec = std::make_shared< std::mutex >();
    m_mutex_img_pose_for_projection = std::make_shared< std::mutex >();
    m_mutex_recent_added_list = std::make_shared< std::mutex >();
    m_mutex_rgb_pts_in_recent_hitted_boxes = std::make_shared< std::mutex >();
    m_mutex_m_box_recent_hitted = std::make_shared< std::mutex >();
    m_mutex_pts_last_visited = std::make_shared< std::mutex >();
    // Allocate memory for pointclouds
    if ( Common_tools::get_total_phy_RAM_size_in_GB() < 12 )
    {
        scope_color( ANSI_COLOR_RED_BOLD );
        std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
        cout << "I have detected your physical memory smaller than 12GB (currently: " << Common_tools::get_total_phy_RAM_size_in_GB()
             << "GB). I recommend you to add more physical memory for improving the overall performance of R3LIVE." << endl;
        cout << "+++++++++++++++++++++++++++++++++++++++++++++++++++" << endl;
        std::this_thread::sleep_for( std::chrono::seconds( 5 ) );
        m_rgb_pts_vec.reserve( 1e8 );
        m_voxel_vec.reserve( 1e6 );
    }
    else
    {
        m_rgb_pts_vec.reserve( 1e9 );
        m_voxel_vec.reserve( 1e7 );
    }
    // m_rgb_pts_in_recent_visited_voxels.reserve( 1e6 );
    if ( if_start_service )
    {
        m_thread_service = std::make_shared< std::thread >( &Global_map::service_refresh_pts_for_projection, this );
    }
    g_rgb_pts_vec = &m_rgb_pts_vec;
}
Global_map::~Global_map(){};

void Global_map::service_refresh_pts_for_projection()
{
    eigen_q                        last_pose_q = eigen_q::Identity();
    Common_tools::Timer            timer;
    std::shared_ptr< Image_frame > img_for_projection = std::make_shared< Image_frame >();
    g_voxel_resolution = m_voxel_resolution;
    while ( 1 )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
        m_mutex_img_pose_for_projection->lock();

        *img_for_projection = m_img_for_projection;
        m_mutex_img_pose_for_projection->unlock();
        if ( img_for_projection->m_img_cols == 0 || img_for_projection->m_img_rows == 0 )
        {
            continue;
        }

        if ( img_for_projection->m_frame_idx == m_updated_frame_index )
        {
            continue;
        }
        timer.tic( " " );
        std::shared_ptr< std::vector< std::shared_ptr< RGB_pts > > > pts_rgb_vec_for_projection =
            std::make_shared< std::vector< std::shared_ptr< RGB_pts > > >();
        if ( m_if_get_all_pts_in_boxes_using_mp )
        {
            std::vector< std::shared_ptr< RGB_pts > > pts_in_recent_hitted_boxes;
            pts_in_recent_hitted_boxes.reserve( 1e6 );
            std::unordered_set< std::shared_ptr< RGB_Voxel > > boxes_recent_hitted;
            m_mutex_m_box_recent_hitted->lock();
            boxes_recent_hitted = m_voxels_recent_visited;
            m_mutex_m_box_recent_hitted->unlock();

            // get_all_pts_in_boxes(boxes_recent_hitted, pts_in_recent_hitted_boxes);
            m_mutex_rgb_pts_in_recent_hitted_boxes->lock();
            // m_rgb_pts_in_recent_visited_voxels = pts_in_recent_hitted_boxes;
            m_mutex_rgb_pts_in_recent_hitted_boxes->unlock();
        }
        selection_points_for_projection( img_for_projection, pts_rgb_vec_for_projection.get(), nullptr, 10.0, 1 );
        m_mutex_pts_vec->lock();
        m_pts_rgb_vec_for_projection = pts_rgb_vec_for_projection;
        m_updated_frame_index = img_for_projection->m_frame_idx;
        // cout << ANSI_COLOR_MAGENTA_BOLD << "Refresh pts_for_projection size = " << m_pts_rgb_vec_for_projection->size()
        //      << " | " << m_rgb_pts_vec.size()
        //      << ", cost time = " << timer.toc() << ANSI_COLOR_RESET << endl;
        m_mutex_pts_vec->unlock();
        last_pose_q = img_for_projection->m_pose_w2c_q;
    }
}

void Global_map::render_points_for_projection( std::shared_ptr< Image_frame >& img_ptr )
{
    m_mutex_pts_vec->lock();
    if ( m_pts_rgb_vec_for_projection != nullptr )
    {
        render_pts_in_voxels( img_ptr, *m_pts_rgb_vec_for_projection );
        // render_pts_in_voxels(img_ptr, m_rgb_pts_vec);
    }
    m_last_updated_frame_idx = img_ptr->m_frame_idx;
    m_mutex_pts_vec->unlock();
}

void Global_map::update_pose_for_projection( std::shared_ptr< Image_frame >& img, double fov_margin )
{
    m_mutex_img_pose_for_projection->lock();
    m_img_for_projection.set_intrinsic( img->m_cam_K );
    m_img_for_projection.m_img_cols = img->m_img_cols;
    m_img_for_projection.m_img_rows = img->m_img_rows;
    m_img_for_projection.m_fov_margin = fov_margin;
    m_img_for_projection.m_frame_idx = img->m_frame_idx;
    m_img_for_projection.m_pose_w2c_q = img->m_pose_w2c_q;
    m_img_for_projection.m_pose_w2c_t = img->m_pose_w2c_t;
    m_img_for_projection.m_img_gray = img->m_img_gray; // clone?
    m_img_for_projection.m_img = img->m_img;           // clone?
    m_img_for_projection.refresh_pose_for_projection();
    m_mutex_img_pose_for_projection->unlock();
}

bool Global_map::is_busy()
{
    return m_in_appending_pts;
}

template int Global_map::append_points_to_global_map< pcl::PointXYZI >( pcl::PointCloud< pcl::PointXYZI >& pc_in, double added_time,
                                                                        std::vector< std::shared_ptr< RGB_pts > >* pts_added_vec, int step,
                                                                        int disable_append );
template int Global_map::append_points_to_global_map< pcl::PointXYZRGB >( pcl::PointCloud< pcl::PointXYZRGB >& pc_in, double added_time,
                                                                          std::vector< std::shared_ptr< RGB_pts > >* pts_added_vec, int step,
                                                                          int disable_append );

vec_3 g_current_lidar_position;

std::vector< RGB_pt_ptr > retrieve_pts_in_voxels( std::unordered_set< std::shared_ptr< RGB_Voxel > >& neighbor_voxels )
{
    std::vector< RGB_pt_ptr > RGB_pt_ptr_vec;
    for ( std::unordered_set< std::shared_ptr< RGB_Voxel > >::iterator it = neighbor_voxels.begin(); it != neighbor_voxels.end(); it++ )
    {
        auto it_s = ( *it )->m_pts_in_grid.begin();
        auto it_e = ( *it )->m_pts_in_grid.end();
        RGB_pt_ptr_vec.insert( RGB_pt_ptr_vec.end(), it_s, it_e );
    }
    return RGB_pt_ptr_vec;
}

std::unordered_set< std::shared_ptr< RGB_Voxel > > voxels_recent_visited;

template < typename T >
int Global_map::append_points_to_global_map( pcl::PointCloud< T >& pc_in, double added_time, std::vector< std::shared_ptr< RGB_pts > >* pts_added_vec,
                                             int step, int disable_append )
{
    m_in_appending_pts = 1;
    Common_tools::Timer tim;
    tim.tic();
    int acc = 0;
    int rej = 0;
    if ( pts_added_vec != nullptr )
    {
        pts_added_vec->clear();
    }

    if ( m_recent_visited_voxel_activated_time == 0 )
    {
        voxels_recent_visited.clear();
    }
    else
    {
        m_mutex_m_box_recent_hitted->lock();
        std::swap( voxels_recent_visited, m_voxels_recent_visited );
        m_mutex_m_box_recent_hitted->unlock();
        for ( Voxel_set_iterator it = voxels_recent_visited.begin(); it != voxels_recent_visited.end(); )
        {

            if ( added_time - ( *it )->m_last_visited_time > m_recent_visited_voxel_activated_time )
            {
                it = voxels_recent_visited.erase( it );
                continue;
            }
            if ( ( *it )->m_pts_in_grid.size() )
            {
                double voxel_dis = ( g_current_lidar_position - vec_3( ( *it )->m_pts_in_grid[ 0 ]->get_pos() ) ).norm();
                // if ( voxel_dis > 30 )
                // {
                //     it = voxels_recent_visited.erase( it );
                //     continue;
                // }
            }

            it++;
        }
        // cout << "Restored voxel number = " << voxels_recent_visited.size() << endl;
    }
    int number_of_voxels_before_add = voxels_recent_visited.size();
    int pt_size = pc_in.points.size();
    // step = 4;

    KDtree_pt_vector     pt_vec_vec;
    std::vector< float > dist_vec;

    RGB_voxel_ptr* temp_box_ptr_ptr;
    for ( long pt_idx = 0; pt_idx < pt_size; pt_idx += step )
    {
        int  add = 1;
        int  grid_x = std::round( pc_in.points[ pt_idx ].x / m_minimum_pts_size );
        int  grid_y = std::round( pc_in.points[ pt_idx ].y / m_minimum_pts_size );
        int  grid_z = std::round( pc_in.points[ pt_idx ].z / m_minimum_pts_size );
        int  box_x = std::round( pc_in.points[ pt_idx ].x / m_voxel_resolution );
        int  box_y = std::round( pc_in.points[ pt_idx ].y / m_voxel_resolution );
        int  box_z = std::round( pc_in.points[ pt_idx ].z / m_voxel_resolution );
        auto pt_ptr = m_hashmap_3d_pts.get_data( grid_x, grid_y, grid_z );
        if ( pt_ptr != nullptr )
        {
            add = 0;
            if ( pts_added_vec != nullptr )
            {
                pts_added_vec->push_back( *pt_ptr );
            }
        }
        RGB_voxel_ptr box_ptr;
        temp_box_ptr_ptr = m_hashmap_voxels.get_data( box_x, box_y, box_z );
        if ( temp_box_ptr_ptr == nullptr )
        {
            box_ptr = std::make_shared< RGB_Voxel >( box_x, box_y, box_z );
            m_hashmap_voxels.insert( box_x, box_y, box_z, box_ptr );
            m_voxel_vec.push_back( box_ptr );
        }
        else
        {
            box_ptr = *temp_box_ptr_ptr;
        }
        voxels_recent_visited.insert( box_ptr );
        box_ptr->m_last_visited_time = added_time;
        if ( add == 0 )
        {
            rej++;
            continue;
        }
        if ( disable_append )
        {
            continue;
        }
        acc++;
        KDtree_pt kdtree_pt( vec_3( pc_in.points[ pt_idx ].x, pc_in.points[ pt_idx ].y, pc_in.points[ pt_idx ].z ), 0 );
        if ( m_kdtree.Root_Node != nullptr )
        {
            m_kdtree.Nearest_Search( kdtree_pt, 1, pt_vec_vec, dist_vec );
            if ( pt_vec_vec.size() )
            {
                if ( sqrt( dist_vec[ 0 ] ) < m_minimum_pts_size )
                {
                    continue;
                }
            }
        }
        std::shared_ptr< RGB_pts > pt_rgb = std::make_shared< RGB_pts >();
        pt_rgb->set_pos( vec_3( pc_in.points[ pt_idx ].x, pc_in.points[ pt_idx ].y, pc_in.points[ pt_idx ].z ) );
        pt_rgb->m_pt_index = m_rgb_pts_vec.size();
        kdtree_pt.m_pt_idx = pt_rgb->m_pt_index;
        m_rgb_pts_vec.push_back( pt_rgb );
        m_hashmap_3d_pts.insert( grid_x, grid_y, grid_z, pt_rgb );
        if ( box_ptr != nullptr )
        {
            box_ptr->m_pts_in_grid.push_back( pt_rgb );
            // box_ptr->add_pt(pt_rgb);
            box_ptr->m_new_added_pts_count++;
            box_ptr->m_meshing_times = 0;
        }
        else
        {
            scope_color( ANSI_COLOR_RED_BOLD );
            for ( int i = 0; i < 100; i++ )
            {
                cout << "box_ptr is nullptr!!!" << endl;
            }
        }
        // Add to kdtree
        m_kdtree.Add_Point( kdtree_pt, false );
        if ( pts_added_vec != nullptr )
        {
            pts_added_vec->push_back( pt_rgb );
        }
    }
    m_in_appending_pts = 0;
    m_mutex_m_box_recent_hitted->lock();
    std::swap( m_voxels_recent_visited, voxels_recent_visited );
    // m_voxels_recent_visited = voxels_recent_visited ;
    m_mutex_m_box_recent_hitted->unlock();
    return ( m_voxels_recent_visited.size() - number_of_voxels_before_add );
}

void Global_map::render_pts_in_voxels( std::shared_ptr< Image_frame >& img_ptr, std::vector< std::shared_ptr< RGB_pts > >& pts_for_render,
                                       double obs_time )
{
    Common_tools::Timer tim;
    tim.tic();
    double u, v;
    int    hit_count = 0;
    int    pt_size = pts_for_render.size();
    m_last_updated_frame_idx = img_ptr->m_frame_idx;
    double min_voxel_dis = 3e8;
    for ( int i = 0; i < pt_size; i++ )
    {
        double pt_cam_dis = ( pts_for_render[ i ]->get_pos() - img_ptr->m_pose_w2c_t ).dot( img_ptr->m_image_norm );
        if ( pt_cam_dis < min_voxel_dis )
        {
            min_voxel_dis = pt_cam_dis;
        }
    }
    double allow_render_dis = std::max( 0.05, g_voxel_resolution * 0.1 );
    for ( int i = 0; i < pt_size; i++ )
    {

        vec_3  pt_w = pts_for_render[ i ]->get_pos();
        double pt_cam_dis = ( pt_w - img_ptr->m_pose_w2c_t ).dot( img_ptr->m_image_norm );
        ;
        if ( ( pt_cam_dis - min_voxel_dis > allow_render_dis ) && ( pts_for_render[ i ]->m_N_rgb > 5 ) )
        {
            continue;
        }
        bool res = img_ptr->project_3d_point_in_this_img( pt_w, u, v, nullptr, 1.0 );
        if ( res == false )
        {
            continue;
        }
        hit_count++;
        vec_2 gama_bak = img_ptr->m_gama_para;
        img_ptr->m_gama_para = vec_2( 1.0, 0.0 ); // Render using normal value?
        double gray = img_ptr->get_grey_color( u, v, 0 );
        vec_3  rgb_color = img_ptr->get_rgb( u, v, 0 );
        if ( rgb_color.maxCoeff() > 255.0 )
        {
            cout << ANSI_COLOR_RED << "Error, render RGB = " << rgb_color.transpose() << ANSI_COLOR_RESET << endl;
        }
        // pts_for_render[i]->update_gray(gray, pt_cam.norm());
        pts_for_render[ i ]->update_rgb( rgb_color, pt_cam_dis, vec_3( image_obs_cov, image_obs_cov, image_obs_cov ), obs_time,
                                         img_ptr->m_image_inverse_exposure_time );
        img_ptr->m_gama_para = gama_bak;
        // m_rgb_pts_vec[i]->update_rgb( vec_3(gray, gray, gray) );
    }
    // cout << "Render cost time = " << tim.toc() << endl;
    // cout << "Total hit count = " << hit_count << endl;
}

Common_tools::Cost_time_logger cost_time_logger_render( "/home/ziv/temp/render_thr.log" );

// SECTION recored average photometric error
// ANCHOR - thread_render_pts_in_voxel
std::atomic< long >  render_pts_count;
extern double        g_maximum_pe_error;
static inline double thread_render_pts_in_voxel( const int& pt_start, const int& pt_end, const std::shared_ptr< Image_frame >& img_ptr,
                                                 const std::vector< RGB_voxel_ptr >* voxels_for_render, const double obs_time )
{
    vec_3               pt_w;
    vec_3               rgb_color;
    double              u, v;
    double              pt_cam_norm;
    Common_tools::Timer tim;
    tim.tic();
    vec_3  pt_radiance;
    double allow_render_dis = std::max( 0.1, g_voxel_resolution * 0.2 );
    for ( int voxel_idx = pt_start; voxel_idx < pt_end; voxel_idx++ )
    {
        // continue;
        RGB_voxel_ptr voxel_ptr = ( *voxels_for_render )[ voxel_idx ];
        double        min_voxel_dis = 3e8;
        // for ( int i = 0; i < voxel_ptr->m_pts_in_grid.size(); i++ )
        // {
        //     double pt_cam_dis = ( voxel_ptr->m_pts_in_grid[i]->get_pos() - img_ptr->m_pose_w2c_t ).dot( img_ptr->m_image_norm );
        //     // double pt_cam_dis = ( voxel_ptr->m_pts_in_grid[i]->get_pos() - img_ptr->m_pose_w2c_t ).norm();
        //     if ( pt_cam_dis < min_voxel_dis )
        //     {
        //         min_voxel_dis = pt_cam_dis;
        //     }
        // }
        for ( int pt_idx = 0; pt_idx < voxel_ptr->m_pts_in_grid.size(); pt_idx++ )
        {
            pt_w = voxel_ptr->m_pts_in_grid[ pt_idx ]->get_pos();
            vec_3  pt_cam_view_vector = pt_w - img_ptr->m_pose_w2c_t;
            double view_dis = pt_cam_view_vector.norm();
            double view_angle = acos( pt_cam_view_vector.dot( img_ptr->m_image_norm ) / ( view_dis + 0.0001 ) ) * 57.3;
            view_angle = std::max( view_angle, 5.0 );
            view_dis = std::max( view_dis, 1.0 );
            if ( view_angle > 30 )
            {
                continue;
            }
            if ( img_ptr->project_3d_point_in_this_img( pt_w, u, v, nullptr, 1.0 ) == false )
            {
                continue;
            }

            vec_3 obs_cov =
                vec_3( image_obs_cov * view_dis * view_angle, image_obs_cov * view_dis * view_angle, image_obs_cov * view_dis * view_angle );
            rgb_color = img_ptr->get_rgb( u, v, 0 );
            if ( voxel_ptr->m_pts_in_grid[ pt_idx ]->update_rgb( rgb_color, view_dis, obs_cov, obs_time, img_ptr->m_image_inverse_exposure_time ) )
            {
                render_pts_count++;
                if ( voxel_ptr->m_pts_in_grid[ pt_idx ]->get_rgb().maxCoeff() > 254 )
                {
                    continue;
                }
                pt_radiance = voxel_ptr->m_pts_in_grid[ pt_idx ]->get_radiance();
                pt_radiance = ( pt_radiance / img_ptr->m_image_inverse_exposure_time );
                if ( pt_radiance.maxCoeff() > 245.0 )
                {
                    continue;
                }
                // double brightness_err = fabs(rgb_color.mean() - pt_radiance.mean());
                double brightness_err = fabs( rgb_color.norm() - pt_radiance.norm() );
                if ( brightness_err > g_maximum_pe_error )
                {
                    brightness_err = g_maximum_pe_error;
                    // continue;
                }
                img_ptr->m_acc_render_count++;
                img_ptr->m_acc_photometric_error = img_ptr->m_acc_photometric_error + brightness_err;
                // img_ptr->m_acc_photometric_error += std::atomic<double>((pt_color- rgb_color).norm());
            }
        }
    }
    double cost_time = tim.toc() * 100;
    return cost_time;
}

std::vector< RGB_voxel_ptr > g_voxel_for_render;
FILE*                        photometric_fp = nullptr;
void render_pts_in_voxels_mp( std::shared_ptr< Image_frame >& img_ptr, std::unordered_set< RGB_voxel_ptr >* _voxels_for_render,
                              const double& obs_time )
{
    Common_tools::Timer tim;
    g_voxel_for_render.clear();
    for ( Voxel_set_iterator it = ( *_voxels_for_render ).begin(); it != ( *_voxels_for_render ).end(); it++ )
    {
        g_voxel_for_render.push_back( *it );
    }
    std::vector< std::future< double > > results;
    tim.tic( "Render_mp" );
    int numbers_of_voxels = g_voxel_for_render.size();
    g_cost_time_logger.record( "Pts_num", numbers_of_voxels );
    render_pts_count = 0;
    img_ptr->m_acc_render_count = 0;
    img_ptr->m_acc_photometric_error = 0;
    if ( USING_OPENCV_TBB )
    {
        cv::parallel_for_( cv::Range( 0, numbers_of_voxels ),
                           [&]( const cv::Range& r ) { thread_render_pts_in_voxel( r.start, r.end, img_ptr, &g_voxel_for_render, obs_time ); } );
    }
    else
    {
        int num_of_threads = std::min( 8 * 2, ( int ) numbers_of_voxels );
        // results.clear();
        results.resize( num_of_threads );
        tim.tic( "Com" );
        for ( int thr = 0; thr < num_of_threads; thr++ )
        {
            // cv::Range range(thr * pt_size / num_of_threads, (thr + 1) * pt_size / num_of_threads);
            int start = thr * numbers_of_voxels / num_of_threads;
            int end = ( thr + 1 ) * numbers_of_voxels / num_of_threads;
            results[ thr ] = m_thread_pool_ptr->commit_task( thread_render_pts_in_voxel, start, end, img_ptr, &g_voxel_for_render, obs_time );
        }
        g_cost_time_logger.record( tim, "Com" );
        tim.tic( "wait_Opm" );
        for ( int thr = 0; thr < num_of_threads; thr++ )
        {
            double cost_time = results[ thr ].get();
            cost_time_logger_render.record( std::string( "T_" ).append( std::to_string( thr ) ), cost_time );
        }
        g_cost_time_logger.record( tim, "wait_Opm" );
        cost_time_logger_render.record( tim, "wait_Opm" );
    }
    // ANCHOR - record photometric error
    // printf( "Image frame = %d, count = %d, acc_PT = %.3f, avr_PE = %.3f\r\n", img_ptr->m_frame_idx, long( img_ptr->m_acc_render_count ),
    //         double( img_ptr->m_acc_photometric_error), double( img_ptr->m_acc_photometric_error) / long(img_ptr->m_acc_render_count ) );
    if ( photometric_fp == nullptr )
    {
        photometric_fp = fopen( std::string( Common_tools::get_home_folder().c_str() ).append( "/r3live_output/photometric.log" ).c_str(), "w+" );
    }
    if ( long( img_ptr->m_acc_render_count ) != 0 )
    {
        fprintf( photometric_fp, "%f %d %d %f %f\r\n", img_ptr->m_timestamp, img_ptr->m_frame_idx, long( img_ptr->m_acc_render_count ),
                 double( img_ptr->m_acc_photometric_error ) / long( img_ptr->m_acc_render_count ), double( img_ptr->m_acc_photometric_error ) );
        fflush( photometric_fp );
    }
    // img_ptr->release_image();
    cost_time_logger_render.flush_d();
    g_cost_time_logger.record( tim, "Render_mp" );
    g_cost_time_logger.record( "Pts_num_r", render_pts_count );
}
//! SECTION

void Global_map::render_with_a_image( std::shared_ptr< Image_frame >& img_ptr, int if_select )
{

    std::vector< std::shared_ptr< RGB_pts > > pts_for_render;
    // pts_for_render = m_rgb_pts_vec;
    if ( if_select )
    {
        selection_points_for_projection( img_ptr, &pts_for_render, nullptr, 1.0 );
    }
    else
    {
        pts_for_render = m_rgb_pts_vec;
    }
    render_pts_in_voxels( img_ptr, pts_for_render );
}

void Global_map::selection_points_for_projection( std::shared_ptr< Image_frame >& image_pose, std::vector< std::shared_ptr< RGB_pts > >* pc_out_vec,
                                                  std::vector< cv::Point2f >* pc_2d_out_vec, double minimum_dis, int skip_step, int use_all_pts )
{
    Common_tools::Timer tim;
    tim.tic();
    if ( pc_out_vec != nullptr )
    {
        pc_out_vec->clear();
    }
    if ( pc_2d_out_vec != nullptr )
    {
        pc_2d_out_vec->clear();
    }
    Hash_map_2d< int, int >   mask_index;
    Hash_map_2d< int, float > mask_depth;

    std::map< int, cv::Point2f > map_idx_draw_center;
    std::map< int, cv::Point2f > map_idx_draw_center_raw_pose;

    int    u, v;
    double u_f, v_f;
    // cv::namedWindow("Mask", cv::WINDOW_FREERATIO);
    int acc = 0;
    int blk_rej = 0;
    // int pts_size = m_rgb_pts_vec.size();
    std::vector< std::shared_ptr< RGB_pts > > pts_for_projection;
    m_mutex_m_box_recent_hitted->lock();
    std::unordered_set< std::shared_ptr< RGB_Voxel > > boxes_recent_hitted = m_voxels_recent_visited;
    m_mutex_m_box_recent_hitted->unlock();
    if ( ( !use_all_pts ) && boxes_recent_hitted.size() )
    {
        m_mutex_rgb_pts_in_recent_hitted_boxes->lock();

        for ( Voxel_set_iterator it = boxes_recent_hitted.begin(); it != boxes_recent_hitted.end(); it++ )
        {
            // pts_for_projection.push_back( (*it)->m_pts_in_grid.back() );
            if ( ( *it )->m_pts_in_grid.size() )
            {
                //  pts_for_projection.push_back( (*it)->m_pts_in_grid.back() );
                pts_for_projection.push_back( ( *it )->m_pts_in_grid[ 0 ] );
                // pts_for_projection.push_back( ( *it )->m_pts_in_grid[ ( *it )->m_pts_in_grid.size()-1 ] );
            }
        }

        m_mutex_rgb_pts_in_recent_hitted_boxes->unlock();
    }
    else
    {
        pts_for_projection = m_rgb_pts_vec;
    }
    int pts_size = pts_for_projection.size();
    for ( int pt_idx = 0; pt_idx < pts_size; pt_idx += skip_step )
    {
        vec_3  pt = pts_for_projection[ pt_idx ]->get_pos();
        double depth = ( pt - image_pose->m_pose_w2c_t ).norm();
        if ( depth > m_maximum_depth_for_projection )
        {
            continue;
        }
        if ( depth < m_minimum_depth_for_projection )
        {
            continue;
        }
        bool res = image_pose->project_3d_point_in_this_img( pt, u_f, v_f, nullptr, 1.0 );
        if ( res == false )
        {
            continue;
        }
        u = std::round( u_f / minimum_dis ) * minimum_dis; // Why can not work
        v = std::round( v_f / minimum_dis ) * minimum_dis;
        if ( ( !mask_depth.if_exist( u, v ) ) || mask_depth.m_map_2d_hash_map[ u ][ v ] > depth )
        {
            acc++;
            if ( mask_index.if_exist( u, v ) )
            {
                // erase old point
                int old_idx = mask_index.m_map_2d_hash_map[ u ][ v ];
                blk_rej++;
                map_idx_draw_center.erase( map_idx_draw_center.find( old_idx ) );
                map_idx_draw_center_raw_pose.erase( map_idx_draw_center_raw_pose.find( old_idx ) );
            }
            mask_index.m_map_2d_hash_map[ u ][ v ] = ( int ) pt_idx;
            mask_depth.m_map_2d_hash_map[ u ][ v ] = ( float ) depth;
            map_idx_draw_center[ pt_idx ] = cv::Point2f( v, u );
            map_idx_draw_center_raw_pose[ pt_idx ] = cv::Point2f( u_f, v_f );
        }
    }

    if ( pc_out_vec != nullptr )
    {
        for ( auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++ )
        {
            // pc_out_vec->push_back(m_rgb_pts_vec[it->first]);
            pc_out_vec->push_back( pts_for_projection[ it->first ] );
        }
    }

    if ( pc_2d_out_vec != nullptr )
    {
        for ( auto it = map_idx_draw_center.begin(); it != map_idx_draw_center.end(); it++ )
        {
            pc_2d_out_vec->push_back( map_idx_draw_center_raw_pose[ it->first ] );
        }
    }
}

void Global_map::save_to_pcd( std::string dir_name, std::string _file_name, int save_pts_with_views )
{
    Common_tools::Timer tim;
    Common_tools::create_dir( dir_name );
    std::string file_name = std::string( dir_name ).append( _file_name );
    scope_color( ANSI_COLOR_BLUE_BOLD );
    cout << "Save Rgb points to " << file_name << endl;
    fflush( stdout );
    pcl::PointCloud< pcl::PointXYZRGB > pc_rgb;
    long                                pt_size = m_rgb_pts_vec.size();
    pc_rgb.resize( pt_size );
    long pt_count = 0;
    for ( long i = pt_size - 1; i > 0; i-- )
    // for (int i = 0; i  <  pt_size; i++)
    {
        if ( i % 1000 == 0 )
        {
            cout << ANSI_DELETE_CURRENT_LINE << "Saving offline map " << ( int ) ( ( pt_size - 1 - i ) * 100.0 / ( pt_size - 1 ) ) << " % ...";
            fflush( stdout );
        }

        if ( m_rgb_pts_vec[ i ]->m_N_rgb < save_pts_with_views )
        {
            continue;
        }
        pcl::PointXYZRGB pt;
        vec_3            pt_rgb = m_rgb_pts_vec[ i ]->get_rgb();
        pc_rgb.points[ pt_count ].x = m_rgb_pts_vec[ i ]->m_pos[ 0 ];
        pc_rgb.points[ pt_count ].y = m_rgb_pts_vec[ i ]->m_pos[ 1 ];
        pc_rgb.points[ pt_count ].z = m_rgb_pts_vec[ i ]->m_pos[ 2 ];

        pc_rgb.points[ pt_count ].r = pt_rgb( 2 );
        pc_rgb.points[ pt_count ].g = pt_rgb( 1 );
        pc_rgb.points[ pt_count ].b = pt_rgb( 0 );
        pt_count++;
    }
    cout << ANSI_DELETE_CURRENT_LINE << "Saving offline map 100% ..." << endl;
    pc_rgb.resize( pt_count );
    cout << "Total have " << pt_count << " points." << endl;
    tim.tic();
    cout << "Now write to: " << file_name << endl;
    pcl::io::savePCDFileBinary( std::string( file_name ).append( ".pcd" ), pc_rgb );
    cout << "Save PCD cost time = " << tim.toc() << endl;
}

void Global_map::save_and_display_pointcloud( std::string dir_name, std::string file_name, int save_pts_with_views )
{
    save_to_pcd( dir_name, file_name, save_pts_with_views );
    scope_color( ANSI_COLOR_WHITE_BOLD );
    cout << "========================================================" << endl;
    cout << "Open pcl_viewer to display point cloud, close the viewer's window to continue mapping process ^_^" << endl;
    cout << "========================================================" << endl;
    // system(std::string("pcl_viewer ").append(dir_name).append("/rgb_pt.pcd").c_str());
    system( std::string( "r3live_viewer " ).append( dir_name ).append( "/rgb_pt.pcd" ).c_str() );
}

vec_3 Global_map::smooth_pts( RGB_pt_ptr& rgb_pt, double smooth_factor, double knn, double maximum_smooth_dis )
{
    std::vector< int >   indices;
    std::vector< float > distances;
    KDtree_pt_vector     kdtree_pt_vec;
    std::vector< float > search_pt_dis;
    vec_3 pt_vec = rgb_pt->get_pos();
    vec_3  pt_vec_neighbor = vec_3( 0, 0, 0 );
    if(maximum_smooth_dis <= 0)
    {
        maximum_smooth_dis = m_voxel_resolution * 0.8;
    }
    m_kdtree.Nearest_Search( KDtree_pt(pt_vec), knn,  kdtree_pt_vec, search_pt_dis  );
    double valid_size = 0.0;
     int size = search_pt_dis.size();
    for ( int k = 1; k < size; k++ )
    {
        if( sqrt(search_pt_dis[k]) < maximum_smooth_dis )
        {
            pt_vec_neighbor += m_rgb_pts_vec[ kdtree_pt_vec[ k ].m_pt_idx ]->get_pos();
            valid_size += 1.0;
        }
    }
    vec_3 pt_vec_smoothed = pt_vec * ( 1.0 - smooth_factor ) + pt_vec_neighbor * smooth_factor / valid_size;
    rgb_pt->set_smooth_pos(pt_vec_smoothed);
    return pt_vec_smoothed;
}