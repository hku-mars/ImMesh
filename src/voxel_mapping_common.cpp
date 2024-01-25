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

void Voxel_mapping::kitti_log( FILE *fp )
{
    // Eigen::Matrix4d T_lidar_to_cam;
    // T_lidar_to_cam << 0.00042768, -0.999967, -0.0080845, -0.01198, -0.00721062,
    //     0.0080811998, -0.99994131, -0.0540398, 0.999973864, 0.00048594,
    //     -0.0072069, -0.292196, 0, 0, 0, 1.0;
    double          time_stamp = 0;
    Eigen::Matrix4d T_lidar_to_cam;
    T_lidar_to_cam << 0.00554604, -0.999971, -0.00523653, 0.0316362, -0.000379382, 0.00523451, -0.999986, 0.0380934, 0.999985, 0.00554795,
        -0.000350341, 0.409066, 0, 0, 0, 1;

    // V3D rot_ang(Log(state.rot_end));
    MD( 4, 4 ) T;
    T.block< 3, 3 >( 0, 0 ) = state.rot_end;
    T.block< 3, 1 >( 0, 3 ) = state.pos_end;
    T( 3, 0 ) = 0;
    T( 3, 1 ) = 0;
    T( 3, 2 ) = 0;
    T( 3, 3 ) = 1;
    T = T_lidar_to_cam * T * T_lidar_to_cam.inverse();

    Eigen::Matrix3d    camera_rotation = T.block< 3, 3 >( 0, 0 );
    Eigen::Vector3d    camera_translation = T.block< 3, 1 >( 0, 3 );
    Eigen::Quaterniond q( camera_rotation );
    fprintf( fp, "%lf %lf %lf %lf %lf %lf %lf %lf\n", m_last_timestamp_lidar, camera_translation[ 0 ], camera_translation[ 1 ],
             camera_translation[ 2 ], q.x(), q.y(), q.z(), q.w() );
    fflush( fp );
}

void Voxel_mapping::SigHandle( int sig )
{
    m_flg_exit = true;
    ROS_WARN( "catch sig %d", sig );
    m_sig_buffer.notify_all();
}

void Voxel_mapping::dump_lio_state_to_log( FILE *fp )
{
#ifdef USE_IKFOM
    // state_ikfom write_state = kf.get_x();
    V3D rot_ang( Log( state_point.rot.toRotationMatrix() ) );
    fprintf( fp, "%lf ", LidarMeasures.lidar_beg_time - first_lidar_time );
    fprintf( fp, "%lf %lf %lf ", rot_ang( 0 ), rot_ang( 1 ), rot_ang( 2 ) ); // Angle
    fprintf( fp, "%lf %lf %lf ", state_point.pos( 0 ), state_point.pos( 1 ),
             state_point.pos( 2 ) );              // Pos
    fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // omega
    fprintf( fp, "%lf %lf %lf ", state_point.vel( 0 ), state_point.vel( 1 ),
             state_point.vel( 2 ) );              // Vel
    fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // Acc
    fprintf( fp, "%lf %lf %lf ", state_point.bg( 0 ), state_point.bg( 1 ),
             state_point.bg( 2 ) ); // Bias_g
    fprintf( fp, "%lf %lf %lf ", state_point.ba( 0 ), state_point.ba( 1 ),
             state_point.ba( 2 ) ); // Bias_a
    fprintf( fp, "%lf %lf %lf ", state_point.grav[ 0 ], state_point.grav[ 1 ],
             state_point.grav[ 2 ] ); // Bias_a
    fprintf( fp, "\r\n" );
    fflush( fp );
#else
    V3D rot_ang( Log( state.rot_end ) );
    fprintf( fp, "%lf ", m_Lidar_Measures.lidar_beg_time - m_first_lidar_time );
    fprintf( fp, "%lf %lf %lf ", rot_ang( 0 ), rot_ang( 1 ), rot_ang( 2 ) ); // Angle
    fprintf( fp, "%lf %lf %lf ", state.pos_end( 0 ), state.pos_end( 1 ),
             state.pos_end( 2 ) );                // Pos
    fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // omega
    fprintf( fp, "%lf %lf %lf ", state.vel_end( 0 ), state.vel_end( 1 ),
             state.vel_end( 2 ) );                // Vel
    fprintf( fp, "%lf %lf %lf ", 0.0, 0.0, 0.0 ); // Acc
    fprintf( fp, "%lf %lf %lf ", state.bias_g( 0 ), state.bias_g( 1 ),
             state.bias_g( 2 ) ); // Bias_g
    fprintf( fp, "%lf %lf %lf ", state.bias_a( 0 ), state.bias_a( 1 ),
             state.bias_a( 2 ) ); // Bias_a
    fprintf( fp, "%lf %lf %lf ", state.gravity( 0 ), state.gravity( 1 ),
             state.gravity( 2 ) ); // Bias_a
    fprintf( fp, "\r\n" );
    fflush( fp );
#endif
}

void Voxel_mapping::pointBodyToWorld( const PointType &pi, PointType &po )
{
    V3D p_body( pi.x, pi.y, pi.z );

    V3D p_global( state.rot_end * ( m_extR * p_body + m_extT ) + state.pos_end );

    po.x = p_global( 0 );
    po.y = p_global( 1 );
    po.z = p_global( 2 );
    po.intensity = pi.intensity;
}

void Voxel_mapping::frameBodyToWorld( const PointCloudXYZI::Ptr &pi, PointCloudXYZI::Ptr &po )
{
    int pi_size = pi->points.size();
    po->resize( pi_size );
    for ( int i = 0; i < pi_size; i++ )
    {
        /* transform to world frame */
        pointBodyToWorld( pi->points[ i ], po->points[ i ] );
    }
}

void Voxel_mapping::get_NED_transform()
{
    if ( false )
    {
        V3D        grav_vec( -0.0463686846197, -0.194593831897, 0.996038079262 );
        double     gravity_correct_ang = std::acos( grav_vec.dot( V3D( 0, 0, 9.8 ) ) / ( grav_vec.norm() * 9.8 ) );
        AngleAxisd gravity_correct_vec( gravity_correct_ang, ( grav_vec.cross( V3D( 0, 0, 9.8 ) ) ).normalized() );
        // gravity_correct_vec = gravity_correct_vec / gravity_correct_vec.norm() *
        // gravity_correct_ang;
        _gravity_correct_rotM = gravity_correct_vec.toRotationMatrix();

        Eigen::Quaterniond _gravity_correct_q;
        _gravity_correct_q.x() = 0.0983599;
        _gravity_correct_q.y() = 0.00420122;
        _gravity_correct_q.z() = -0.377381;
        _gravity_correct_q.w() = 0.92081;

        _gravity_correct_rotM = _gravity_correct_q.toRotationMatrix().transpose();

        cout << "gravity_correct_rotM: " << _gravity_correct_rotM << endl;
        cout << "corrected gravity: " << grav_vec.transpose() * _gravity_correct_rotM.transpose() << endl;
    }
}

void Voxel_mapping::RGBpointBodyToWorld( PointType const *const pi, PointType *const po )
{
    V3D p_body( pi->x, pi->y, pi->z );
#ifdef USE_IKFOM
    // state_ikfom transfer_state = kf.get_x();
    V3D p_global( state_point.rot * ( state_point.offset_R_L_I * p_body + state_point.offset_T_L_I ) + state_point.pos );
#else
    V3D p_global( state.rot_end * ( m_extR * p_body + m_extT ) + state.pos_end );
#endif

    p_global = _gravity_correct_rotM * p_global;

    po->x = p_global( 0 );
    po->y = p_global( 1 );
    po->z = p_global( 2 );
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - floor( intensity );

    int reflection_map = intensity * 10000;
}

void Voxel_mapping::RGBpointBodyLidarToIMU( PointType const *const pi, PointType *const po )
{
    V3D p_body_lidar( pi->x, pi->y, pi->z );
#ifdef USE_IKFOM
    // state_ikfom transfer_state = kf.get_x();
    V3D p_body_imu( state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I );
#else
    V3D p_body_imu( m_extR * p_body_lidar + m_extT );
#endif

    po->x = p_body_imu( 0 );
    po->y = p_body_imu( 1 );
    po->z = p_body_imu( 2 );
    po->intensity = pi->intensity;
}

void Voxel_mapping::points_cache_collect()
{
    PointVector points_history;
    m_ikdtree.acquire_removed_points( points_history );
    m_points_cache_size = points_history.size();
}

void Voxel_mapping::laser_map_fov_segment()
{
    m_cub_need_rm.clear();
    m_kdtree_delete_counter = 0;
    m_kdtree_delete_time = 0.0;
    pointBodyToWorld( m_XAxis_Point_body, m_XAxis_Point_world );
#ifdef USE_IKFOM
    // state_ikfom fov_state = kf.get_x();
    // V3D pos_LiD = fov_state.pos + fov_state.rot * fov_state.offset_T_L_I;
    V3D pos_LiD = pos_lid;
#else
    V3D pos_LiD = state.pos_end;
#endif
    if ( !m_localmap_Initialized )
    {
        // if (cube_len <= 2.0 * MOV_THRESHOLD * DETECTION_RANGE) throw
        // std::invalid_argument("[Error]: Local Map Size is too small! Please
        // change parameter \"cube_side_length\" to larger than %d in the launch
        // file.\n");
        for ( int i = 0; i < 3; i++ )
        {
            m_LocalMap_Points.vertex_min[ i ] = pos_LiD( i ) - m_cube_len / 2.0;
            m_LocalMap_Points.vertex_max[ i ] = pos_LiD( i ) + m_cube_len / 2.0;
        }
        m_localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
    // LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[ 3 ][ 2 ];
    bool  need_move = false;
    for ( int i = 0; i < 3; i++ )
    {
        dist_to_map_edge[ i ][ 0 ] = fabs( pos_LiD( i ) - m_LocalMap_Points.vertex_min[ i ] );
        dist_to_map_edge[ i ][ 1 ] = fabs( pos_LiD( i ) - m_LocalMap_Points.vertex_max[ i ] );
        if ( dist_to_map_edge[ i ][ 0 ] <= MOV_THRESHOLD * DETECTION_RANGE || dist_to_map_edge[ i ][ 1 ] <= MOV_THRESHOLD * DETECTION_RANGE )
            need_move = true;
    }
    if ( !need_move )
        return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = m_LocalMap_Points;
    float mov_dist = max( ( m_cube_len - 2.0 * MOV_THRESHOLD * DETECTION_RANGE ) * 0.5 * 0.9, double( DETECTION_RANGE * ( MOV_THRESHOLD - 1 ) ) );
    for ( int i = 0; i < 3; i++ )
    {
        tmp_boxpoints = m_LocalMap_Points;
        if ( dist_to_map_edge[ i ][ 0 ] <= MOV_THRESHOLD * DETECTION_RANGE )
        {
            New_LocalMap_Points.vertex_max[ i ] -= mov_dist;
            New_LocalMap_Points.vertex_min[ i ] -= mov_dist;
            tmp_boxpoints.vertex_min[ i ] = m_LocalMap_Points.vertex_max[ i ] - mov_dist;
            m_cub_need_rm.push_back( tmp_boxpoints );
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
            // tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        }
        else if ( dist_to_map_edge[ i ][ 1 ] <= MOV_THRESHOLD * DETECTION_RANGE )
        {
            New_LocalMap_Points.vertex_max[ i ] += mov_dist;
            New_LocalMap_Points.vertex_min[ i ] += mov_dist;
            tmp_boxpoints.vertex_max[ i ] = m_LocalMap_Points.vertex_min[ i ] + mov_dist;
            m_cub_need_rm.push_back( tmp_boxpoints );
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n",
            // tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        }
    }
    m_LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if ( m_cub_need_rm.size() > 0 )
        m_kdtree_delete_counter = m_ikdtree.Delete_Point_Boxes( m_cub_need_rm );
    m_kdtree_delete_time = omp_get_wtime() - delete_begin;
    // printf( "Delete time: %0.6f, delete size: %d\n", m_kdtree_delete_time, m_kdtree_delete_counter );
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}

void Voxel_mapping::standard_pcl_cbk( const sensor_msgs::PointCloud2::ConstPtr &msg )
{
    if ( !m_lidar_en )
        return;
    m_mutex_buffer.lock();
    // cout<<"got feature"<<endl;
    if ( msg->header.stamp.toSec() < m_last_timestamp_lidar )
    {
        ROS_ERROR( "lidar loop back, clear buffer" );
        m_lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr( new PointCloudXYZI() );
    m_p_pre->process( msg, ptr );
    m_lidar_buffer.push_back( ptr );
    m_time_buffer.push_back( msg->header.stamp.toSec() );
    m_last_timestamp_lidar = msg->header.stamp.toSec();

    m_mutex_buffer.unlock();
    m_sig_buffer.notify_all();
}

void Voxel_mapping::livox_pcl_cbk( const livox_ros_driver::CustomMsg::ConstPtr &msg )
{
    if ( !m_lidar_en )
        return;
    m_mutex_buffer.lock();
    // ROS_INFO( "get LiDAR, its header time: %.6f", msg->header.stamp.toSec() );
    if ( msg->header.stamp.toSec() < m_last_timestamp_lidar )
    {
        ROS_ERROR( "lidar loop back, clear buffer" );
        m_lidar_buffer.clear();
    }
    // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr( new PointCloudXYZI() );
    m_p_pre->process( msg, ptr );
    m_lidar_buffer.push_back( ptr );
    m_time_buffer.push_back( msg->header.stamp.toSec() );
    m_last_timestamp_lidar = msg->header.stamp.toSec();

    m_mutex_buffer.unlock();
    m_sig_buffer.notify_all();
}

void Voxel_mapping::imu_cbk( const sensor_msgs::Imu::ConstPtr &msg_in )
{
    if ( !m_imu_en )
        return;

    if ( m_last_timestamp_lidar < 0.0 )
        return;
    m_publish_count++;
    // ROS_INFO("get imu at time: %.6f", msg_in->header.stamp.toSec());
    sensor_msgs::Imu::Ptr msg( new sensor_msgs::Imu( *msg_in ) );

    double timestamp = msg->header.stamp.toSec();
    m_mutex_buffer.lock();

    if ( m_last_timestamp_imu > 0.0 && timestamp < m_last_timestamp_imu )
    {
        m_mutex_buffer.unlock();
        m_sig_buffer.notify_all();
        ROS_ERROR( "imu loop back \n" );
        return;
    }
    // old 0.2
    if ( m_last_timestamp_imu > 0.0 && timestamp > m_last_timestamp_imu + 0.4 )
    {
        m_mutex_buffer.unlock();
        m_sig_buffer.notify_all();
        ROS_WARN( "imu time stamp Jumps %0.4lf seconds \n", timestamp - m_last_timestamp_imu );
        return;
    }

    m_last_timestamp_imu = timestamp;

    m_imu_buffer.push_back( msg );
    // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
    m_mutex_buffer.unlock();
    m_sig_buffer.notify_all();
}

bool Voxel_mapping::sync_packages( LidarMeasureGroup &meas )
{
    if ( !m_imu_en )
    {
        if ( !m_lidar_buffer.empty() )
        {
            meas.lidar = m_lidar_buffer.front();
            meas.lidar_beg_time = m_time_buffer.front();
            m_lidar_buffer.pop_front();
            m_time_buffer.pop_front();
            return true;
        }

        return false;
    }

    if ( m_lidar_buffer.empty() || m_imu_buffer.empty() )
    {
        return false;
    }
    /*** push a lidar scan ***/
    if ( !m_lidar_pushed )
    {
        meas.lidar = m_lidar_buffer.front();
        if ( meas.lidar->points.size() <= 1 )
        {
            m_lidar_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = m_time_buffer.front();
        m_lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double( 1000 );
        m_lidar_pushed = true;
    }
    if ( m_last_timestamp_imu < m_lidar_end_time )
    {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/

    // no img topic, means only has lidar topic
    if ( m_imu_en && m_last_timestamp_imu < m_lidar_end_time )
    { // imu message needs to be larger than lidar_end_time, keep complete propagate.
        // ROS_ERROR("out sync");
        return false;
    }
    struct MeasureGroup m; // standard method to keep imu message.
    if ( !m_imu_buffer.empty() )
    {
        double imu_time = m_imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        m_mutex_buffer.lock();
        while ( ( !m_imu_buffer.empty() && ( imu_time < m_lidar_end_time ) ) )
        {
            imu_time = m_imu_buffer.front()->header.stamp.toSec();
            if ( imu_time > m_lidar_end_time )
                break;
            m.imu.push_back( m_imu_buffer.front() );
            m_imu_buffer.pop_front();
        }
    }
    m_lidar_buffer.pop_front();
    m_time_buffer.pop_front();
    m_mutex_buffer.unlock();
    m_sig_buffer.notify_all();
    m_lidar_pushed = false;   // sync one whole lidar scan.
    meas.is_lidar_end = true; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back( m );

    return true;
}

void Voxel_mapping::publish_voxel_point( const ros::Publisher &pubLaserCloudVoxel, const PointCloudXYZI::Ptr &pcl_wait_pub )
{
    uint                  size = pcl_wait_pub->points.size();
    PointCloudXYZRGB::Ptr laserCloudWorldRGB( new PointCloudXYZRGB( size, 1 ) );
    for ( int i = 0; i < size; i++ )
    {
        PointTypeRGB pointRGB;

        pointRGB.x = pcl_wait_pub->points[ i ].x;
        pointRGB.y = pcl_wait_pub->points[ i ].y;
        pointRGB.z = pcl_wait_pub->points[ i ].z;

        V3D point( pointRGB.x, pointRGB.y, pointRGB.z );
        V3F pixel = RGBFromVoxel( point, m_max_voxel_size, m_layer_size, m_min_eigen_value, m_feat_map );
        pointRGB.r = pixel[ 0 ];
        pointRGB.g = pixel[ 1 ];
        pointRGB.b = pixel[ 2 ];
        laserCloudWorldRGB->push_back( pointRGB );
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    if ( m_img_en )
    {
        cout << "RGB pointcloud size: " << laserCloudWorldRGB->size() << endl;
        pcl::toROSMsg( *laserCloudWorldRGB, laserCloudmsg );
    }
    else
    {
        pcl::toROSMsg( *pcl_wait_pub, laserCloudmsg );
    }
    laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudVoxel.publish( laserCloudmsg );
    m_publish_count -= PUBFRAME_PERIOD;
}

void Voxel_mapping::publish_visual_world_map( const ros::Publisher &pubVisualCloud )
{
    PointCloudXYZI::Ptr laserCloudFullRes( m_map_cur_frame_point );
    int                 size = laserCloudFullRes->points.size();
    if ( size == 0 )
        return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->coints[i]);
    // }
    // mutex_buffer_pointcloud.lock();
    *m_pcl_visual_wait_pub = *laserCloudFullRes;
    if ( 1 ) // if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg( *m_pcl_visual_wait_pub, laserCloudmsg );
        laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubVisualCloud.publish( laserCloudmsg );
        m_publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mutex_buffer_pointcloud.unlock();
}

void Voxel_mapping::publish_visual_world_sub_map( const ros::Publisher &pubSubVisualCloud )
{
    PointCloudXYZI::Ptr laserCloudFullRes( m_sub_map_cur_frame_point );
    int                 size = laserCloudFullRes->points.size();
    if ( size == 0 )
        return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->points[i]);
    // }
    // mutex_buffer_pointcloud.lock();
    *m_sub_pcl_visual_wait_pub = *laserCloudFullRes;
    if ( 1 ) // if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg( *m_sub_pcl_visual_wait_pub, laserCloudmsg );
        laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubSubVisualCloud.publish( laserCloudmsg );
        m_publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mutex_buffer_pointcloud.unlock();
}

void Voxel_mapping::publish_effect_world( const ros::Publisher &pubLaserCloudEffect )
{
    PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI( m_effct_feat_num, 1 ) );
    for ( int i = 0; i < m_effct_feat_num; i++ )
    {
        RGBpointBodyToWorld( &m_laserCloudOri->points[ i ], &laserCloudWorld->points[ i ] );
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg( *laserCloudWorld, laserCloudFullRes3 );
    laserCloudFullRes3.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish( laserCloudFullRes3 );
}

void Voxel_mapping::publish_map( const ros::Publisher &pubLaserCloudMap )
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg( *m_featsFromMap, laserCloudMap );
    laserCloudMap.header.stamp = ros::Time::now();
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish( laserCloudMap );
}

void Voxel_mapping::publish_odometry( const ros::Publisher &pubOdomAftMapped )
{
    m_odom_aft_mapped.header.frame_id = "camera_init";
    m_odom_aft_mapped.child_frame_id = "aft_mapped";
    m_odom_aft_mapped.header.stamp = ros::Time::now(); //.ros::Time()fromSec(last_timestamp_lidar);
    set_pose_timestamp( m_odom_aft_mapped.pose.pose );
    // odomAftMapped.twist.twist.linear.x = state_point.vel(0);
    // odomAftMapped.twist.twist.linear.y = state_point.vel(1);
    // odomAftMapped.twist.twist.linear.z = state_point.vel(2);
    // if (Measures.imu.size()>0) {
    //     Vector3d tmp(Measures.imu.back()->angular_velocity.x,
    //     Measures.imu.back()->angular_velocity.y,Measures.imu.back()->angular_velocity.z);
    //     odomAftMapped.twist.twist.angular.x = tmp[0] - state_point.bg(0);
    //     odomAftMapped.twist.twist.angular.y = tmp[1] - state_point.bg(1);
    //     odomAftMapped.twist.twist.angular.z = tmp[2] - state_point.bg(2);
    // }
    // static tf::TransformBroadcaster br;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // transform.setOrigin(tf::Vector3(state.pos_end(0), state.pos_end(1),
    // state.pos_end(2))); q.setW(geoQuat.w); q.setX(geoQuat.x);
    // q.setY(geoQuat.y);
    // q.setZ(geoQuat.z);
    // transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform,
    // odomAftMapped.header.stamp, "camera_init", "aft_mapped" ) );
    pubOdomAftMapped.publish( m_odom_aft_mapped );
}

void Voxel_mapping::publish_mavros( const ros::Publisher &mavros_pose_publisher )
{
    m_msg_body_pose.header.stamp = ros::Time::now();
    m_msg_body_pose.header.frame_id = "camera_odom_frame";
    set_pose_timestamp( m_msg_body_pose.pose );
    mavros_pose_publisher.publish( m_msg_body_pose );
}

void Voxel_mapping::publish_frame_world( const ros::Publisher &pubLaserCloudFullRes, const int point_skip )
{
    PointCloudXYZI::Ptr laserCloudFullRes( m_dense_map_en ? m_feats_undistort : m_feats_down_body );
    int                 size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI( size, 1 ) );
    for ( int i = 0; i < size; i++ )
    {
        RGBpointBodyToWorld( &laserCloudFullRes->points[ i ], &laserCloudWorld->points[ i ] );
    }
    PointCloudXYZI::Ptr laserCloudWorldPub( new PointCloudXYZI );
    for ( int i = 0; i < size; i += point_skip )
    {
        laserCloudWorldPub->points.push_back( laserCloudWorld->points[ i ] );
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg( *laserCloudWorldPub, laserCloudmsg );
    laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish( laserCloudmsg );
}

void Voxel_mapping::publish_path( const ros::Publisher pubPath )
{
    set_pose_timestamp( m_msg_body_pose.pose );
    m_msg_body_pose.header.stamp = ros::Time::now();
    m_msg_body_pose.header.frame_id = "camera_init";
    m_pub_path.poses.push_back( m_msg_body_pose );
    pubPath.publish( m_pub_path );
}

void Voxel_mapping::read_ros_parameters( ros::NodeHandle &nh )
{
    nh.param< int >( "dense_map_enable", m_dense_map_en, 1 );
    nh.param< int >( "img_enable", m_img_en, 1 );
    nh.param< int >( "lidar_enable", m_lidar_en, 1 );
    nh.param< int >( "debug", m_debug, 0 );
    nh.param< int >( "max_iteration", NUM_MAX_ITERATIONS, 4 );
    nh.param< int >( "min_img_count", MIN_IMG_COUNT, 1000 );
    nh.param< string >( "pc_name", m_pointcloud_file_name, " " );

    nh.param< int >( "gui_font_size", m_GUI_font_size, 14 );
    
    nh.param< double >( "cam_fx", cam_fx, 453.483063 );
    nh.param< double >( "cam_fy", cam_fy, 453.254913 );
    nh.param< double >( "cam_cx", cam_cx, 318.908851 );
    nh.param< double >( "cam_cy", cam_cy, 234.238189 );

    nh.param< double >( "laser_point_cov", LASER_POINT_COV, 0.001 );
    nh.param< double >( "img_point_cov", IMG_POINT_COV, 10 );
    nh.param< string >( "map_file_path", m_map_file_path, "" );
    nh.param< string >( "common/lid_topic", m_lid_topic, "/livox/lidar" );
    nh.param< string >( "common/imu_topic", m_imu_topic, "/livox/imu" );
    nh.param< string >( "hilti/seq", m_hilti_seq_name, "01" );
    nh.param< bool >( "hilti/en", m_hilti_en, false );
    nh.param< string >( "camera/img_topic", m_img_topic, "/usb_cam/image_raw" );
    nh.param< double >( "filter_size_corner", m_filter_size_corner_min, 0.5 );
    nh.param< double >( "filter_size_surf", m_filter_size_surf_min, 0.5 );
    nh.param< double >( "filter_size_map", m_filter_size_map_min, 0.5 );
    nh.param< double >( "cube_side_length", m_cube_len, 200 );
    nh.param< double >( "mapping/fov_degree", m_fov_deg, 180 );
    nh.param< double >( "mapping/gyr_cov", m_gyr_cov, 1.0 );
    nh.param< double >( "mapping/acc_cov", m_acc_cov, 1.0 );
    nh.param< int >( "voxel/max_points_size", m_max_points_size, 100 );
    nh.param< int >( "voxel/max_layer", m_max_layer, 2 );
    nh.param< vector< int > >( "voxel/layer_init_size", m_layer_init_size, vector< int >() );
    nh.param< int >( "mapping/imu_int_frame", m_imu_int_frame, 3 );
    nh.param< bool >( "mapping/imu_en", m_imu_en, false );
    nh.param< bool >( "voxel/voxel_map_en", m_use_new_map, false );
    nh.param< bool >( "voxel/pub_plane_en", m_is_pub_plane_map, false );
    nh.param< double >( "voxel/match_eigen_value", m_match_eigen_value, 0.0025 );
    nh.param< int >( "voxel/layer", m_voxel_layer, 1 );
    nh.param< double >( "voxel/match_s", m_match_s, 0.90 );
    nh.param< double >( "voxel/voxel_size", m_max_voxel_size, 1.0 );
    nh.param< double >( "voxel/min_eigen_value", m_min_eigen_value, 0.01 );
    nh.param< double >( "voxel/sigma_num", m_sigma_num, 3 );
    nh.param< double >( "voxel/beam_err", m_beam_err, 0.02 );
    nh.param< double >( "voxel/dept_err", m_dept_err, 0.05 );
    nh.param< double >( "preprocess/blind", m_p_pre->blind, 0.01 );
    nh.param< double >( "image_save/rot_dist", m_keyf_rotd, 0.01 );
    nh.param< double >( "image_save/pos_dist", m_keyf_posd, 0.01 );
    nh.param< int >( "preprocess/lidar_type", m_p_pre->lidar_type, AVIA );
    nh.param< int >( "preprocess/scan_line", m_p_pre->N_SCANS, 16 );
    nh.param< int >( "preprocess/timestamp_unit", m_p_pre->time_unit, US );
    nh.param< bool >( "preprocess/calib_laser", m_p_pre->calib_laser, false );
    nh.param< int >( "point_filter_num", m_p_pre->point_filter_num, 2 );
    nh.param< int >( "pcd_save/interval", m_pcd_save_interval, -1 );
    nh.param< int >( "image_save/interval", m_img_save_interval, 1 );
    nh.param< int >( "pcd_save/type", m_pcd_save_type, 0 );
    nh.param< bool >( "pcd_save/pcd_save_en", m_pcd_save_en, false );
    nh.param< bool >( "image_save/img_save_en", m_img_save_en, false );
    nh.param< bool >( "feature_extract_enable", m_p_pre->feature_enabled, false );
    nh.param< vector< double > >( "mapping/extrinsic_T", m_extrin_T, vector< double >() );
    nh.param< vector< double > >( "mapping/extrinsic_R", m_extrin_R, vector< double >() );
    nh.param< vector< double > >( "camera/Pcl", m_camera_extrin_T, vector< double >() );
    nh.param< vector< double > >( "camera/Rcl", m_camera_extrin_R, vector< double >() );
    nh.param< int >( "grid_size", m_grid_size, 40 );
    nh.param< int >( "patch_size", m_patch_size, 4 );
    nh.param< double >( "outlier_threshold", m_outlier_threshold, 100 );
    nh.param< bool >( "publish/effect_point_pub", m_effect_point_pub, false );
    nh.param< int >( "publish/pub_point_skip", m_pub_point_skip, 1 );
    nh.param< double >( "meshing/distance_scale", m_meshing_distance_scale, 1.0 );
    nh.param< double >( "meshing/points_minimum_scale", m_meshing_points_minimum_scale, 0.1 );
    nh.param< double >( "meshing/voxel_resolution", m_meshing_voxel_resolution, 0.4 );
    nh.param< double >( "meshing/region_size", m_meshing_region_size, 10.0 );
    nh.param< int >( "meshing/if_draw_mesh", m_if_draw_mesh, 1.0 );
    nh.param< int >( "meshing/enable_mesh_rec", m_if_enable_mesh_rec, 1 );
    nh.param< int >( "meshing/maximum_thread_for_rec_mesh", m_meshing_maximum_thread_for_rec_mesh, 12 );
    nh.param< int >( "meshing/number_of_pts_append_to_map", m_meshing_number_of_pts_append_to_map, 10000 );

    m_p_pre->blind_sqr = m_p_pre->blind * m_p_pre->blind;
    cout << "Ranging cov:" << m_dept_err << " , angle cov:" << m_beam_err << std::endl;
    cout << "Meshing distance scale:" << m_meshing_distance_scale << " , points minimum scale:" << m_meshing_points_minimum_scale << std::endl;
}

void Voxel_mapping::transformLidar( const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                                    pcl::PointCloud< pcl::PointXYZI >::Ptr &trans_cloud )
{
    trans_cloud->clear();
    for ( size_t i = 0; i < input_cloud->size(); i++ )
    {
        pcl::PointXYZINormal p_c = input_cloud->points[ i ];
        Eigen::Vector3d      p( p_c.x, p_c.y, p_c.z );
        // p = rot * p + t;
        p = ( rot * ( m_extR * p + m_extT ) + t );
        pcl::PointXYZI pi;
        pi.x = p( 0 );
        pi.y = p( 1 );
        pi.z = p( 2 );
        pi.intensity = p_c.intensity;
        trans_cloud->points.push_back( pi );
    }
}
