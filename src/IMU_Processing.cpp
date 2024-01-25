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
#include "IMU_Processing.h"

Eigen::Matrix3d Eye3d = Eigen::Matrix3d::Identity();
Eigen::Matrix3f Eye3f = Eigen::Matrix3f::Identity();
Eigen::Vector3d Zero3d = Eigen::Vector3d::Zero();
Eigen::Vector3f Zero3f = Eigen::Vector3f::Zero();

const bool time_list( PointType &x, PointType &y ) { return ( x.curvature < y.curvature ); }

ImuProcess::ImuProcess() : b_first_frame_( true ), imu_need_init_( true ), start_timestamp_( -1 )
{
    init_iter_num = 1;
#ifdef USE_IKFOM
    Q = process_noise_cov();
#endif
    cov_acc = V3D( 0.1, 0.1, 0.1 );
    cov_gyr = V3D( 0.1, 0.1, 0.1 );
    // cov_acc_scale = V3D(1, 1, 1);
    // cov_gyr_scale = V3D(1, 1, 1);
    cov_bias_gyr = V3D( 0.1, 0.1, 0.1 );
    cov_bias_acc = V3D( 0.1, 0.1, 0.1 );
    mean_acc = V3D( 0, 0, -1.0 );
    mean_gyr = V3D( 0, 0, 0 );
    angvel_last = Zero3d;
    acc_s_last = Zero3d;
    Lid_offset_to_IMU = Zero3d;
    Lid_rot_to_IMU = Eye3d;
    last_imu_.reset( new sensor_msgs::Imu() );
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset()
{
    ROS_WARN( "Reset ImuProcess" );
    mean_acc = V3D( 0, 0, -1.0 );
    mean_gyr = V3D( 0, 0, 0 );
    angvel_last = Zero3d;
    imu_need_init_ = true;
    start_timestamp_ = -1;
    init_iter_num = 1;
    v_imu_.clear();
    IMUpose.clear();
    last_imu_.reset( new sensor_msgs::Imu() );
    cur_pcl_un_.reset( new PointCloudXYZI() );
}

void ImuProcess::disable_imu()
{
    cout << "IMU disabled !!!!!" << endl;
    imu_en = false;
    imu_need_init_ = false;
}

void ImuProcess::push_update_state( double offs_t, StatesGroup state )
{
    // V3D acc_tmp(last_acc), angvel_tmp(last_ang), vel_imu(state.vel_end), pos_imu(state.pos_end);
    // M3D R_imu(state.rot_end);
    // angvel_tmp -= state.bias_g;
    // acc_tmp   = acc_tmp * G_m_s2 / mean_acc.norm() - state.bias_a;
    // acc_tmp  = R_imu * acc_tmp + state.gravity;
    // IMUpose.push_back(set_pose6d(offs_t, acc_tmp, angvel_tmp, vel_imu, pos_imu, R_imu));
    V3D acc_tmp = acc_s_last, angvel_tmp = angvel_last, vel_imu( state.vel_end ), pos_imu( state.pos_end );
    M3D R_imu( state.rot_end );
    IMUpose.push_back( set_pose6d( offs_t, acc_tmp, angvel_tmp, vel_imu, pos_imu, R_imu ) );
}

void ImuProcess::set_extrinsic( const MD( 4, 4 ) & T )
{
    Lid_offset_to_IMU = T.block< 3, 1 >( 0, 3 );
    Lid_rot_to_IMU = T.block< 3, 3 >( 0, 0 );
}

void ImuProcess::set_extrinsic( const V3D &transl )
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU.setIdentity();
}

void ImuProcess::set_extrinsic( const V3D &transl, const M3D &rot )
{
    Lid_offset_to_IMU = transl;
    Lid_rot_to_IMU = rot;
}

void ImuProcess::set_gyr_cov_scale( const V3D &scaler ) { cov_gyr = scaler; }

void ImuProcess::set_acc_cov_scale( const V3D &scaler ) { cov_acc = scaler; }

void ImuProcess::set_gyr_bias_cov( const V3D &b_g ) { cov_bias_gyr = b_g; }

void ImuProcess::set_acc_bias_cov( const V3D &b_a ) { cov_bias_acc = b_a; }

void ImuProcess::set_imu_init_frame_num( const int &num ) { MAX_INI_COUNT = num; }

#ifdef USE_IKFOM
void ImuProcess::IMU_init( const MeasureGroup &meas, esekfom::esekf< state_ikfom, 12, input_ikfom > &kf_state, int &N )
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    ROS_INFO( "IMU Initializing: %.1f %%", double( N ) / MAX_INI_COUNT * 100 );
    V3D cur_acc, cur_gyr;

    if ( b_first_frame_ )
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        // first_lidar_time = meas.lidar_beg_time;
        // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
    }

    for ( const auto &imu : meas.imu )
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += ( cur_acc - mean_acc ) / N;
        mean_gyr += ( cur_gyr - mean_gyr ) / N;

        cov_acc = cov_acc * ( N - 1.0 ) / N + ( cur_acc - mean_acc ).cwiseProduct( cur_acc - mean_acc ) * ( N - 1.0 ) / ( N * N );
        cov_gyr = cov_gyr * ( N - 1.0 ) / N + ( cur_gyr - mean_gyr ).cwiseProduct( cur_gyr - mean_gyr ) * ( N - 1.0 ) / ( N * N );

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N++;
    }
    state_ikfom init_state = kf_state.get_x();
    init_state.grav = S2( -mean_acc / mean_acc.norm() * G_m_s2 );

    // state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    init_state.bg = mean_gyr;
    init_state.offset_T_L_I = Lid_offset_to_IMU;
    init_state.offset_R_L_I = Lid_rot_to_IMU;
    kf_state.change_x( init_state );

    esekfom::esekf< state_ikfom, 12, input_ikfom >::cov init_P = kf_state.get_P() * 0.001;
    kf_state.change_P( init_P );
    last_imu_ = meas.imu.back();
}
#else
void ImuProcess::IMU_init( const MeasureGroup &meas, StatesGroup &state_inout, int &N )
{
    /** 1. initializing the gravity, gyro bias, acc and gyro covariance
     ** 2. normalize the acceleration measurenments to unit gravity **/
    ROS_INFO( "IMU Initializing: %.1f %%", double( N ) / MAX_INI_COUNT * 100 );
    V3D cur_acc, cur_gyr;

    if ( b_first_frame_ )
    {
        Reset();
        N = 1;
        b_first_frame_ = false;
        const auto &imu_acc = meas.imu.front()->linear_acceleration;
        const auto &gyr_acc = meas.imu.front()->angular_velocity;
        mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
        // first_lidar_time = meas.lidar_beg_time;
        // cout<<"init acc norm: "<<mean_acc.norm()<<endl;
    }

    for ( const auto &imu : meas.imu )
    {
        const auto &imu_acc = imu->linear_acceleration;
        const auto &gyr_acc = imu->angular_velocity;
        cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
        cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

        mean_acc += ( cur_acc - mean_acc ) / N;
        mean_gyr += ( cur_gyr - mean_gyr ) / N;

        // cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        // cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

        N++;
    }

    state_inout.gravity = -mean_acc / mean_acc.norm() * G_m_s2;

    state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
    state_inout.bias_g = Zero3d; // mean_gyr;

    last_imu_ = meas.imu.back();
}
#endif

#ifdef USE_IKFOM
void ImuProcess::UndistortPcl( const MeasureGroup &meas, esekfom::esekf< state_ikfom, 12, input_ikfom > &kf_state, PointCloudXYZI &pcl_out )
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front( last_imu_ );
    const double &             imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time IMUpose.push_back( set_pose6d( offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu ) );
    time *** / pcl_out = *( meas.lidar );
    sort( pcl_out.points.begin(), pcl_out.points.end(), time_list );
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double( 1000 );
    // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

    /*** Initialize IMU pose ***/
    state_ikfom imu_state = kf_state.get_x();
    IMUpose.clear();
    IMUpose.push_back( set_pose6d( 0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix() ) );

    /*** forward propagation at each imu point ***/
    V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    M3D R_imu;

    double dt = 0;

    input_ikfom in;
    for ( auto it_imu = v_imu.begin(); it_imu < ( v_imu.end() - 1 ); it_imu++ )
    {
        auto &&head = *( it_imu );
        auto &&tail = *( it_imu + 1 );

        if ( tail->header.stamp.toSec() < last_lidar_end_time_ )
            continue;

        angvel_avr << 0.5 * ( head->angular_velocity.x + tail->angular_velocity.x ), 0.5 * ( head->angular_velocity.y + tail->angular_velocity.y ),
            0.5 * ( head->angular_velocity.z + tail->angular_velocity.z );
        acc_avr << 0.5 * ( head->linear_acceleration.x + tail->linear_acceleration.x ),
            0.5 * ( head->linear_acceleration.y + tail->linear_acceleration.y ), 0.5 * ( head->linear_acceleration.z + tail->linear_acceleration.z );

        // #ifdef DEBUG_PRINT
        fout_imu << setw( 10 ) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose()
                 << endl;
        // #endif

        acc_avr = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;

        if ( head->header.stamp.toSec() < last_lidar_end_time_ )
        {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
            // dt = tail->header.stamp.toSec() - pcl_beg_time;
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }

        in.acc = acc_avr;
        in.gyro = angvel_avr;
        Q.block< 3, 3 >( 0, 0 ).diagonal() = cov_gyr;
        Q.block< 3, 3 >( 3, 3 ).diagonal() = cov_acc;
        kf_state.predict( dt, Q, in );

        /* save the poses at each IMU measurements */
        imu_state = kf_state.get_x();
        angvel_last = angvel_avr - imu_state.bg;
        acc_s_last = imu_state.rot * ( acc_avr - imu_state.ba );
        for ( int i = 0; i < 3; i++ )
        {
            acc_s_last[ i ] += imu_state.grav[ i ];
        }
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back( set_pose6d( offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix() ) );
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * ( pcl_end_time - imu_end_time );
    kf_state.predict( dt, Q, in );

    imu_state = kf_state.get_x();
    last_imu_ = meas.imu.back();
    last_lidar_end_time_ = pcl_end_time;

#ifdef DEBUG_PRINT
    esekfom::esekf< state_ikfom, 12, input_ikfom >::cov P = kf_state.get_P();
    cout << "[ IMU Process ]: vel " << imu_state.vel.transpose() << " pos " << imu_state.pos.transpose() << " ba" << imu_state.ba.transpose()
         << " bg " << imu_state.bg.transpose() << endl;
    cout << "propagated cov: " << P.diagonal().transpose() << endl;
#endif

    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    for ( auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp-- )
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY( head->rot );
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY( head->vel );
        pos_imu << VEC_FROM_ARRAY( head->pos );
        acc_imu << VEC_FROM_ARRAY( tail->acc );
        angvel_avr << VEC_FROM_ARRAY( tail->gyr );

        for ( ; it_pcl->curvature / double( 1000 ) > head->offset_time; it_pcl-- )
        {
            dt = it_pcl->curvature / double( 1000 ) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            M3D R_i( R_imu * Exp( angvel_avr, dt ) );

            V3D P_i( it_pcl->x, it_pcl->y, it_pcl->z );
            V3D T_ei( pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos );
            V3D P_compensate = imu_state.offset_R_L_I.conjugate() *
                               ( imu_state.rot.conjugate() * ( R_i * ( imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I ) + T_ei ) -
                                 imu_state.offset_T_L_I ); // not accurate!

            // save Undistorted points and their rotation
            it_pcl->x = P_compensate( 0 );
            it_pcl->y = P_compensate( 1 );
            it_pcl->z = P_compensate( 2 );

            if ( it_pcl == pcl_out.points.begin() )
                break;
        }
    }
}
#else

void ImuProcess::Forward( const MeasureGroup &meas, StatesGroup &state_inout, double pcl_beg_time, double end_time )
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    auto v_imu = meas.imu;
    v_imu.push_front( last_imu_ );
    const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();

    // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

    // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
    if ( IMUpose.empty() )
    {
        IMUpose.push_back( set_pose6d( 0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end ) );
    }

    /*** forward propagation at each imu point ***/
    V3D acc_imu = acc_s_last, angvel_avr = angvel_last, acc_avr, vel_imu( state_inout.vel_end ), pos_imu( state_inout.pos_end );
    M3D R_imu( state_inout.rot_end );
    //  last_state = state_inout;
    MD( DIM_STATE, DIM_STATE ) F_x, cov_w;

    double dt = 0;
    for ( auto it_imu = v_imu.begin(); it_imu < ( v_imu.end() - 1 ); it_imu++ )
    {
        auto &&head = *( it_imu );
        auto &&tail = *( it_imu + 1 );

        if ( tail->header.stamp.toSec() < last_lidar_end_time_ )
            continue;

        angvel_avr << 0.5 * ( head->angular_velocity.x + tail->angular_velocity.x ), 0.5 * ( head->angular_velocity.y + tail->angular_velocity.y ),
            0.5 * ( head->angular_velocity.z + tail->angular_velocity.z );

        // angvel_avr<<tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z;

        acc_avr << 0.5 * ( head->linear_acceleration.x + tail->linear_acceleration.x ),
            0.5 * ( head->linear_acceleration.y + tail->linear_acceleration.y ), 0.5 * ( head->linear_acceleration.z + tail->linear_acceleration.z );
        last_acc = acc_avr;
        last_ang = angvel_avr;
        // #ifdef DEBUG_PRINT
        fout_imu << setw( 10 ) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose()
                 << endl;
        // #endif

        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

        if ( head->header.stamp.toSec() < last_lidar_end_time_ )
        {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }
        // cout<<setw(20)<<"dt: "<<dt<<endl;
        /* covariance propagation */
        M3D acc_avr_skew;
        M3D Exp_f = Exp( angvel_avr, dt );
        acc_avr_skew << SKEW_SYM_MATRX( acc_avr );

        F_x.setIdentity();
        cov_w.setZero();

        F_x.block< 3, 3 >( 0, 0 ) = Exp( angvel_avr, -dt );
        F_x.block< 3, 3 >( 0, 9 ) = -Eye3d * dt;
        // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
        F_x.block< 3, 3 >( 3, 6 ) = Eye3d * dt;
        F_x.block< 3, 3 >( 6, 0 ) = -R_imu * acc_avr_skew * dt;
        F_x.block< 3, 3 >( 6, 12 ) = -R_imu * dt;
        F_x.block< 3, 3 >( 6, 15 ) = Eye3d * dt;

        cov_w.block< 3, 3 >( 0, 0 ).diagonal() = cov_gyr * dt * dt;
        cov_w.block< 3, 3 >( 6, 6 ) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        cov_w.block< 3, 3 >( 9, 9 ).diagonal() = cov_bias_gyr * dt * dt;   // bias gyro covariance
        cov_w.block< 3, 3 >( 12, 12 ).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

        /* propogation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* propogation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        IMUpose.push_back( set_pose6d( offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu ) );
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    double note = end_time > imu_end_time ? 1.0 : -1.0;
    dt = note * ( end_time - imu_end_time );
    state_inout.vel_end = vel_imu + note * acc_imu * dt;
    state_inout.rot_end = R_imu * Exp( V3D( note * angvel_avr ), dt );
    state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;

    last_imu_ = v_imu.back();
    last_lidar_end_time_ = end_time;

    // auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
    // auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;

#ifdef DEBUG_PRINT
    cout << "[ IMU Process ]: vel " << state_inout.vel_end.transpose() << " pos " << state_inout.pos_end.transpose() << " ba"
         << state_inout.bias_a.transpose() << " bg " << state_inout.bias_g.transpose() << endl;
    cout << "propagated cov: " << state_inout.cov.diagonal().transpose() << endl;
#endif
}

void ImuProcess::Forward_without_imu( LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out )
{
    const double &pcl_beg_time = meas.lidar_beg_time;

    /*** sort point clouds by offset time ***/
    pcl_out = *( meas.lidar );
    // sort(pcl_out->points.begin(), pcl_out->points.end(), time_list);
    const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double( 1000 );
    // V3D acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end),
    //     pos_imu(state_inout.pos_end);
    // M3D R_imu(state_inout.rot_end);
    meas.last_update_time = pcl_end_time;
    MD( DIM_STATE, DIM_STATE )
    F_x, cov_w;
    double dt = 0;

    if ( b_first_frame_ )
    {
        dt = 0.1;
        b_first_frame_ = false;
    }
    else
    {
        dt = pcl_beg_time - time_last_scan;
    }
    // std::cout << "[No imu] dt:" << dt << std::endl;
    time_last_scan = pcl_beg_time;
    // for (size_t i = 0; i < pcl_out->points.size(); i++) {
    //   if (dt < pcl_out->points[i].curvature) {
    //     dt = pcl_out->points[i].curvature;
    //   }
    // }
    // dt = dt / (double)1000;
    // std::cout << "dt:" << dt << std::endl;
    // double dt = pcl_out->points.back().curvature / double(1000);

    /* covariance propagation */
    // M3D acc_avr_skew;
    M3D Exp_f = Exp( state_inout.bias_g, dt );

    F_x.setIdentity();
    cov_w.setZero();

    F_x.block< 3, 3 >( 0, 0 ) = Exp( state_inout.bias_g, -dt );
    F_x.block< 3, 3 >( 0, 9 ) = Eye3d * dt;
    F_x.block< 3, 3 >( 3, 6 ) = Eye3d * dt;
    // F_x.block<3, 3>(6, 0)  = - R_imu * acc_avr_skew * dt;
    // F_x.block<3, 3>(6, 12) = - R_imu * dt;
    // F_x.block<3, 3>(6, 15) = Eye3d * dt;

    cov_w.block< 3, 3 >( 9, 9 ).diagonal() = cov_gyr * dt * dt; // for omega in constant model
    cov_w.block< 3, 3 >( 6, 6 ).diagonal() = cov_acc * dt * dt; // for velocity in constant model
    // cov_w.block<3, 3>(6, 6) =
    //     R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
    // cov_w.block<3, 3>(9, 9).diagonal() =
    //     cov_bias_gyr * dt * dt; // bias gyro covariance
    // cov_w.block<3, 3>(12, 12).diagonal() =
    //     cov_bias_acc * dt * dt; // bias acc covariance

    // std::cout << "before propagete:" << state_inout.cov.diagonal().transpose()
    //           << std::endl;
    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;
    // std::cout << "cov_w:" << cov_w.diagonal().transpose() << std::endl;
    // std::cout << "after propagete:" << state_inout.cov.diagonal().transpose()
    //           << std::endl;
    state_inout.rot_end = state_inout.rot_end * Exp_f;
    state_inout.pos_end = state_inout.pos_end + state_inout.vel_end * dt;
}

void ImuProcess::Backward( const LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out )
{
    /*** undistort each lidar point (backward propagation) ***/
    M3D    R_imu;
    V3D    acc_imu, angvel_avr, vel_imu, pos_imu;
    double dt;
    auto   pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
    auto   it_pcl = pcl_out.points.end() - 1;
    for ( auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp-- )
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY( head->rot );
        acc_imu << VEC_FROM_ARRAY( head->acc );
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY( head->vel );
        pos_imu << VEC_FROM_ARRAY( head->pos );
        angvel_avr << VEC_FROM_ARRAY( head->gyr );
        for ( ; it_pcl->curvature / double( 1000 ) > head->offset_time; it_pcl-- )
        {
            dt = it_pcl->curvature / double( 1000 ) - head->offset_time;

            /* Transform to the 'end' frame, using only the rotation
             * Note: Compensation direction is INVERSE of Frame's moving direction
             * So if we want to compensate a point at timestamp-i to the frame-e
             * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
            M3D R_i( R_imu * Exp( angvel_avr, dt ) );
            V3D T_ei( pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lid_offset_to_IMU - pos_liD_e );

            V3D P_i( it_pcl->x, it_pcl->y, it_pcl->z );
            V3D P_compensate = state_inout.rot_end.transpose() * ( R_i * P_i + T_ei );

            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate( 0 );
            it_pcl->y = P_compensate( 1 );
            it_pcl->z = P_compensate( 2 );

            if ( it_pcl == pcl_out.points.begin() )
                break;
        }
    }
}
#endif

#ifdef USE_IKFOM
void ImuProcess::Process( const LidarMeasureGroup &lidar_meas, esekfom::esekf< state_ikfom, 12, input_ikfom > &kf_state,
                          PointCloudXYZI::Ptr cur_pcl_un_ )
{
    double t1, t2, t3;
    t1 = omp_get_wtime();
    MeasureGroup meas = lidar_meas.measures.back();
    if ( meas.imu.empty() )
    {
        return;
    };
    ROS_ASSERT( meas.lidar != nullptr );

    if ( imu_need_init_ )
    {
        /// The very first lidar frame
        IMU_init( meas, kf_state, init_iter_num );

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        state_ikfom imu_state = kf_state.get_x();
        if ( init_iter_num > MAX_INI_COUNT )
        {
            cov_acc *= pow( G_m_s2 / mean_acc.norm(), 2 );
            imu_need_init_ = false;
            ROS_INFO( "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: "
                      "%.8f %.8f %.8f",
                      imu_state.grav[ 0 ], imu_state.grav[ 1 ], imu_state.grav[ 2 ], mean_acc.norm(), cov_acc_scale[ 0 ], cov_acc_scale[ 1 ],
                      cov_acc_scale[ 2 ], cov_acc[ 0 ], cov_acc[ 1 ], cov_acc[ 2 ], cov_gyr[ 0 ], cov_gyr[ 1 ], cov_gyr[ 2 ] );
            cov_acc = cov_acc.cwiseProduct( cov_acc_scale );
            cov_gyr = cov_gyr.cwiseProduct( cov_gyr_scale );
            // cout<<"mean acc: "<<mean_acc<<" acc measures in word frame:"<<state.rot_end.transpose()*mean_acc<<endl;
            ROS_INFO( "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: "
                      "%.8f %.8f %.8f",
                      imu_state.grav[ 0 ], imu_state.grav[ 1 ], imu_state.grav[ 2 ], mean_acc.norm(), cov_bias_gyr[ 0 ], cov_bias_gyr[ 1 ],
                      cov_bias_gyr[ 2 ], cov_acc[ 0 ], cov_acc[ 1 ], cov_acc[ 2 ], cov_gyr[ 0 ], cov_gyr[ 1 ], cov_gyr[ 2 ] );
            fout_imu.open( DEBUG_FILE_DIR( "imu.txt" ), ios::out );
        }

        return;
    }

    /// Undistort points： the first point is assummed as the base frame
    /// Compensate lidar points with IMU rotation (with only rotation now)
    if ( lidar_meas.is_lidar_end )
    {
        UndistortPcl( lidar_meas, kf_state, *cur_pcl_un_ );
    }

    t2 = omp_get_wtime();

    // {
    //   static ros::Publisher pub_UndistortPcl =
    //       nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
    //   sensor_msgs::PointCloud2 pcl_out_msg;
    //   pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    //   pcl_out_msg.header.stamp = ros::Time().fromSec(meas.lidar_beg_time);
    //   pcl_out_msg.header.frame_id = "/livox";
    //   pub_UndistortPcl.publish(pcl_out_msg);
    // }

    t3 = omp_get_wtime();

    // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}
#else

void ImuProcess::Process( LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_ )
{
    double t1, t2, t3;
    t1 = omp_get_wtime();
    ROS_ASSERT( lidar_meas.lidar != nullptr );
    MeasureGroup meas = lidar_meas.measures.back();

    if ( imu_need_init_ )
    {
        if ( meas.imu.empty() )
        {
            return;
        };
        /// The very first lidar frame
        IMU_init( meas, stat, init_iter_num );

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        if ( init_iter_num > MAX_INI_COUNT )
        {
            cov_acc *= pow( G_m_s2 / mean_acc.norm(), 2 );
            imu_need_init_ = false;
            ROS_INFO( "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f", stat.gravity[ 0 ],
                      stat.gravity[ 1 ], stat.gravity[ 2 ], mean_acc.norm(), cov_acc[ 0 ], cov_acc[ 1 ], cov_acc[ 2 ], cov_gyr[ 0 ], cov_gyr[ 1 ],
                      cov_gyr[ 2 ] );

            // cout<<"mean acc: "<<mean_acc<<" acc measures in word frame:"<<state.rot_end.transpose()*mean_acc<<endl;
            fout_imu.open( DEBUG_FILE_DIR( "imu.txt" ), ios::out );
        }

        return;
    }

    /// Undistort points： the first point is assummed as the base frame
    /// Compensate lidar points with IMU rotation (with only rotation now)
    if ( lidar_meas.is_lidar_end )
    {
        /*** sort point clouds by offset time ***/
        *cur_pcl_un_ = *( lidar_meas.lidar );
        sort( cur_pcl_un_->points.begin(), cur_pcl_un_->points.end(), time_list );
        const double &pcl_beg_time = lidar_meas.lidar_beg_time;
        const double &pcl_end_time = pcl_beg_time + lidar_meas.lidar->points.back().curvature / double( 1000 );
        if ( imu_en )
        {
            Forward( meas, stat, pcl_beg_time, pcl_end_time );
            Backward( lidar_meas, stat, *cur_pcl_un_ );
            last_lidar_end_time_ = pcl_end_time;
            IMUpose.clear();
        }
        else
        {
            Forward_without_imu( lidar_meas, stat, *cur_pcl_un_ );
        }
        // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
    //        <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;
        // cout<<"Time:";
        // for (auto it = IMUpose.begin(); it != IMUpose.end(); ++it) {
        //   cout<<it->offset_time<<" ";
        // }
        // cout<<endl<<"size:"<<IMUpose.size()<<endl;
    }
    else
    {
        const double &pcl_beg_time = lidar_meas.lidar_beg_time;
        const double &img_end_time = pcl_beg_time + meas.img_offset_time;
        Forward( meas, stat, pcl_beg_time, img_end_time );
    }

    t2 = omp_get_wtime();

    // {
    //   static ros::Publisher pub_UndistortPcl =
    //       nh.advertise<sensor_msgs::PointCloud2>("/livox_undistort", 100);
    //   sensor_msgs::PointCloud2 pcl_out_msg;
    //   pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    //   pcl_out_msg.header.stamp = ros::Time().fromSec(meas.lidar_beg_time);
    //   pcl_out_msg.header.frame_id = "/livox";
    //   pub_UndistortPcl.publish(pcl_out_msg);
    // }

    t3 = omp_get_wtime();

    // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}

void ImuProcess::UndistortPcl( LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out )
{
    /*** add the imu of the last frame-tail to the of current frame-head ***/
    MeasureGroup meas;
    meas = lidar_meas.measures.back();
    auto v_imu = meas.imu;
    v_imu.push_front( last_imu_ );
    const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
    const double &imu_end_time = v_imu.back()->header.stamp.toSec();
    const double  pcl_beg_time = MAX( lidar_meas.lidar_beg_time, lidar_meas.last_update_time );
    // const double &pcl_beg_time = meas.lidar_beg_time;

    /*** sort point clouds by offset time ***/
    // pcl_out.clear();
    // auto         pcl_it = lidar_meas.lidar->points.begin() + lidar_meas.lidar_scan_index_now;
    // auto         pcl_it_end = lidar_meas.lidar->points.end();
    // const double pcl_end_time = lidar_meas.is_lidar_end ? lidar_meas.lidar_beg_time + lidar_meas.lidar->points.back().curvature / double( 1000 )
    //                                                     : lidar_meas.lidar_beg_time + lidar_meas.measures.back().img_offset_time;
    // // std::cout << "pcl end time:" << pcl_end_time << " pcl beg time:" << lidar_meas.lidar_beg_time << std::endl;
    // const double pcl_offset_time = ( pcl_end_time - lidar_meas.lidar_beg_time ) * double( 1000 );
    // while ( pcl_it != pcl_it_end && pcl_it->curvature <= pcl_offset_time )
    // {
    //     pcl_out.push_back( *pcl_it );
    //     pcl_it++;
    //     lidar_meas.lidar_scan_index_now++;
    // }
    // cout << "pcl_offset_time:  " << pcl_offset_time << "   pcl_it->curvature:  " << pcl_it->curvature << endl;
    // cout << "lidar_meas.lidar_scan_index_now:" << lidar_meas.lidar_scan_index_now << endl;
    pcl_out = *( lidar_meas.lidar );
    sort( pcl_out.points.begin(), pcl_out.points.end(), time_list );
    const double pcl_end_time = lidar_meas.is_lidar_end ? lidar_meas.lidar_beg_time + lidar_meas.lidar->points.back().curvature / double( 1000 )
                                                        : lidar_meas.lidar_beg_time + lidar_meas.measures.back().img_offset_time;
    lidar_meas.last_update_time = pcl_end_time;
    if ( lidar_meas.is_lidar_end )
    {
        lidar_meas.lidar_scan_index_now = 0;
    }
    // sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
    // lidar_meas.debug_show();
    // cout<<"UndistortPcl [ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;
    // cout<<"[ IMU Process ]: point size: "<<lidar_meas.lidar->points.size()<<endl;
    /*** Initialize IMU pose ***/
    IMUpose.clear();
    // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
    IMUpose.push_back( set_pose6d( 0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end ) );

    /*** forward propagation at each imu point ***/
    V3D acc_imu( acc_s_last ), angvel_avr( angvel_last ), acc_avr, vel_imu( state_inout.vel_end ), pos_imu( state_inout.pos_end );
    M3D R_imu( state_inout.rot_end );
    MD( DIM_STATE, DIM_STATE ) F_x, cov_w;

    double dt = 0;
    for ( auto it_imu = v_imu.begin(); it_imu != v_imu.end() - 1; it_imu++ )
    {
        auto &&head = *( it_imu );
        auto &&tail = *( it_imu + 1 );

        if ( tail->header.stamp.toSec() < last_lidar_end_time_ )
            continue;

        angvel_avr << 0.5 * ( head->angular_velocity.x + tail->angular_velocity.x ), 0.5 * ( head->angular_velocity.y + tail->angular_velocity.y ),
            0.5 * ( head->angular_velocity.z + tail->angular_velocity.z );

        // angvel_avr<<tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z;

        acc_avr << 0.5 * ( head->linear_acceleration.x + tail->linear_acceleration.x ),
            0.5 * ( head->linear_acceleration.y + tail->linear_acceleration.y ), 0.5 * ( head->linear_acceleration.z + tail->linear_acceleration.z );

        // #ifdef DEBUG_PRINT
        fout_imu << setw( 10 ) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose()
                 << endl;
        // #endif

        angvel_avr -= state_inout.bias_g;
        acc_avr = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

        if ( head->header.stamp.toSec() < last_lidar_end_time_ )
        {
            dt = tail->header.stamp.toSec() - last_lidar_end_time_;
        }
        else
        {
            dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
        }

        /* covariance propagation */
        M3D acc_avr_skew;
        M3D Exp_f = Exp( angvel_avr, dt );
        acc_avr_skew << SKEW_SYM_MATRX( acc_avr );

        F_x.setIdentity();
        cov_w.setZero();

        F_x.block< 3, 3 >( 0, 0 ) = Exp( angvel_avr, -dt );
        F_x.block< 3, 3 >( 0, 9 ) = -Eye3d * dt;
        // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
        F_x.block< 3, 3 >( 3, 6 ) = Eye3d * dt;
        F_x.block< 3, 3 >( 6, 0 ) = -R_imu * acc_avr_skew * dt;
        F_x.block< 3, 3 >( 6, 12 ) = -R_imu * dt;
        F_x.block< 3, 3 >( 6, 15 ) = Eye3d * dt;

        cov_w.block< 3, 3 >( 0, 0 ).diagonal() = cov_gyr * dt * dt;
        cov_w.block< 3, 3 >( 6, 6 ) = R_imu * cov_acc.asDiagonal() * R_imu.transpose() * dt * dt;
        cov_w.block< 3, 3 >( 9, 9 ).diagonal() = cov_bias_gyr * dt * dt;   // bias gyro covariance
        cov_w.block< 3, 3 >( 12, 12 ).diagonal() = cov_bias_acc * dt * dt; // bias acc covariance

        state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

        /* propogation of IMU attitude */
        R_imu = R_imu * Exp_f;

        /* Specific acceleration (global frame) of IMU */
        acc_imu = R_imu * acc_avr + state_inout.gravity;

        /* propogation of IMU */
        pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

        /* velocity of IMU */
        vel_imu = vel_imu + acc_imu * dt;

        /* save the poses at each IMU measurements */
        angvel_last = angvel_avr;
        acc_s_last = acc_imu;
        double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
        // cout<<setw(20)<<"offset_t: "<<offs_t<<"tail->header.stamp.toSec(): "<<tail->header.stamp.toSec()<<endl;
        IMUpose.push_back( set_pose6d( offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu ) );
    }

    /*** calculated the pos and attitude prediction at the frame-end ***/
    if ( imu_end_time > pcl_beg_time )
    {
        double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;
        dt = note * ( pcl_end_time - imu_end_time );
        state_inout.vel_end = vel_imu + note * acc_imu * dt;
        state_inout.rot_end = R_imu * Exp( V3D( note * angvel_avr ), dt );
        state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
    }
    else
    {
        double note = pcl_end_time > pcl_beg_time ? 1.0 : -1.0;
        dt = note * ( pcl_end_time - pcl_beg_time );
        state_inout.vel_end = vel_imu + note * acc_imu * dt;
        state_inout.rot_end = R_imu * Exp( V3D( note * angvel_avr ), dt );
        state_inout.pos_end = pos_imu + note * vel_imu * dt + note * 0.5 * acc_imu * dt * dt;
    }

    last_imu_ = v_imu.back();
    last_lidar_end_time_ = pcl_end_time;

    // auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lid_offset_to_IMU;
    // auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;

    // cout<<"[ IMU Process ]: vel "<<state_inout.vel_end.transpose()<<" pos "<<state_inout.pos_end.transpose()<<"
    // ba"<<state_inout.bias_a.transpose()<<" bg "<<state_inout.bias_g.transpose()<<endl; cout<<"propagated cov:
    // "<<state_inout.cov.diagonal().transpose()<<endl;

    //   cout<<"UndistortPcl Time:";
    //   for (auto it = IMUpose.begin(); it != IMUpose.end(); ++it) {
    //     cout<<it->offset_time<<" ";
    //   }
    //   cout<<endl<<"UndistortPcl size:"<<IMUpose.size()<<endl;
    //   cout<<"Undistorted pcl_out.size: "<<pcl_out.size()
    //          <<"lidar_meas.size: "<<lidar_meas.lidar->points.size()<<endl;
    if ( pcl_out.points.size() < 1 )
        return;
    /*** undistort each lidar point (backward propagation) ***/
    auto it_pcl = pcl_out.points.end() - 1;
    // cout << __FILE__ << " " << __LINE__ << " " << __FUNCTION__ << " " << it_pcl->curvature << endl;
    // cout << "R mat :\r\n" << Lid_rot_to_IMU << endl;
    for ( auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp-- )
    {
        auto head = it_kp - 1;
        auto tail = it_kp;
        R_imu << MAT_FROM_ARRAY( head->rot );
        acc_imu << VEC_FROM_ARRAY( head->acc );
        // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
        vel_imu << VEC_FROM_ARRAY( head->vel );
        pos_imu << VEC_FROM_ARRAY( head->pos );
        angvel_avr << VEC_FROM_ARRAY( head->gyr );

        for ( ; it_pcl->curvature / double( 1000 ) > head->offset_time; it_pcl-- )
        {
            dt = it_pcl->curvature / double( 1000 ) - head->offset_time;

            /* Transform to the 'end' frame */
            M3D R_i( R_imu * Exp( angvel_avr, dt ) );
            V3D T_ei( pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - state_inout.pos_end );

            V3D P_i( it_pcl->x, it_pcl->y, it_pcl->z );
            V3D P_compensate =
                Lid_rot_to_IMU.transpose() *
                ( state_inout.rot_end.transpose() * ( R_i * ( Lid_rot_to_IMU * P_i + Lid_offset_to_IMU ) + T_ei ) - Lid_offset_to_IMU );
            /// save Undistorted points and their rotation
            it_pcl->x = P_compensate( 0 );
            it_pcl->y = P_compensate( 1 );
            it_pcl->z = P_compensate( 2 );

            if ( it_pcl == pcl_out.points.begin() )
                break;
        }
    }
    // cout<<"[ IMU Process ]: undistort size: "<<pcl_out.points.size()<<endl;
}

void ImuProcess::Process2( LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_ )
{
    double t1, t2, t3;
    t1 = omp_get_wtime();
    ROS_ASSERT( lidar_meas.lidar != nullptr );
    if ( !imu_en )
    {
        Forward_without_imu( lidar_meas, stat, *cur_pcl_un_ );
        return;
    }

    MeasureGroup meas = lidar_meas.measures.back();

    if ( imu_need_init_ )
    {
        double pcl_end_time = lidar_meas.is_lidar_end ? lidar_meas.lidar_beg_time + lidar_meas.lidar->points.back().curvature / double( 1000 )
                                                      : lidar_meas.lidar_beg_time + lidar_meas.measures.back().img_offset_time;
        lidar_meas.last_update_time = pcl_end_time;

        if ( meas.imu.empty() )
        {
            return;
        };
        /// The very first lidar frame
        IMU_init( meas, stat, init_iter_num );

        imu_need_init_ = true;

        last_imu_ = meas.imu.back();

        if ( init_iter_num > MAX_INI_COUNT )
        {
            // cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);
            imu_need_init_ = false;
            ROS_INFO( "IMU Initials: Gravity: %.4f %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f \n",
                      stat.gravity[ 0 ], stat.gravity[ 1 ], stat.gravity[ 2 ], mean_acc.norm(), cov_acc[ 0 ], cov_acc[ 1 ], cov_acc[ 2 ],
                      cov_gyr[ 0 ], cov_gyr[ 1 ], cov_gyr[ 2 ] );
            ROS_INFO( "IMU Initials: ba covarience: %.8f %.8f %.8f; bg covarience: %.8f %.8f %.8f", cov_bias_acc[ 0 ], cov_bias_acc[ 1 ],
                      cov_bias_acc[ 2 ], cov_bias_gyr[ 0 ], cov_bias_gyr[ 1 ], cov_bias_gyr[ 2 ] );
            fout_imu.open( DEBUG_FILE_DIR( "imu.txt" ), ios::out );
        }

        return;
    }

    UndistortPcl( lidar_meas, stat, *cur_pcl_un_ );
}

#endif
