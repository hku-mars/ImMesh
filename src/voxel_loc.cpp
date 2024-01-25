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
#include "voxel_loc.hpp"

int g_plane_id = 0;
int g_max_layer = 4;
int g_max_points = 1000;

void OctoTree::init_plane( const std::vector< Point_with_var > &points, Plane *plane )
{
    plane->m_plane_var = Eigen::Matrix< double, 6, 6 >::Zero();
    plane->m_covariance = Eigen::Matrix3d::Zero();
    plane->m_center = Eigen::Vector3d::Zero();
    plane->m_normal = Eigen::Vector3d::Zero();
    plane->m_points_size = points.size();
    plane->m_radius = 0;
    for ( auto pv : points )
    {
        plane->m_covariance += pv.m_point * pv.m_point.transpose();
        plane->m_center += pv.m_point;
    }
    plane->m_center = plane->m_center / plane->m_points_size;
    plane->m_covariance = plane->m_covariance / plane->m_points_size - plane->m_center * plane->m_center.transpose();
    Eigen::EigenSolver< Eigen::Matrix3d > es( plane->m_covariance );
    Eigen::Matrix3cd                      evecs = es.eigenvectors();
    Eigen::Vector3cd                      evals = es.eigenvalues();
    Eigen::Vector3d                       evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff( &evalsMin );
    evalsReal.rowwise().sum().maxCoeff( &evalsMax );
    int             evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col( evalsMin );
    Eigen::Vector3d evecMid = evecs.real().col( evalsMid );
    Eigen::Vector3d evecMax = evecs.real().col( evalsMax );
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->m_points_size, 0, 0, 0, 1.0 / plane->m_points_size, 0, 0, 0, 1.0 / plane->m_points_size;
    // && evalsReal(evalsMid) > 0.05
    //&& evalsReal(evalsMid) > 0.01
    if ( evalsReal( evalsMin ) < m_planer_threshold_ )
    {
        for ( int i = 0; i < points.size(); i++ )
        {
            Eigen::Matrix< double, 6, 3 > J;
            Eigen::Matrix3d               F;
            for ( int m = 0; m < 3; m++ )
            {
                if ( m != ( int ) evalsMin )
                {
                    Eigen::Matrix< double, 1, 3 > F_m = ( points[ i ].m_point - plane->m_center ).transpose() /
                                                        ( ( plane->m_points_size ) * ( evalsReal[ evalsMin ] - evalsReal[ m ] ) ) *
                                                        ( evecs.real().col( m ) * evecs.real().col( evalsMin ).transpose() +
                                                          evecs.real().col( evalsMin ) * evecs.real().col( m ).transpose() );
                    F.row( m ) = F_m;
                }
                else
                {
                    Eigen::Matrix< double, 1, 3 > F_m;
                    F_m << 0, 0, 0;
                    F.row( m ) = F_m;
                }
            }
            J.block< 3, 3 >( 0, 0 ) = evecs.real() * F;
            J.block< 3, 3 >( 3, 0 ) = J_Q;
            plane->m_plane_var += J * points[ i ].m_var * J.transpose();
        }

        plane->m_normal << evecs.real()( 0, evalsMin ), evecs.real()( 1, evalsMin ), evecs.real()( 2, evalsMin );
        plane->m_min_eigen_value = evalsReal( evalsMin );
        plane->m_radius = sqrt( evalsReal( evalsMax ) );
        plane->m_d = -( plane->m_normal( 0 ) * plane->m_center( 0 ) + plane->m_normal( 1 ) * plane->m_center( 1 ) +
                        plane->m_normal( 2 ) * plane->m_center( 2 ) );
        plane->m_p_center.x = plane->m_center( 0 );
        plane->m_p_center.y = plane->m_center( 1 );
        plane->m_p_center.z = plane->m_center( 2 );
        plane->m_p_center.normal_x = plane->m_normal( 0 );
        plane->m_p_center.normal_y = plane->m_normal( 1 );
        plane->m_p_center.normal_z = plane->m_normal( 2 );
        plane->m_is_plane = true;
        plane->m_is_update = true;
        if ( !plane->m_is_init )
        {
            plane->m_id = g_plane_id;
            g_plane_id++;
            plane->m_is_init = true;
        }

        // Calc Normal and center covariance
    }
    else
    {
        if ( !plane->m_is_init )
        {
            plane->m_id = g_plane_id;
            g_plane_id++;
            plane->m_is_init = true;
        }
        plane->m_is_update = true;
        plane->m_is_plane = false;
    }
}

void OctoTree::init_octo_tree()
{
    if ( m_temp_points_.size() > m_octo_init_size_ )
    {
        init_plane( m_temp_points_, m_plane_ptr_ );
        if ( m_plane_ptr_->m_is_plane == true )
        {
            m_octo_state_ = 0;
        }
        else
        {
            m_octo_state_ = 1;
            cut_octo_tree();
        }
        m_init_octo_ = true;
        m_new_points_ = 0;
        //      m_temp_points_.clear();
    }
}

void OctoTree::cut_octo_tree()
{
    if ( m_layer_ >= m_max_layer_ )
    {
        m_octo_state_ = 0;
        return;
    }
    for ( size_t i = 0; i < m_temp_points_.size(); i++ )
    {
        int xyz[ 3 ] = { 0, 0, 0 };
        if ( m_temp_points_[ i ].m_point[ 0 ] > m_voxel_center_[ 0 ] )
        {
            xyz[ 0 ] = 1;
        }
        if ( m_temp_points_[ i ].m_point[ 1 ] > m_voxel_center_[ 1 ] )
        {
            xyz[ 1 ] = 1;
        }
        if ( m_temp_points_[ i ].m_point[ 2 ] > m_voxel_center_[ 2 ] )
        {
            xyz[ 2 ] = 1;
        }
        int leafnum = 4 * xyz[ 0 ] + 2 * xyz[ 1 ] + xyz[ 2 ];
        if ( m_leaves_[ leafnum ] == nullptr )
        {
            m_leaves_[ leafnum ] = new OctoTree( m_max_layer_, m_layer_ + 1, m_layer_init_num_, m_max_points_size_, m_planer_threshold_ );
            m_leaves_[ leafnum ]->m_voxel_center_[ 0 ] = m_voxel_center_[ 0 ] + ( 2 * xyz[ 0 ] - 1 ) * m_quater_length_;
            m_leaves_[ leafnum ]->m_voxel_center_[ 1 ] = m_voxel_center_[ 1 ] + ( 2 * xyz[ 1 ] - 1 ) * m_quater_length_;
            m_leaves_[ leafnum ]->m_voxel_center_[ 2 ] = m_voxel_center_[ 2 ] + ( 2 * xyz[ 2 ] - 1 ) * m_quater_length_;
            m_leaves_[ leafnum ]->m_quater_length_ = m_quater_length_ / 2;
        }
        m_leaves_[ leafnum ]->m_temp_points_.push_back( m_temp_points_[ i ] );
        m_leaves_[ leafnum ]->m_new_points_++;
    }
    for ( uint i = 0; i < 8; i++ )
    {
        if ( m_leaves_[ i ] != nullptr )
        {
            if ( m_leaves_[ i ]->m_temp_points_.size() > m_leaves_[ i ]->m_octo_init_size_ )
            {
                init_plane( m_leaves_[ i ]->m_temp_points_, m_leaves_[ i ]->m_plane_ptr_ );
                if ( m_leaves_[ i ]->m_plane_ptr_->m_is_plane )
                {
                    m_leaves_[ i ]->m_octo_state_ = 0;
                }
                else
                {
                    m_leaves_[ i ]->m_octo_state_ = 1;
                    m_leaves_[ i ]->cut_octo_tree();
                }
                m_leaves_[ i ]->m_init_octo_ = true;
                m_leaves_[ i ]->m_new_points_ = 0;
                // leaves_[i]->temp_points_.clear();
            }
        }
    }
}

void OctoTree::UpdateOctoTree( const Point_with_var &pv )
{
    if ( !m_init_octo_ )
    {
        m_new_points_++;
        m_temp_points_.push_back( pv );
        if ( m_temp_points_.size() > m_octo_init_size_ )
        {
            init_octo_tree();
        }
    }
    else
    {
        if ( m_plane_ptr_->m_is_plane )
        {
            if ( m_update_enable_ )
            {
                m_new_points_++;
                m_temp_points_.push_back( pv );
                if ( m_new_points_ > m_update_size_threshold_ )
                {
                    init_plane( m_temp_points_, m_plane_ptr_ );
                    m_new_points_ = 0;
                }
                if ( m_temp_points_.size() >= m_max_points_size_ )
                {
                    m_update_enable_ = false;
                    std::vector< Point_with_var >().swap( m_temp_points_ );
                    m_new_points_ = 0;
                }
            }
        }
        else
        {
            if ( m_layer_ < m_max_layer_ )
            {
                if ( m_temp_points_.size() != 0 )
                {
                    std::vector< Point_with_var >().swap( m_temp_points_ );
                }
                int xyz[ 3 ] = { 0, 0, 0 };
                if ( pv.m_point[ 0 ] > m_voxel_center_[ 0 ] )
                {
                    xyz[ 0 ] = 1;
                }
                if ( pv.m_point[ 1 ] > m_voxel_center_[ 1 ] )
                {
                    xyz[ 1 ] = 1;
                }
                if ( pv.m_point[ 2 ] > m_voxel_center_[ 2 ] )
                {
                    xyz[ 2 ] = 1;
                }
                int leafnum = 4 * xyz[ 0 ] + 2 * xyz[ 1 ] + xyz[ 2 ];
                if ( m_leaves_[ leafnum ] != nullptr )
                {
                    m_leaves_[ leafnum ]->UpdateOctoTree( pv );
                }
                else
                {
                    m_leaves_[ leafnum ] = new OctoTree( m_max_layer_, m_layer_ + 1, m_layer_init_num_, m_max_points_size_, m_planer_threshold_ );
                    m_leaves_[ leafnum ]->m_voxel_center_[ 0 ] = m_voxel_center_[ 0 ] + ( 2 * xyz[ 0 ] - 1 ) * m_quater_length_;
                    m_leaves_[ leafnum ]->m_voxel_center_[ 1 ] = m_voxel_center_[ 1 ] + ( 2 * xyz[ 1 ] - 1 ) * m_quater_length_;
                    m_leaves_[ leafnum ]->m_voxel_center_[ 2 ] = m_voxel_center_[ 2 ] + ( 2 * xyz[ 2 ] - 1 ) * m_quater_length_;
                    m_leaves_[ leafnum ]->m_quater_length_ = m_quater_length_ / 2;
                    m_leaves_[ leafnum ]->UpdateOctoTree( pv );
                }
            }
            else
            {
                if ( m_update_enable_ )
                {
                    m_new_points_++;
                    m_temp_points_.push_back( pv );
                    if ( m_new_points_ > m_update_size_threshold_ )
                    {
                        init_plane( m_temp_points_, m_plane_ptr_ );
                        m_new_points_ = 0;
                    }
                    if ( m_temp_points_.size() > g_max_points )
                    {
                        m_update_enable_ = false;
                        std::vector< Point_with_var >().swap( m_temp_points_ );
                        // m_temp_points_.clear();
                    }
                }
            }
        }
    }
}

void OctoTree::updatePlane()
{
    if ( m_temp_points_.size() >= m_update_size_threshold_ )
    {
        Eigen::Matrix3d old_covariance = m_plane_ptr_->m_covariance;
        Eigen::Vector3d old_center = m_plane_ptr_->m_center;
        Eigen::Matrix3d sum_ppt =
            ( m_plane_ptr_->m_covariance + m_plane_ptr_->m_center * m_plane_ptr_->m_center.transpose() ) * m_plane_ptr_->m_points_size;
        Eigen::Vector3d sum_p = m_plane_ptr_->m_center * m_plane_ptr_->m_points_size;
        for ( size_t i = 0; i < m_temp_points_.size(); i++ )
        {
            const Point_with_var &pv = m_temp_points_[ i ];
            sum_ppt += pv.m_point * pv.m_point.transpose();
            sum_p += pv.m_point;
        }
        m_plane_ptr_->m_points_size = m_plane_ptr_->m_points_size + m_temp_points_.size();
        m_plane_ptr_->m_center = sum_p / m_plane_ptr_->m_points_size;
        m_plane_ptr_->m_covariance = sum_ppt / m_plane_ptr_->m_points_size - m_plane_ptr_->m_center * m_plane_ptr_->m_center.transpose();
        Eigen::EigenSolver< Eigen::Matrix3d > es( m_plane_ptr_->m_covariance );
        Eigen::Matrix3cd                      evecs = es.eigenvectors();
        Eigen::Vector3cd                      evals = es.eigenvalues();
        Eigen::Vector3d                       evalsReal; //注意这里定义的MatrixXd里没有c
        evalsReal = evals.real();                        //获取特征值实数部分
        Eigen::Matrix3f::Index evalsMin, evalsMax;
        evalsReal.rowwise().sum().minCoeff( &evalsMin );
        evalsReal.rowwise().sum().maxCoeff( &evalsMax );
        // std::cout << "min eigen value:" << evalsReal(evalsMin) <<
        // std::endl;
        if ( evalsReal( evalsMin ) < m_planer_threshold_ )
        {
            m_plane_ptr_->m_normal << evecs.real()( 0, evalsMin ), evecs.real()( 1, evalsMin ), evecs.real()( 2, evalsMin );
            m_plane_ptr_->m_min_eigen_value = evalsReal( evalsMin );
            m_plane_ptr_->m_radius = sqrt( evalsReal( evalsMax ) );
            m_plane_ptr_->m_d =
                -( m_plane_ptr_->m_normal( 0 ) * m_plane_ptr_->m_center( 0 ) + m_plane_ptr_->m_normal( 1 ) * m_plane_ptr_->m_center( 1 ) +
                   m_plane_ptr_->m_normal( 2 ) * m_plane_ptr_->m_center( 2 ) );
            m_plane_ptr_->m_p_center.x = m_plane_ptr_->m_center( 0 );
            m_plane_ptr_->m_p_center.y = m_plane_ptr_->m_center( 1 );
            m_plane_ptr_->m_p_center.z = m_plane_ptr_->m_center( 2 );
            m_plane_ptr_->m_p_center.normal_x = m_plane_ptr_->m_normal( 0 );
            m_plane_ptr_->m_p_center.normal_y = m_plane_ptr_->m_normal( 1 );
            m_plane_ptr_->m_p_center.normal_z = m_plane_ptr_->m_normal( 2 );
            m_plane_ptr_->m_is_plane = true;
            // m_temp_points_.clear();
            m_new_points_ = 0;
            m_plane_ptr_->m_is_update = true;
        }
        else
        {
            // plane_ptr_->is_plane = false;
            m_plane_ptr_->m_is_update = true;
            m_plane_ptr_->m_covariance = old_covariance;
            m_plane_ptr_->m_center = old_center;
            m_plane_ptr_->m_points_size = m_plane_ptr_->m_points_size - m_temp_points_.size();
            // m_temp_points_.clear();
            m_new_points_ = 0;
        }
    }
}