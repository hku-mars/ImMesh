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

#include "tools_my_shader_common.h"

namespace Common_tools
{
    
class Point_cloud_shader
{
  public:
    Point_cloud_shader()
    {
    }
    ~Point_cloud_shader()
    {
        glDeleteVertexArrays( 1, &m_VAO );
        glDeleteBuffers( 1, &m_VBO );
    }
    std::shared_ptr< Shader >    m_pt_shader=nullptr, m_pt_shader_single_color=nullptr;
    std::vector< Point_element > m_pts_vector;
    unsigned int                 m_VBO, m_VAO;
    bool                         m_if_set_pt_buffer = false;
    unsigned char                m_pt_stencil_id = 0xF0;
    int                          m_pts_size = 10;
    double                       m_pts_alpha = 1.0;
    int                          m_edge_size = 0;
    glm::vec4                    m_edge_color = glm::vec4( 1.0, 1.0, 1.0, 1.0 );
    int                          m_rviz_like_cluster_number = 100;
    int                          m_point_step = 1;
    int                          m_draw_points_number = -1;
    void find_min_max_of_points(int s_index, int e_index, Point_element & min_pt, Point_element & max_pt)
    {
       
        if ( s_index < 0 )
            s_index = 0;
        if ( e_index > m_pts_vector.size() )
            e_index = m_pts_vector.size();
        min_pt = m_pts_vector[ s_index ];
        max_pt = m_pts_vector[ s_index ];
        for ( int i = s_index; i < e_index; i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                min_pt.m_pos[ j ] = std::min( min_pt.m_pos[ j ], m_pts_vector[ i ].m_pos[ j ] );
                max_pt.m_pos[ j ] = std::max( max_pt.m_pos[ j ], m_pts_vector[ i ].m_pos[ j ] );
            }
        }
    }

    void make_rviz_like_color()
    {
        // int cluster_number = m_pts_vector.size() / m_rviz_like_cluster_number;
        int cluster_number = 1e3;
        for(int cluster_idx = 0; cluster_idx < m_pts_vector.size(); cluster_idx+=cluster_number)
        {
            Point_element pt_min, pt_max;
            int s_index = cluster_idx;
            int e_index = cluster_idx+ cluster_number;
            
            find_min_max_of_points( s_index, e_index, pt_min, pt_max );
            float min_value = pt_min.m_pos[2];
            float max_value = pt_max.m_pos[2];
            float mix_max = (pt_max.m_pos[2] - pt_min.m_pos[2]);
            
            for(int pt_idx = s_index; pt_idx < e_index; pt_idx++ )
            {
                float val = (m_pts_vector[pt_idx].m_pos[2] - min_value) / mix_max;
                tinycolormap::Color c = tinycolormap::GetColor(1.0-val, tinycolormap::ColormapType::Heat);
                m_pts_vector[pt_idx].pack_color(c.r(), c.g(), c.b());
            }
        }
    }

    void init_single_color_shader(const std::string & shader_path )
    {
        std::string vertex_shader_single = std::string( shader_path ).append( "points_shader_rgb.vs" );
        std::string fragment_single_color_shader = std::string( shader_path ).append( "points_shader_single_color.fs" );
        m_pt_shader_single_color = std::make_shared< Shader >( vertex_shader_single.c_str(), fragment_single_color_shader.c_str() );
        m_pt_shader_single_color->use();
        m_pt_shader_single_color->setFloat( "pointAlpha", 1.0 );
        m_pt_shader_single_color->setFloat( "pointSize", 10.0 );
        m_pt_shader_single_color->setVec4("edgeColor", m_edge_color );
    }

    void init( const std::string & shader_path )
    {
        std::string vertex_shader = std::string( shader_path ).append( "points_shader_rgb.vs" );
        std::string fragment_shader = std::string( shader_path ).append( "points_shader_rgb.fs" );
        m_pt_shader = std::make_shared< Shader >( vertex_shader.c_str(), fragment_shader.c_str() );
        m_pt_shader->use();
        m_pt_shader->setFloat( "pointAlpha", 1.0 );
        m_pt_shader->setFloat( "pointSize", 1 );
    }

    void set_point_attr( int pts_size = 10, int edge_size = 10, double pts_alpha = 1.0 )
    {
        m_pts_size = pts_size;
        m_edge_size = edge_size;
        m_pts_alpha = pts_alpha;
    }

    void init_data_buffer()
    {
        // cout << "=== Init data buffer === " << endl;
        if(m_if_set_pt_buffer)
        {
            glDeleteVertexArrays( 1, &m_VAO );
            glDeleteBuffers( 1, &m_VBO );
        }
        m_point_step = std::max(1 , m_point_step);

        glGenVertexArrays( 1, &m_VAO );
        glGenBuffers( 1, &m_VBO );

        glBindVertexArray( m_VAO );
        glBindBuffer( GL_ARRAY_BUFFER, m_VBO );
        
        int stride = 3 * sizeof( float ) + sizeof( float );
        glBufferData( GL_ARRAY_BUFFER, m_pts_vector.size() * ( stride ), m_pts_vector.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride * m_point_step, ( void* ) 0 );
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 1, GL_FLOAT, GL_FALSE, stride * m_point_step, ( void* ) ( 3 * sizeof( float ) ) );
        m_if_set_pt_buffer = true;

    }
    
    template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
    void set_pointcloud( std::vector< Eigen::Matrix< T, 6, 1, option > >& _pts_color_of_maps, int if_rviz_like_color = 1 )
    {
        m_pts_vector.resize(_pts_color_of_maps.size());
        for ( int i = 0; i < _pts_color_of_maps.size(); i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_vector[ i ].m_pos[j] = _pts_color_of_maps[ i ]( j );
            }
            m_pts_vector[ i ].pack_color( _pts_color_of_maps[ i ]( 3 ), _pts_color_of_maps[ i ]( 4 ), _pts_color_of_maps[ i ]( 5 ) );
        }
        if ( if_rviz_like_color )
        {
            make_rviz_like_color();
        }
        init_data_buffer();
    }

    template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
    void set_pointcloud( std::vector< Eigen::Matrix< T, 3, 1, option > >& _pts_color_of_maps, vec_3 pt_color = vec_3( 1.0, 1.0, 1.0 ),
                         bool if_rviz_like_color = false )
    {
        m_pts_vector.resize( _pts_color_of_maps.size() );
        for ( int i = 0; i < _pts_color_of_maps.size(); i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_vector[ i ].m_pos[j] = _pts_color_of_maps[ i ]( j );
            }
            m_pts_vector[ i ].pack_color( pt_color(0),  pt_color(1),  pt_color(2));
        }
        if ( if_rviz_like_color )
        {
            make_rviz_like_color();
        }
        init_data_buffer();
    }

    template < typename T, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
    void set_pointcloud( std::vector< Eigen::Matrix< T, 4, 1, option > >& _pts_color_of_maps, vec_3 pt_color = vec_3( 1.0, 1.0, 1.0 ),
                         bool if_rviz_like_color = false )
    {
        m_pts_vector.resize( _pts_color_of_maps.size() );
        for ( int i = 0; i < _pts_color_of_maps.size(); i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_vector[ i ].m_pos[j] = _pts_color_of_maps[ i ]( j );
            }
            m_pts_vector[ i ].pack_color( pt_color(0),  pt_color(1),  pt_color(2));
        }
        if ( if_rviz_like_color )
        {
            make_rviz_like_color();
        }
        init_data_buffer();
    }

    void set_pointcloud( std::vector< eigen_vec_f<3> >& _pts_pos, std::vector< eigen_vec_uc<4> >& _pts_color, bool if_rviz_like_color = false  )
    {
        m_pts_vector.resize(_pts_pos.size());
        for ( int i = 0; i < _pts_pos.size(); i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_vector[ i ].m_pos[j] = _pts_pos[ i ]( j );
            }
            m_pts_vector[ i ].pack_color( _pts_color[ i ]( 0 ), _pts_color[ i ]( 1 ), _pts_color[ i ]( 2 ) );
            
        }
        if ( if_rviz_like_color )
        {
            make_rviz_like_color();
        }
        init_data_buffer();
    }

    void set_pointcloud( std::vector< eigen_vec_f<3> >& _pts_pos, bool if_rviz_like_color = false )
    {
        m_pts_vector.resize(_pts_pos.size());
        for ( int i = 0; i < _pts_pos.size(); i++ )
        {
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_vector[ i ].m_pos[j] = _pts_pos[ i ]( j );
            }
            m_pts_vector[ i ].pack_color( 1.0, 1.0, 1.0 );
        }
        if ( if_rviz_like_color )
        {
            make_rviz_like_color();
        }
        init_data_buffer();
    }

    void draw( glm::mat4 proj_mat, glm::mat4 pose_mat, GLenum draw_mode = GL_POINTS  )
    {
        Common_tools::Timer tim;
        tim.tic();
        if(m_if_set_pt_buffer == false)
        {
            return;
        }
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
        glEnable( GL_BLEND );
        glm::mat4 projection_mul_view = proj_mat * pose_mat;
        m_pt_shader->use();
        m_pt_shader->setFloat( "pointAlpha", m_pts_alpha );
        m_pt_shader->setFloat( "pointSize", m_pts_size );
        m_pt_shader->setMat4( "projection_mul_view", projection_mul_view );
        int draw_count = m_pts_vector.size();
        if ( m_draw_points_number >= 0 )
        {
            draw_count = std::min( m_draw_points_number, ( int ) m_pts_vector.size() );
            // cout << "Draw count  = " << draw_count << " | " << m_pts_vector.size() << endl;
        }

        if ( draw_mode == GL_LINES || draw_mode == GL_LINE_STRIP || draw_mode == GL_LINE_LOOP )
        {
            glEnable( GL_LINE_WIDTH );
            glLineWidth( m_pts_size );
        }
        else
        {
            if(m_pts_size >=0)
            {
                glEnable( GL_VERTEX_PROGRAM_POINT_SIZE );
                glPointSize( m_pts_size );
            }
        }
        glBindVertexArray( m_VAO );
        glDrawArrays( draw_mode, 0, draw_count / m_point_step );
        glDepthFunc( GL_LESS );
        // cout << "Shader cost time_3 = " << tim.toc_string() << endl;
    }

    void draw( const Eigen::Matrix< double, 4, 4 > proj_mat, const Eigen::Matrix< double, 4, 4 > pose_mat )
    {
        draw( eigen2glm( proj_mat ), Common_tools::eigen2glm( pose_mat ) );
    }
};

}