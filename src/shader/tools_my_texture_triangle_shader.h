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
#define HAVE_OPENCV 1
#include "tools_my_shader_common.h"
#include <glm/glm.hpp>

namespace Common_tools
{
class Triangle_facet_shader
// class Point_cloud_shader
{
  public:
    Triangle_facet_shader()
    {
    }
    ~Triangle_facet_shader()
    {
        glDeleteVertexArrays( 1, &m_vao );
        glDeleteBuffers( 3, m_vbo );
    }
    std::shared_ptr< Shader > m_shader_facet;
    std::vector< vec_3f >     m_pts_pos_vector;
    std::vector< vec_3f >     m_pts_normal_vector;
    std::vector< float >      m_pts_color_vector;
    bool                      m_if_draw_face = true;
    unsigned int              m_vbo[ 3 ], m_vao;
    unsigned char             m_pt_stencil_id = 0xF0;
    int                       m_line_width = 1;
    double                    m_pts_alpha = 1;
    int                       m_edge_size = 0;
    int                       m_if_draw_edge = true;
    int                       m_rviz_like_cluster_number = 100;

    void init( std::string shader_path )
    {
        std::string vertex_shader = std::string( shader_path ).append( "triangle_facets.vs" );
        std::string fragment_shader = std::string( shader_path ).append( "triangle_facets.fs" );
        m_shader_facet = std::make_shared< Shader >( vertex_shader.c_str(), fragment_shader.c_str() );
    }

    void set_point_attr( int pts_size = 10, int edge_size = 10, double pts_alpha = 1.0 )
    {
        m_edge_size = edge_size;
        m_line_width = pts_size;
        m_pts_alpha = pts_alpha;
    }

    // template < typename T >
    // void set_pointcloud( std::vector< Eigen::Matrix< float, 6, 1 > >& _pts_color_of_maps )
    int if_have_init_data_buffer = false;
    void init_data_buffer()
    {
        if ( if_have_init_data_buffer )
        {
        }
        else
        {
            glGenVertexArrays( 1, &m_vao );
            // glGenBuffers( 2, m_vbo );
            glGenBuffers( 3, m_vbo );
        }
        glBindVertexArray( m_vao );
        int stride = 3 * sizeof( float );

        glBindBuffer( GL_ARRAY_BUFFER, m_vbo[ 0 ] );
        glBufferData( GL_ARRAY_BUFFER, m_pts_pos_vector.size() * ( stride ), m_pts_pos_vector.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride, ( void* ) 0 );

        glBindBuffer( GL_ARRAY_BUFFER, m_vbo[ 1 ] );
        glBufferData( GL_ARRAY_BUFFER, m_pts_normal_vector.size() * ( stride ), m_pts_normal_vector.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 3, GL_FLOAT, GL_FALSE, stride, ( void* ) 0 );

        stride = sizeof( float );
        glBindBuffer( GL_ARRAY_BUFFER, m_vbo[ 2 ] );
        glBufferData( GL_ARRAY_BUFFER, m_pts_color_vector.size() * ( stride ), m_pts_color_vector.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 2 );
        glVertexAttribPointer( 2, 1, GL_FLOAT, GL_FALSE, stride, ( void* ) 0 );
        
        if_have_init_data_buffer = true;
    }

    void set_color_by_axis(vec_3f * axis_min_max = nullptr, int select_axis = 2)
    {
        if ( axis_min_max != nullptr )
        {
            m_pts_color_vector.resize( m_pts_pos_vector.size() );
            for ( int i = 0; i < m_pts_pos_vector.size(); i++ )
            {
                float val_min = axis_min_max[ 0 ][ select_axis ];
                float val_max = axis_min_max[ 1 ][ select_axis ];
                float val = (m_pts_pos_vector[ i ][ select_axis ] - val_min) / (val_max - val_min);
                tinycolormap::Color color = tinycolormap::GetColor( 1.0 - val,  tinycolormap::ColormapType::Heat );
                unsigned char r = ( unsigned char ) ( color.r() * 255 );
                unsigned char g = ( unsigned char ) ( color.g() * 255 );
                unsigned char b = ( unsigned char ) ( color.b() * 255 );
                m_pts_color_vector[i] = ( r << 16 ) | ( g << 8 ) | b;
            }
        }
    }

    void set_pointcloud( std::vector< eigen_vec_f<3> >& _pts_pos, vec_3f * axis_min_max = nullptr, int select_axis = 2)
    {
        m_pts_pos_vector.resize(_pts_pos.size());
        m_pts_normal_vector.resize(_pts_pos.size());
        m_pts_color_vector.resize( _pts_pos.size() );
        for ( int i = 0; i < _pts_pos.size(); i+=3 )
        {
            vec_3f normal = ( _pts_pos[ i + 1 ] - _pts_pos[ i ] ).cross( _pts_pos[ i + 2 ] - _pts_pos[ i ] );
            for ( int j = 0; j < 3; j++ )
            {
                m_pts_pos_vector[ i + j ] = _pts_pos[ i + j ];
                m_pts_normal_vector[ i + j ] = normal;
                if(axis_min_max == nullptr)
                    m_pts_color_vector[ i + j ] = ( 255 << 16 ) | ( 255 << 8 ) | 255;
            }
        }
        set_color_by_axis(axis_min_max, select_axis);
        init_data_buffer();
    }

    

    void draw( glm::mat4 proj_mat, glm::mat4 pose_mat )
    {
        Common_tools::Timer tim;
        tim.tic();

        // pts_vector[0].debug();
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
        glEnable( GL_BLEND );
       
        glm::mat4 projection_mul_view = proj_mat * pose_mat;
        glDepthFunc( GL_LESS );
    
        glLineWidth(m_line_width);
        
        glm::mat4 pose_mat_inv = glm::inverse(pose_mat);
        glm::vec3 lightPos(pose_mat_inv[3][0], pose_mat_inv[3][1], pose_mat_inv[3][2]);

        m_shader_facet->use();

        m_shader_facet->setVec3("lightColor", 1.0f, 1.0f, 1.0f);
        m_shader_facet->setVec3("lightPos", lightPos);

        m_shader_facet->setMat4( "projection", proj_mat );
        m_shader_facet->setMat4( "view", pose_mat );
        // m_shader->setFloat( "pointAlpha", m_pts_alpha );
        // m_shader->setFloat( "pointSize", m_line_width );
        // m_shader->setMat4( "projection_mul_view", projection_mul_view );
        // cout << "================================" << endl;
        // render boxes
        glBindVertexArray( m_vao );
        if(m_if_draw_face)
        {
            m_shader_facet->setBool("if_light", true);
            glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        }
        else
        {
            m_shader_facet->setBool("if_light", false);
            glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        }
        glDrawArrays( GL_TRIANGLES, 0, m_pts_pos_vector.size() );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glDepthFunc( GL_LESS );

    }

    void draw( const Eigen::Matrix< double, 4, 4 > proj_mat, const Eigen::Matrix< double, 4, 4 > pose_mat )
    {
        draw( eigen2glm( proj_mat ), Common_tools::eigen2glm( pose_mat ) );
    }
};


} // namespace Common_tools
