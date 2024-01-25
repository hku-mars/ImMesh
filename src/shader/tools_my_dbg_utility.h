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

class Axis_shader
{
  public:
    Axis_shader()
    {
    }
    ~Axis_shader()
    {
        if ( m_if_set_pt_buffer )
        {
            glDeleteVertexArrays( 1, &m_VAO );
            glDeleteBuffers( 1, &m_VBO );
        }
    }
    std::shared_ptr< Shader >    m_pt_shader;
    std::vector< Point_element > m_pts_vector;
    unsigned int                 m_VBO, m_VAO;
    bool                         m_if_set_pt_buffer = false;
    unsigned char                m_pt_stencil_id = 0xF0;
    int                          m_pts_size = 10;
    int                          m_point_step = 1;

    void init_pts( float axis_scale = 10 )
    {
        m_pts_vector.clear();
        m_pts_vector.push_back( Point_element( 0, 0, 0, 1.0, 0.0, 0.0 ) );
        m_pts_vector.push_back( Point_element( axis_scale, 0, 0, 1.0, 0.0, 0.0 ) );

        m_pts_vector.push_back( Point_element( 0, 0, 0, 0.0, 1.0, 0.0 ) );
        m_pts_vector.push_back( Point_element( 0, axis_scale, 0, 0.0, 1.0, 0.0 ) );

        m_pts_vector.push_back( Point_element( 0, 0, 0, 0.0, 0.0, 1.0 ) );
        m_pts_vector.push_back( Point_element( 0, 0, axis_scale, 0.0, 0.0, 1.0 ) );
        init_data_buffer();
    }

    void init( std::string shader_path, float axis_scale = 10 )
    {
        std::string vertex_shader = std::string( shader_path ).append( "/points_shader_rgb.vs" );
        std::string fragment_shader = std::string( shader_path ).append( "/points_shader_rgb.fs" );
        m_pt_shader = std::make_shared< Shader >( vertex_shader.c_str(), fragment_shader.c_str() );
        init_pts( axis_scale );
    }

    void init_data_buffer()
    {
        // cout << "=== Init data buffer === " << endl;
        if ( m_if_set_pt_buffer )
        {
            glDeleteVertexArrays( 1, &m_VAO );
            glDeleteBuffers( 1, &m_VBO );
        }
        m_if_set_pt_buffer = true;

        m_point_step = std::max( 1, m_point_step );

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
    }

    void draw( glm::mat4 proj_mat, glm::mat4 pose_mat )
    {
        // printf_line;
        Common_tools::Timer tim;
        tim.tic();

        glm::mat4 projection_mul_view = proj_mat * pose_mat;
        // activate shader
        // glDepthFunc( GL_ALWAYS );
        m_pt_shader->use();
        m_pt_shader->setFloat( "pointAlpha", 1.0 );
        m_pt_shader->setFloat( "pointSize", 1 );
        m_pt_shader->setMat4( "projection_mul_view", projection_mul_view );

        glEnable( GL_LINE_WIDTH );
        glLineWidth( m_pts_size );

        glBindVertexArray( m_VAO );
        glDrawArrays( GL_LINES, 0, m_pts_vector.size() );
        // glDepthFunc( GL_LESS );
    }

    void draw( const Eigen::Matrix< double, 4, 4 > proj_mat, const Eigen::Matrix< double, 4, 4 > pose_mat )
    {
        draw( eigen2glm( proj_mat ), Common_tools::eigen2glm( pose_mat ) );
    }
};

class Ground_plane_shader
{
  public:
    Ground_plane_shader()
    {
    }
    ~Ground_plane_shader()
    {
        if ( m_if_set_pt_buffer )
        {
            glDeleteVertexArrays( 1, &m_VAO );
            glDeleteBuffers( 1, &m_VBO );
        }
    }
    std::shared_ptr< Shader >    m_pt_shader;
    std::vector< Point_element > m_pts_vector;
    unsigned int                 m_VBO, m_VAO;
    bool                         m_if_set_pt_buffer = false;
    unsigned char                m_pt_stencil_id = 0xF0;
    int                          m_pts_size = 1;
    int                          m_point_step = 1;

    void init_pts( float grid_scale = 10, int cell_count = 5 )
    {
        m_pts_vector.clear();
        vec_3 color = vec_3( 0.45, 0.45, 0.45 );
        for ( int i = -cell_count; i <= cell_count; i++ )
        {
            m_pts_vector.push_back( Point_element( i * grid_scale, -cell_count * grid_scale, 0, color( 0 ), color( 1 ), color( 2 ) ) );
            m_pts_vector.push_back( Point_element( i * grid_scale, cell_count * grid_scale, 0, color( 0 ), color( 1 ), color( 2 ) ) );

            m_pts_vector.push_back( Point_element( -cell_count * grid_scale, i * grid_scale, 0, color( 0 ), color( 1 ), color( 2 ) ) );
            m_pts_vector.push_back( Point_element( cell_count * grid_scale, i * grid_scale, 0, color( 0 ), color( 1 ), color( 2 ) ) );
        }

        init_data_buffer();
    }

    void init( std::string shader_path, float axis_scale = 1 , float grid_count = 5 )
    {
        std::string vertex_shader = std::string( shader_path ).append( "/points_shader_rgb.vs" );
        std::string fragment_shader = std::string( shader_path ).append( "/points_shader_rgb.fs" );
        m_pt_shader = std::make_shared< Shader >( vertex_shader.c_str(), fragment_shader.c_str() );
        init_pts( axis_scale, grid_count);
    }

    void init_data_buffer()
    {
        // cout << "=== Init data buffer === " << endl;
        if ( m_if_set_pt_buffer )
        {
            glDeleteVertexArrays( 1, &m_VAO );
            glDeleteBuffers( 1, &m_VBO );
        }
        m_if_set_pt_buffer = true;

        m_point_step = std::max( 1, m_point_step );

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
    }

    void draw( glm::mat4 proj_mat, glm::mat4 pose_mat )
    {
        Common_tools::Timer tim;
        tim.tic();

        glm::mat4 projection_mul_view = proj_mat * pose_mat;

        m_pt_shader->use();
        m_pt_shader->setFloat( "pointAlpha", 1.0 );
        m_pt_shader->setFloat( "pointSize", 1 );
        m_pt_shader->setMat4( "projection_mul_view", projection_mul_view );

        glEnable( GL_LINE_WIDTH );
        glLineWidth( m_pts_size );

        glBindVertexArray( m_VAO );
        glDrawArrays( GL_LINES, 0, m_pts_vector.size() );
        // glDepthFunc( GL_LESS );
    }

    void draw( const Eigen::Matrix< double, 4, 4 > proj_mat, const Eigen::Matrix< double, 4, 4 > pose_mat )
    {
        draw( eigen2glm( proj_mat ), Common_tools::eigen2glm( pose_mat ) );
    }
};

} // namespace Common_tools