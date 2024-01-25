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
class Camera_pose_shader
{
  public:
    Camera_pose_shader()
    {
    }
    ~Camera_pose_shader()
    {
        glDeleteVertexArrays( 1, &m_frame_vao );
        glDeleteBuffers( 1, &m_frame_vbo );
        glDeleteVertexArrays( 1, &m_image_vao );
        glDeleteBuffers( 2, m_image_vbo );
    }
    std::shared_ptr< Shader >    m_pt_shader, m_image_shader;
    std::vector< Point_element > m_pts_vector;
    unsigned int                 m_frame_vbo, m_frame_vao;
    unsigned int                 m_image_vbo[2], m_image_vao;
    bool                         m_if_set_pt_buffer = false;
    unsigned char                pt_stencil_id = 0xF0;
    int                          m_pts_size = 20;
    double                       m_draw_alpha = 1.0;
    int                          m_edge_size = 50;
    int                          if_draw_edge = true;
    glm::vec4                    m_edge_color = glm::vec4( 1.0, 1.0, 1.0, 1.0 );
    int                          m_rviz_like_cluster_number = 100;
    int                          m_point_step = 1;
    glm::vec3                    m_draw_color = glm::vec3( 1.0, 1.0, 1.0 );
    std::vector< vec_3f > init_camera_frustum_vertex( Eigen::Matrix3f cam_K, int w, int h, float scale )
    {
        std::vector< vec_3f > vertex_vec;
        Eigen::Matrix3f       Kinv = cam_K.inverse();
        float                 u0 = Kinv( 0, 2 );
        float                 v0 = Kinv( 1, 2 );
        float                 fu = Kinv( 0, 0 );
        float                 fv = Kinv( 1, 1 );
        const float           xl = scale * u0;
        const float           xh = scale * ( w * fu + u0 );
        const float           yl = scale * v0;
        const float           yh = scale * ( h * fv + v0 );
        vertex_vec.push_back( vec_3f( xl, yl, scale ) );
        vertex_vec.push_back( vec_3f( xh, yl, scale ) );
        vertex_vec.push_back( vec_3f( xh, yh, scale ) );
        vertex_vec.push_back( vec_3f( xl, yh, scale ) );
        vertex_vec.push_back( vec_3f( xl, yl, scale ) );
        vertex_vec.push_back( vec_3f( 0, 0, 0 ) );
        vertex_vec.push_back( vec_3f( xh, yl, scale ) );
        vertex_vec.push_back( vec_3f( 0, 0, 0 ) );
        vertex_vec.push_back( vec_3f( xl, yh, scale ) );
        vertex_vec.push_back( vec_3f( 0, 0, 0 ) );
        vertex_vec.push_back( vec_3f( xh, yh, scale ) );
        return vertex_vec;
    }

    std::vector< vec_3f > cam_frustum_vertex_vec;
    std::vector< vec_2f > tex_cor_vec;
    std::vector< vec_3f > texture_pts_vec;
    float draw_camera_size = 1.0;
    void init_vertices()
    {
        Eigen::Matrix3f cam_K;
        Eigen::Matrix3d lidar_frame_to_camera_frame;
        // For R3LIVE camera
        // cam_K << 863.4241, 0.0, 640.6808, 0.0, 863.4171, 518.3392, 0.0, 0.0, 1.0;
        // Eigen::Matrix3f       K = cam_K.cast< float >();
        // cam_frustum_vertex_vec = init_camera_frustum_vertex( cam_K, 1280, 1024, 1.5 * draw_camera_size );
        
        // For oxford data
        cam_K << 704.504153913163, 0.0, 708.8501128392297, 0.0, 703.6790228113778, 562.7041625276061, 0.0, 0.0, 1.0 ;
        Eigen::Matrix3f       K = cam_K.cast< float >();
        cam_frustum_vertex_vec = init_camera_frustum_vertex( cam_K, 1440, 1080, 1.5 * draw_camera_size );
        // ===========
        texture_pts_vec.push_back( cam_frustum_vertex_vec[ 0 ] );
        texture_pts_vec.push_back( cam_frustum_vertex_vec[ 1 ] );
        texture_pts_vec.push_back( cam_frustum_vertex_vec[ 2 ] );
        texture_pts_vec.push_back( cam_frustum_vertex_vec[ 3 ] );
        tex_cor_vec.push_back( vec_2f( 0, 0 ) );
        tex_cor_vec.push_back( vec_2f( 1, 0 ) );
        tex_cor_vec.push_back( vec_2f( 1, 1 ) );
        tex_cor_vec.push_back( vec_2f( 0, 1 ) );
        // cout << ANSI_COLOR_GREEN << "init_vertices finish!!!" << cam_K << ANSI_COLOR_RESET << endl;
    }


    void init( std::string shader_path )
    {
        std::string vertex_shader = std::string( shader_path ).append( "/points_shader.vs" );
        std::string fragment_shader = std::string( shader_path ).append( "/points_shader.fs" );
        m_pt_shader = std::make_shared< Shader >( vertex_shader.c_str(), fragment_shader.c_str() );

        m_pt_shader->use();
        m_pt_shader->setFloat( "pointAlpha", 1.0 );
        m_pt_shader->setFloat( "pointSize", 10 );
        m_pt_shader->setVec3( "pointColor", glm::vec3( 1.0, 1.0, 1.0 ) );

        m_image_shader = std::make_shared< Shader >( std::string( shader_path ).append( "/image_shader.vs" ).c_str(),
                                                     std::string( shader_path ).append( "/image_shader.fs" ).c_str() );

        init_vertices();
        init_data_buffer();
    }

    void set_point_attr( int wireframe_size = 10, int edge_size = 10, double pts_alpha = 1.0 )
    {
        m_pts_size = wireframe_size;
        m_edge_size = edge_size;
        m_draw_alpha = pts_alpha;
    }

    void init_data_buffer()
    {
        // cout << "=== Init data buffer === " << endl;
        if(m_if_set_pt_buffer)
        {
            glDeleteVertexArrays( 1, &m_frame_vao );
            glDeleteBuffers( 1, &m_frame_vbo );
        }
        m_point_step = std::max(1 , m_point_step);
        // Init frame vao and vbo
        glGenVertexArrays( 1, &m_frame_vao );
        glGenBuffers( 1, &m_frame_vbo );

        glBindVertexArray( m_frame_vao );
        glBindBuffer( GL_ARRAY_BUFFER, m_frame_vbo );
        
        int stride = 3 * sizeof( float );
        glBufferData( GL_ARRAY_BUFFER, cam_frustum_vertex_vec.size() * ( stride ), cam_frustum_vertex_vec.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride * m_point_step, ( void* ) 0 );

        // Init image vao and vbo
        glGenVertexArrays( 1, &m_image_vao );
        glGenBuffers( 2, m_image_vbo );

        glBindVertexArray( m_image_vao );
        glBindBuffer( GL_ARRAY_BUFFER, m_image_vbo[0] );
        stride = 3 * sizeof( float );
        glBufferData( GL_ARRAY_BUFFER, texture_pts_vec.size() * ( stride ), texture_pts_vec.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 0 );
        glVertexAttribPointer( 0, 3, GL_FLOAT, GL_FALSE, stride * m_point_step, ( void* ) 0 );
        
        glBindBuffer( GL_ARRAY_BUFFER, m_image_vbo[1] );
        stride = 2 * sizeof( float );
        glBufferData( GL_ARRAY_BUFFER, tex_cor_vec.size() * ( stride ), tex_cor_vec.data(), GL_STATIC_DRAW );
        glEnableVertexAttribArray( 1 );
        glVertexAttribPointer( 1, 2, GL_FLOAT, GL_FALSE, stride * m_point_step, ( void* ) 0 );
        
        m_if_set_pt_buffer = true;

    }

    Eigen::Quaterniond m_pose_q;
    vec_3              m_pose_t;
    double             m_scale;
    Eigen::Matrix4d pose_inv_d  = Eigen::Matrix4d::Identity();
    void set_camera_pose_and_scale( Eigen::Quaterniond pose_q, vec_3 pose_t, double scale )
    {
        m_pose_q = pose_q;
        m_pose_t = pose_t;
        m_scale = scale;
        Eigen::Matrix4f    pose = Eigen::Matrix4f::Identity();
        pose.block( 0, 0, 3, 3 ) = ( pose_q.toRotationMatrix() ).cast< float >();
        pose.block( 0, 3, 3, 1 ) = pose_t.cast< float >();
        pose_inv_d = pose.inverse().cast<double>();
        pose_inv_d.block( 0, 0, 3, 3 ) *= scale;

    }

    void draw( glm::mat4 proj_mat, glm::mat4 pose_mat, int texture_id = -1  )
    {
        // printf_line;
        GLenum draw_mode = GL_LINE_STRIP;
        Common_tools::Timer tim;
        tim.tic();

        // pts_vector[0].debug();
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
        glEnable( GL_BLEND );
        // 1st. render pass, draw objects as normal, writing to the stencil buffer
        // --------------------------------------------------------------------
        glStencilFunc( GL_ALWAYS, pt_stencil_id, 0xFF );
        glStencilMask( pt_stencil_id );
        glm::mat4 projection_mul_view =  proj_mat * pose_mat * eigen2glm<double>(pose_inv_d);
        glActiveTexture( GL_TEXTURE0 );
        if ( texture_id >= 0 )
        {
            glBindTexture( GL_TEXTURE_2D, texture_id );
            glBindVertexArray( m_image_vao );
            m_image_shader->use();
            m_image_shader->setMat4( "projection_mul_view", projection_mul_view );
            m_image_shader->setInt( "TexID", 0 );
            // glLineWidth( m_pts_size + 3 );
            // cout << "Number of pts = " << cam_frustum_vertex_vec.size() << "" << endl;
            glDrawArrays( GL_QUADS, 0, 4 );
            // printf("Draw[%d], ", texture_id);
            // fflush(stdout);
        }
        // activate shader
        // glDepthFunc( GL_LESS );
        m_pt_shader->use();
        m_pt_shader->setVec3( "pointColor", m_draw_color );
        m_pt_shader->setFloat( "pointAlpha", m_draw_alpha );
        m_pt_shader->setFloat( "pointSize", m_pts_size );
        m_pt_shader->setMat4( "projection_mul_view", projection_mul_view );
        if ( draw_mode == GL_LINES ||  draw_mode == GL_LINE_STRIP || draw_mode == GL_LINE_LOOP)
        {
            glEnable(GL_LINE_WIDTH);
            glLineWidth( m_pts_size );
        }
        glBindVertexArray( m_frame_vao );
        // cout << "Number of pts = " << cam_frustum_vertex_vec.size() << "" << endl;
        glDrawArrays( draw_mode, 0, cam_frustum_vertex_vec.size() / m_point_step );
    }

    void draw( const Eigen::Matrix< double, 4, 4 > proj_mat, const Eigen::Matrix< double, 4, 4 > pose_mat )
    {
        draw( eigen2glm( proj_mat ), Common_tools::eigen2glm( pose_mat ) );
    }
};
};