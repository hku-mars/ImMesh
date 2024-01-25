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
#include "openGL_camera.hpp"

void Cam_view::refresh_pose()
{
    m_camera_pose_mat44 = Sophus::SE3d( m_camera_rot, m_camera_pos ).matrix();
    m_camera_pose_mat44_inverse = m_camera_pose_mat44.inverse();
}

double* Cam_view::get_GL_modelview_matrix_ptr()
{
    refresh_pose();
    return m_camera_pose_mat44_inverse.data();
}

vec_3 Cam_view::get_eula_angle()
{
    refresh_pose();
    m_camera_pose_eula_angle = m_camera_rot.eulerAngles( 0, 1, 2 );
    return m_camera_pose_eula_angle;
}


Eigen::Matrix< float, 4, 4, Eigen::ColMajor > opencvToOpenGLProjectionMatrix(float fx, float fy, float cx, float cy, float near, float far, int windowWidth, int windowHeight)
{
    // Generate for chatGPT
    float aspect = static_cast<float>(windowWidth) / windowHeight;

    float top = near * tanf(0.5f * fy / fx);
    float bottom = -top;
    float right = aspect * top;
    float left = -right;

    Eigen::Matrix< float, 4, 4, Eigen::ColMajor > projectionMatrix;
    projectionMatrix << 
        2 * near / (right - left), 0, (right + left) / (right - left), 0,
        0, 2 * near / (top - bottom), (top + bottom) / (top - bottom), 0,
        0, 0, -(far + near) / (far - near), -2 * far * near / (far - near),
        0, 0, -1, 0;

    return projectionMatrix;
}

void Cam_view::set_gl_projection( int width , int height , double focus )
{
    if ( width * height != 0 )
    {
        m_display_w = width;
        m_display_h = height;
    }
    if ( focus != 0 )
    {
        m_camera_focus = focus;
    }
    if ( m_display_w % 4 != 0 )
    {
        // Keep data memory aligned to 4 bytes
        // cout << ANSI_COLOR_YELLOW_BOLD << "WARNING: m_display_w is not aligned to 4 bytes, it is " << m_display_w  << endl;
        m_display_w = m_display_w + ( 4 - m_display_w % 4 );
        // cout << "m_display_w is set to " << m_display_w << ANSI_COLOR_RESET << endl;
    }
    m_camera_intrinsic << m_camera_focus, 0, m_display_w / 2, 0, m_camera_focus, m_display_h / 2, 0, 0, 1;
    m_camera_intrinsic_inv = m_camera_intrinsic.inverse();

    float fov_y = 2 * atan2( m_display_h / 2, m_camera_focus );
    m_camera_fov_h = fov_y;
    m_camera_fov_w = 2 * atan2( m_display_w / 2, m_camera_focus );

    if ( 1 )
    {
        m_glm_projection_mat = glm::perspective( fov_y, ( float ) m_display_w / ( float ) m_display_h, m_camera_z_near, m_camera_z_far );
        glMatrixMode( GL_PROJECTION );
        glLoadMatrixf( &m_glm_projection_mat[ 0 ][ 0 ] );
        glViewport( 0, 0, m_display_w, m_display_h );
    }
    else
    {
        glMatrixMode( GL_PROJECTION );
        Eigen::Matrix< float, 4, 4, Eigen::ColMajor > proj_mat = opencvToOpenGLProjectionMatrix(
            m_camera_focus, m_camera_focus, m_display_w / 2, m_display_h / 2, m_camera_z_near, m_camera_z_far, m_display_w, m_display_h );
        for ( int i = 0; i < 4; i++ )
        {
            for ( int j = 0; j < 4; j++ )
            {
                m_glm_projection_mat[ i ][ j ] = proj_mat( j, i );
            }
        }
        glLoadMatrixf( proj_mat.data() );
        glViewport( 0, 0, m_display_w, m_display_h );
    }
}

void Cam_view::set_horizon_fov( double fov_deg )
{
    m_camera_fov_w = fov_deg / (180.0 / M_PI) ;
    m_camera_focus = ( m_display_w / 2.0 ) / tan( m_camera_fov_w / 2.0 );
    float fov_y = 2 * atan2( m_display_h / 2, m_camera_focus );
    m_camera_fov_h = fov_y;
    m_glm_projection_mat = glm::perspective( fov_y, ( float ) m_display_w / ( float ) m_display_h, m_camera_z_near, m_camera_z_far );

    glMatrixMode( GL_PROJECTION );
    glLoadMatrixf( &m_glm_projection_mat[ 0 ][ 0 ] );
    glViewport( 0, 0, m_display_w, m_display_h );
}

void  Cam_view::init_color_bar()
{
    // cout << ANSI_COLOR_BLUE_BOLD <<"Init color bar" << ANSI_COLOR_RESET << endl;
    m_extra_info_vec.push_back( "" );
    m_depth_color_bar_height = m_display_h * 0.05;
    m_depth_color_bar_height = std::max(30, m_depth_color_bar_height);
    m_depth_color_bar.resize( m_depth_color_bar_height*3, m_display_w  );
    m_depth_color_bar.setZero();
    // m_depth_color_bar = cv::Mat::zeros( m_depth_color_bar_height, m_display_w, CV_8UC3 );
    for ( int i = 0; i < m_display_w; i++ )
    {
        tinycolormap::Color color = tinycolormap::GetColor( ( ( float ) i ) / ( ( float ) m_display_w ), m_color_bar_theme );
        for ( int j = 0; j < m_depth_color_bar_height; j++ )
        {
            m_depth_color_bar.data()[ j * m_display_w * 3 + i * 3 + 0 ] = color.bi();
            m_depth_color_bar.data()[ j * m_display_w * 3 + i * 3 + 1 ] = color.gi();
            m_depth_color_bar.data()[ j * m_display_w * 3 + i * 3 + 2 ] = color.ri();
        }
    }

    int m_gl_image_width = 0;
    int m_gl_image_height = 0;
    m_gl_depth_image_color_texture = 0;
    texture_from_eigen_mat( m_depth_color_bar, &m_gl_depth_image_color_texture, &m_gl_image_width, &m_gl_image_height );
}

void Cam_view::draw_depth_image( bool* win_open)
{
    std::unique_lock<std::mutex> lock( m_mutex_depth_image);
    convert_depth_buffer_to_truth_depth(&m_depth_buffer_mat);
    bool res = depth_image_to_gl_image();
    if ( res )
    {
        ImGuiWindowFlags win_tag;
        int display_h = m_display_h;
        int display_w = m_display_w;
        float horizon_fov = m_camera_fov_w * (180.0 / M_PI) ;
        set_horizon_fov(horizon_fov);
        if ( !m_gui_dy_config )
        {
            win_tag = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar;
        }
        else
        {
            win_tag = ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking;
        }
        // ImGui::Begin( "OpenGL Texture test", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoBackground |
        // ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar  );
        ImGui::Begin( "Depth image", win_open, win_tag );
        // init_color_bar();

        ImGui::PushStyleColor( ImGuiCol_WindowBg, ImVec4( 0.15f, 0.15f, 0.15f, 1.0f ) ); // Set window background color

        imgui_text_horizon_ratio( string_format( "LiDAR pointcloud reinforcement" ), 0.5 );

        if ( m_gui_dy_config )
        {
            ImGui::Checkbox( "Dynamic configuration enabled", &m_gui_dy_config );
        }
        else
        {
            ImGui::Checkbox( "Dynamic configuration disabled", &m_gui_dy_config );
        }
        if ( m_gui_dy_config )
        {
            imgui_text_horizon_ratio( string_format( "Image resolution = %d x %d", m_display_w, m_display_h ), 0.5 );
            // imgui_text_horizon_ratio( string_format( "Focus = %.2f", m_camera_focus) , 0.5 );
            // ImGui::SliderFloat( "Focus ", &m_camera_focus, 100, 5000.0, "%lf", ImGuiSliderFlags_Logarithmic );
            //ImGui::Text( "pointer = %p , %p", m_gl_depth_image_texture, m_gl_depth_image_color_texture  );
            ImGui::InputInt( "Image width", &m_display_w, 50, 100 );
            ImGui::InputInt( "Image height", &m_display_h, 50, 100 );
            imgui_text_horizon_ratio( string_format( "Depth FoV = [ %.1f° x %.1f° ]", m_camera_fov_w * (180.0 / M_PI), m_camera_fov_h * (180.0 / M_PI) ), 0.5 );
            ImGui::SliderFloat( "Horizontal FoV", &horizon_fov, 10.0, 179.9, "%.1f");
            ImGui::InputFloat( "Focus", &m_camera_focus, m_camera_focus * 0.05, m_camera_focus * 0.1, "%.1f" );
            ImGui::InputFloat( "Z-Near", &m_camera_z_near, m_camera_z_near * 0.05, m_camera_z_near * 0.1, "%.1f" );
            ImGui::InputFloat( "Z-Far", &m_camera_z_far, m_camera_z_far * 0.05, m_camera_z_far * 0.1, "%.1f" );
            ImGui::InputFloat( "Focus", &m_camera_focus, m_camera_focus * 0.05, m_camera_focus * 0.1, "%.1f" );
            ImGui::Checkbox("Draw 3D Points", & m_if_draw_depth_pts);
            ImGui::SameLine();
            ImGui::Text(" number (%d)", m_depth_avail_pts_count);
            if ( m_if_draw_depth_pts )
            {
                ImGui::SliderInt( "Unprojected point size", &m_draw_depth_pts_size, 0, 10 );
                ImGui::SliderInt( "LiDAR point size", &m_draw_LiDAR_pts_size, 0, 10 );
            }
            // if(m_if_draw_depth_pts)
            // {
            //     ImGui::SliderFloat( "Point size", &m_draw_depth_pts_size, 0.5, 5.0 );
            //     ImGui::SliderFloat4( "Point Color", m_draw_depth_color.data(), 0, 1.0 );
            //     ImGui::SliderFloat( "Downsample resolution", &m_depth_downsample_resolution, 0.0, 1.0 );
            // }
            if ( 1 )
            {
                for ( std::string str : m_extra_info_vec )
                {
                    if ( str.size() > 3 )
                    {
                        ImGui::Text( str.c_str() );
                    }
                }
            }
            // m_camera_focus = std::max( 50.0f, m_camera_focus );
            m_camera_focus = std::min( 5000.0f, m_camera_focus );
            m_display_h = std::max( 50, m_display_h );
            m_display_w = std::max( 50, m_display_w );
            // m_display_h = std::min( m_display_w, m_display_h );
            // m_display_w = std::max( m_display_w, m_display_h );
            m_display_w = std::min( 1920, m_display_w );
            if ( m_display_w % 4 != 0 )
            {
                m_display_w = m_display_w + ( 4 - m_display_w % 4 );
            }
        }
        // ImGui::Text( "Image resolution = %d x %d", m_display_w, m_display_h );
        ImGui::Image( ( void* ) ( intptr_t ) m_gl_depth_image_texture, ImVec2( display_w, display_h ) );

        imgui_text_horizon_ratio( string_format( "Depth (meter)" ), 0.5 );        
        ImGui::Image( ( void* ) ( intptr_t ) m_gl_depth_image_color_texture, ImVec2( display_w, m_depth_color_bar_height ) );
        // imgui_text_horizon_ratio( std::to_string( m_depth_val_min ), 0 );
        imgui_text_horizon_ratio( string_format( "%.3f", m_depth_val_min ), 0 );
        ImGui::SameLine();
        imgui_text_horizon_ratio( string_format( "%.3f", m_depth_val_max ), 1.0 );
        ImGui::SameLine();
        imgui_text_horizon_ratio( string_format( "%.3f", ( m_depth_val_min + m_depth_val_max ) * 0.5 ), 0.5 );

        ImGui::End();
        set_horizon_fov(horizon_fov);
#ifdef USE_OPENCV
        cv::cvtColor( m_cv_depth_image_BGR, m_cv_depth_image_RGB, cv::COLOR_RGB2BGR );
#endif
    }
}

void Cam_view::set_gl_model_view()
{
    glViewport( 0, 0, m_display_w, m_display_h );
    glMatrixMode( GL_MODELVIEW );
    glLoadMatrixd( get_GL_modelview_matrix_ptr() );
}

void Cam_view::set_camera_pose( const mat_3_3& rot, const vec_3 t_vec )
{
    m_camera_rot = rot;
    m_camera_pos = t_vec;
}

void Cam_view::set_gl_matrix()
{
    set_gl_projection();
    set_gl_model_view();
}

void Cam_view::draw_depth_3d_pts( double pts_size, vec_4f color )
{
    if(m_depth_avail_pts_count < 10)
    {
        return;
    }
    if(pts_size <= 0 )
    {
        return;
    }
    // cout << "Draw depth pts, point count = " << m_depth_avail_pts_count << endl;
    glPointSize( pts_size );
    glColor4f( color.x(), color.y(), color.z(), color.w() );
    glVertexPointer( 3, GL_FLOAT, sizeof( vec_3f ), m_depth_pts_vec.data() ); // Should make sure that the data format.
    glEnableClientState( GL_VERTEX_ARRAY );
    glDrawArrays( GL_POINTS, 0, m_depth_avail_pts_count );
    glDisableClientState( GL_VERTEX_ARRAY );
}

void Cam_view::convert_depth_buffer_to_truth_depth(EIGEN_DEPTH_BUFFER_MAT_TYPE * depth_mat)
{
    m_depth_val_min = m_camera_z_far;
    m_depth_val_max = m_camera_z_near;
    int width = depth_mat->cols();
    int height = depth_mat->rows();
    int img_size = width * height;
    // cout << "convert_depth_buffer_to_truth_depth size = " << width << " x " << height << endl;
    for ( int i = 0; i < img_size; i++ )
    {
        int pos_x = i / width;
        int pos_y = i % width;
        // float val = depth_mat.at< float >( pos_x, pos_y );
        float val = (*depth_mat)( pos_x, pos_y );
        val = get_truth_depth(val, m_camera_z_near, m_camera_z_far);
        if( val < m_camera_z_far*0.99 )
        {
            if(val > m_depth_val_max)
            {
                m_depth_val_max = val;
            }
            if(val < m_depth_val_min)
            {
                m_depth_val_min = val;
            }
            (*depth_mat)( pos_x, pos_y ) = val;
        }
        else
        {
            (*depth_mat)( pos_x, pos_y ) = -1;
        }
    }
    m_depth_val_max =  std::min( m_maximum_disp_depth, m_depth_val_max );

    float depth_min_max = m_depth_val_max - m_depth_val_min;
    // depth_min_max = std::min( m_maximum_disp_depth, depth_min_max );
    // m_depth_image.release();
    m_depth_RGB_image.setZero();

    m_depth_avail_pts_count = 0;
    // cout << "Width = " << width << ", height = " << height << endl;
    clear_downsample_hash();
    for ( int i = 0; i < img_size ; i++ )
    {
        int   img_pos_y = i / width;
        int   img_pos_x = i % width;
        img_pos_y = std::max( 1, img_pos_y );
        img_pos_y = std::min( height - 1, img_pos_y );
        float val = (*depth_mat)( height - img_pos_y, img_pos_x );
        // float val = (*depth_mat)( img_pos_y, img_pos_x );
        // float val = (*depth_mat)( img_pos_y, img_pos_x );
        if ( val >= 0 )
        {
            if ( m_if_draw_depth_pts )
            {
                vec_3f depth_pt = unproject_point( img_pos_x, img_pos_y, val ).cast< float >();
                if ( downsample_pts_result( depth_pt ) )
                {
                    m_depth_pts_vec[ m_depth_avail_pts_count ] = depth_pt;
                    m_depth_avail_pts_count++;
                }
            }
            val = ( val - m_depth_val_min ) / depth_min_max;
            tinycolormap::Color color = tinycolormap::GetColor( val, m_color_bar_theme );
            // printf("[%d, %d], ", height - img_pos_y, img_pos_x + 0 );
            // fflush(stdout);
            m_depth_RGB_image( i * 3 + 0 ) = color.bi();
            m_depth_RGB_image( i * 3 + 1 ) = color.gi();
            m_depth_RGB_image( i * 3 + 2 ) = color.ri();
        }
    }
}

int Cam_view::depth_image_to_3d_points()
{
    m_depth_avail_pts_count = 0;
    int width = m_depth_buffer_mat.cols();
    int height = m_depth_buffer_mat.rows();
    int img_size = width * height;
    for ( int i = 0; i < img_size; i++ )
    {
        int   img_pos_y = i / width;
        int   img_pos_x = i % width;
        float val = m_depth_buffer_mat( height - img_pos_y, img_pos_x );
        if ( val >= 0 )
        {
            m_depth_pts_vec[ m_depth_avail_pts_count ] = unproject_point( img_pos_x, img_pos_y, val ).cast< float >();
            m_depth_avail_pts_count++;
        }
    }
}

vec_3 Cam_view::unproject_point( float cursor_x, float cursor_y, float pt_depth )
{
    vec_3 pt_in_screen_coord = m_camera_intrinsic_inv * vec_3( cursor_x, cursor_y, 1 );
    pt_in_screen_coord = pt_in_screen_coord * pt_depth / pt_in_screen_coord( 2 );
    vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
    vec_3 clicked_pts = m_camera_rot * pt_in_gl_camera_coord + m_camera_pos;
    return clicked_pts;
}


void Cam_view::read_depth(  int depth_buffer_precision  )
{
    std::unique_lock<std::mutex> lock( m_mutex_depth_image);
    if ( depth_buffer_precision == 16 )
    {
        try
        {
            if((m_depth_buffer_mat.cols() != m_display_w) || (m_depth_buffer_mat.rows() != m_display_h))
            {
                // cout << ANSI_COLOR_CYAN_BOLD << "Allocating memory for depth image: " <<  m_display_w << " x " <<  m_display_h << ANSI_COLOR_RESET << endl;

                // m_image_mat = cv::Mat::zeros( m_display_h, m_display_w, CV_32F );
                // m_image_aft_flip = cv::Mat::zeros( m_display_h, m_display_w, CV_32F );
                m_depth_buffer_mat.resize( m_display_h, m_display_w );
                m_depth_RGB_image.resize( m_display_h * 3 , m_display_w );
                m_depth_pts_vec.resize( m_display_w * m_display_h );
                m_downsample_kd_hash.m_map_3d_hash_map.reserve( m_display_w * m_display_h );
                m_gl_depth_image_texture = 0;
                m_gl_depth_image_color_texture = 0;
                init_color_bar();
#ifdef USE_OPENCV
                // cv::Mat m_depth_image = cv::Mat::zeros( m_display_h, m_display_w, CV_8UC3 );
                m_cv_depth_image_BGR = cv::Mat( m_depth_RGB_image.rows() / 3, m_depth_RGB_image.cols(), CV_8UC3, m_depth_RGB_image.data() );
#endif
                // m_image_mat = cv::Mat::zeros( m_display_h, m_display_w, CV_16UC1 );
                // m_image_aft_flip = cv::Mat::zeros( m_display_h, m_display_w, CV_16UC1 );
            }
            Common_tools::Timer tim;
            tim.tic();
            // glReadPixels( 0, 0, m_display_w, m_display_h, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT, m_image_mat.data );
            glReadPixels( 0, 0, m_display_w, m_display_h, GL_DEPTH_COMPONENT, GL_FLOAT, m_depth_buffer_mat.data() );
            // cout << "Read depth cost time = " << tim.toc() << " ms" ;
            // cv::flip( m_image_mat, m_image_aft_flip, 0 );

            //  cout << ", show cost time = " << tim.toc() << " ms" ;
            
            // cout << ", total cost time = " << tim.toc() << " ms" << endl;
        }
        catch ( ... )
        {
            printf( "Error reading depth frame\n" );
            printf_line;
            return;
        }
    }
    else if ( depth_buffer_precision == 24 )
    {
        // TODO
    }
    else if ( depth_buffer_precision == 32 )
    {
        // TODO
    }
    else if ( depth_buffer_precision == 48 )
    {
        // TODO
    }
}