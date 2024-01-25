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
// #include "glad.h"
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "tools/tools_eigen.hpp"
#include "tools/tools_logger.hpp"
#include "tools/lib_sophus/se3.hpp"
#include "tools/tools_kd_hash.hpp"
#include "tools/tools_serialization.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#include <set>
#include "gl_draw_founction.hpp"
#include "tinycolormap.hpp"
#define OPENGL_CAMERA_VERSION "V1.2"

#define USE_OPENCV
#ifdef USE_OPENCV
#include "opencv2/opencv.hpp"
#endif

template < typename T >
inline Eigen::Matrix< T, 4, 4, Eigen::RowMajor > init_projection_matrix( int win_w, int win_h, T fx, T fy, T win_center_w, T win_center_h,
                                                                         T clip_z_near, T clip_z_far )
{
    // http://www.songho.ca/opengl/gl_projectionmatrix.html
    // scope_color(ANSI_COLOR_YELLOW_BOLD);
    // cout << "Win_w = " << win_w << ", win_h = " << win_h << endl;
    const T n = clip_z_near;
    const T f = clip_z_far;
    const T l = +( win_center_w ) *n / -fx;
    const T t = +( win_center_h ) *n / fy;
    const T r = -( win_w - win_center_w ) * n / -fx;
    const T b = -( win_h - win_center_h ) * n / fy;

    Eigen::Matrix< T, 4, 4, Eigen::ColMajor > mat_proj;
    mat_proj.setZero();
    mat_proj( 0, 0 ) = 2 * n / ( r - l );
    mat_proj( 1, 1 ) = 2 * n / ( t - b );
    mat_proj( 2, 2 ) = -( f + n ) / ( f - n );
    mat_proj( 2, 0 ) = ( r + l ) / ( r - l );
    mat_proj( 2, 1 ) = ( t + b ) / ( t - b );
    mat_proj( 2, 3 ) = -1.0;
    mat_proj( 3, 2 ) = -( 2 * f * n ) / ( f - n );
    // cout  << "Projection matrix is: " << endl << mat_proj  << endl;
    return mat_proj;
}

struct imgui_key_funcs
{
    static bool IsLegacyNativeDupe( ImGuiKey key )
    {
        return key < 512 && ImGui::GetIO().KeyMap[ key ] != -1;
    }
};

inline bool texture_from_cv_mat( const cv::Mat& cv_image, GLuint* out_texture, int* out_width, int* out_height )
{
    // Load from file
    int image_width = 0;
    int image_height = 0;

    if ( cv_image.cols == 0 )
        return false;
    image_width = cv_image.cols;
    image_height = cv_image.rows;

    // Create a OpenGL texture identifier
    GLuint image_texture = 3e8;
    if ( *out_texture == 0 )
    {
        glGenTextures( 1, &image_texture );
        *out_texture = image_texture;
    }
    glBindTexture( GL_TEXTURE_2D, *out_texture );
    // Setup filtering parameters for display
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE ); // This is required on WebGL for non power-of-two textures
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE ); // Same
    // Upload pixels into texture
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, cv_image.data );
    // glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB_INTEGER, GL_UNSIGNED_SHORT, cv_image.data);
    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;
    return true;
}


inline bool texture_from_eigen_mat( const Eigen::Matrix< unsigned char, -1, -1 > & eigen_image, GLuint* out_texture, int* out_width, int* out_height )
{
    // Load from file
    int image_width = 0;
    int image_height = 0;

    if ( eigen_image.cols() == 0 )
        return false;
    
    image_width = eigen_image.cols() ;
    image_height = eigen_image.rows() / 3 ;
    // cout << "Eigen mat size = " << image_width << " x " << image_height << endl;
    // Create a OpenGL texture identifier
    GLuint image_texture = 3e8;
    if ( *out_texture == 0 )
    {
        glGenTextures( 1, &image_texture );
        *out_texture = image_texture;
    }
    glBindTexture( GL_TEXTURE_2D, *out_texture );
    // Setup filtering parameters for display
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE ); // This is required on WebGL for non power-of-two textures
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE ); // Same
    // Upload pixels into texture
    // glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, eigen_image.data() );
    glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, eigen_image.data());
    *out_texture = image_texture;
    *out_width = image_width;
    *out_height = image_height;
    return true;
}

template < typename... Args >
inline std::string string_format( const std::string& format, Args... args )
{
    int size_s = std::snprintf( nullptr, 0, format.c_str(), args... ) + 1; // Extra space for '\0'
    if ( size_s <= 0 )
    {
        throw std::runtime_error( "Error during formatting." );
    }
    auto                      size = static_cast< size_t >( size_s );
    std::unique_ptr< char[] > buf( new char[ size ] );
    std::snprintf( buf.get(), size, format.c_str(), args... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

typedef Eigen::Matrix< float, -1, -1, Eigen::DontAlign | Eigen::RowMajor  > EIGEN_DEPTH_BUFFER_MAT_TYPE;

struct Cam_view
{
    int   m_display_w = 640;
    int   m_display_h = 480;
    float m_camera_focus = 3000.0;
    float m_camera_z_near = 0.05;
    float m_camera_z_far = 200;
    double m_camera_fov_w = 0;
    double m_camera_fov_h = 0;
    // cv::Mat m_image_mat ;
    // cv::Mat m_image_aft_flip;

    EIGEN_DEPTH_BUFFER_MAT_TYPE                                                m_depth_buffer_mat;
    Eigen::Matrix< unsigned char, -1, -1, Eigen::DontAlign | Eigen::ColMajor > m_depth_RGB_image; // I don't know why here is colMajor
    Eigen::Matrix< unsigned char, -1, -1, Eigen::DontAlign | Eigen::ColMajor > m_depth_color_bar;
#ifdef USE_OPENCV
    cv::Mat m_cv_depth_image_BGR, m_cv_depth_image_RGB;
#endif
    GLuint                         m_gl_depth_image_texture = 0;
    std::mutex                     m_mutex_depth_image;
    vec_3                          m_camera_pos = vec_3( 0, 0, 1 );
    Eigen::Matrix< double, 3, 3 >  m_camera_rot = Eigen::Matrix< double, 3, 3 >::Identity();
    Eigen::Matrix< double, 4, 4 >  m_camera_pose_mat44;
    Eigen::Matrix< double, 4, 4 >  m_camera_pose_mat44_inverse;
    Eigen::Matrix< double, 3, 3 >  m_camera_intrinsic;
    Eigen::Matrix< double, 3, 3 >  m_camera_intrinsic_inv;
    glm::mat4                      m_glm_projection_mat;
    vec_3                           m_camera_pose_eula_angle = vec_3( 0, 0, 0 );
    
    // For depth points downsample
    Hash_map_3d<long, vec_3f> m_downsample_kd_hash;
    float m_depth_downsample_resolution = 0.01;
    std::vector< vec_3f > m_depth_pts_vec, m_camera_frame_pts_vec;
    
    int m_depth_avail_pts_count = 0;
    tinycolormap::ColormapType m_color_bar_theme =  tinycolormap::ColormapType::Heat;
    
    int m_depth_color_bar_height = 20;
    
    GLuint m_gl_depth_image_color_texture = 0; 
    std::vector<std::string> m_extra_info_vec;
    double m_cost_time = 0.0;
    bool m_gui_dy_config = false;
    float m_depth_val_min;
    float m_depth_val_max;
    bool  m_if_draw_depth_pts = false;
    int m_draw_depth_pts_size = 2;
    int m_draw_LiDAR_pts_size = 2;
    float                   m_maximum_disp_depth = 100;
    vec_4f m_draw_depth_color = vec_4f( 1.0, 0, 1.0, 1.0 );
    void    refresh_pose();
    double* get_GL_modelview_matrix_ptr();
    vec_3   get_eula_angle();
    void    set_gl_projection( int width = 0, int height = 0, double focus = 0 );
    void    set_gl_model_view();
    void    set_camera_pose( const mat_3_3& rot, const vec_3 t_vec );
    void    set_gl_matrix();
    void    read_depth(  int depth_buffer_precision = 16 );
    vec_3   unproject_point( float cursor_x, float cursor_y, float pt_depth );
    void init_color_bar();
    int     depth_image_to_3d_points();
    void    draw_depth_3d_pts( double pts_size, vec_4f color = vec_4f( 1.0, 1.0, 1.0, 0.5 ) );
    void    draw_depth_3d_pts()
    {
        draw_depth_3d_pts( m_draw_depth_pts_size, m_draw_depth_color );
    }

    void set_horizon_fov(double fov);

    float get_truth_depth( float z_read, float const z_near, const float z_far )
    {
        // https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
        float z_n = 2.0 * z_read - 1.0;
        float z_depth_value = 2.0 * z_near * z_far / ( z_far + z_near - z_n * ( z_far - z_near ) );
        return z_depth_value;
    }
    
    void clear_downsample_hash()
    {
        m_downsample_kd_hash.m_map_3d_hash_map.clear();
    }

    bool downsample_pts_result( const vec_3f& pt )
    {
        if ( m_depth_downsample_resolution <= 0 )
        {
            return true;
        }
        int pt_x = std::round( pt( 0 ) / m_depth_downsample_resolution );
        int pt_y = std::round( pt( 1 ) / m_depth_downsample_resolution );
        int pt_z = std::round( pt( 2 ) / m_depth_downsample_resolution );

        int pt_hash = m_downsample_kd_hash.if_exist( pt_x, pt_y, pt_z );
        if ( pt_hash == 0 )
        {
            m_downsample_kd_hash.insert( pt_x, pt_y, pt_z, pt );
            return true;
        }
        else
        {
            return false;
        }
    }

    bool    depth_image_to_gl_image()
    {
        int m_gl_image_width = 0;
        int m_gl_image_height = 0;
        return texture_from_eigen_mat( m_depth_RGB_image, &m_gl_depth_image_texture, &m_gl_image_width, &m_gl_image_height );
    } 

    void convert_depth_buffer_to_truth_depth(EIGEN_DEPTH_BUFFER_MAT_TYPE * depth_mat);

    void imgui_text_horizon_ratio( std::string text, float ratio  )
    {
        // std::string text = std::to_string( m_depth_val_min );
        // long        windowWidth = ImGui::GetWindowSize().x;
        long        windowWidth = m_display_w + 10;
        long        textWidth = ImGui::CalcTextSize( text.c_str() ).x;
        if ( ratio >= 1.0 )
        {
            ratio = 0.985;
        }
        if ( ratio <= 0.0 )
        {
            ratio = 0.015;
        }
        ImGui::SetCursorPosX( ( windowWidth - textWidth ) * ratio );
        ImGui::Text( text.c_str() );
    }

   

    void draw_depth_image( bool * win_open = nullptr );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        boost::serialization::split_free( ar, *this, version );
    }
};

template < typename Archive >
inline void save( Archive& ar, const Cam_view& cam_view, const unsigned int /*version*/ )
{
     ar << std::string(OPENGL_CAMERA_VERSION);
    ar << cam_view.m_display_w;
    ar << cam_view.m_display_h;
    ar << cam_view.m_camera_z_near;
    ar << cam_view.m_camera_z_far;
    ar << cam_view.m_camera_focus;
    ar << cam_view.m_camera_pos;
    ar << cam_view.m_camera_pose_eula_angle;
    ar << cam_view.m_camera_rot;
    ar << cam_view.m_camera_intrinsic;
    ar << cam_view.m_camera_intrinsic_inv;
    ar << cam_view.m_camera_pose_mat44;
}

template < typename Archive >
inline void load( Archive& ar, Cam_view& cam_view, const unsigned int /*version*/ )
{ 
    std::string version_str;
    ar >> version_str;
    if ( version_str.compare( std::string( OPENGL_CAMERA_VERSION ) ) == 0 )
    {
        ar >> cam_view.m_display_w;
        ar >> cam_view.m_display_h;
        ar >> cam_view.m_camera_z_near;
        ar >> cam_view.m_camera_z_far;
        ar >> cam_view.m_camera_focus;
        ar >> cam_view.m_camera_pos;
        ar >> cam_view.m_camera_pose_eula_angle;
        ar >> cam_view.m_camera_rot;
        ar >> cam_view.m_camera_intrinsic;
        ar >> cam_view.m_camera_intrinsic_inv;
        ar >> cam_view.m_camera_pose_mat44;
    }
    else
    {
        cout << ANSI_COLOR_RED_BOLD << "OPENGL_CAMERA_VERSION mismatch !!!" << ANSI_COLOR_RESET << endl;
    }
}

struct GL_camera
{
    // See the camera system in openGL: https://learnopengl.com/Getting-started/Camera
    
    Cam_view                      m_gl_cam;
    vec_3                         m_cursor_clicked_pts;
    float                         g_rot_sensetive = 0.15;
    float                         g_vec_sensetive = 0.0001;

    float    m_last_clicked_depth = 1.0;
    int      mouse_counter = 0;
    int      scroll_counter = 0;
    float    m_z_depth_value = 0;
    bool     in_left_drag = false, in_right_drag = false, in_middle_drag = false;
    ImGuiIO* m_gui_io = nullptr;
    float    m_last_cursor_depth = 1.0;

    vec_3   m_drag_start_clicked_pt_world;
    vec_2   m_drag_start_clicked_pt_screen;
    double  m_drag_start_clicked_pt_depth = 1.0;
    vec_3   m_drag_start_clicked_camera_pos;
    mat_3_3 m_drag_start_clicked_camera_rot;

    GLFWwindow* m_glfw_window;
    GLFWwindow* m_glfw_window_depth;
    // OpenGL light and material
    GLfloat m_light0_ambient[ 4 ] = { 0.3, 0.3f, 0.3f, 0.0f };
    GLfloat m_light0_diffuse[ 4 ] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat m_light0_specular[ 4 ] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat m_light0_position[ 4 ] = { 0.0, 0.0, 1e4, 1.0 };
    GLfloat m_light0_direction[ 4 ] = { 0.0, 0.0, 2.0, 1.0 };

    GLfloat m_mat_ambient[ 4 ] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat m_mat_diffuse[ 4 ] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat m_mat_specular[ 4 ] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat m_mat_shininess[ 4 ] = { 0.0 };
    GLfloat m_mat_emission[ 4 ] = { 0.0, 0.0, 0.0, 1.0 };

    vec_3   m_track_last_follow_translation = vec_3( 0, 0, 0 );
    mat_3_3 m_track_last_follow_rotation = mat_3_3::Identity();
    vec_3   m_track_last_openGL_translation = vec_3( 0, 0, 0 );
    mat_3_3 m_track_last_openGL_rotation = mat_3_3::Identity();
    bool                    m_need_reset_tracking_camera_pos = true;
    void                    draw_cursor_3d_pt();
    std::set< std::string > m_pressed_key;
    double                  m_moving_sensitive = 0.1;
    int                     m_gravity_direction = 2;  // 0: x-axis, 1: y-axis, 2: z-axis
    GL_camera()
    {
        m_gl_cam.m_camera_pos = vec_3( 0, 0, 20 );
        m_gl_cam.m_camera_rot.setIdentity();
        m_gl_cam.refresh_pose();
    }

    void    init_ImGUI_IO( ImGuiIO* io );
    void    set_clicked_depth( float clicked_depth );
    void    set_windows_projection_matrix( float camera_fx, float z_near = 0.1, float z_far = 1000.0 );
    void    set_position( const vec_3& pos );
    void    set_rotation( const vec_3& rot_vec );
    void    rotate_by_roll( double degree );
    void    rotate_by_pitch( double degree );
    void    rotate_by_yaw( double degree );
    void    rotate_by_rpy( double roll, double pitch, double yaw );
    void    rotate_by_rpy( vec_3 roll_pitch_yaw );
    void    move_forward_backward( double dis );
    void    move_left_right( double dis );
    void    move_up_down( double dis );
    void    servo_moving_camera();

    double* get_GL_modelview_matrix_ptr();
    mat_4_4 get_GL_modelview_matrix();
    vec_3   get_eula_angle();
    void    framebuffer_size_callback( GLFWwindow* window, int width, int height );
    void    scroll_callback( GLFWwindow* window, double x_offset, double yoffset );
    void    mouse_callback( GLFWwindow* window, double xpos_In, double ypos_In );
    void    mouse_button_callback( GLFWwindow* window, int button, int action, int mods );
    void    bind_gl_callback_function( GLFWwindow* window );
    float   get_truth_depth( float z_read, float const z_near, const float z_far );
    float   get_screen_pixel_depth( int pos_x, int pos_y );
    float   get_nearest_avail_pixel_depth( int pos_x, int pos_y, int max_layer = 5 );
    double  get_cursor_depth();
    vec_3   unproject_point( float cursor_x, float cursor_y, float pt_depth );
    vec_3   get_cursor_clicked_pts();

    void        set_gl_camera_pose_matrix();
    void        display_parameters();
    void        draw_debug_objects();
    void        draw_camera_window( bool& win_open );
    void        save_camera( std::string filename );
    void        load_camera( std::string filename );
    GLFWwindow* init_openGL_and_ImGUI( const char* window_title, int full_screen = 1 , int font_size_bias = 4);
    void        light_on( bool& light_open, bool show_win = true );
    void        draw_frame_start( ImVec4 clear_color = ImVec4( 0, 0, 0, 1.0 ) );
    void        tracking_camera( const Eigen::Quaterniond& current_camera_q, const vec_3& current_camera_pos );

   
    void set_last_tracking_camera_pos( Eigen::Quaterniond current_camera_q, vec_3 current_camera_pos );
    void draw_frame_finish( int flush_camera_matrix = 1 );
    int if_windows_should_close();
    void destroy_window();
    void init_dock_space();
    void get_pressed_keys();
    bool if_press_key( std::string key );

  private:
    friend class boost::serialization::access;
    template < typename Archive >
    void serialize( Archive& ar, const unsigned int version )
    {
        boost::serialization::split_free( ar, *this, version );
    }
};



template < typename Archive >
inline void save( Archive& ar, const GL_camera& gl_camera, const unsigned int /*version*/ )
{
    ar << std::string(OPENGL_CAMERA_VERSION);
    ar << gl_camera.m_gl_cam;
    ar << gl_camera.m_cursor_clicked_pts;
    ar << gl_camera.g_rot_sensetive;
    ar << gl_camera.g_vec_sensetive;
    ar << gl_camera.m_gravity_direction;
}

template < typename Archive >
inline void load( Archive& ar, GL_camera& gl_camera, const unsigned int /*version*/ )
{
    std::string version_str;
    ar >> version_str;
    if ( version_str.compare( std::string( OPENGL_CAMERA_VERSION ) ) == 0 )
    {
        ar >> gl_camera.m_gl_cam;
        ar >> gl_camera.m_cursor_clicked_pts;
        ar >> gl_camera.g_rot_sensetive;
        ar >> gl_camera.g_vec_sensetive;
        ar >> gl_camera.m_gravity_direction;
    }
    else
    {
        cout << ANSI_COLOR_RED_BOLD << "OPENGL_CAMERA_VERSION mismatch !!!" << ANSI_COLOR_RESET << endl;
    }
}   
