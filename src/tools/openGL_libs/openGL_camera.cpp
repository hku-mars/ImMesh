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
#include "gl_draw_founction.hpp"

static void glfw_error_callback( int error, const char* description )
{
    fprintf( stderr, "Glfw Error %d: %s\n", error, description );
}

GLFWwindow* GL_camera::init_openGL_and_ImGUI( const char* window_title, int full_screen, int font_size_bias )
{
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    glfwSetErrorCallback( glfw_error_callback );
    if ( !glfwInit() )
        return nullptr;

    const char* glsl_version = "#version 130";
    glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
    glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 0 );
    GLFWwindow* window = glfwCreateWindow( 1280, 720, window_title, nullptr, NULL );
    if ( window == NULL )
        return nullptr;
    m_glfw_window = window;
    bind_gl_callback_function( m_glfw_window );
    glfwMakeContextCurrent( m_glfw_window );
    glfwSwapInterval( 1 );
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    if ( full_screen )
    {
        glfwMaximizeWindow( m_glfw_window );
    }
    ImGuiIO& io = ImGui::GetIO();
    
    init_ImGUI_IO( &io );
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL( m_glfw_window, true );
    ImGui_ImplOpenGL3_Init( glsl_version );

    set_windows_projection_matrix( 2000.0, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    const GLFWvidmode* vidmode = glfwGetVideoMode( glfwGetPrimaryMonitor() );
    double             font_scale = std::ceil( vidmode->width / ( 3840.0 / 20.0 ) ) + font_size_bias;

    set_windows_projection_matrix( vidmode->width / ( 3840.0 / 2000.0 ), m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
#ifndef FONT_DIR
#else
    cout << "Load font from: " << std::string( FONT_DIR ).append( "/Roboto-Medium.ttf" ) << endl;
    cout << "Display width = " << vidmode->width << endl;
    cout << "Font sclae = " << font_scale << endl;
    io.Fonts->AddFontFromFileTTF( std::string( FONT_DIR ).append( "/Roboto-Medium.ttf" ).c_str(), font_scale );
#endif

    // window = glfwCreateWindow( 640, 480, std::string(window_title).append("_depth").c_str(), nullptr, NULL );
    // if (window == NULL)
    //     return nullptr;
    // m_glfw_window_depth = window;
    // bind_gl_callback_function( m_glfw_window_depth );
    return m_glfw_window;
}

void GL_camera::set_last_tracking_camera_pos( Eigen::Quaterniond current_camera_q, vec_3 current_camera_pos )
{
    m_track_last_follow_translation = current_camera_pos;
    m_track_last_openGL_translation = m_gl_cam.m_camera_pos;

    m_track_last_follow_rotation = current_camera_q.toRotationMatrix();
    m_track_last_openGL_rotation = m_gl_cam.m_camera_rot;

    m_need_reset_tracking_camera_pos = false;
    // cout << ANSI_COLOR_BLUE_BOLD << "Set last camera translation: " << m_track_last_follow_translation.transpose() << ANSI_COLOR_RESET << endl;
}

void GL_camera::draw_frame_start( ImVec4 clear_color )
{
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    glClearColor( clear_color.x, clear_color.y, clear_color.z, clear_color.w );
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_BLEND );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

    glDisable( GL_POINT_SMOOTH );
    glDisable( GL_LINE_SMOOTH );
    // glEnable( GL_LINE_SMOOTH );
    // glEnable( GL_POLYGON_SMOOTH );
    glHint( GL_POINT_SMOOTH_HINT, GL_FASTEST );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    // glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

    glfwWindowHint(GLFW_SAMPLES, 4);
    glEnable(GL_MULTISAMPLE);  

    get_pressed_keys();
    servo_moving_camera();
    init_dock_space();
}

void GL_camera::draw_frame_finish( int flush_camera_matrix )
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );
    glfwSwapBuffers( m_glfw_window );
}

int GL_camera::if_windows_should_close()
{
    return glfwWindowShouldClose( m_glfw_window );
}

void GL_camera::destroy_window()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow( m_glfw_window );
    glfwTerminate();
}

void GL_camera::init_dock_space()
{
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking;
    // If you strip some features of, this demo is pretty much equivalent to calling DockSpaceOverViewport()!
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None | ImGuiDockNodeFlags_PassthruCentralNode | ImGuiDockNodeFlags_AutoHideTabBar;

    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos( viewport->WorkPos );
    ImGui::SetNextWindowSize( viewport->WorkSize );
    ImGui::SetNextWindowViewport( viewport->ID );

    ImGui::PushStyleVar( ImGuiStyleVar_WindowRounding, 0.0f );
    ImGui::PushStyleVar( ImGuiStyleVar_ChildBorderSize, 0.0f );
    ImGui::PushStyleVar( ImGuiStyleVar_WindowBorderSize, 10.0f );
    // ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f)); // Set window background to red
    window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    window_flags |= ImGuiWindowFlags_NoBackground;

    ImGui::Begin( "DockSpace", nullptr, window_flags );
    ImGui::PopStyleVar( 3 );

    // Submit the DockSpace
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGuiID dockspace_id = ImGui::GetID( "MyDockSpace" );
    ImGui::DockSpace( dockspace_id, ImVec2( 0.0f, 0.0f ), dockspace_flags );
    ImGui::End();
}

void GL_camera::get_pressed_keys()
{
    m_pressed_key.clear();
    for ( ImGuiKey key = 0; key < ImGuiKey_COUNT; key++ )
    {
        if ( imgui_key_funcs::IsLegacyNativeDupe( key ) )
            continue;
        if ( ImGui::IsKeyPressed( key ) )
        {
            m_pressed_key.insert( ImGui::GetKeyName( key ) );
        }
    }
}

bool GL_camera::if_press_key( std::string key )
{
    if ( m_pressed_key.find( key ) != m_pressed_key.end() )
    {
        return true;
    }
    return false;
}

void GL_camera::init_ImGUI_IO( ImGuiIO* io )
{
    m_gui_io = io;
}

void GL_camera::set_clicked_depth( float clicked_depth )
{
    if ( clicked_depth < m_gl_cam.m_camera_z_far * 0.8 && clicked_depth > m_gl_cam.m_camera_z_near * 1.2 )
    {
        m_last_clicked_depth = clicked_depth;
    }
}

void GL_camera::set_position( const vec_3& pos )
{
    m_gl_cam.m_camera_pos = pos;
}

void GL_camera::set_rotation( const vec_3& rot_vec )
{
    m_gl_cam.m_camera_rot = Sophus::SO3d::exp( vec_3( rot_vec( 0 ) / 57.3, rot_vec( 1 ) / 57.3, rot_vec( 2 ) / 57.3 ) ).matrix();
}

void GL_camera::rotate_by_roll( double degree )
{
    m_gl_cam.m_camera_rot = m_gl_cam.m_camera_rot * Sophus::SO3d::rotZ( degree / 57.3 ).matrix();
}

void GL_camera::rotate_by_pitch( double degree )
{
    m_gl_cam.m_camera_rot = m_gl_cam.m_camera_rot * Sophus::SO3d::rotX( degree / 57.3 ).matrix();
}

void GL_camera::rotate_by_yaw( double degree )
{
    m_gl_cam.m_camera_rot = m_gl_cam.m_camera_rot * Sophus::SO3d::rotY( degree / 57.3 ).matrix();
}

void GL_camera::rotate_by_rpy( double roll, double pitch, double yaw )
{
    m_gl_cam.m_camera_rot = m_gl_cam.m_camera_rot * ( Sophus::SO3d::exp( vec_3( pitch, yaw, roll ) / 57.3 ).matrix() );
}

void GL_camera::rotate_by_rpy( vec_3 roll_pitch_yaw )
{
    rotate_by_rpy( roll_pitch_yaw( 0 ), roll_pitch_yaw( 1 ), roll_pitch_yaw( 2 ) );
}

void GL_camera::move_forward_backward( double dis )
{
    // printf_line;
    // vec_3  move_vec = (m_gl_cam.m_camera_rot.col(2) * dis) ;
    // cout << "Move_forward_backward, dis = " << move_vec.transpose() << endl;
    m_gl_cam.m_camera_pos += (m_gl_cam.m_camera_rot.col(2) * dis);
}

void GL_camera::move_left_right( double dis )
{
    m_gl_cam.m_camera_pos += (m_gl_cam.m_camera_rot.col(0) * dis);
}
void GL_camera::move_up_down( double dis )
{
    m_gl_cam.m_camera_pos += (m_gl_cam.m_camera_rot.col(1) * dis);
}
void GL_camera::servo_moving_camera()
{
    if(if_press_key("UpArrow"))
    {   
        move_up_down(m_moving_sensitive * m_last_clicked_depth);
    }
    if(if_press_key("DownArrow"))
    {
        move_up_down(-m_moving_sensitive * m_last_clicked_depth);
    }
    if(if_press_key("LeftArrow"))
    {
        move_left_right(-m_moving_sensitive * m_last_clicked_depth);
    }
    if(if_press_key("RightArrow"))
    {
        move_left_right(m_moving_sensitive * m_last_clicked_depth);
    }
    if(if_press_key("PageUp"))
    {
        move_forward_backward(-m_moving_sensitive * m_last_clicked_depth);
    }
    if(if_press_key("PageDown"))
    {
        move_forward_backward(m_moving_sensitive * m_last_clicked_depth);
    }

    if(if_press_key("Home"))
    {
        rotate_by_yaw(-30 );
    }
    if(if_press_key("End"))
    {
        rotate_by_yaw(30 );
    }
    
}

void GL_camera::set_windows_projection_matrix( float camera_focus, float z_near, float z_far )
{
    glfwGetFramebufferSize( m_glfw_window, &m_gl_cam.m_display_w, &m_gl_cam.m_display_h );
    m_gl_cam.m_camera_intrinsic << camera_focus, 0, m_gl_cam.m_display_w / 2, 0, camera_focus, m_gl_cam.m_display_h / 2, 0, 0, 1;
    m_gl_cam.m_camera_intrinsic_inv = m_gl_cam.m_camera_intrinsic.inverse();
    m_gl_cam.m_camera_focus = camera_focus;
    if ( 0 )
    {
        Eigen::Matrix< float, 4, 4 > projection_matrix =
            init_projection_matrix< float >( m_gl_cam.m_display_w, m_gl_cam.m_display_h, camera_focus, camera_focus, m_gl_cam.m_display_w * 0.5,
                                             m_gl_cam.m_display_h * 0.5, z_near, z_far );
        glMatrixMode( GL_PROJECTION );
        glLoadMatrixf( projection_matrix.data() );
    }
    else
    {
        m_gl_cam.set_gl_projection( 0, 0, camera_focus );
        // float fov_y = 2 * atan2( m_gl_cam.m_display_h / 2, camera_focus );
        // m_gl_cam.m_camera_z_near = z_near;
        // m_gl_cam.m_camera_z_far = z_far;
        // m_gl_cam.m_glm_projection_mat = glm::perspective( fov_y, ( float ) m_gl_cam.m_display_w / ( float ) m_gl_cam.m_display_h, z_near, z_far );
        // glMatrixMode( GL_PROJECTION );
        // glLoadMatrixf( &m_gl_cam.m_glm_projection_mat[ 0 ][ 0 ] );
        // if ( 0 )
        // {
        //     // if_first_print = false;
        //     cout << "Display width: " << m_gl_cam.m_display_w << ", heigh: " << m_gl_cam.m_display_h << endl;
        //     cout << "Camera focus = " << camera_focus << ", fov = " << fov_y << endl;
        //     cout << "Camera matrix = \r\n" << m_gl_cam.m_camera_intrinsic << endl;
        //     cout << "Projection matrix is: \r\n" << eigen_mat_f< 4, 4 >( &m_gl_cam.m_glm_projection_mat[ 0 ][ 0 ] ) << endl;
        //     cout << "Computed project mat is: \r\n"
        //          << init_projection_matrix< float >( m_gl_cam.m_display_w, m_gl_cam.m_display_h, camera_focus, camera_focus, m_gl_cam.m_display_w *
        //          0.5, m_gl_cam.m_display_h * 0.5,
        //                                              z_near, z_far )
        //          << endl;
        // }
    }
}

float GL_camera::get_screen_pixel_depth( int pos_x, int pos_y )
{
    float z_read;
    pos_y = m_gl_cam.m_display_h - pos_y;
    if ( pos_x < 1 || ( pos_x > m_gl_cam.m_camera_intrinsic( 0, 2 ) * 2 - 1 ) )
    {
        return m_gl_cam.m_camera_z_far;
    }
    if ( pos_y < 1 || ( pos_y > m_gl_cam.m_camera_intrinsic( 1, 2 ) * 2 - 1 ) )
    {
        return m_gl_cam.m_camera_z_far;
    }
    glReadPixels( pos_x, pos_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z_read );
    float truth_depth = get_truth_depth( z_read, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    return truth_depth;
}

vec_3 GL_camera::unproject_point( float cursor_x, float cursor_y, float pt_depth )
{
    return m_gl_cam.unproject_point(cursor_x, cursor_y, pt_depth);
    // vec_3 pt_in_screen_coord = m_gl_cam.m_camera_intrinsic_inv * vec_3( cursor_x, cursor_y, 1 );
    // pt_in_screen_coord = pt_in_screen_coord * pt_depth / pt_in_screen_coord( 2 );
    // vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
    // vec_3 clicked_pts = m_gl_cam.m_camera_rot * pt_in_gl_camera_coord + m_gl_cam.m_camera_pos;
    // return clicked_pts;
}

vec_3 GL_camera::get_cursor_clicked_pts()
{
    vec_3 clicked_pts( 3e8, 3e8, 3e8 );
    // float depth = get_screen_pixel_depth( m_gui_io->MousePos.x, m_gui_io->MousePos.y );
    float cursor_x = m_gui_io->MousePos.x;
    float cursor_y = m_gui_io->MousePos.y;
    float depth = get_nearest_avail_pixel_depth( cursor_x, cursor_y, 10 );
    if ( depth > 0.8 * m_gl_cam.m_camera_z_far )
    {
        depth = m_last_cursor_depth;
        // return clicked_pts;
    }
    m_last_cursor_depth = depth;
    m_z_depth_value = depth;
    m_cursor_clicked_pts = unproject_point( cursor_x, cursor_y, depth );
    return m_cursor_clicked_pts;
};

void GL_camera::draw_cursor_3d_pt()
{
    // glColor3f( 1, 1, 0 );
    // glPointSize( 20 );
    // glBegin( GL_POINTS );
    // double pos_scale = 0.01;
    // for ( int i = -1; i <= 1; i++ )
    //     for ( int j = -1; j <= 1; j++ )
    //         for ( int k = -1; k <= 1; k++ )
    //         {
    //             glVertex3f( m_cursor_clicked_pts( 0 ) + ( i * pos_scale ), m_cursor_clicked_pts( 1 ) + ( j * pos_scale ),
    //                         m_cursor_clicked_pts( 2 ) + ( k * pos_scale ) );
    //         }
    // // cout << "Draw  pt: " << m_cursor_clicked_pts.transpose() << endl;
    // glEnd();
    if ( in_left_drag || in_middle_drag || in_right_drag )
    {
        glMatrixMode( GL_MODELVIEW );
        glPushMatrix();
        glTranslatef( m_cursor_clicked_pts( 0 ), m_cursor_clicked_pts( 1 ), m_cursor_clicked_pts( 2 ) );
        glColor3f( 0.0, 0.7, 0.0 );
        glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
        glDrawSphere( m_last_cursor_depth * 0.01, 20, 20, 0.2 );
        glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
        glPopMatrix();
    }
}

void GL_camera::set_gl_camera_pose_matrix()
{
    m_gl_cam.set_gl_matrix();
    // set_windows_projection_matrix(m_gl_cam.m_camera_focus, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far);
    // glViewport(0, 0, m_gl_cam.m_display_w, m_gl_cam.m_display_h);
    // glMatrixMode( GL_MODELVIEW );
    // glLoadMatrixd( get_GL_modelview_matrix_ptr() );
}

void GL_camera::framebuffer_size_callback( GLFWwindow* window, int width, int height )
{
    // cout << "Framebuffer_size_callback: " << width << " " << height << endl;
    // cout << "Call member function: " << __FUNCTION__ << endl;
    set_windows_projection_matrix( m_gl_cam.m_camera_focus, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    // z_depth_value = get_truth_depth(z_depth_value, 0.1, 1000.0);
}

void GL_camera::draw_debug_objects()
{
    draw_cursor_3d_pt();
}

void GL_camera::scroll_callback( GLFWwindow* window, double x_offset, double yoffset )
{
    if ( m_gui_io->WantCaptureMouse )
    {
        return;
    }
    // cout << "scroll_callback: " << scroll_counter++ << ", x_offset = " << x_offset << ", y_offset = " << yoffset  << endl;
    vec_3 translation_vec = m_last_cursor_depth * vec_3( 0, 0, -yoffset * 0.1 );
    translation_vec = m_gl_cam.m_camera_rot * translation_vec;
    m_gl_cam.m_camera_pos += translation_vec;

    m_need_reset_tracking_camera_pos = true;
}

void GL_camera::mouse_button_callback( GLFWwindow* window, int button, int action, int mods )
{
    mouse_counter++;
    if ( m_gui_io->WantCaptureMouse )
    {
        in_left_drag = false;
        in_right_drag = false;
        in_middle_drag = false;
        return;
    }

    vec_3 pt_clicked = get_cursor_clicked_pts();
    m_drag_start_clicked_pt_screen = vec_2( m_gui_io->MousePos.x, m_gui_io->MousePos.y );
    m_drag_start_clicked_camera_rot = m_gl_cam.m_camera_rot;
    m_drag_start_clicked_camera_pos = m_gl_cam.m_camera_pos;

    if ( pt_clicked( 0 ) != 3e8 )
    {
        m_drag_start_clicked_pt_world = get_cursor_clicked_pts();
        m_drag_start_clicked_pt_depth = m_z_depth_value;
        // cout << "Refresh mouse button clicked point: " << button << ", click: " << m_drag_start_clicked_pt_world.transpose() << endl;
    }
    else
    {
        m_drag_start_clicked_pt_world = unproject_point( m_gui_io->MousePos.x, m_gui_io->MousePos.y, m_drag_start_clicked_pt_depth );
    }
}

void GL_camera::save_camera( std::string filename )
{
    Common_tools::dump_obj_to_file( this, filename );
}

void GL_camera::load_camera( std::string filename )
{
    if ( Common_tools::if_file_exist( filename ) )
    {
        Common_tools::load_obj_from_file( this, filename );
        cout << ANSI_COLOR_YELLOW_BOLD << "Load camera from file: " << filename << ANSI_COLOR_RESET << endl;
    }
    else
    {
        cout << "File: " << filename << " does not exist, using default camera configuration." << endl;
    }
}

void GL_camera::mouse_callback( GLFWwindow* window, double xpos_In, double ypos_In )
{
    if ( m_gui_io->WantCaptureMouse )
    {
        // cout << "Not wanted" << endl;
        return;
    }

    in_left_drag = ( m_gui_io->MouseDown[ 0 ] == true );
    in_right_drag = ( m_gui_io->MouseDown[ 1 ] == true );
    in_middle_drag = ( m_gui_io->MouseDown[ 2 ] == true );

    double maximum_rot_speed = 1;
    vec_3  rot_rpy = vec_3( 0, 0, 0 );

    if ( in_right_drag && in_left_drag ) // Rotate camera's roll axis
    {
        // ANCHOR - moving camera roll
        // float rotate_roll = ( m_gui_io->MousePos.y - m_drag_start_clicked_pt_screen(1) )  * g_rot_sensetive;
        float rotate_roll = ( m_gui_io->MousePos.x - m_drag_start_clicked_pt_screen( 0 ) ) * g_rot_sensetive;
        m_gl_cam.m_camera_rot = m_drag_start_clicked_camera_rot * Sophus::SO3d::exp( vec_3( 0, 0, rotate_roll ) / 57.3 ).matrix();

        vec_3 pt_in_screen_coord =
            m_gl_cam.m_camera_intrinsic_inv * vec_3( m_drag_start_clicked_pt_screen( 0 ), m_drag_start_clicked_pt_screen( 1 ), 1 );
        pt_in_screen_coord = pt_in_screen_coord * m_drag_start_clicked_pt_depth / pt_in_screen_coord( 2 );
        vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
        m_gl_cam.m_camera_pos = m_drag_start_clicked_pt_world - m_gl_cam.m_camera_rot * pt_in_gl_camera_coord;
    }
    else if ( in_right_drag || in_middle_drag ) // moving camera position
    {
        // ANCHOR - moving camera position
        // cout << "Move camera translation" << endl;
        vec_3 old_pos = m_gl_cam.m_camera_pos;
        vec_3 pt_in_screen_coord = m_gl_cam.m_camera_intrinsic_inv * vec_3( m_gui_io->MousePos.x, m_gui_io->MousePos.y, 1 );
        pt_in_screen_coord = pt_in_screen_coord * m_drag_start_clicked_pt_depth / pt_in_screen_coord( 2 );
        vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
        m_gl_cam.m_camera_pos = m_drag_start_clicked_pt_world - m_gl_cam.m_camera_rot * pt_in_gl_camera_coord;
        // cout << "New pos: " << m_gl_cam.m_camera_pos.transpose()  << " | " << " old = " << old_pos.transpose() << " | del = " << (m_gl_cam.m_camera_pos -  old_pos).transpose() << endl;
    }
    else if ( in_left_drag )
    {
        // cout << "Move camera rotation" << endl;
        // ANCHOR - moving camera rotation
        float   rotate_yaw = ( m_gui_io->MousePos.x - m_drag_start_clicked_pt_screen( 0 ) ) * g_rot_sensetive;
        float   rotate_pitch = ( m_gui_io->MousePos.y - m_drag_start_clicked_pt_screen( 1 ) ) * g_rot_sensetive;
        mat_3_3 temp_mat;
        if(m_gravity_direction == 0)
        {
            temp_mat = Sophus::SO3d::rotX( rotate_yaw / 57.3 ).matrix() * m_drag_start_clicked_camera_rot * Sophus::SO3d::rotX( -rotate_pitch / 57.3 ).matrix();
        }
        else if(m_gravity_direction == 1)
        {
            temp_mat = Sophus::SO3d::rotY( -rotate_yaw / 57.3 ).matrix() * m_drag_start_clicked_camera_rot * Sophus::SO3d::rotX( -rotate_pitch / 57.3 ).matrix();
        }
        else
        {
            temp_mat = Sophus::SO3d::rotZ( -rotate_yaw / 57.3 ).matrix() * m_drag_start_clicked_camera_rot * Sophus::SO3d::rotX( -rotate_pitch / 57.3 ).matrix();
        }
        // m_gl_cam.m_camera_rot = temp_mat;
        m_gl_cam.m_camera_rot = Eigen::Quaterniond( temp_mat ).normalized().toRotationMatrix();

        // vec_3 pt_in_screen_coord = m_camera_intrinsic_inv * vec_3(  m_gui_io->MousePos.x, m_gui_io->MousePos.y, 1 );
        vec_3 pt_in_screen_coord =
            m_gl_cam.m_camera_intrinsic_inv * vec_3( m_drag_start_clicked_pt_screen( 0 ), m_drag_start_clicked_pt_screen( 1 ), 1 );
        pt_in_screen_coord = pt_in_screen_coord * m_drag_start_clicked_pt_depth / pt_in_screen_coord( 2 );
        vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
        m_gl_cam.m_camera_pos = m_drag_start_clicked_pt_world - m_gl_cam.m_camera_rot * pt_in_gl_camera_coord;
    }
    else if ( in_middle_drag && in_left_drag )
    {
        // ANCHOR - moving camera rotation
        float rotate_yaw = ( m_gui_io->MousePos.x - m_drag_start_clicked_pt_screen( 0 ) ) * g_rot_sensetive;
        float rotate_pitch = ( m_gui_io->MousePos.y - m_drag_start_clicked_pt_screen( 1 ) ) * g_rot_sensetive;
        // mat_3_3 temp_mat =  m_drag_start_clicked_camera_rot * Sophus::SO3d::rotX( -rotate_pitch / 57.3 ).matrix() * Sophus::SO3d::rotY( rotate_yaw
        // / 57.3 ).matrix();
        mat_3_3 temp_mat =
            Sophus::SO3d::rotX( rotate_yaw / 57.3 ).matrix() * m_drag_start_clicked_camera_rot * Sophus::SO3d::rotX( -rotate_pitch / 57.3 ).matrix();
        m_gl_cam.m_camera_rot = temp_mat;
        m_gl_cam.m_camera_rot = Eigen::Quaterniond( m_gl_cam.m_camera_rot ).normalized().toRotationMatrix();

        // vec_3 pt_in_screen_coord = m_camera_intrinsic_inv * vec_3(  m_gui_io->MousePos.x, m_gui_io->MousePos.y, 1 );
        vec_3 pt_in_screen_coord =
            m_gl_cam.m_camera_intrinsic_inv * vec_3( m_drag_start_clicked_pt_screen( 0 ), m_drag_start_clicked_pt_screen( 1 ), 1 );
        pt_in_screen_coord = pt_in_screen_coord * m_drag_start_clicked_pt_depth / pt_in_screen_coord( 2 );
        vec_3 pt_in_gl_camera_coord = vec_3( pt_in_screen_coord( 0 ), -pt_in_screen_coord( 1 ), -pt_in_screen_coord( 2 ) );
        m_gl_cam.m_camera_pos = m_drag_start_clicked_pt_world - m_gl_cam.m_camera_rot * pt_in_gl_camera_coord;
    }
    if ( in_left_drag || in_right_drag || in_middle_drag )
    {
        m_need_reset_tracking_camera_pos = true;
    }
}

float GL_camera::get_nearest_avail_pixel_depth( int pos_x, int pos_y, int max_layer )
{
    float z_val = get_screen_pixel_depth( pos_x, pos_y );
    if ( z_val < 0.8 * m_gl_cam.m_camera_z_far )
    {
        m_z_depth_value = z_val;
        return z_val;
    }
    for ( int i = 1; i < max_layer; i++ )
    {
        float angle_step = 360 / ( 2 * i * 2 * i );
        for ( double angle = 0; angle < 360; angle += angle_step )
        {
            int pos_x_tmp = pos_x + i * cos( angle / 57.295779513 );
            int pos_y_tmp = pos_y + i * sin( angle / 57.295779513 );
            z_val = get_screen_pixel_depth( pos_x_tmp, pos_y_tmp );
            if ( z_val < 0.8 * m_gl_cam.m_camera_z_far )
            {
                pos_x = pos_x_tmp;
                pos_y = pos_y_tmp;
                m_z_depth_value = z_val;
                return z_val;
            }
        }
    }
    return m_gl_cam.m_camera_z_far;
}

void GL_camera::bind_gl_callback_function( GLFWwindow* window )
{
    glfwSetWindowUserPointer( window, this );
    auto framebuffer_size_callback_func = []( GLFWwindow* w, int width, int height ) {
        static_cast< GL_camera* >( glfwGetWindowUserPointer( w ) )->framebuffer_size_callback( w, width, height );
    };

    auto mouse_button_callback_func = []( GLFWwindow* w, int button, int action, int mods ) {
        static_cast< GL_camera* >( glfwGetWindowUserPointer( w ) )->mouse_button_callback( w, button, action, mods );
    };

    auto mouse_callback_func = []( GLFWwindow* w, double xpos_In, double ypos_In ) {
        static_cast< GL_camera* >( glfwGetWindowUserPointer( w ) )->mouse_callback( w, xpos_In, ypos_In );
    };

    auto scroll_callback_func = []( GLFWwindow* w, double x_offset, double yoffset ) {
        static_cast< GL_camera* >( glfwGetWindowUserPointer( w ) )->scroll_callback( w, x_offset, yoffset );
    };
    glfwSetFramebufferSizeCallback( window, framebuffer_size_callback_func );
    glfwSetMouseButtonCallback( window, mouse_button_callback_func );
    glfwSetCursorPosCallback( window, mouse_callback_func );
    glfwSetScrollCallback( window, scroll_callback_func );
}

float GL_camera::get_truth_depth( float z_read, float const z_near, const float z_far )
{
    // https://stackoverflow.com/questions/6652253/getting-the-true-z-value-from-the-depth-buffer
    float z_n = 2.0 * z_read - 1.0;
    float z_depth_value = 2.0 * z_near * z_far / ( z_far + z_near - z_n * ( z_far - z_near ) );
    return z_depth_value;
}

double GL_camera::get_cursor_depth()
{
    m_z_depth_value = get_screen_pixel_depth( m_gui_io->MousePos.x, m_gui_io->MousePos.y );
    return m_z_depth_value;
}

void GL_camera::draw_camera_window( bool& win_open )
{
    static int e = 0;

    ImGui::Begin( "Camera Pose", &win_open, ImGuiWindowFlags_NoDocking );
    ImGui::Text( "Camera focus: %.5f", m_gl_cam.m_camera_focus );
    ImGui::Text( "Screen cursor pts: %f, %f", m_gui_io->MousePos.x, m_gui_io->MousePos.y );
    ImGui::Text( "Z_value: %.5f", m_z_depth_value );
    ImGui::Text( "Cursor pts: %.3f, %.3f, %.3f ", m_cursor_clicked_pts( 0 ), m_cursor_clicked_pts( 1 ), m_cursor_clicked_pts( 2 ) );
    // gravity direction
    ImGui::RadioButton( "Gravity-X ", &m_gravity_direction, 0 );
    ImGui::SameLine();
    ImGui::RadioButton( "Gravity-Y ", &m_gravity_direction, 1 );
    ImGui::SameLine();
    ImGui::RadioButton( "Gravity-Z", &m_gravity_direction, 2 );
    
    ImGui::Text( "Focus: %.2f", m_gl_cam.m_camera_focus );
    ImGui::Text( "Camera clipping near: %.2f", m_gl_cam.m_camera_z_near );
    ImGui::Text( "Camera clipping far: %.2f", m_gl_cam.m_camera_z_far );

    // if ( ImGui::SliderFloat( "Camera focus", &m_gl_cam.m_camera_focus, 500, 5000, "%.1f", ImGuiSliderFlags_Logarithmic ) )
    if (ImGui::InputFloat( "Camera focus", &m_gl_cam.m_camera_focus, m_gl_cam.m_camera_focus * 0.05, m_gl_cam.m_camera_focus * 0.1, "%.1f" ))
    {
        m_gl_cam.m_camera_focus = std::max( 500.0f, m_gl_cam.m_camera_focus );
        m_gl_cam.m_camera_focus = std::min( 5000.0f, m_gl_cam.m_camera_focus );
        set_windows_projection_matrix( m_gl_cam.m_camera_focus, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    };
    // if ( ImGui::SliderFloat( "Camera z-near", &m_gl_cam.m_camera_z_near, 0.001, 100.0, "%lf", ImGuiSliderFlags_Logarithmic ) )
    if (ImGui::InputFloat( "Camera z-near", &m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_near * 0.05, m_gl_cam.m_camera_z_near * 0.1, "%.1f" ))
    {
        set_windows_projection_matrix( m_gl_cam.m_camera_focus, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    };
    static float max_z_far = m_gl_cam.m_camera_z_far;
    // if ( ImGui::SliderFloat( "Camera z-far", &m_gl_cam.m_camera_z_far, 0.001, max_z_far * 2, "%lf", ImGuiSliderFlags_Logarithmic ) )
    if (ImGui::InputFloat( "Camera z-far", &m_gl_cam.m_camera_z_far, m_gl_cam.m_camera_z_far * 0.05, m_gl_cam.m_camera_z_far * 0.1, "%.1f" ))
    {
        set_windows_projection_matrix( m_gl_cam.m_camera_focus, m_gl_cam.m_camera_z_near, m_gl_cam.m_camera_z_far );
    };
    ImGui::SliderFloat( "Rotate sensetive", &g_rot_sensetive, 0.001, 5, "%.3f", ImGuiSliderFlags_NoRoundToFormat | ImGuiSliderFlags_Logarithmic );

    // glm::mat4x4 temp_matrix;
    // glGetFloatv( GL_PROJECTION_MATRIX, &temp_matrix[ 0 ][ 0 ] );

    // ImGui::Text( "Current Projection matrix:" );
    // for ( int i = 0; i < 4; i++ )
    // {
    //     ImGui::Text( "%.3f %.3f %.3f %.3f", temp_matrix[ 0 ][ i ], temp_matrix[ 1 ][ i ], temp_matrix[ 2 ][ i ], temp_matrix[ 3 ][ i ] );
    // }

    // glGetFloatv( GL_MODELVIEW_MATRIX, &temp_matrix[ 0 ][ 0 ] );
    // ImGui::Text( "Current view matrix:" );
    // for ( int i = 0; i < 4; i++ )
    // {
    //     ImGui::Text( "%.3f %.3f %.3f %.3f", temp_matrix[ 0 ][ i ], temp_matrix[ 1 ][ i ], temp_matrix[ 2 ][ i ], temp_matrix[ 3 ][ i ] );
    // }
    // ImGui::Text( "Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate );
    ImGui::End();
}

void GL_camera::tracking_camera( const Eigen::Quaterniond& current_camera_q, const vec_3& current_camera_pos )
{
    if ( m_need_reset_tracking_camera_pos )
    {
        set_last_tracking_camera_pos( current_camera_q, current_camera_pos );
    }
    m_gl_cam.m_camera_pos = m_track_last_openGL_translation + ( current_camera_pos - m_track_last_follow_translation );
    m_gl_cam.m_camera_rot =
        m_track_last_openGL_rotation * ( Eigen::Quaterniond( m_track_last_follow_rotation.transpose() ) * current_camera_q ).toRotationMatrix();
}

void GL_camera::light_on( bool& light_open, bool show_win )
{
    glEnable( GL_NORMALIZE );
    if ( 1 )
    {
        m_light0_position[ 0 ] = m_gl_cam.m_camera_pos( 0 );
        m_light0_position[ 1 ] = m_gl_cam.m_camera_pos( 1 );
        m_light0_position[ 2 ] = m_gl_cam.m_camera_pos( 2 );
    }
    if ( show_win )
    {
        ImGui::Begin( "Light", &light_open );

        ImGui::SliderFloat3( "light0_position", m_light0_position, -100.0f, 100.0f );
        float w = ( ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.y ) * 0.5;
        ImGui::Text( "Light source color" );
        ImGui::SetNextItemWidth( w );
        ImGui::ColorPicker3( "##MyColor##5", ( float* ) &m_light0_diffuse,
                             ImGuiColorEditFlags_PickerHueBar | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoInputs );
        ImGui::SameLine();
        ImGui::SetNextItemWidth( w );
        ImGui::ColorPicker3( "##MyColor##6", ( float* ) &m_light0_diffuse,
                             ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar |
                                 ImGuiColorEditFlags_NoSidePreview );
        ImGui::SetNextItemWidth( w * 2 );
        // ImGui::ColorEdit4( "", ( float* ) &light0_diffuse,
        //                    ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_Uint8
        //                    );
        ImGui::SetNextItemWidth( w * 2 );
        // ImGui::ColorEdit4( "", ( float* ) &light0_diffuse,
        //                    ImGuiColorEditFlags_DisplayHSV | ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_Float
        //                    );
        ImGui::SetNextItemWidth( w * 2 );
        ImGui::Text( "  " );
        ImGui::Text( "Environment light" );
        ImGui::SetNextItemWidth( w );
        ImGui::ColorPicker3( "##MyColor##7", ( float* ) &m_light0_ambient,
                             ImGuiColorEditFlags_PickerHueBar | ImGuiColorEditFlags_NoSidePreview | ImGuiColorEditFlags_NoInputs );
        ImGui::SameLine();
        ImGui::SetNextItemWidth( w );
        ImGui::ColorPicker3( "##MyColor##8", ( float* ) &m_light0_ambient,
                             ImGuiColorEditFlags_PickerHueWheel | ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar |
                                 ImGuiColorEditFlags_NoSidePreview );
        ImGui::SetNextItemWidth( w * 2 );
        // ImGui::ColorEdit4( "", ( float* ) &light0_ambient,
        //                    ImGuiColorEditFlags_DisplayRGB | ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_Uint8
        //                    );
        ImGui::SetNextItemWidth( w * 2 );
        // ImGui::ColorEdit4( "", ( float* ) &light0_ambient,
        //                    ImGuiColorEditFlags_DisplayHSV | ImGuiColorEditFlags_NoLabel | ImGuiColorEditFlags_InputHSV | ImGuiColorEditFlags_Float
        //                    );
        ImGui::SetNextItemWidth( w * 2 );

        // glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);

        if ( 1 )
        {
            ImGui::Text( "  " );
            ImGui::SliderFloat3( "mat_ambient", m_mat_ambient, 0.0f, 1.0f );
            ImGui::SliderFloat3( "mat_diffuse", m_mat_diffuse, 0.0f, 1.0f );
            ImGui::SliderFloat3( "mat_specular", m_mat_specular, 0.0f, 1.0f );
            ImGui::SliderFloat3( "mat_shininess", m_mat_shininess, 0.0f, 1.0f );
            ImGui::SliderFloat3( "mat_emission", m_mat_emission, 0.0f, 1.0f );
        }
        ImGui::End();
    }

    glLightfv( GL_LIGHT0, GL_POSITION, m_light0_position );
    glLightfv( GL_LIGHT0, GL_AMBIENT, m_light0_ambient );
    glLightfv( GL_LIGHT0, GL_DIFFUSE, m_light0_diffuse );
    glLightfv( GL_LIGHT0, GL_SPECULAR, m_light0_specular );
    glLightfv( GL_LIGHT0, GL_SPOT_DIRECTION, m_light0_direction );

    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, m_mat_ambient );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, m_mat_diffuse );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, m_mat_specular );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SHININESS, m_mat_shininess );
    glMaterialfv( GL_FRONT_AND_BACK, GL_EMISSION, m_mat_emission );

    glEnable( GL_NORMALIZE );
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);
    glShadeModel( GL_FLAT );
    glEnable( GL_LIGHTING );
    glEnable( GL_LIGHT0 );
    glEnable( GL_DEPTH_TEST );
    glEnable( GL_COLOR_MATERIAL );
    glColorMaterial( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE );
}
