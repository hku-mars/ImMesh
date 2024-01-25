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

#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <csignal>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <ros/ros.h>
#include <so3_math.h>
#include <thread>
#include <unistd.h>

#include "IMU_Processing.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "preprocess.h"
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include <opencv2/opencv.hpp>
#include "voxel_mapping.hpp"

#include "tools/tools_color_printf.hpp"
#include "tools/tools_data_io.hpp"
#include "tools/tools_logger.hpp"
#include "tools/tools_color_printf.hpp"
#include "tools/tools_eigen.hpp"
#include "tools/tools_random.hpp"
#include "tools/lib_sophus/so3.hpp"
#include "tools/lib_sophus/se3.hpp"
#include "mesh_rec_display.hpp"
#include "mesh_rec_geometry.hpp"

double g_maximum_pe_error = 40;
double g_initial_camera_exp_tim = 1.0;

double              g_max_incidence_angle = 90;
Common_tools::Timer g_cost_time_logger;

Eigen::Matrix3d g_camera_K;

std::string data_path_file = std::string( Common_tools::get_home_folder() ).append( "/ImMesh_output/" );

int    appending_pts_frame = ( int ) 5e3;
double threshold_scale = 1.0; // normal
double region_size = 10.0;    // normal

double minimum_pts = 0.1 * threshold_scale;
double g_meshing_voxel_size = 0.4 * threshold_scale;

GL_camera g_gl_camera;

bool                        g_if_automatic_save_mesh = false;
float                       g_alpha = 1.0;
extern std::vector< vec_3 > pts_of_maps;

bool             show_background_color = false;
float            camera_focus = 2000;
Global_map       g_map_rgb_pts_mesh( 0 );
Triangle_manager g_triangles_manager;

std::vector< Image_frame > g_image_frame_vec;
std::vector< GLuint >      g_texture_id_vec;
long                       img_width = 0, img_heigh = 0;

LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;
bool                         g_flag_pause = false;
int                          if_first_call = 1;

std::string          g_debug_string;
int                  g_current_frame = -1;

// GUI settting
bool   g_display_mesh = true;
int    g_enable_mesh_rec = true;
int    g_save_to_offline_bin = false;
int    g_display_face = 1;
bool   g_draw_LiDAR_point = true;
float  g_draw_path_size = 2.0;
float  g_display_camera_size = 1.0;
float  g_ply_smooth_factor = 1.0; 
int    g_ply_smooth_k = 20.0;
bool   g_display_main_window = true;
bool   g_display_camera_pose_window = false;
bool   g_display_help_win = false;
bool   g_follow_cam = true;
bool   g_mesh_if_color = false;
bool   g_if_draw_z_plane = true;
bool   g_if_draw_wireframe = false;
bool   g_if_draw_depth = false;
bool   g_if_depth_bind_cam = true;

bool   g_force_refresh_triangle = false;

extern Common_tools::Axis_shader         g_axis_shader;
extern Common_tools::Ground_plane_shader g_ground_plane_shader;

ImVec4 g_mesh_color = ImVec4( 1.0, 1.0, 1.0, 1.0 );

Voxel_mapping voxel_mapping;

void print_help_window( bool *if_display_help_win )
{
    ImGui::Begin( "--- Help ---", if_display_help_win );
    ImGui::Text( "[H]     | Display/Close main windows" );
    ImGui::Text( "[C]     | Show/Close camera pose windows" );
    ImGui::Text( "[T]     | Follow the camera" );
    ImGui::Text( "[D]     | Show/close the depth image" );
    ImGui::Text( "[L]     | Show/close the LiDAR points" );
    ImGui::Text( "[M]     | Show/close the mesh" );
    ImGui::Text( "[S]     | Save camera view" );
    ImGui::Text( "[Z]     | Load camera view" );
    ImGui::Text( "[+/-]   | Increase/Decrease the line width" );
    ImGui::Text( "[F1]   | Display help window" );
    ImGui::Text( "[Space] | To pause the program" );
    ImGui::Text( "[Esc]   | Exit the program" );
    ImGui::End();
}

void get_last_avr_pose( int current_frame_idx, Eigen::Quaterniond &q_avr, vec_3 &t_vec )
{
    const int win_ssd = 1;
    mat_3_3   lidar_frame_to_camera_frame;
    // Clang-format off
    lidar_frame_to_camera_frame << 0, 0, -1, -1, 0, 0, 0, 1, 0;
    // Clang-format on
    q_avr = Eigen::Quaterniond::Identity();
    t_vec = vec_3::Zero();
    if ( current_frame_idx < 1 )
    {
        return;
    }
    int                frame_count = 0;
    int                frame_s = std::max( 0, current_frame_idx - win_ssd );
    vec_3              log_angle_acc = vec_3( 0, 0, 0 );
    Eigen::Quaterniond q_first;
    for ( int frame_idx = frame_s; frame_idx < current_frame_idx; frame_idx++ )
    {
        if ( g_eigen_vec_vec[ frame_idx ].second.size() != 0 )
        {
            Eigen::Quaterniond pose_q( g_eigen_vec_vec[ frame_idx ].second.head< 4 >() );
            pose_q.normalize();
            if ( frame_count == 0 )
            {
                q_first = pose_q;
            }
            q_avr = q_avr * pose_q;
            log_angle_acc += Sophus::SO3d( q_first.inverse() * pose_q ).log();
            t_vec = t_vec + g_eigen_vec_vec[ frame_idx ].second.block( 4, 0, 3, 1 );
            frame_count++;
        }
    }
    t_vec = t_vec / frame_count;
    q_avr = q_first * Sophus::SO3d::exp( log_angle_acc / frame_count ).unit_quaternion();
    q_avr.normalize();
    q_avr = q_avr * Eigen::Quaterniond( lidar_frame_to_camera_frame );
}

int main( int argc, char **argv )
{
    // Setup window
    pcl::console::setVerbosityLevel( pcl::console::L_ALWAYS );
    Common_tools::printf_software_version();
    printf_program( "ImMesh: An Immediate LiDAR Localization and Meshing Framework" );
    
    ros::init( argc, argv, "laserMapping" );
    voxel_mapping.init_ros_node();

    GLFWwindow *window = g_gl_camera.init_openGL_and_ImGUI( "ImMesh: An Immediate LiDAR Localization and Meshing Framework", 1, voxel_mapping.m_GUI_font_size );
    if ( !gladLoadGLLoader( ( GLADloadproc ) glfwGetProcAddress ) )
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }
    Common_tools::Point_cloud_shader  g_pt_shader;
    init_openGL_shader();
    if ( window == nullptr )
    {
        cout << "Window == nullptr" << endl;
        return 0;
    }

    
    g_enable_mesh_rec = voxel_mapping.m_if_enable_mesh_rec;
    cout << "Offline point cloud name: " << ANSI_COLOR_GREEN_BOLD << voxel_mapping.m_pointcloud_file_name << ANSI_COLOR_RESET << endl;
    if ( Common_tools::if_file_exist( voxel_mapping.m_pointcloud_file_name ) )
    {
        pcl::PointCloud< pcl::PointXYZI > offline_pts;
        cout << "Loading data..." ;
        fflush( stdout );
        pcl::io::loadPCDFile( voxel_mapping.m_pointcloud_file_name, offline_pts );
        cout << " total of pts = " << offline_pts.points.size() << endl;
        cout << "g_map_rgb_pts_mesh.m_minimum_pts_size = " << g_map_rgb_pts_mesh.m_minimum_pts_size << endl;
        reconstruct_mesh_from_pointcloud( offline_pts.makeShared() );
    }
    else if(voxel_mapping.m_pointcloud_file_name.length() > 5)
    {
        cout << ANSI_COLOR_RED_BOLD << "Offline point cloud file: " << voxel_mapping.m_pointcloud_file_name <<" NOT exist!!!, Please check!!!" << ANSI_COLOR_RESET << endl;
        while(1);
    }
    
    
    cout << "====Loading parameter=====" << endl;

    threshold_scale = voxel_mapping.m_meshing_distance_scale;
    minimum_pts = voxel_mapping.m_meshing_points_minimum_scale * voxel_mapping.m_meshing_distance_scale;
    g_meshing_voxel_size = voxel_mapping.m_meshing_voxel_resolution * voxel_mapping.m_meshing_distance_scale;
    appending_pts_frame = voxel_mapping.m_meshing_number_of_pts_append_to_map;
    region_size = voxel_mapping.m_meshing_region_size * voxel_mapping.m_meshing_distance_scale;
    g_display_mesh = voxel_mapping.m_if_draw_mesh;
    scope_color( ANSI_COLOR_YELLOW_BOLD );
    cout << "=========Meshing config ========= " << endl;
    cout << "Threshold scale = " << threshold_scale << endl;
    cout << "Minimum pts distance = " << minimum_pts << endl;
    cout << "Voxel size = " << g_meshing_voxel_size << endl;
    cout << "Region size = " << region_size << endl;
 
    g_current_frame = -3e8;
    g_triangles_manager.m_pointcloud_map = &g_map_rgb_pts_mesh;
    g_map_rgb_pts_mesh.set_minimum_dis( minimum_pts );
    g_map_rgb_pts_mesh.set_voxel_resolution( g_meshing_voxel_size );
    g_triangles_manager.m_region_size = region_size;
    g_map_rgb_pts_mesh.m_recent_visited_voxel_activated_time = 0;
    cout << "==== Loading parameter end =====" << endl;

    std::thread thr_mapping = std::thread( &Voxel_mapping::service_LiDAR_update, &voxel_mapping );
    std::thread thr = std::thread( service_refresh_and_synchronize_triangle, 100 );
    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
    Common_tools::Timer disp_tim;

    g_flag_pause = false;

    std::string gl_camera_file_name = Common_tools::get_home_folder().append( "/ImMeshing.gl_camera" );
    g_gl_camera.load_camera( gl_camera_file_name );
    g_gl_camera.m_gl_cam.m_camera_z_far = 1500;
    g_gl_camera.m_gl_cam.m_camera_z_near = 0.1;
    
    // Rasterization configuration
    Cam_view m_depth_view_camera;
    m_depth_view_camera.m_display_w = 640;
    m_depth_view_camera.m_display_h = 480;
    m_depth_view_camera.m_camera_focus = 400;
    m_depth_view_camera.m_maximum_disp_depth = 150.0;
    m_depth_view_camera.m_draw_depth_pts_size = 2;
    m_depth_view_camera.m_draw_LiDAR_pts_size = m_depth_view_camera.m_draw_depth_pts_size;
    m_depth_view_camera.m_if_draw_depth_pts = true;
    vec_3 ext_rot_angle = vec_3( 0, 0, 0 );

    while ( !glfwWindowShouldClose( window ) )
    {
        g_gl_camera.draw_frame_start();

        Eigen::Quaterniond q_last_avr;
        vec_3              t_last_avr;
        get_last_avr_pose( g_current_frame, q_last_avr, t_last_avr );
        if ( g_if_draw_depth )
        {
            Common_tools::Timer tim;
            tim.tic();
            m_depth_view_camera.m_camera_z_far = g_gl_camera.m_gl_cam.m_camera_z_far;
            m_depth_view_camera.m_camera_z_near = g_gl_camera.m_gl_cam.m_camera_z_near;

            if ( g_if_depth_bind_cam )
                m_depth_view_camera.set_camera_pose( q_last_avr.toRotationMatrix(), t_last_avr );
            else
                m_depth_view_camera.set_camera_pose( g_gl_camera.m_gl_cam.m_camera_rot, g_gl_camera.m_gl_cam.m_camera_pos );
            
            m_depth_view_camera.set_gl_matrix();
            
            draw_triangle( m_depth_view_camera );
            
            m_depth_view_camera.read_depth();
            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
            m_depth_view_camera.draw_depth_image();
            g_gl_camera.set_gl_camera_pose_matrix();
            if(m_depth_view_camera.m_if_draw_depth_pts)
            {
                g_draw_LiDAR_point = true;
            }
        }

        if ( g_display_main_window )
        {
            ImGui::Begin( "ImMesh's Main_windows", &g_display_main_window );               // Create a window called "Hello, world!" and append into it.
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.75f), "ImMesh: An Immediate LiDAR Localization and Meshing Framework"); 
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.75f), "Github:  ");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "https://github.com/hku-mars/ImMesh");
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 1.0f, 0.75f), "Author:  ");
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.5f, 0.5f, 1.0f, 1.0f), "Jiarong Lin & Chongjian Yuan");
            if (ImGui::TreeNode("Help"))
            {
                ImGui::Text( "[H]    | Display/Close main windows" );
                ImGui::Text( "[C]    | Show/Close camera pose windows" );
                ImGui::Text( "[T]    | Follow the camera" );
                ImGui::Text( "[D]    | Show/close the depth image" );
                ImGui::Text( "[L]    | Show/close the LiDAR points" );
                ImGui::Text( "[M]    | Show/close the mesh" );
                ImGui::Text( "[S]    | Save camera view" );
                ImGui::Text( "[Z]    | Load camera view" );
                ImGui::Text( "[+/-]  | Increase/Decrease the line width" );
                ImGui::Text( "[F1]   | Display help window" );
                ImGui::Text( "[Space]| To pause the program" );
                ImGui::Text( "[Esc]  | Exit the program" );
                ImGui::TreePop();
                ImGui::Separator();
            }
            ImGui::SetNextItemOpen(true, 1);
            // ImGui::SetNextTreeNodeOpen();
            if (ImGui::TreeNode("Draw Online reconstructed mesh options:"))
            {   
                ImGui::RadioButton("Draw mesh's Facet", &g_display_face, 1); 
                ImGui::RadioButton("Draw mesh's Wireframe", &g_display_face, 0);
                if(ImGui::Checkbox( "Draw mesh with color", &g_mesh_if_color ))
                {
                    g_force_refresh_triangle = true;
                }
                ImGui::TreePop();
                ImGui::Separator();
            } 
            if (ImGui::TreeNode("LiDAR pointcloud reinforcement"))
            {
                ImGui::Checkbox( "Enable", &g_if_draw_depth );
                if(g_if_draw_depth)
                {
                    ImGui::Checkbox( "If depth in sensor frame", &g_if_depth_bind_cam );
                }
                ImGui::SliderInt( "Reinforced point size", &m_depth_view_camera.m_draw_depth_pts_size, 0, 10 );
                ImGui::SliderInt( "LiDAR point size", &m_depth_view_camera.m_draw_LiDAR_pts_size, 0, 10 );
                ImGui::TreePop();
                ImGui::Separator();
            }
            ImGui::Checkbox( "Move follow camera", &g_follow_cam );
            ImGui::Checkbox( "Mapping pause", &g_flag_pause );
            ImGui::Checkbox( "Draw LiDAR point", &g_draw_LiDAR_point );
            if(g_draw_LiDAR_point)
            {
                ImGui::SliderInt( "LiDAR point size", & m_depth_view_camera.m_draw_LiDAR_pts_size, 0, 10 );
            }
            ImGui::Checkbox( "Axis and Z_plane", &g_if_draw_z_plane );
            
            ImGui::SliderFloat( "Path width", &g_draw_path_size, 1.0, 10.0f );
            ImGui::SliderFloat( "Camera size", &g_display_camera_size, 0.01, 10.0, "%lf", ImGuiSliderFlags_Logarithmic );
            
            if ( ImGui::Button( "  Save Mesh to PLY file  " ) )
            {
                int temp_flag = g_flag_pause;
                g_flag_pause = true;
                Common_tools::create_dir( data_path_file );
                save_to_ply_file( std::string( data_path_file ).append( "/rec_mesh.ply" ), g_ply_smooth_factor, g_ply_smooth_k );
                g_flag_pause = temp_flag;
            }

            if ( ImGui::Button( "Load Camera view" ) )
            {
                g_gl_camera.load_camera( gl_camera_file_name );
            }

            if ( ImGui::Button( "Save Camera view" ) )
            {
                cout << "Save view to " << gl_camera_file_name << endl;
                g_gl_camera.save_camera( gl_camera_file_name );
            }
            ImGui::Checkbox( "Show OpenGL camera paras", &g_display_camera_pose_window ); 
            ImGui::Text( "Refresh rate %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate );
            if ( ImGui::Button( "      Exit Program      " ) ) // Buttons return true when clicked (most widgets return true when edited/activated)
                glfwSetWindowShouldClose( window, 1 );

            ImGui::End();
        }

        if(g_draw_LiDAR_point)
        {
            if ( m_depth_view_camera.m_if_draw_depth_pts )
            {
                if ( m_depth_view_camera.m_draw_LiDAR_pts_size > 0 )
                {
                    display_current_LiDAR_pts( g_current_frame, m_depth_view_camera.m_draw_LiDAR_pts_size, vec_4f( 1.0, 1.0, 1.0, 0.85 ) );
                }
                display_reinforced_LiDAR_pts( m_depth_view_camera.m_depth_pts_vec, m_depth_view_camera.m_draw_depth_pts_size, vec_3f( 1.0, 0.0, 1.0 ) );
            }
        }

        if ( g_follow_cam )
        {
            if ( g_current_frame > 1 )
            {
                g_gl_camera.tracking_camera( Eigen::Quaterniond::Identity(), t_last_avr );
            }
        }

        if ( g_display_help_win )
        {
            // display help window
            print_help_window( &g_display_help_win );
        }

        if ( g_display_camera_pose_window )
        {
            g_gl_camera.draw_camera_window( g_display_camera_pose_window );
        }

        if ( g_if_draw_z_plane )
        {
            g_axis_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat,
                                Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ) );
            g_ground_plane_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat,
                                        Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ) );
        }

        if ( g_display_mesh )
        {
            draw_triangle( g_gl_camera.m_gl_cam );
        }

        if ( g_current_frame >= 0 )
        {
            draw_camera_pose( g_current_frame, g_draw_path_size, g_display_camera_size );
            draw_camera_trajectory( g_current_frame + 1, g_draw_path_size);
        }
        
        // For Key-board control
        if ( g_gl_camera.if_press_key( "H" ) )
        {
            g_display_main_window = !g_display_main_window;
        }
        if ( g_gl_camera.if_press_key( "C" ) )
        {
            g_display_camera_pose_window = !g_display_camera_pose_window;
        }
        if ( g_gl_camera.if_press_key( "F" ) )
        {
            g_display_face = !g_display_face;
        }
        if ( g_gl_camera.if_press_key( "Space" ) )
        {
            g_flag_pause = !g_flag_pause;
        }
        if ( g_gl_camera.if_press_key( "S" ) )
        {
            g_gl_camera.save_camera( gl_camera_file_name );
        }
        if ( g_gl_camera.if_press_key( "Z" ) )
        {
            g_gl_camera.load_camera( gl_camera_file_name );
        }
        if ( g_gl_camera.if_press_key( "D" ) )
        {
            g_if_draw_depth = !g_if_draw_depth;
        }
        if ( g_gl_camera.if_press_key( "M" ) )
        {
            g_display_mesh = !g_display_mesh;
        }
        if ( g_gl_camera.if_press_key( "T" ) )
        {
            g_follow_cam = !g_follow_cam;
            if ( g_current_frame > 1 )
            {
                g_gl_camera.set_last_tracking_camera_pos( q_last_avr, t_last_avr );
            }
        }
        if ( g_gl_camera.if_press_key( "Escape" ) )
        {
            glfwSetWindowShouldClose( window, 1 );
        }
        
        if ( g_gl_camera.if_press_key( "F1" ) )
        {
            g_display_help_win = !g_display_help_win;
        }

        g_gl_camera.set_gl_camera_pose_matrix();
        g_gl_camera.draw_frame_finish();
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow( window );
    glfwTerminate();

    return 0;
}
