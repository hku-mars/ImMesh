#pragma once
#include "tools/openGL_libs/glad.h"
#include "tools/openGL_libs/openGL_camera.hpp"
#include "shader/tools_my_texture_triangle_shader.h"
#include "shader/tools_my_camera_pose_shader.h"
#include "shader/tools_my_dbg_utility.h"
#include "shader/tools_my_point_shader.h"
#include "mesh_rec_geometry.hpp"

void init_openGL_shader();
void draw_triangle( const Cam_view &  gl_cam);
void draw_camera_trajectory(int current_frame_idx, float pt_disp_size = 5);
void draw_camera_pose( int current_frame_idx, float pt_disp_size = 5, float display_cam_size = 1.0);
void display_current_LiDAR_pts( int current_frame_idx, double pts_size, vec_4f color = vec_4f( 1.0, 1.0, 1.0, 0.5 ) );
void display_reinforced_LiDAR_pts(  std::vector< vec_3f > & pt_vec, double pts_size, vec_3f color = vec_3f (1.0, 0.0, 1.0) );
void service_refresh_and_synchronize_triangle(double sleep_time = 1);