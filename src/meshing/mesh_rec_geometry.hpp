#pragma once

#include <tbb/tbb.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include "opencv2/opencv.hpp"

#include "pointcloud_rgbd.hpp"
#include "triangle.hpp"
#include <iostream>

#include <CGAL/IO/PLY.h>

#include "tools/tools_color_printf.hpp"
#include "tools/tools_data_io.hpp"
#include "tools/tools_logger.hpp"
#include "tools/tools_color_printf.hpp"
#include "tools/tools_eigen.hpp"
#include "tools/tools_random.hpp"
#include "tools/tools_serialization.hpp"
#include "tools/tools_graphics.hpp"

#include "ikd_Tree.h"


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>

#define NUMBER_OF_POSE_SIZE -1
typedef std::vector< std::pair< std::vector< vec_4 >, Eigen::Matrix< double, NUMBER_OF_POSE_SIZE, 1 > > > LiDAR_frame_pts_and_pose_vec;

typedef CGAL::Exact_predicates_inexact_constructions_kernel                     K;

typedef pcl::PointXYZINormal  PointType ;
using PointVector = std::vector< PointType, Eigen::aligned_allocator< PointType > >;

void save_to_ply_file(std::string ply_file = std::string("/home/ziv/temp/ply/rec_mesh_smoothed.ply"), double smooth_factor = 0.1, double knn = 20);
void smooth_all_pts(double smooth_factor = 0.1, double knn = 20);
void triangle_compare( const Triangle_set & remove_triangles, const std::vector< long > & add_triangles, 
Triangle_set & res_remove_triangles, Triangle_set & res_add_triangles, Triangle_set * exist_triangles = nullptr );

void correct_triangle_index(Triangle_ptr & ptr , const vec_3 & camera_center, const vec_3 & _short_axis);
std::vector<RGB_pt_ptr> remove_outlier_pts(const std::vector<RGB_pt_ptr> & rgb_pts_vec, const RGB_voxel_ptr & voxel_ptr );
std::vector<RGB_pt_ptr> retrieve_neighbor_pts_kdtree(const std::vector<RGB_pt_ptr> & rgb_pts_vec );

// Compute angle of vector(pb-pa) and (pc-pa)
double compute_angle( vec_2 &pa, vec_2 &pb, vec_2 &pc );
bool is_face_is_ok( Common_tools::Delaunay2::Face &face, double maximum_angle = 180  );
std::vector< long > delaunay_triangulation( std::vector< RGB_pt_ptr > &rgb_pt_vec, vec_3 &long_axis, vec_3 &mid_axis, vec_3 &short_axis,
                             std::set< long > &convex_hull_index, std::set< long > & inner_hull_index );

std::vector< vec_3f > init_camera_frustum_vertex(Eigen::Matrix3f cam_K, int w, int h, float scale );
void draw_all_camera_pose( float draw_camera_size = 1.0 );
