#include "mesh_rec_geometry.hpp"
#include "tinycolormap.hpp"
#include <pcl/io/ply_io.h>
#include <tbb/tbb.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <pcl/kdtree/kdtree_flann.h>

extern double g_color_val_min, g_color_val_max;
extern int    g_force_update_flag;

extern Global_map       g_map_rgb_pts_mesh;
extern Triangle_manager g_triangles_manager;
// extern Delaunay g_delaunay;
extern std::vector< std::pair< std::vector< vec_4 >, Eigen::Matrix< double, NUMBER_OF_POSE_SIZE, 1 > > > g_eigen_vec_vec;

double minimum_cell_volume = 0.2 * 0.2 * 0.2 * 0.0;
double minimum_height = 0.000;
double hit_scale = 0.10;
int    skip_count = 0;

// Compute angle of vector(pb-pa) and (pc-pa)
double compute_angle( vec_2 &pa, vec_2 &pb, vec_2 &pc )
{
    vec_2 vec_ab = pb - pa;
    vec_2 vec_ac = pc - pa;
    return acos( ( vec_ab.dot( vec_ac ) ) / ( vec_ab.norm() * vec_ac.norm() ) ) * 57.3;
}

bool is_face_is_ok( Common_tools::Delaunay2::Face &face, double maximum_angle )
{
    // return true;
    maximum_angle = 150;
    if ( maximum_angle == 180 || maximum_angle <= 0 )
    {
        return true;
    }
    vec_2 pt[ 3 ];
    for ( int i = 0; i < 3; i++ )
    {
        pt[ i ] = vec_2( face.vertex( i )->point().x(), face.vertex( i )->point().y() );
    }
    if ( compute_angle( pt[ 0 ], pt[ 1 ], pt[ 2 ] ) > maximum_angle )
    {
        return false;
    }
    if ( compute_angle( pt[ 1 ], pt[ 0 ], pt[ 2 ] ) > maximum_angle )
    {
        return false;
    }
    if ( compute_angle( pt[ 2 ], pt[ 0 ], pt[ 1 ] ) > maximum_angle )
    {
        return false;
    }
    return true;
}

extern double                    g_kd_tree_accept_pt_dis;
void smooth_all_pts( double smooth_factor, double knn )
{
    long num_of_pt_size = g_map_rgb_pts_mesh.m_rgb_pts_vec.size();
    tbb::parallel_for( tbb::blocked_range< size_t >( 0, num_of_pt_size, num_of_pt_size / 12 ), [&]( const tbb::blocked_range< size_t > &r ) {
        for ( long i = r.begin(); i != r.end(); i++ )
        {
            g_map_rgb_pts_mesh.smooth_pts( g_map_rgb_pts_mesh.m_rgb_pts_vec[ i ], smooth_factor, knn, g_kd_tree_accept_pt_dis );
        }
    } );
}

void save_to_ply_file( std::string ply_file, double smooth_factor, double knn )
{
    // std::string ply_file = std::string("/home/ziv/temp/ply/rec_mesh_smooth.ply");
    pcl::PolygonMesh mesh_obj;
    cout << "Save to file " << ply_file << endl;
    long                                num_of_pt_size = g_map_rgb_pts_mesh.m_rgb_pts_vec.size();
    pcl::PointCloud< pcl::PointXYZ > rgb_cloud;
    rgb_cloud.points.resize( num_of_pt_size );

    tbb::parallel_for( tbb::blocked_range< size_t >( 0, num_of_pt_size, num_of_pt_size / 12 ), [&]( const tbb::blocked_range< size_t > &r ) {
        for ( long i = r.begin(); i != r.end(); i++ )
        {
            vec_3  pt_vec_smoothed;
            if(smooth_factor!= 0)
            {
                pt_vec_smoothed = g_map_rgb_pts_mesh.smooth_pts( g_map_rgb_pts_mesh.m_rgb_pts_vec[ i ], smooth_factor, knn, g_kd_tree_accept_pt_dis );
            }
            else
            {
                pt_vec_smoothed =  g_map_rgb_pts_mesh.m_rgb_pts_vec[ i ]->get_pos(0);
            }
            rgb_cloud.points[ i ].x = pt_vec_smoothed( 0 );
            rgb_cloud.points[ i ].y = pt_vec_smoothed( 1 );
            rgb_cloud.points[ i ].z = pt_vec_smoothed( 2 );
        }
    } );
    int                         pt_idx = 0;
    std::vector< Triangle_set > triangle_set_vec;
    int                         total_size = g_triangles_manager.get_all_triangle_list( triangle_set_vec, nullptr, 0 );
    mesh_obj.polygons.reserve( total_size );
    for ( int vec_idx = 0; vec_idx < triangle_set_vec.size(); vec_idx++ )
    {
        for ( Triangle_set::iterator it = triangle_set_vec[ vec_idx ].begin(); it != triangle_set_vec[ vec_idx ].end(); it++ )
        {
            pcl::Vertices face_pcl;
            if ( ( *it )->m_vis_score < 0 )
            {
                continue;
            }
            if ( !( *it )->m_index_flip == 0 )
            {
                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 0 ] );
                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 1 ] );
                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 2 ] );
            }
            else
            {

                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 0 ] );
                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 2 ] );
                face_pcl.vertices.push_back( ( *it )->m_tri_pts_id[ 1 ] );
            }
            mesh_obj.polygons.push_back( face_pcl );
        }
    }
    pcl::toPCLPointCloud2( rgb_cloud, mesh_obj.cloud );
    // int ok = CGAL::IO::write_PLY( ply_file, pts_vec, polygons );
    pcl::io::savePLYFileBinary( ply_file, mesh_obj );
    pcl::io::savePCDFileBinary( std::string(ply_file).append(".pcd"), rgb_cloud );
    cout << "=== Save to " << ply_file << ", finish !!! === " << endl;
}

extern int                  g_current_frame;
extern std::vector< vec_3 > dbg_line_vec;
extern std::mutex           dbg_line_mutex;

void triangle_compare( const Triangle_set &remove_triangles, const std::vector< long > &add_triangles, Triangle_set &res_remove_triangles,
                                 Triangle_set &res_add_triangles, Triangle_set *exist_triangles )
{
    Hash_map_3d< long, std::pair< Triangle_ptr, bool > > all_remove_triangles_list;
    for ( const Triangle_ptr &tri_ptr : remove_triangles )
    {
        all_remove_triangles_list.insert( tri_ptr->m_tri_pts_id[ 0 ], tri_ptr->m_tri_pts_id[ 1 ], tri_ptr->m_tri_pts_id[ 2 ],
                                          std::make_pair( tri_ptr, true ) );
    }
    for ( int i = 0; i < add_triangles.size(); i += 3 )
    {
        Triangle                         tri( add_triangles[ i ], add_triangles[ i + 1 ], add_triangles[ i + 2 ] );
        std::pair< Triangle_ptr, bool > *temp_pair_ptr =
            all_remove_triangles_list.get_data( tri.m_tri_pts_id[ 0 ], tri.m_tri_pts_id[ 1 ], tri.m_tri_pts_id[ 2 ] );
        if ( temp_pair_ptr != nullptr )
        {
            temp_pair_ptr->second = false;
            if ( exist_triangles != nullptr )
            {
                exist_triangles->insert( temp_pair_ptr->first );
            }
        }
        else
        {
            res_add_triangles.insert( std::make_shared< Triangle >( tri ) );
        }
    }

    for ( auto &it : all_remove_triangles_list.m_map_3d_hash_map )
    {
        if ( it.second.second )
        {
            res_remove_triangles.insert( it.second.first );
        }
    }
}

std::vector< long > delaunay_triangulation( std::vector< RGB_pt_ptr > &rgb_pt_vec, vec_3 &long_axis, vec_3 &mid_axis, vec_3 &short_axis,
                                                std::set< long > &convex_hull_index, std::set< long > &inner_hull_index )
{
    std::vector< int >  triangle_indices;
    std::vector< long > tri_rgb_pt_indices;
    Common_tools::Timer tim;
    tim.tic();
    int             pt_size = rgb_pt_vec.size();
    Eigen::MatrixXd pc_mat;
    pc_mat.resize( pt_size, 3 );
    if ( rgb_pt_vec.size() < 3 )
    {
        return tri_rgb_pt_indices;
    }
    for ( int i = 0; i < rgb_pt_vec.size(); i++ )
    {
        pc_mat.row( i ) = rgb_pt_vec[ i ]->get_pos();
    }

    vec_3           pc_center = pc_mat.colwise().mean().transpose();
    Eigen::MatrixXd pt_sub_center = pc_mat.rowwise() - pc_center.transpose();

    if ( short_axis.norm() == 0 )
    {
        Eigen::Matrix3d                                  cov = ( pt_sub_center.transpose() * pt_sub_center ) / double( pc_mat.rows() );
        Eigen::SelfAdjointEigenSolver< Eigen::Matrix3d > eigen_solver;
        eigen_solver.compute( cov );
        short_axis = eigen_solver.eigenvectors().col( 0 );
        mid_axis = eigen_solver.eigenvectors().col( 1 );
        // vec_3 long_axis = es.eigenvectors().col(2);
        if ( pt_sub_center.row( 0 ).dot( short_axis ) < 0 )
        {
            short_axis *= -1;
        }
        if ( pt_sub_center.row( 1 ).dot( mid_axis ) < 0 )
        {
            mid_axis *= -1;
        }
        long_axis = short_axis.cross( mid_axis );
    }
    tim.tic();
    std::vector< std::pair< Common_tools::D2_Point, long > > points;
    points.resize( rgb_pt_vec.size() );
    std::vector< Common_tools::D2_Point > pts_for_hull( rgb_pt_vec.size() );
    std::vector< std::size_t >            indices( pts_for_hull.size() );
    int                                   avail_idx = 0;
    for ( int i = 0; i < rgb_pt_vec.size(); i++ )
    {
        // ANCHOR - remove off plane points
        // if(pt_sub_center.row( i ).dot( short_axis ) > 0.1 )
        // {
        //     continue;
        // }
        Common_tools::D2_Point cgal_pt = Common_tools::D2_Point( pt_sub_center.row( i ).dot( long_axis ), pt_sub_center.row( i ).dot( mid_axis ) );
        points[ avail_idx ] = std::make_pair( cgal_pt, rgb_pt_vec[ i ]->m_pt_index );
        pts_for_hull[ avail_idx ] = cgal_pt;
        avail_idx++;
    }
    points.resize( avail_idx );
    pts_for_hull.resize( avail_idx );

    std::iota( indices.begin(), indices.end(), 0 );
    std::vector< std::size_t > out;
    if ( 1 )
    {
        CGAL::convex_hull_2( indices.begin(), indices.end(), std::back_inserter( out ),
                             Common_tools::Convex_hull_traits_2( CGAL::make_property_map( pts_for_hull ) ) );
        for ( auto p : out )
        {
            convex_hull_index.insert( points[ p ].second );
        }
        for ( auto p : points )
        {
            if ( convex_hull_index.find( p.second ) == convex_hull_index.end() )
            {
                inner_hull_index.insert( p.second );
            }
        }
    }
    Common_tools::Delaunay2 T;
    T.insert( points.begin(), points.end() );
    Common_tools::Delaunay2::Finite_faces_iterator fit;
    Common_tools::Delaunay2::Face                  face;
    if ( T.number_of_faces() == 0 )
    {
        return tri_rgb_pt_indices;
    }
    tri_rgb_pt_indices.resize( T.number_of_faces() * 3 );
    long idx = 0;
    for ( fit = T.finite_faces_begin(); fit != T.finite_faces_end(); fit++ )
    {
        face = *fit;
        double max_angle = 180;
        int    hull_count = 0;
        for ( int pt_idx = 0; pt_idx < 3; pt_idx++ )
        {
            if ( convex_hull_index.find( face.vertex( pt_idx )->info() ) != convex_hull_index.end() )
            {
                hull_count++;
            }
        }
        if ( hull_count >= 1 )
        {
            max_angle = 180;
        }
        if ( !is_face_is_ok( face, max_angle ) )
        {
            continue;
        }
        tri_rgb_pt_indices[ idx + 0 ] = face.vertex( 0 )->info();
        tri_rgb_pt_indices[ idx + 1 ] = face.vertex( 1 )->info();
        tri_rgb_pt_indices[ idx + 2 ] = face.vertex( 2 )->info();
        idx += 3;
    }
    tri_rgb_pt_indices.resize( idx );
    char dbg_str[ 1024 ];
    sprintf( dbg_str, "Cost time=%.2f ms, pts=%d, tri=%d", tim.toc(), ( int ) tri_rgb_pt_indices.size(), ( int ) tri_rgb_pt_indices.size() / 3 );
    // g_debug_string = std::string( dbg_str );
    // cout << g_debug_string << endl;

    return tri_rgb_pt_indices;
}

extern std::string bin_file_name;
extern double      minimum_pts;
extern double      g_meshing_voxel_size;
FILE *             g_fp_cost_time = nullptr;
FILE *             g_fp_lio_state = nullptr;
extern bool        g_flag_pause;
extern const int   number_of_frame;
extern int         appending_pts_frame;
// ANCHOR - mesh_reconstruction

Triangle_manager          legal_triangles;
std::vector< RGB_pt_ptr > retrieve_neighbor_pts( const std::vector< RGB_pt_ptr > &rgb_pts_vec )
{
    std::vector< RGB_pt_ptr > res_pts_vec;
    std::set< long >          neighbor_indices;
    for ( int i = 0; i < rgb_pts_vec.size(); i++ )
    {
        int idx_a = rgb_pts_vec[ i ]->m_pt_index;
        neighbor_indices.insert( idx_a );
        for ( Triangle_set::iterator it = legal_triangles.m_map_pt_triangle[ idx_a ].begin(); it != legal_triangles.m_map_pt_triangle[ idx_a ].end();
              it++ )
        {
            neighbor_indices.insert( ( *it )->m_tri_pts_id[ 0 ] );
            neighbor_indices.insert( ( *it )->m_tri_pts_id[ 1 ] );
            neighbor_indices.insert( ( *it )->m_tri_pts_id[ 2 ] );
        }
    }
    res_pts_vec.reserve( neighbor_indices.size() );
    for ( std::set< long >::iterator it = neighbor_indices.begin(); it != neighbor_indices.end(); it++ )
    {
        res_pts_vec.push_back( g_map_rgb_pts_mesh.m_rgb_pts_vec[ *it ] );
    }
    // cout << "Before retrieve: " << rgb_pts_vec.size() << ", appended = " <<res_pts_vec.size() << endl;
    return res_pts_vec;
}

// ANCHOR - retrieve_neighbor_pts_kdtree
float                     smooth_factor = 1.0;
double                    g_kd_tree_accept_pt_dis  = 0.32;
std::vector< RGB_pt_ptr > retrieve_neighbor_pts_kdtree( const std::vector< RGB_pt_ptr > &rgb_pts_vec )
{
    std::vector< RGB_pt_ptr > res_pt_vec;
    std::set< long >          new_pts_index;
    KDtree_pt_vector          kdtree_pt_vector;
    std::vector< float >      pt_dis_vector;
    // g_kd_tree_accept_pt_dis = g_meshing_voxel_size * 0.8;
    g_kd_tree_accept_pt_dis = g_meshing_voxel_size*1.25;
    for ( int i = 0; i < rgb_pts_vec.size(); i++ )
    {
        std::vector< int >   indices;
        std::vector< float > distances;
        vec_3                pt_vec = rgb_pts_vec[ i ]->get_pos();
        KDtree_pt            kdtree_pt( pt_vec );
        g_map_rgb_pts_mesh.m_kdtree.Nearest_Search( kdtree_pt, 20, kdtree_pt_vector, pt_dis_vector );
        int   size = kdtree_pt_vector.size();
        vec_3 smooth_vec = vec_3( 0, 0, 0 );
        int   smooth_count = 0;
        for ( int k = 0; k < size; k++ )
        {
            if ( sqrt( pt_dis_vector[ k ] ) < g_kd_tree_accept_pt_dis )
            {
                new_pts_index.insert( kdtree_pt_vector[ k ].m_pt_idx );
            }
            
            if(sqrt( pt_dis_vector[ k ] ) < g_kd_tree_accept_pt_dis*2)
            {
                smooth_count++;
                smooth_vec += g_map_rgb_pts_mesh.m_rgb_pts_vec[ kdtree_pt_vector[ k ].m_pt_idx ]->get_pos();
            }
        }
        smooth_vec /= smooth_count;
        smooth_vec = smooth_vec * ( smooth_factor ) + pt_vec * ( 1 - smooth_factor );
        rgb_pts_vec[ i ]->set_smooth_pos( smooth_vec );
    }
   
    for ( auto p : new_pts_index )
    {
        res_pt_vec.push_back( g_map_rgb_pts_mesh.m_rgb_pts_vec[ p ] );
    }
    return res_pt_vec;
}

std::vector< RGB_pt_ptr > remove_outlier_pts( const std::vector< RGB_pt_ptr > &rgb_pts_vec, const RGB_voxel_ptr &voxel_ptr )
{
    int                       remove_count = 0;
    std::vector< RGB_pt_ptr > res_pt_vec;
    for ( int i = 0; i < rgb_pts_vec.size(); i++ )
    {
        if ( rgb_pts_vec[ i ]->m_is_inner_pt == 1 )
        {
            if ( rgb_pts_vec[ i ]->m_parent_voxel != voxel_ptr )
            {
                remove_count++;
                continue;
            }
        }
        res_pt_vec.push_back( rgb_pts_vec[ i ] );
    }
    res_pt_vec = rgb_pts_vec;
    return res_pt_vec;
}

void correct_triangle_index( Triangle_ptr &ptr, const vec_3 &camera_center, const vec_3 &_short_axis )
{
    vec_3 pt_a = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ptr->m_tri_pts_id[ 0 ] ]->get_pos( 1 );
    vec_3 pt_b = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ptr->m_tri_pts_id[ 1 ] ]->get_pos( 1 );
    vec_3 pt_c = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ptr->m_tri_pts_id[ 2 ] ]->get_pos( 1 );
    vec_3 pt_ab = pt_b - pt_a;
    vec_3 pt_ac = pt_c - pt_a;
    vec_3 pt_tri_cam = camera_center - pt_a;
    vec_3 short_axis = _short_axis;
    ptr->m_normal = pt_ab.cross( pt_ac );
    if ( ptr->m_normal.norm() != 0 )
    {
        ptr->m_normal.normalize();
    }
    else
    {
        ptr->m_normal = vec_3( 0, 0, 1 );
    }
    if ( short_axis.dot( pt_tri_cam ) < 0 )
    {
        short_axis *= -1;
    }
    if ( short_axis.dot( ptr->m_normal ) < 0 )
    {
        ptr->m_index_flip = 0;
    }
    else
    {
        ptr->m_index_flip = 1;
    }
    if ( ptr->m_normal( 2 ) < 0 )
    {
        ptr->m_normal *= -1;
    }
}

