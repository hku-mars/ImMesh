#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_MSC_SECURE_CRT
#include "mesh_rec_display.hpp"
#include "tools/openGL_libs/gl_draw_founction.hpp"

#include "tools_timer.hpp"
#include "tinycolormap.hpp"
#include "tools/openGL_libs/openGL_camera.hpp"

extern Global_map                   g_map_rgb_pts_mesh;
extern Triangle_manager             g_triangles_manager;
extern LiDAR_frame_pts_and_pose_vec g_eigen_vec_vec;

extern Eigen::Matrix3d g_camera_K;
// extern Eigen::Matrix3d lidar_frame_to_camera_frame;

template < int M, int option = EIGEN_DATA_TYPE_DEFAULT_OPTION >
using eigen_vec_uc = Eigen::Matrix< unsigned char, M, 1, option >;

extern GL_camera g_gl_camera;

Common_tools::Point_cloud_shader    g_path_shader;
Common_tools::Point_cloud_shader    g_LiDAR_point_shader;
Common_tools::Triangle_facet_shader g_triangle_facet_shader;
Common_tools::Axis_shader           g_axis_shader;
Common_tools::Ground_plane_shader   g_ground_plane_shader;
Common_tools::Camera_pose_shader    g_camera_pose_shader;

// std::map< Visibility_region_ptr, Common_tools::Triangle_facet_shader  > g_map_region_triangle_shader;

// ANCHOR - draw_triangle
#include <chrono>
#include <thread>
std::mutex mutex_triangle_vec;

extern unsigned int vbo;
extern unsigned int vbo_color;
extern int          g_current_frame;
#define ENABLE_BUFFER 0

extern bool g_display_mesh;

extern float g_ply_smooth_factor;
extern int   g_ply_smooth_k;

extern double g_kd_tree_accept_pt_dis;
extern bool   g_force_refresh_triangle;
vec_3f        g_axis_min_max[ 2 ];

struct Region_triangles_shader
{
    std::vector< vec_3f >               m_triangle_pt_vec;
    Common_tools::Triangle_facet_shader m_triangle_facet_shader;
    int                                 m_need_init_shader = true;
    int                                 m_need_refresh_shader = true;
    int                                 m_if_set_color = false;
    std::shared_ptr< std::mutex >       m_mutex_ptr = nullptr;

    Region_triangles_shader() { m_mutex_ptr = std::make_shared< std::mutex >(); }

    void init_openGL_shader() { m_triangle_facet_shader.init( SHADER_DIR ); }

    Common_tools::Triangle_facet_shader *get_shader_ptr() { return &m_triangle_facet_shader; }

    void init_pointcloud()
    {
        std::unique_lock< std::mutex > lock( *m_mutex_ptr );
        if ( m_if_set_color )
        {
            m_triangle_facet_shader.set_pointcloud( m_triangle_pt_vec, g_axis_min_max, 2 );
        }
        else
        {
            m_triangle_facet_shader.set_pointcloud( m_triangle_pt_vec );
        }
    }

    void unparse_triangle_set_to_vector( const Triangle_set &tri_angle_set )
    {
        // TODO: synchronized data buffer here:
        std::unique_lock< std::mutex > lock( *m_mutex_ptr );
        m_triangle_pt_vec.resize( tri_angle_set.size() * 3 );
        // cout << "Number of pt_size = " << m_triangle_pt_list.size() << endl;
        int count = 0;
        for ( Triangle_set::iterator it = tri_angle_set.begin(); it != tri_angle_set.end(); it++ )
        {
            for ( size_t pt_idx = 0; pt_idx < 3; pt_idx++ )
            {
                if ( g_map_rgb_pts_mesh.m_rgb_pts_vec[ ( *it )->m_tri_pts_id[ pt_idx ] ]->m_smoothed == false )
                {
                    g_map_rgb_pts_mesh.smooth_pts( g_map_rgb_pts_mesh.m_rgb_pts_vec[ ( *it )->m_tri_pts_id[ pt_idx ] ], g_ply_smooth_factor,
                                                   g_ply_smooth_k, g_kd_tree_accept_pt_dis );
                }
            }
            vec_3 pt_a = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ( *it )->m_tri_pts_id[ 0 ] ]->get_pos( 1 );
            vec_3 pt_b = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ( *it )->m_tri_pts_id[ 1 ] ]->get_pos( 1 );
            vec_3 pt_c = g_map_rgb_pts_mesh.m_rgb_pts_vec[ ( *it )->m_tri_pts_id[ 2 ] ]->get_pos( 1 );
            m_triangle_pt_vec[ count ] = pt_a.cast< float >();
            m_triangle_pt_vec[ count + 1 ] = pt_b.cast< float >();
            m_triangle_pt_vec[ count + 2 ] = pt_c.cast< float >();
            count = count + 3;
        }
    }

    void get_axis_min_max( vec_3f *axis_min_max = nullptr )
    {
        if ( axis_min_max != nullptr )
        {
            for ( int i = 0; i < m_triangle_pt_vec.size(); i++ )
            {
                if ( axis_min_max[ 0 ]( 0 ) > m_triangle_pt_vec[ i ]( 0 ) )
                {
                    axis_min_max[ 0 ]( 0 ) = m_triangle_pt_vec[ i ]( 0 );
                }
                if ( axis_min_max[ 0 ]( 1 ) > m_triangle_pt_vec[ i ]( 1 ) )
                {
                    axis_min_max[ 0 ]( 1 ) = m_triangle_pt_vec[ i ]( 1 );
                }
                if ( axis_min_max[ 0 ]( 2 ) > m_triangle_pt_vec[ i ]( 2 ) )
                {
                    axis_min_max[ 0 ]( 2 ) = m_triangle_pt_vec[ i ]( 2 );
                }
                if ( axis_min_max[ 1 ]( 0 ) < m_triangle_pt_vec[ i ]( 0 ) )
                {
                    axis_min_max[ 1 ]( 0 ) = m_triangle_pt_vec[ i ]( 0 );
                }
                if ( axis_min_max[ 1 ]( 1 ) < m_triangle_pt_vec[ i ]( 1 ) )
                {
                    axis_min_max[ 1 ]( 1 ) = m_triangle_pt_vec[ i ]( 1 );
                }
                if ( axis_min_max[ 1 ]( 2 ) < m_triangle_pt_vec[ i ]( 2 ) )
                {
                    axis_min_max[ 1 ]( 2 ) = m_triangle_pt_vec[ i ]( 2 );
                }
            }
        }
    }

    void synchronized_from_region( Sync_triangle_set *sync_triangle_set, vec_3f *axis_min_max = nullptr )
    {
        if ( sync_triangle_set == nullptr )
        {
            cout << "sync_triangle_set == nullptr" << endl;
            return;
        }

        if ( sync_triangle_set->m_if_required_synchronized )
        {
            Triangle_set triangle_set;
            sync_triangle_set->get_triangle_set( triangle_set, true );
            unparse_triangle_set_to_vector( triangle_set );
            get_axis_min_max( axis_min_max );
            std::this_thread::sleep_for( std::chrono::microseconds( 100 ) );
            m_need_refresh_shader = true;
        }
    }

    void draw( const Cam_view &gl_cam )
    {
        if ( m_need_init_shader )
        {
            init_openGL_shader();
            m_need_init_shader = false;
        }
        if ( m_triangle_pt_vec.size() < 3 )
        {
            return;
        }
        if ( m_need_refresh_shader )
        {
            init_pointcloud();
            m_need_refresh_shader = false;
        }
        m_triangle_facet_shader.draw( gl_cam.m_glm_projection_mat, Common_tools::eigen2glm( gl_cam.m_camera_pose_mat44_inverse ) );
    }
};

void display_current_LiDAR_pts( int current_frame_idx, double pts_size, vec_4f color )
{
    if ( current_frame_idx < 1 )
    {
        return;
    }
    g_LiDAR_point_shader.set_point_attr( pts_size );
    g_LiDAR_point_shader.set_pointcloud( g_eigen_vec_vec[ current_frame_idx ].first, vec_3( 1.0, 1.0, 1.0 ) );
    g_LiDAR_point_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat,
                               Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ) );
}

void display_reinforced_LiDAR_pts( std::vector< vec_3f > &pt_vec, double pts_size, vec_3f color )
{
    g_LiDAR_point_shader.set_point_attr( pts_size );
    g_LiDAR_point_shader.set_pointcloud( pt_vec, color.cast< double >() );
    g_LiDAR_point_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat,
                               Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ) );
}

void init_openGL_shader()
{
    g_LiDAR_point_shader.init( SHADER_DIR );
    g_path_shader.init( SHADER_DIR );
    // Init axis buffer
    g_axis_shader.init( SHADER_DIR, 1 );
    // Init ground shader
    g_ground_plane_shader.init( SHADER_DIR, 10, 10 );
    g_camera_pose_shader.init( SHADER_DIR );
    g_triangle_facet_shader.init( SHADER_DIR );
}

std::mutex                                                                  g_region_triangle_shader_mutex;
std::vector< std::shared_ptr< Region_triangles_shader > >                   g_region_triangles_shader_vec;
std::map< Sync_triangle_set *, std::shared_ptr< Region_triangles_shader > > g_map_region_triangles_shader;
extern bool                                                                 g_mesh_if_color;
extern float                                                                g_wireframe_width;

extern bool          g_display_face;
std::vector< vec_3 > pt_camera_traj;

// ANCHOR - synchronize_triangle_list_for_disp
void synchronize_triangle_list_for_disp()
{
    int region_size = g_triangles_manager.m_triangle_set_vector.size();
    bool if_force_refresh = g_force_refresh_triangle;
    for ( int region_idx = 0; region_idx < region_size; region_idx++ )
    {
        Sync_triangle_set *                        sync_triangle_set_ptr = g_triangles_manager.m_triangle_set_vector[ region_idx ];
        std::shared_ptr< Region_triangles_shader > region_triangles_shader_ptr = nullptr;
        if ( g_map_region_triangles_shader.find( sync_triangle_set_ptr ) == g_map_region_triangles_shader.end() )
        {
            // new a shader
            region_triangles_shader_ptr = std::make_shared< Region_triangles_shader >();
            g_region_triangle_shader_mutex.lock();
            g_map_region_triangles_shader.insert( std::make_pair( sync_triangle_set_ptr, region_triangles_shader_ptr ) );
            g_region_triangles_shader_vec.push_back( region_triangles_shader_ptr );
            g_region_triangle_shader_mutex.unlock();
        }
        else
        {
            region_triangles_shader_ptr = g_map_region_triangles_shader[ sync_triangle_set_ptr ];
        }

        if ( region_triangles_shader_ptr != nullptr )
        {
            if ( g_force_refresh_triangle && if_force_refresh == false )
            {
                if_force_refresh = true;
                region_idx = -1; // refresh from the start
            }
            if(if_force_refresh)
            {
                sync_triangle_set_ptr->m_if_required_synchronized = true;
            }
            region_triangles_shader_ptr->synchronized_from_region( sync_triangle_set_ptr, g_axis_min_max );
        }
    }
    if ( g_force_refresh_triangle )
    {
        g_force_refresh_triangle = false;
    }
}

void service_refresh_and_synchronize_triangle( double sleep_time )
{
    g_axis_min_max[ 0 ] = vec_3f( 1e8, 1e8, 1e8 );
    g_axis_min_max[ 1 ] = vec_3f( -1e8, -1e8, -1e8 );
    while ( 1 )
    {
        std::this_thread::sleep_for( std::chrono::milliseconds( ( int ) sleep_time ) );
        synchronize_triangle_list_for_disp();
    }
}

void draw_triangle( const Cam_view &gl_cam )
{
    int region_size = g_region_triangles_shader_vec.size();
    for ( int region_idx = 0; region_idx < region_size; region_idx++ )
    {
        g_region_triangles_shader_vec[ region_idx ]->m_triangle_facet_shader.m_if_draw_face = g_display_face;
        g_region_triangles_shader_vec[ region_idx ]->m_if_set_color = g_mesh_if_color;
        g_region_triangles_shader_vec[ region_idx ]->draw( gl_cam );
    }
}

void display_camera_traj( float display_size )
{
    if ( pt_camera_traj.size() == 0 )
    {
        return;
    }
    g_path_shader.set_pointcloud( pt_camera_traj );
    g_path_shader.set_point_attr( display_size + 2, 0, 1.0 );
    g_path_shader.m_draw_points_number = pt_camera_traj.size();
    g_path_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat, Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ),
                        GL_LINE_STRIP );
}

void draw_camera_pose( int current_frame_idx, float pt_disp_size, float display_cam_size )
{

    Eigen::Quaterniond pose_q( g_eigen_vec_vec[ current_frame_idx ].second.head< 4 >() );
    vec_3              pose_t = g_eigen_vec_vec[ current_frame_idx ].second.block( 4, 0, 3, 1 );
    mat_3_3            lidar_frame_to_camera_frame;
    lidar_frame_to_camera_frame << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    pose_q = Eigen::Quaterniond( pose_q.toRotationMatrix() * lidar_frame_to_camera_frame );

    pose_t = pose_q.inverse() * ( pose_t * -1.0 );
    pose_q = pose_q.inverse();

    g_camera_pose_shader.set_camera_pose_and_scale( pose_q, pose_t, display_cam_size );
    g_camera_pose_shader.set_point_attr( 5, 0, 1.0 );
    g_camera_pose_shader.draw( g_gl_camera.m_gl_cam.m_glm_projection_mat, Common_tools::eigen2glm( g_gl_camera.m_gl_cam.m_camera_pose_mat44_inverse ),
                               -1 );
}

void draw_camera_trajectory( int current_frame_idx, float pt_disp_size )
{
    pt_camera_traj.clear();
    for ( int i = 0; i < current_frame_idx; i++ )
    {
        if ( g_eigen_vec_vec[ i ].second.size() >= 7 )
        {
            pt_camera_traj.push_back( g_eigen_vec_vec[ i ].second.block( 4, 0, 3, 1 ) );
        }
    }
    display_camera_traj( pt_disp_size );
}
