#pragma once
#include <set>
#include <unordered_set>
#include "tools_kd_hash.hpp"
#include "pointcloud_rgbd.hpp"
// class RGB_pts;
// class RGB_Voxel;
class Global_map;
class Triangle
{
  public:
    int    m_tri_pts_id[ 3 ] = { 0 };
    vec_3  m_normal = vec_3( 0, 0, 0 );
    int    m_projected_texture_id = 0;
    vec_2f m_texture_coor[ 3 ];
    double m_vis_score = 0;
    float  last_texture_distance = 3e8;
    int    m_index_flip = 0;

    void sort_id()
    {
        std::sort( std::begin( m_tri_pts_id ), std::end( m_tri_pts_id ) );
    }
    Triangle() = default;
    ~Triangle() = default;

    Triangle( int id_0, int id_1, int id_2 )
    {
        m_tri_pts_id[ 0 ] = id_0;
        m_tri_pts_id[ 1 ] = id_1;
        m_tri_pts_id[ 2 ] = id_2;
        sort_id();
    }
};

using Triangle_ptr = std::shared_ptr< Triangle >;
// using Triangle_set = std::unordered_set<Triangle_ptr>;
using Triangle_set = std::set< Triangle_ptr >;

struct Sync_triangle_set
{
    Triangle_set m_triangle_set;
    std::shared_ptr< std::mutex> m_mutex_ptr = nullptr;
    int          m_frame_idx = 0;
    bool         m_if_required_synchronized = true;
    Sync_triangle_set() 
    {
        m_mutex_ptr = std::make_shared<std::mutex>();
    }

    void lock()
    {
        m_mutex_ptr->lock();
    }

    void unlock()
    {
        m_mutex_ptr->unlock();
    }

    void insert( const Triangle_ptr& tri_ptr )
    {
        lock();
        m_triangle_set.insert( tri_ptr );
        m_if_required_synchronized = true;
        unlock();
    }

    void erase( const Triangle_ptr& tri_ptr )
    {
        lock();
        auto it1 = m_triangle_set.find( tri_ptr );
        if ( it1 != m_triangle_set.end() )
        {
            m_triangle_set.erase( it1 );
        }
        m_if_required_synchronized = true;
        unlock();
    }

    void clear()
    {
        lock();
        m_triangle_set.clear();
        m_if_required_synchronized = true;
        unlock();
    }

    int get_triangle_set_size()
    {
        int return_size = 0;
        lock();
        return_size = m_triangle_set.size();
        unlock();
        return return_size;
    }

    Triangle_set * get_triangle_set_ptr()
    {
        return &m_triangle_set;
    }
    
    void get_triangle_set(Triangle_set & ret_set, bool reset_status = false)
    {
        lock();
        ret_set = m_triangle_set;
        if ( reset_status )
        {
            m_if_required_synchronized = false;
        }
        unlock();
    }
};

class Triangle_manager
{
  public:
    Global_map*                             m_pointcloud_map = nullptr;
    Hash_map_3d< int, Triangle_ptr >        m_triangle_hash;
    double                                  m_region_size = 10.0;
    std::vector< Sync_triangle_set* >            m_triangle_set_vector;
    // Hash_map_3d< int, Triangle_set >        m_triangle_set_in_region;
    Hash_map_3d< int, Sync_triangle_set >        m_triangle_set_in_region;

    std::unordered_map< int, Triangle_set > m_map_pt_triangle;

    // Triangle_set m_triangle_list;

    std::mutex m_mutex_triangle_hash;

    Hash_map_2d< int, Triangle_set > m_map_edge_triangle;
    Hash_map_2d< int, Triangle_set > m_map_edge_triangle_conflicted;
    int                              m_enable_map_edge_triangle = 0;
    int                              m_newest_rec_frame_idx = 0;
    
    void clear()
    {
        m_triangle_hash.clear();
        m_map_pt_triangle.clear();
        // m_triangle_list.clear();
        for ( auto& triangle_set : m_triangle_set_in_region.m_map_3d_hash_map )
        {
            triangle_set.second.clear();
        }

        m_map_edge_triangle.clear();
        m_map_edge_triangle_conflicted.clear();
    }

    Triangle_manager()
    {
        m_triangle_hash.m_map_3d_hash_map.reserve( 1e7 );
        m_map_pt_triangle.reserve( 1e7 );
    };
    ~Triangle_manager() = default;

    vec_3 get_triangle_center( const Triangle_ptr& tri_ptr );
    void  insert_triangle_to_list( const Triangle_ptr& tri_ptr, const int& frame_idx = 0 );
    void  erase_triangle_from_list( const Triangle_ptr& tri_ptr, const int& frame_idx = 0 );
    int   get_all_triangle_list( std::vector< Triangle_set >& triangle_list, std::mutex* mutex = nullptr, int sleep_us_each_query = 10 );
    int   get_triangle_list_size();
    // void 

    void erase_triangle( const Triangle_ptr& tri_ptr )
    {
        int idx[ 3 ];
        idx[ 0 ] = tri_ptr->m_tri_pts_id[ 0 ];
        idx[ 1 ] = tri_ptr->m_tri_pts_id[ 1 ];
        idx[ 2 ] = tri_ptr->m_tri_pts_id[ 2 ];
        // printf_line;
        // erase triangle in list
        erase_triangle_from_list( tri_ptr, m_newest_rec_frame_idx );
        // auto it1 = m_triangle_list.find( tri_ptr );
        // if ( it1 != m_triangle_list.end() )
        // {
        //     m_triangle_list.erase( m_triangle_list.find( tri_ptr ) );
        // }

        for ( int tri_idx = 0; tri_idx < 3; tri_idx++ )
        {
            auto it3 = m_map_pt_triangle[ idx[ tri_idx ] ].find( tri_ptr );
            if ( it3 != m_map_pt_triangle[ idx[ tri_idx ] ].end() )
            {
                m_map_pt_triangle[ idx[ tri_idx ] ].erase( it3 );
            }
        }
        if ( m_enable_map_edge_triangle )
        {
            // printf_line;
            // erase triangle in edge-triangle list
            auto it2 = m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 1 ] ].find( tri_ptr );
            if ( it2 != m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 1 ] ].end() )
            {
                m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 1 ] ].erase( it2 );
            }

            it2 = m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 2 ] ].find( tri_ptr );
            if ( it2 != m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 2 ] ].end() )
            {
                m_map_edge_triangle.m_map_2d_hash_map[ idx[ 0 ] ][ idx[ 2 ] ].erase( it2 );
            }

            it2 = m_map_edge_triangle.m_map_2d_hash_map[ idx[ 1 ] ][ idx[ 2 ] ].find( tri_ptr );
            if ( it2 != m_map_edge_triangle.m_map_2d_hash_map[ idx[ 1 ] ][ idx[ 2 ] ].end() )
            {
                m_map_edge_triangle.m_map_2d_hash_map[ idx[ 1 ] ][ idx[ 2 ] ].erase( it2 );
            }
        }
        // printf_line;
    }

    void remove_triangle_list( const Triangle_set& triangle_list, const int frame_idx = 0 )
    {
        m_mutex_triangle_hash.lock();
        m_newest_rec_frame_idx = std::max( frame_idx, m_newest_rec_frame_idx );
        for ( auto tri_ptr : triangle_list )
        {
            erase_triangle( tri_ptr );
        }
        m_mutex_triangle_hash.unlock();
    }

    template < typename T >
    Triangle_set find_relative_triangulation_combination( std::set< T >& set_index )
    {
        // std::set< T >::iterator it;
        Triangle_set triangle_ptr_list;
        // m_mutex_triangle_hash.lock();
        for ( typename std::set< T >::iterator it = set_index.begin(); it != set_index.end(); it++ )
        {
            if ( m_map_pt_triangle.find( *it ) != m_map_pt_triangle.end() )
            {
                for ( Triangle_set::iterator tri_it = m_map_pt_triangle[ *it ].begin(); tri_it != m_map_pt_triangle[ *it ].end(); tri_it++ )
                {
                    if ( ( set_index.find( ( *tri_it )->m_tri_pts_id[ 0 ] ) != set_index.end() ) &&
                         ( set_index.find( ( *tri_it )->m_tri_pts_id[ 1 ] ) != set_index.end() ) &&
                         ( set_index.find( ( *tri_it )->m_tri_pts_id[ 2 ] ) != set_index.end() ) )
                    {
                        triangle_ptr_list.insert( *tri_it );
                    }
                }
            }
        }
        // m_mutex_triangle_hash.unlock();
        return triangle_ptr_list;
    }

    template < typename T >
    void remove_relative_triangulation_combination( std::set< T >& set_index )
    {
        // std::set< T >::iterator it;
        Triangle_set triangle_ptr_list = find_relative_triangulation_combination( set_index );

        // cout << ANSI_COLOR_YELLOW_BOLD << "In conflict triangle size = " << triangle_ptr_list.size() << ANSI_COLOR_RESET << endl;
        remove_triangle_list( triangle_ptr_list );
    }

    template < typename T >
    void remove_relative_triangulation_combination( std::vector< T >& vector_index )
    {
        std::set< T > index_set;
        for ( auto p : vector_index )
        {
            index_set.insert( p );
        }
        remove_relative_triangulation_combination( index_set );
    }

    template < typename T >
    Triangle_set get_inner_hull_triangle_list( std::set< T >& inner_hull_indices )
    {
        Triangle_set triangle_list;
        for ( auto p : inner_hull_indices )
        {
            if ( m_map_pt_triangle.find( p ) != m_map_pt_triangle.end() )
            {
                for ( auto pp : m_map_pt_triangle[ p ] )
                {
                    triangle_list.insert( pp );
                }
            }
        }
        return triangle_list;
    }

    template < typename T >
    void remove_inner_hull_triangle( std::set< T >& inner_hull_indices )
    {
        Triangle_set triangle_list = get_inner_hull_triangle_list( inner_hull_indices );
        remove_triangle_list( triangle_list );
    }

    bool if_triangle_exist( int& id_0, int& id_1, int& id_2 )
    {
        int ids[ 3 ];
        ids[ 0 ] = id_0;
        ids[ 1 ] = id_1;
        ids[ 2 ] = id_2;
        std::sort( std::begin( ids ), std::end( ids ) );
        if ( m_triangle_hash.if_exist( ids[ 0 ], ids[ 1 ], ids[ 2 ] ) )
        {
            // This triangle exist
            return true;
        }
        else
        {
            return false;
        }
    }

    Triangle_ptr find_triangle( int id_0, int id_1, int id_2 )
    {
        int ids[ 3 ];
        ids[ 0 ] = id_0;
        ids[ 1 ] = id_1;
        ids[ 2 ] = id_2;
        std::sort( std::begin( ids ), std::end( ids ) );
        if ( m_triangle_hash.if_exist( ids[ 0 ], ids[ 1 ], ids[ 2 ] ) )
        {
            // This triangle exist
            // return m_triangle_hash.m_map_3d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ][ ids[ 2 ] ];
            return *m_triangle_hash.get_data( ids[ 0 ], ids[ 1 ], ids[ 2 ] );
        }
        else
        {
            return nullptr;
        }
    }

    Triangle_ptr insert_triangle( int id_0, int id_1, int id_2, int build_triangle_map = false, const int & frame_idx = 0 )
    {
        int ids[ 3 ];
        ids[ 0 ] = id_0;
        ids[ 1 ] = id_1;
        ids[ 2 ] = id_2;
        std::sort( std::begin( ids ), std::end( ids ) );
        Triangle_ptr triangle_ptr;
        if ( m_triangle_hash.if_exist( ids[ 0 ], ids[ 1 ], ids[ 2 ] ) )
        {
            // This triangle exist
            // triangle_ptr = m_triangle_hash.m_map_3d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ][ ids[ 2 ] ];
            triangle_ptr = *m_triangle_hash.get_data( ids[ 0 ], ids[ 1 ], ids[ 2 ] );
        }
        else
        {
            // This triangle is not exist.
            // Step 1: new a triangle
            triangle_ptr = std::make_shared< Triangle >( ids[ 0 ], ids[ 1 ], ids[ 2 ] );
            triangle_ptr->m_vis_score = 1;
            m_mutex_triangle_hash.lock();
            m_triangle_hash.insert( ids[ 0 ], ids[ 1 ], ids[ 2 ], triangle_ptr );
            m_mutex_triangle_hash.unlock();
            // return m_map_pt_triangle.size();
            // return m_triangle_list.size();
            // return triangle_ptr;
        }

        m_mutex_triangle_hash.lock();
        // m_triangle_list.insert( triangle_ptr );
        insert_triangle_to_list( triangle_ptr, frame_idx );
        // ins
        if ( build_triangle_map )
        {
            // Step 2: add this triangle to points list:
            m_map_pt_triangle[ ids[ 0 ] ].insert( triangle_ptr );
            m_map_pt_triangle[ ids[ 1 ] ].insert( triangle_ptr );
            m_map_pt_triangle[ ids[ 2 ] ].insert( triangle_ptr );

            if ( m_enable_map_edge_triangle )
            {
                m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ].insert( triangle_ptr );
                m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 2 ] ].insert( triangle_ptr );
                m_map_edge_triangle.m_map_2d_hash_map[ ids[ 1 ] ][ ids[ 2 ] ].insert( triangle_ptr );
                // Find conflict triangle
                if ( m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ].size() > 2 )
                {
                    m_map_edge_triangle_conflicted.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ] =
                        m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 1 ] ];
                }
                if ( m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 2 ] ].size() > 2 )
                {
                    m_map_edge_triangle_conflicted.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 2 ] ] =
                        m_map_edge_triangle.m_map_2d_hash_map[ ids[ 0 ] ][ ids[ 2 ] ];
                }
                if ( m_map_edge_triangle.m_map_2d_hash_map[ ids[ 1 ] ][ ids[ 2 ] ].size() > 2 )
                {
                    m_map_edge_triangle_conflicted.m_map_2d_hash_map[ ids[ 1 ] ][ ids[ 2 ] ] =
                        m_map_edge_triangle.m_map_2d_hash_map[ ids[ 1 ] ][ ids[ 2 ] ];
                }
            }
        }
        m_mutex_triangle_hash.unlock();

        return triangle_ptr;
    }
    std::set< int > get_conflict_triangle_pts()
    {
        std::set< int > conflict_triangle_pts;
        if ( m_enable_map_edge_triangle )
        {
            for ( auto it : m_map_edge_triangle_conflicted.m_map_2d_hash_map )
            {
                for ( auto it_it : it.second )
                {
                    Triangle_set triangle_list = it_it.second;
                    for ( auto tri : triangle_list )
                    {
                        // g_triangle_manager.erase_triangle( tri );
                        // conflict_triangle++;

                        conflict_triangle_pts.insert( tri->m_tri_pts_id[ 0 ] );
                        conflict_triangle_pts.insert( tri->m_tri_pts_id[ 1 ] );
                        conflict_triangle_pts.insert( tri->m_tri_pts_id[ 2 ] );
                    }
                    // printf_line;
                }
            }
        }
        return conflict_triangle_pts;
    }

    void clear_conflicted_triangles_list()
    {
        if ( m_enable_map_edge_triangle )
        {
            m_map_edge_triangle_conflicted.clear();
        }
    }
};
