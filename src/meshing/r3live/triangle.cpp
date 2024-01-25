#include "triangle.hpp"

vec_3 Triangle_manager::get_triangle_center(const Triangle_ptr& tri_ptr)
{
    vec_3 triangle_pos = ( m_pointcloud_map->m_rgb_pts_vec[ tri_ptr->m_tri_pts_id[ 0 ] ]->get_pos() +
                           m_pointcloud_map->m_rgb_pts_vec[ tri_ptr->m_tri_pts_id[ 1 ] ]->get_pos() +
                           m_pointcloud_map->m_rgb_pts_vec[ tri_ptr->m_tri_pts_id[ 2 ] ]->get_pos() );
    triangle_pos = triangle_pos / 3.0;
    return triangle_pos;
}

int Triangle_manager::get_all_triangle_list(std::vector< Triangle_set > & triangle_list, std::mutex * mutex, int sleep_us_each_query)
{
    int all_triangle_num = 0;
    triangle_list.clear();
    for ( auto &sync_triangle_set : m_triangle_set_in_region.m_map_3d_hash_map )
    {
        if ( mutex != nullptr )
            mutex->lock();
        Triangle_set tri_set;
        sync_triangle_set.second.get_triangle_set(tri_set) ;
        triangle_list.push_back( tri_set );
        all_triangle_num += triangle_list.back().size();
        if ( mutex != nullptr )
            mutex->unlock();
        if ( sleep_us_each_query != 0 )
        {
            // std::this_thread::yield();
            std::this_thread::sleep_for( std::chrono::microseconds( sleep_us_each_query ) );
        }
    }
    return all_triangle_num;
}

void Triangle_manager::insert_triangle_to_list( const Triangle_ptr& tri_ptr , const int& frame_idx)
{
    vec_3         triangle_pos = get_triangle_center( tri_ptr );
    int          hash_3d_x = std::round( triangle_pos( 0 ) / m_region_size );
    int          hash_3d_y = std::round( triangle_pos( 1 ) / m_region_size );
    int          hash_3d_z = std::round( triangle_pos( 2 ) / m_region_size );
    Sync_triangle_set* sync_triangle_set_ptr = m_triangle_set_in_region.get_data( hash_3d_x, hash_3d_y, hash_3d_z );
    if ( sync_triangle_set_ptr == nullptr )
    {
        sync_triangle_set_ptr = new Sync_triangle_set();
        sync_triangle_set_ptr->insert( tri_ptr );
        m_triangle_set_in_region.insert( hash_3d_x, hash_3d_y, hash_3d_z, *sync_triangle_set_ptr );
        m_triangle_set_vector.push_back( m_triangle_set_in_region.get_data( hash_3d_x, hash_3d_y, hash_3d_z ) );
    }
    else
    {
        sync_triangle_set_ptr->insert( tri_ptr );
    }
}

void Triangle_manager::erase_triangle_from_list( const Triangle_ptr& tri_ptr , const int & frame_idx)
{
    vec_3         triangle_pos = get_triangle_center( tri_ptr );
    int          hash_3d_x = std::round( triangle_pos( 0 ) / m_region_size );
    int          hash_3d_y = std::round( triangle_pos( 1 ) / m_region_size );
    int          hash_3d_z = std::round( triangle_pos( 2 ) / m_region_size );
    Sync_triangle_set* triangle_set_ptr = m_triangle_set_in_region.get_data( hash_3d_x, hash_3d_y, hash_3d_z );
    if ( triangle_set_ptr == nullptr )
    {
        return;
    }
    else
    {
        triangle_set_ptr->erase( tri_ptr );
    }
}

int Triangle_manager::get_triangle_list_size()
{
    int tri_list_size = 0;
    for ( auto& triangle_set : m_triangle_set_in_region.m_map_3d_hash_map )
    {
        tri_list_size += triangle_set.second.get_triangle_set_size();
    }
    return tri_list_size;
}