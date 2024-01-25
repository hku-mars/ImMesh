#pragma once
#include <CGAL/Point_2.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Polygon_2.h>

// #include <CGAL/Boolean_set_operations_2.h>
// https://doc.cgal.org/latest/Kernel_23/index.html#Kernel_23Kernel
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
// #include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

#include <CGAL/convex_hull_2.h>
#include <CGAL/Convex_hull_traits_adapter_2.h>

namespace Common_tools
{
typedef CGAL::Simple_cartesian<double> CGAL_Kernel;
// typedef CGAL::Exact_predicates_inexact_constructions_kernel CGAL_Kernel;
// typedef CGAL::Exact_predicates_exact_constructions_kernel CGAL_Kernel;
typedef CGAL_Kernel::Point_2                                Point_2;
typedef CGAL_Kernel::Segment_2                              Segment_2;
typedef CGAL_Kernel::Line_2                                 Line_2;
typedef CGAL_Kernel::Intersect_2                            Intersect_2;
typedef CGAL_Kernel::Triangle_2                             Triangle_2; 
typedef CGAL::Polygon_2< CGAL_Kernel >                      Polygon_2;

typedef CGAL_Kernel::Point_3    Point_3;
typedef CGAL_Kernel::Triangle_3 Triangle_3;
typedef CGAL_Kernel::Segment_3  Segment_3;

typedef CGAL::Convex_hull_traits_adapter_2< CGAL_Kernel, CGAL::Pointer_property_map< Point_2 >::type > Convex_hull_traits_2;

typedef CGAL::Triangulation_vertex_base_with_info_2< unsigned, CGAL_Kernel >                  TVb2;
typedef CGAL::Triangulation_data_structure_2< TVb2 >                                          Tds2;
typedef CGAL::Delaunay_triangulation_2< CGAL_Kernel, Tds2 >                                   Delaunay2;
typedef Delaunay2::Point                                                                      D2_Point;
typedef Delaunay2::Vertex_handle                                                              D2_Vertex_handle;
typedef CGAL::Convex_hull_traits_adapter_2< CGAL_Kernel, CGAL::Pointer_property_map< D2_Point >::type > Convex_hull_traits_2;

using std::cout;
using std::endl;

void inline printf_triangle_pair( Triangle_2 &triangle_a, Triangle_2 &triangle_b )
{
    // clang-format off
    printf("Tri {(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)} V.S. {(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)}", 
        triangle_a.vertex(0).x(), triangle_a.vertex(0).y(), 
        triangle_a.vertex(1).x(), triangle_a.vertex(1).y(), 
        triangle_a.vertex(2).x(), triangle_a.vertex(2).y(), 
        triangle_b.vertex(0).x(), triangle_b.vertex(0).y(), 
        triangle_a.vertex(1).x(), triangle_b.vertex(1).y(), 
        triangle_b.vertex(2).x(), triangle_b.vertex(2).y() );
    // clang-format on
}

inline int triangle_intersect_triangle( Triangle_2 &triangle_a, Triangle_2 &triangle_b, double area_threshold = 0.001 )
{
    CGAL::Intersection_traits< CGAL_Kernel, Triangle_2, Triangle_2 >::result_type result = CGAL::intersection( triangle_a, triangle_b );
    
    // cout << "=== Test triangle intersection result ===" << endl;
    // printf_triangle_pair(triangle_a, triangle_b);
    if ( result )
    {
        // cout << "Intersect!" << endl;
        if ( const Point_2 *p = boost::get< Point_2 >( &*result ) )
        {
            // const Point_2 *p = boost::get< Point_2 >( &*result );
            // std::cout << "Intersection as points: "<< *p << std::endl;
            return 1;
        }
        else if ( const Segment_2 *s = boost::get< Segment_2 >( &*result ) )
        {
            // std::cout << "Intersection as Segment: "  << *s << std::endl;
            return 2;
        }
        else if ( const Triangle_2 *tri = boost::get< Triangle_2 >( &*result ) )
        {
            // const Point_2 *p = boost::get< Point_2 >( &*result );
            // printf("Intersection as triangle: {(%.2f, %.2f), (%.2f, %.2f), (%.2f, %.2f)}\r\n", 
            // (*tri).vertex(0).x(), (*tri).vertex(0).y(), 
            // (*tri).vertex(1).x(), (*tri).vertex(1).y(), 
            // (*tri).vertex(2).x(), (*tri).vertex(2).y() );
            // std::cout << "Intersection as triangle: "<< *tri << ", area = " << (*tri).area() << std::endl;
            if((*tri).area() < area_threshold)
            {
                return 0;
            }
            std::cout << "Intersection as triangle: "<< *tri << ", area = " << (*tri).area() << std::endl;
            return 3;
        }
        else if ( const std::vector<Point_2> *vec_pts = boost::get< std::vector<Point_2> >( &*result ) )
        {   
            Polygon_2 poly(vec_pts->begin(), vec_pts->end());
            // cout << "Intersection as polygon: "<< vec_pts->size() << ", area = " << poly.area()  << endl;
            // for(int i =0 ;i < (*vec_pts).size(); i++)
            // {
            //     cout << "("<< (*vec_pts)[i] << "), ";
            // }
            // cout << endl;
            if( poly.area() < area_threshold)
            {
                return 0;
            }
            cout << "Intersection as polygon: "<< vec_pts->size() << ", area = " << poly.area()  << endl;
            return (4 + vec_pts->size());
        }
    }
    else
    {
        // cout << "No intersection" << endl;
        return 0;
    }
    cout << "=== Test triangle intersection   end ===" << endl;
    
}
} // namespace Common_tools