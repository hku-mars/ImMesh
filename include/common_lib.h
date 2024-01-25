#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#pragma once

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <so3_math.h>
#include <types.h>
// #include <fast_lio/States.h>
// #include <fast_lio/Pose6D.h>
#include <boost/shared_ptr.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <unordered_map>
// #include <point.h>

using namespace std;
using namespace Eigen;
// using namespace Sophus;

// #define DEBUG_PRINT
#define USE_ikdtree
// #define USE_ikdforest
//#define USE_IKFOM
// #define USE_FOV_Checker

#define print_line std::cout << __FILE__ << ", " << __LINE__ << std::endl;
#define PI_M ( 3.14159265358 )
#define G_m_s2 ( 9.81 )   // Gravaty const in GuangDong/China
#define DIM_STATE ( 18 )  // Dimension of states (Let Dim(SO(3)) = 3)
#define DIM_PROC_N ( 12 ) // Dimension of process noise (Let Dim(SO(3)) = 3)
#define CUBE_LEN ( 6.0 )
#define LIDAR_SP_LEN ( 2 )
#define INIT_COV ( 0.0000001 )
#define CALIB_ANGLE_COV ( 0.01 )
#define NUM_MATCH_POINTS ( 5 )
#define MAX_MEAS_DIM ( 10000 )

#define VEC_FROM_ARRAY( v ) v[ 0 ], v[ 1 ], v[ 2 ]
#define MAT_FROM_ARRAY( v ) v[ 0 ], v[ 1 ], v[ 2 ], v[ 3 ], v[ 4 ], v[ 5 ], v[ 6 ], v[ 7 ], v[ 8 ]
#define CONSTRAIN( v, min, max ) ( ( v > min ) ? ( ( v < max ) ? v : max ) : min )
#define ARRAY_FROM_EIGEN( mat ) mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN( mat ) vector< decltype( mat )::Scalar >( mat.data(), mat.data() + mat.rows() * mat.cols() )
#define DEBUG_FILE_DIR( name ) ( string( string( ROOT_DIR ) + "Log/" + name ) )

#define HASH_P 116101
#define MAX_N 10000000000

extern M3D      Eye3d;
extern M3F      Eye3f;
extern V3D      Zero3d;
extern V3F      Zero3f;
extern Vector3d Lidar_offset_to_IMU;

enum LID_TYPE
{
    AVIA = 1,
    VELO16 = 2,
    OUST64 = 3,
    L515 = 4,
    XT32 = 5,
    VELO32 = 6
}; //{1, 2, 3}

enum TIME_UNIT
{
    SEC = 0,
    MS = 1,
    US = 2,
    NS = 3
};
namespace lidar_selection
{
class Point;
class VOXEL_POINTS
{
  public:
    std::vector< Point * > voxel_points;
    int                    count;
    // bool is_visited;

    VOXEL_POINTS( int num ) : count( num ) {}
};

class Warp
{
  public:
    Matrix2d A_cur_ref;
    int      search_level;
    // bool is_visited;

    Warp( int level, Matrix2d warp_matrix ) : search_level( level ), A_cur_ref( warp_matrix ) {}
};
} // namespace lidar_selection

// Key of hash table
class VOXEL_KEY
{

  public:
    int64_t x;
    int64_t y;
    int64_t z;

    VOXEL_KEY( int64_t vx = 0, int64_t vy = 0, int64_t vz = 0 ) : x( vx ), y( vy ), z( vz ) {}

    bool operator==( const VOXEL_KEY &other ) const { return ( x == other.x && y == other.y && z == other.z ); }

    bool operator<( const VOXEL_KEY &p ) const
    {
        if ( x < p.x )
            return true;
        if ( x > p.x )
            return false;
        if ( y < p.y )
            return true;
        if ( y > p.y )
            return false;
        if ( z < p.z )
            return true;
        if ( z > p.z )
            return false;
    }
};

// Hash value
namespace std
{
template <>
struct hash< VOXEL_KEY >
{
    size_t operator()( const VOXEL_KEY &s ) const
    {
        using std::hash;
        using std::size_t;

        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:
        //   return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^
        //   (hash<int64_t>()(s.z) << 1);
        return ( ( ( hash< int64_t >()( s.z ) * HASH_P ) % MAX_N + hash< int64_t >()( s.y ) ) * HASH_P ) % MAX_N + hash< int64_t >()( s.x );
    }
};
} // namespace std

struct MeasureGroup
{
    double                              img_offset_time;
    deque< sensor_msgs::Imu::ConstPtr > imu;
    cv::Mat                             img;
    MeasureGroup() { img_offset_time = 0.0; };
};

struct LidarMeasureGroup
{
    double                       lidar_beg_time;
    double                       last_update_time;
    PointCloudXYZI::Ptr          lidar;
    deque< struct MeasureGroup > measures;
    bool                         is_lidar_end;
    int                          lidar_scan_index_now;
    LidarMeasureGroup()
    {
        lidar_beg_time = 0.0;
        is_lidar_end = false;
        this->lidar.reset( new PointCloudXYZI() );
        this->measures.clear();
        lidar_scan_index_now = 0;
        last_update_time = 0.0;
    };
    void debug_show()
    {
        int i = 0;
        ROS_WARN( "Lidar selector debug:" );
        std::cout << "last_update_time:" << setprecision( 20 ) << this->last_update_time << endl;
        std::cout << "lidar_beg_time:" << setprecision( 20 ) << this->lidar_beg_time << endl;
        // for (auto it = this->measures.begin(); it != this->measures.end();
        // ++it,++i) {
        //     std::cout<<"In "<<i<<" measures: ";
        //     for (auto it_meas=it->imu.begin(); it_meas!=it->imu.end();++it_meas)
        //     {
        //         std::cout<<setprecision(20)<<(*it_meas)->header.stamp.toSec()-this->lidar_beg_time<<"
        //         ";
        //     }
        //     std::cout<<"img_time:"<<setprecision(20)<<it->img_offset_time<<endl;
        // }
        std::cout << "is_lidar_end:" << this->is_lidar_end << "lidar_end_time:" << this->lidar->points.back().curvature / double( 1000 ) << endl;
        std::cout << "lidar_.points.size(): " << this->lidar->points.size() << endl << endl;
    };
};

struct StatesGroup
{
    StatesGroup()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
        this->bias_g = Zero3d;
        this->bias_a = Zero3d;
        this->gravity = Zero3d;
        this->cov = Matrix< double, DIM_STATE, DIM_STATE >::Identity() * INIT_COV;
        // this->cov.block< 9, 9 >( 9, 9 ) = MD( 9, 9 )::Identity() * 0.00001;
    };

    StatesGroup( const StatesGroup &b )
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
    };

    StatesGroup &operator=( const StatesGroup &b )
    {
        this->rot_end = b.rot_end;
        this->pos_end = b.pos_end;
        this->vel_end = b.vel_end;
        this->bias_g = b.bias_g;
        this->bias_a = b.bias_a;
        this->gravity = b.gravity;
        this->cov = b.cov;
        return *this;
    };

    StatesGroup operator+( const Matrix< double, DIM_STATE, 1 > &state_add )
    {
        StatesGroup a;
        a.rot_end = this->rot_end * Exp( state_add( 0, 0 ), state_add( 1, 0 ), state_add( 2, 0 ) );
        a.pos_end = this->pos_end + state_add.block< 3, 1 >( 3, 0 );
        a.vel_end = this->vel_end + state_add.block< 3, 1 >( 6, 0 );
        a.bias_g = this->bias_g + state_add.block< 3, 1 >( 9, 0 );
        a.bias_a = this->bias_a + state_add.block< 3, 1 >( 12, 0 );
        a.gravity = this->gravity + state_add.block< 3, 1 >( 15, 0 );
        a.cov = this->cov;
        return a;
    };

    StatesGroup &operator+=( const Matrix< double, DIM_STATE, 1 > &state_add )
    {
        this->rot_end = this->rot_end * Exp( state_add( 0, 0 ), state_add( 1, 0 ), state_add( 2, 0 ) );
        this->pos_end += state_add.block< 3, 1 >( 3, 0 );
        this->vel_end += state_add.block< 3, 1 >( 6, 0 );
        this->bias_g += state_add.block< 3, 1 >( 9, 0 );
        this->bias_a += state_add.block< 3, 1 >( 12, 0 );
        this->gravity += state_add.block< 3, 1 >( 15, 0 );
        return *this;
    };

    Matrix< double, DIM_STATE, 1 > operator-( const StatesGroup &b )
    {
        Matrix< double, DIM_STATE, 1 > a;
        M3D                            rotd( b.rot_end.transpose() * this->rot_end );
        a.block< 3, 1 >( 0, 0 ) = Log( rotd );
        a.block< 3, 1 >( 3, 0 ) = this->pos_end - b.pos_end;
        a.block< 3, 1 >( 6, 0 ) = this->vel_end - b.vel_end;
        a.block< 3, 1 >( 9, 0 ) = this->bias_g - b.bias_g;
        a.block< 3, 1 >( 12, 0 ) = this->bias_a - b.bias_a;
        a.block< 3, 1 >( 15, 0 ) = this->gravity - b.gravity;
        return a;
    };

    void resetpose()
    {
        this->rot_end = M3D::Identity();
        this->pos_end = Zero3d;
        this->vel_end = Zero3d;
    }

    M3D rot_end;                                    // the estimated attitude (rotation matrix) at the end lidar
                                                    // point
    V3D                                    pos_end; // the estimated position at the end lidar point (world frame)
    V3D                                    vel_end; // the estimated velocity at the end lidar point (world frame)
    V3D                                    bias_g;  // gyroscope bias
    V3D                                    bias_a;  // accelerator bias
    V3D                                    gravity; // the estimated gravity acceleration
    Matrix< double, DIM_STATE, DIM_STATE > cov;     // states covariance
};

template < typename T >
T rad2deg( T radians )
{
    return radians * 180.0 / PI_M;
}

template < typename T >
T deg2rad( T degrees )
{
    return degrees * PI_M / 180.0;
}

template < typename T >
auto set_pose6d( const double t, const Matrix< T, 3, 1 > &a, const Matrix< T, 3, 1 > &g, const Matrix< T, 3, 1 > &v, const Matrix< T, 3, 1 > &p,
                 const Matrix< T, 3, 3 > &R )
{
    Pose6D rot_kp;
    rot_kp.offset_time = t;
    for ( int i = 0; i < 3; i++ )
    {
        rot_kp.acc[ i ] = a( i );
        rot_kp.gyr[ i ] = g( i );
        rot_kp.vel[ i ] = v( i );
        rot_kp.pos[ i ] = p( i );
        for ( int j = 0; j < 3; j++ )
            rot_kp.rot[ i * 3 + j ] = R( i, j );
    }
    // Map<M3D>(rot_kp.rot, 3,3) = R;
    return move( rot_kp );
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template < typename T >
bool esti_normvector( Matrix< T, 3, 1 > &normvec, const PointVector &point, const T &threshold, const int &point_num )
{
    MatrixXf A( point_num, 3 );
    MatrixXf b( point_num, 1 );
    b.setOnes();
    b *= -1.0f;

    for ( int j = 0; j < point_num; j++ )
    {
        A( j, 0 ) = point[ j ].x;
        A( j, 1 ) = point[ j ].y;
        A( j, 2 ) = point[ j ].z;
    }
    normvec = A.colPivHouseholderQr().solve( b );

    for ( int j = 0; j < point_num; j++ )
    {
        if ( fabs( normvec( 0 ) * point[ j ].x + normvec( 1 ) * point[ j ].y + normvec( 2 ) * point[ j ].z + 1.0f ) > threshold )
        {
            return false;
        }
    }

    normvec.normalize();
    return true;
}

template < typename T >
bool esti_plane( Matrix< T, 4, 1 > &pca_result, const PointVector &point, const T &threshold )
{
    Matrix< T, NUM_MATCH_POINTS, 3 > A;
    Matrix< T, NUM_MATCH_POINTS, 1 > b;
    b.setOnes();
    b *= -1.0f;

    for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
    {
        A( j, 0 ) = point[ j ].x;
        A( j, 1 ) = point[ j ].y;
        A( j, 2 ) = point[ j ].z;
    }

    Matrix< T, 3, 1 > normvec = A.colPivHouseholderQr().solve( b );

    T n = normvec.norm();
    pca_result( 0 ) = normvec( 0 ) / n;
    pca_result( 1 ) = normvec( 1 ) / n;
    pca_result( 2 ) = normvec( 2 ) / n;
    pca_result( 3 ) = 1.0 / n;

    for ( int j = 0; j < NUM_MATCH_POINTS; j++ )
    {
        if ( fabs( pca_result( 0 ) * point[ j ].x + pca_result( 1 ) * point[ j ].y + pca_result( 2 ) * point[ j ].z + pca_result( 3 ) ) > threshold )
        {
            return false;
        }
    }

    // for (int j = 0; j < NUM_MATCH_POINTS; j++)
    // {
    //     if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2)
    //     * point[j].z + 1.0f) > threshold)
    //     {
    //         return false;
    //     }
    // }

    // T n = normvec.norm();
    // pca_result(0) = normvec(0) / n;
    // pca_result(1) = normvec(1) / n;
    // pca_result(2) = normvec(2) / n;
    // pca_result(3) = 1.0 / n;
    return true;
}

#endif