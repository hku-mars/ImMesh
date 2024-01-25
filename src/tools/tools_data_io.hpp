/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

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
#ifndef __TOOLS_DATA_IO_HPP__
#define __TOOLS_DATA_IO_HPP__
#include <iostream>
#include <Eigen/Eigen>
#include <string>
#include <vector>
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"

// #define DATA_IO_VERSION      "V1.0"
// #define DATA_IO_VERSION_INFO "Init repo."

// #define DATA_IO_VERSION      "V1.1"
// #define DATA_IO_VERSION_INFO "More robustness, enable split with given char."

#define DATA_IO_VERSION      "V1.2"
#define DATA_IO_VERSION_INFO "Make more robust, detect NaN and skip this line."

namespace Common_tools
{
using std::cout;
using std::endl;

template<typename T=float>
inline void save_matrix_to_txt( std::string file_name, Eigen::Matrix<T, -1, -1> mat )
{

    FILE *fp = fopen( file_name.c_str(), "w+" );
    int   cols_size = mat.cols();
    int   rows_size = mat.rows();
    for ( int i = 0; i < rows_size; i++ )
    {
        for ( int j = 0; j < cols_size; j++ )
        {
            fprintf( fp, "%.15f ", mat( i, j ) );
        }
        fprintf( fp, "\r\n" );
    }
    fclose( fp );
}

inline void save_matrix_to_txt( std::string file_name, Eigen::SparseMatrix< double > mat )
{
    save_matrix_to_txt( file_name, mat.toDense() );
}

template<typename T=float>
inline Eigen::Matrix<T, -1, -1> mat_from_data_vec(const std::vector< std::vector< T > > & data_vec, char split_char = ' '  )
{   
    Eigen::Matrix<T, -1, -1> res_mat;
    if(data_vec.size() ==0 )
    {
        return res_mat;
    }
    res_mat.resize( data_vec.size(), data_vec[ 0 ].size() );
    for (unsigned int i = 0; i < data_vec.size(); i++ )
    {
        for (unsigned int j = 0; j < data_vec[i].size(); j++ )
        {
            res_mat( i, j ) = data_vec[ i ][ j ];
        }
    }
    return res_mat;
}
  
template<typename T=float>
inline std::vector<std::vector<T>> load_data_from_txt(std::string file_name, char split_char = ' ' )
{
    static const int DATA_RESERVE_SIZE = 1e8;
    std::vector<std::vector<T>> data_mat;
    FILE * fp;
    // cout << "Load date from: " << file_name.c_str() << endl;
    fp = fopen(file_name.c_str(), "r");
    if(fp == nullptr)
    {
        cout << "Can not load data from " << file_name << ", please check!" << endl;
    }
    else
    { 
        std::string line_str;
        line_str.reserve( DATA_RESERVE_SIZE );
    read_line:
        while ( fgets( ( char * ) line_str.data(), DATA_RESERVE_SIZE, fp ) != NULL )
        {

            std::vector< T > data_vec;
            data_vec.reserve( 1e5 );
            T                 data = -3e8;
            std::stringstream ss( line_str.c_str() );
            std::string       single_s;
            while ( std::getline( ss, single_s, split_char ) )
            {
                try
                {
                    data = std::stod( single_s );
                    if ( std::isnan( data ) )
                    {
                        cout << "Nan data detected, skip this line!!! " << endl;
                        goto read_line;
                    }
                }
                catch ( ... )
                {
                    break;
                }
                data_vec.push_back( data );
                
            }
            data_mat.push_back( data_vec );
            line_str = std::string(" ");
        }
        fclose(fp);
    }
    for( int i = 1; i <data_mat.size(); i++  )
    {
        if( data_mat[i].size() != data_mat[0].size() )
        {
            cout << "[" << i << "] " << " row exception, current size = " << data_mat[i].size() << ", but size of first row is " << data_mat[0].size() << endl;
            while(1);
        }
    }
    return data_mat;
}

template<typename T=float>
inline Eigen::Matrix<T, -1, -1> load_mat_from_txt(std::string file_name, char split_char = ' ')
{
    return mat_from_data_vec<T>(load_data_from_txt<T>(file_name, split_char));
}

}
#endif