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
#ifndef __TOOOLS_MY_SHADER_COMMON_H__
#define __TOOOLS_MY_SHADER_COMMON_H__

#include "shader_m.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "stb_image.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "learnopengl/filesystem.h"
#include "learnopengl/shader_m.h"
#include "learnopengl/camera.h"
#include "learnopengl/model.h"
#include "shader_m.h"
#include "tools/tools_eigen.hpp"
#include "tools/tools_timer.hpp"
#include "tools/tools_logger.hpp"
#include "tools/tools_color_printf.hpp"
#include "tools/tinycolormap.hpp"

#define HAVE_OPENCV 1
#if HAVE_OPENCV
#include "opencv2/opencv.hpp"
#endif

namespace Common_tools
{

#if HAVE_OPENCV
inline cv::Mat get_stencil_buffer( float scaled = 0.5, std::string window_name = std::string( "" ) )
{

    // Visualize stencil buffer
    const GLFWvidmode* vidmode = glfwGetVideoMode( glfwGetPrimaryMonitor() );
    int                win_w = vidmode->width;
    int                win_h = vidmode->height;
    cout << "Window size = " << win_w << " x " << win_h << endl;
    cv::Mat stencilImage = cv::Mat::zeros( win_h, win_w, CV_8UC1 );
    glReadPixels( 0, 0, win_w, win_h, GL_STENCIL_INDEX, GL_UNSIGNED_BYTE, stencilImage.data );
    // glReadPixels( 0, 0, win_w, win_h, GL_STENCIL_VALUE_MASK, GL_UNSIGNED_BYTE, stencilImage.data );
    stencilImage = stencilImage;
    cv::Mat stencilImage_resize;
    if ( scaled != 1.0 )
    {
        cv::resize( stencilImage, stencilImage_resize, cv::Size( 0, 0 ), scaled, scaled );
    }
    else
    {
        stencilImage_resize = stencilImage;
    }
    if ( window_name.length() > 1 )
    {
        cv::imshow( window_name, stencilImage_resize );
        cv::waitKey( 1 );
    }
    return stencilImage_resize;
}
#endif

template < typename T >
inline glm::mat4 eigen2glm( const Eigen::Matrix< T, 4, 4 >& eigen_mat )
{
    glm::mat4 temp_mat;
    for ( int i = 0; i < 4; i++ )
    {
        for ( int j = 0; j < 4; j++ )
        {
            temp_mat[ i ][ j ] = eigen_mat( j, i );
        }
    }
    return temp_mat;
}

inline unsigned int loadTexture( char const* path )
{
    unsigned int textureID;
    glGenTextures( 1, &textureID );

    int            width, height, nrComponents;
    unsigned char* data = stbi_load( path, &width, &height, &nrComponents, 0 );
    if ( data )
    {
        GLenum format;
        if ( nrComponents == 1 )
            format = GL_RED;
        else if ( nrComponents == 3 )
            format = GL_RGB;
        else if ( nrComponents == 4 )
            format = GL_RGBA;

        glBindTexture( GL_TEXTURE_2D, textureID );
        glTexImage2D( GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data );
        glGenerateMipmap( GL_TEXTURE_2D );

        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR );
        glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );

        stbi_image_free( data );
    }
    else
    {
        std::cout << "Texture failed to load at path: " << path << std::endl;
        stbi_image_free( data );
    }

    return textureID;
}

// loads a cubemap texture from 6 individual texture faces
// order:
// +X (right)
// -X (left)
// +Y (top)
// -Y (bottom)
// +Z (front)
// -Z (back)
// -------------------------------------------------------
inline unsigned int loadCubemap( vector< std::string > faces )
{
    unsigned int textureID;
    glGenTextures( 1, &textureID );
    glBindTexture( GL_TEXTURE_CUBE_MAP, textureID );

    int width, height, nrChannels;
    for ( unsigned int i = 0; i < faces.size(); i++ )
    {
        unsigned char* data = stbi_load( faces[ i ].c_str(), &width, &height, &nrChannels, 0 );
        if ( data )
        {
            glTexImage2D( GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data );
            stbi_image_free( data );
        }
        else
        {
            std::cout << "Cubemap texture failed to load at path: " << faces[ i ] << std::endl;
            stbi_image_free( data );
        }
    }
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );
    glTexParameteri( GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE );

    return textureID;
}

struct Point_element
{
    float m_pos[ 3 ];
    float m_color = 0x000000FF;
    Point_element() = default;
    void pack_color(const unsigned char r, const unsigned char g, const unsigned char b)
    {
        m_color =  (r << 16) | (g << 8) | b;
    }

    void pack_color(const float rf, const float gf, const float bf)
    {
        unsigned char r = (unsigned char)(rf * 255);
        unsigned char g = (unsigned char)(gf * 255);
        unsigned char b = (unsigned char)(bf * 255);
        m_color =  (r << 16) | (g << 8) | b;
    }

    void pack_color(const double rf, const double gf, const double bf)
    {
        unsigned char r = (unsigned char)(rf * 255);
        unsigned char g = (unsigned char)(gf * 255);
        unsigned char b = (unsigned char)(bf * 255);
        m_color =  (r << 16) | (g << 8) | b;
    }

    Point_element( const float& x, const float& y, const float& z, const float& rf, const float& gf, const float& bf )
    {
        m_pos[ 0 ] = x;
        m_pos[ 1 ] = y;
        m_pos[ 2 ] = z;
        pack_color( ( float ) rf, ( float ) gf, ( float ) bf );
    }
    Point_element( const float& x, const float& y, const float& z, const double& rf, const double& gf, const double& bf )
    {
        m_pos[ 0 ] = x;
        m_pos[ 1 ] = y;
        m_pos[ 2 ] = z;
        pack_color( ( float ) rf, ( float ) gf, ( float ) bf );
    }
    Point_element( const float& x, const float& y, const float& z, const unsigned char& r, const unsigned char& g, const unsigned char& b )
    {
        m_pos[ 0 ] = x;
        m_pos[ 1 ] = y;
        m_pos[ 2 ] = z;
        pack_color( r, g, b );
    }
    void debug()
    {
        int m_color_cast_int = int(m_color);
        vec_3 fragColor = vec_3( (m_color_cast_int >> 16) & 0xFF, (m_color_cast_int >> 8) & 0xFF, m_color_cast_int & 0xFF) / 255.0;
        printf("Bit = %x, ", m_color_cast_int);
        cout <<"Frag_color = " << fragColor.transpose() << endl;
    }

};

} // namespace Common_tools

#endif