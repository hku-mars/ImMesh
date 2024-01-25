#pragma once
// this part of codes is copied from Pangolin: https://github.com/stevenlovegrove/Pangolin
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <Eigen/Core>
#include <Eigen/Geometry>


#ifndef GL_LUMINANCE32UI_EXT // I don't know why this is not defined if include GLAD
#include "GL/glext.h"
#endif


template<typename T>
struct GlFormatTraits;
//{
//    static const GLint glinternalformat = 0;
//    static const GLenum glformat = 0;
//    static const GLenum gltype = 0;
//    static const T glmin = 0;
//    static const T glmax = 0;
//};

template<>
struct GlFormatTraits<unsigned char>
{
    static const GLint glinternalformat = GL_LUMINANCE8;
    static const GLenum glformat = GL_LUMINANCE;
    static const GLenum gltype = GL_UNSIGNED_BYTE;
    static const size_t components = 1;
};

template<>
struct GlFormatTraits<unsigned short>
{
    static const GLint glinternalformat = GL_LUMINANCE16;
    static const GLenum glformat = GL_LUMINANCE;
    static const GLenum gltype = GL_UNSIGNED_SHORT;
    static const size_t components = 1;
};

template<>
struct GlFormatTraits<unsigned int>
{
    static const GLint glinternalformat = GL_LUMINANCE32UI_EXT;
    static const GLenum glformat = GL_LUMINANCE_INTEGER_EXT;
    static const GLenum gltype = GL_UNSIGNED_INT;
    static const size_t components = 1;
};

template<>
struct GlFormatTraits<int>
{
    static const GLint glinternalformat = GL_LUMINANCE32I_EXT;
    static const GLenum glformat = GL_LUMINANCE_INTEGER_EXT;
    static const GLenum gltype = GL_INT;
    static const size_t components = 1;
};

template<>
struct GlFormatTraits<float>
{
    static const GLint glinternalformat = GL_LUMINANCE32F_ARB;
    static const GLenum glformat = GL_LUMINANCE;
    static const GLenum gltype = GL_FLOAT;
    static const size_t components = 1;
};

template<>
struct GlFormatTraits<double>
{
    static const GLint glinternalformat = GL_LUMINANCE32F_ARB;
    static const GLenum glformat = GL_LUMINANCE;
    static const GLenum gltype = GL_DOUBLE;
    static const size_t components = 1;
};


//////////////////////////////////////////////////////////////////

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned char,2,1>>
{
    static const GLint glinternalformat = GL_RG8;
    static const GLenum glformat = GL_RG;
    static const GLenum gltype = GL_UNSIGNED_BYTE;
    static const size_t components = 2;
};

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned short,2,1>>
{
    static const GLint glinternalformat = GL_RG16;
    static const GLenum glformat = GL_RG;
    static const GLenum gltype = GL_UNSIGNED_SHORT;
    static const size_t components = 2;
};

template <>
struct GlFormatTraits<Eigen::Vector2i>
{
    static const GLint glinternalformat = GL_RG32I;
    static const GLenum glformat = GL_RG;
    static const GLenum gltype = GL_INT;
    static const size_t components = 2;
};

template <>
struct GlFormatTraits<Eigen::Vector2f>
{
    static const GLint glinternalformat = GL_RG32F;
    static const GLenum glformat = GL_RG;
    static const GLenum gltype = GL_FLOAT;
    static const size_t components = 2;
};

template <>
struct GlFormatTraits<Eigen::Vector2d>
{
    static const GLint glinternalformat = GL_RG32F;
    static const GLenum glformat = GL_RG;
    static const GLenum gltype = GL_DOUBLE;
    static const size_t components = 2;
};

//////////////////////////////////////////////////////////////////

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned char,3,1>>
{
    static const GLint glinternalformat = GL_RGB8;
    static const GLenum glformat = GL_RGB;
    static const GLenum gltype = GL_UNSIGNED_BYTE;
    static const size_t components = 3;
};

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned short,3,1>>
{
    static const GLint glinternalformat = GL_RGBA16;
    static const GLenum glformat = GL_RGB;
    static const GLenum gltype = GL_UNSIGNED_SHORT;
    static const size_t components = 3;
};

template <>
struct GlFormatTraits<Eigen::Vector3f>
{
    static const GLint glinternalformat = GL_RGB32F;
    static const GLenum glformat = GL_RGB;
    static const GLenum gltype = GL_FLOAT;
    static const size_t components = 3;
};

template <>
struct GlFormatTraits<Eigen::Vector3d>
{
    static const GLint glinternalformat = GL_RGB32F;
    static const GLenum glformat = GL_RGB;
    static const GLenum gltype = GL_DOUBLE;
    static const size_t components = 3;
};

//////////////////////////////////////////////////////////////////

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned char,4,1>>
{
    static const GLint glinternalformat = GL_RGBA8;
    static const GLenum glformat = GL_RGBA;
    static const GLenum gltype = GL_UNSIGNED_BYTE;
    static const size_t components = 4;
};

template <>
struct GlFormatTraits<Eigen::Matrix<unsigned short,4,1>>
{
    static const GLint glinternalformat = GL_RGBA16;
    static const GLenum glformat = GL_RGBA;
    static const GLenum gltype = GL_UNSIGNED_SHORT;
    static const size_t components = 4;
};

template <>
struct GlFormatTraits<Eigen::Vector4f>
{
    static const GLint glinternalformat = GL_RGBA32F;
    static const GLenum glformat = GL_RGBA;
    static const GLenum gltype = GL_FLOAT;
    static const size_t components = 4;
};

template <>
struct GlFormatTraits<Eigen::Vector4d>
{
    static const GLint glinternalformat = GL_RGBA32F;
    static const GLenum glformat = GL_RGBA;
    static const GLenum gltype = GL_DOUBLE;
    static const size_t components = 4;
};

