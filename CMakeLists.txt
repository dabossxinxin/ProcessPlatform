cmake_minimum_required(VERSION 3.0.0)

# 为了兼容新版本的cmake，屏蔽一定的警告
IF(POLICY CMP0071)
	CMAKE_POLICY(SET CMP0071 NEW)
ENDIF()

# 设置工程的名称
PROJECT(ImgaeProcessPlatform)

# CMAKE_SOURCE_DIR为源码所在最顶层路径
# 在搜寻默认路径前先搜寻这个路径下的CMake模块
SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake)
# DEBUG下的加后缀
SET(CMAKE_DEBUG_POSTFIX "D")

MESSAGE(STATUS "CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR})
MESSAGE(STATUS "MSVC: " ${MSVC})

IF(MSVC)
  IF(CMAKE_CL_64)
    MESSAGE(STATUS "Build the x64 project.")
  ELSE()
    MESSAGE(FATAL_ERROR "FATAL: The project only construct by x64 system, can not construct x86 system.")
    RETURN()
  ENDIF()

  IF(MSVC_VERSION EQUAL 1400)  # Visual Studio 2005
    MESSAGE(FATAL_ERROR "FATAL: The compiler should not lower than visual studio 2013.")
    RETURN()
  ELSEIF(MSVC_VERSION EQUAL 1500)  # Visual Studio 2008
    MESSAGE(FATAL_ERROR "FATAL: The compiler should not lower than visual studio 2013.")
    RETURN()
  ELSEIF(MSVC_VERSION EQUAL 1600)  # Visual Studio 2010
    MESSAGE(FATAL_ERROR "FATAL: The compiler should not lower than visual studio 2013.")
    RETURN()
  ELSEIF(MSVC_VERSION EQUAL 1700)  # Visual Studio 2012
    MESSAGE(FATAL_ERROR "FATAL: The compiler should not lower than visual studio 2013.")
    RETURN()
  ELSEIF(MSVC_VERSION EQUAL 1800)  # Visual Studio 2013
    INCLUDE(msvc2013)
  ELSEIF(MSVC_VERSION EQUAL 1900)  # Visual Studio 2015
    INCLUDE(msvc)
  ELSE()
    INCLUDE(msvc)
  ENDIF()
ELSEIF()
  MESSAGE(FATAL_ERROR "FATAL: The source can only use the MSVC GENERATOR")
  RETURN()
ENDIF()

