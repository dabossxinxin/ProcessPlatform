
MESSAGE(STATUS "Find basic library: vtk Qt PCL OpenCV")
 
INCLUDE(FindVTK)
INCLUDE(FindQt)
INCLUDE(FindOpenCV)
INCLUDE(FindPCL)
#INCLUDE(FindCUDA)
INCLUDE(FindBoost)

# 设置添加当前目录
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
# 设置自动生成Qt MOC文件
# TODO Qt MOC文件有什么作用
SET(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)

# 添加指定目录中所有的源文件
AUX_SOURCE_DIRECTORY(. Srcs)

# 查找指定文件目录中的头文件
FILE(GLOB Hdrs "./*.h"  "./PointCloud/*.h")

INCLUDE_DIRECTORIES(${Hdrs})

# 查找指定文件目录中的Ui文件
FILE(GLOB UIs "./*.ui" "./PointCloud/*.ui")

# 生成工程中包含的Ui文件,生成的.h文件放在UIs中，.cxx文件放在Views中
QT5_WRAP_UI(Warps ${UIs} ${Views} )

# 添加资源文件
Qt5_ADD_RESOURCES(qrcs findcirclesapplication.qrc)

# 在VS工程创建文件目录
SOURCE_GROUP("Form Files" FILES ${UIs} ${Views})
SOURCE_GROUP("Generated Files" FILES ${Warps})
SOURCE_GROUP("Resource Files" FILES ${qrcs} findcirclesapplication.qrc)

message(STATUS "PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

SET(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/../lib)
SET(BIN_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/../bin)

ADD_EXECUTABLE(${PROJECT_NAME} ${Srcs} ${Hdrs} ${Warps} ${qrcs})
#set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES 35 52 60 61 70 72 75 80 90)
#set_target_properties(${PROJECT_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# 设置release的属性，屏蔽控制台窗口, 同时设置两项，不能分开写
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")

# 对输出文件进行重命名
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "QiXiang")

# 添加Qt库
MESSAGE(STATUS "Qt5_INCLUDE_DIR: " ${Qt5_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${Qt5_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui Qt5::Xml Qt5::PrintSupport )

# 添加VTK类库文件
INCLUDE_DIRECTORIES(${VTK_INCLUDE_DIRS})
FOREACH(_var ${VTK_LIBRARIES})
    TARGET_LINK_LIBRARIES(${PROJECT_NAME} debug ${VTK_LIB_DIR}/${_var}d.lib
        optimized ${VTK_LIB_DIR}/${_var}.lib)
ENDFOREACH()

# 添加Eigen3的依赖
FIND_PACKAGE(Eigen3)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

# 添加Boost类库文件
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
FOREACH(_var ${Boost_LIBRARIES})
	 TARGET_LINK_LIBRARIES(${PROJECT_NAME} debug ${Boost_LIB_DIR}/${_var}-gd-x64-1_68.lib
	optimized ${Boost_LIB_DIR}/${_var}-x64-1_68.lib)
ENDFOREACH()

# 添加cuda库文件
#INCLUDE_DIRECTORIES(${CUDA_INCLUDE_DIRS})
#FOREACH(_var ${CUDA_LIBRARIES})
#    TARGET_LINK_LIBRARIES(${PROJECT_NAME} debug ${CUDA_LIB_DIR}/${_var}.lib
#        optimized ${CUDA_LIB_DIR}/${_var}.lib)
#ENDFOREACH()

# 添加OpenCV

message(STATUS "OpenCV4_INCLUDE_DIR: " ${OpenCV4_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OpenCV4_INCLUDE_DIR})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} debug ${OpenCV4_DEBUG_LIB} optimized ${OpenCV4_RELEASE_LIB})

# 添加pcl
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
FOREACH(_var ${PCL_LIBRARIES})
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} debug ${PCL_LIB_DIR}/${_var}debug.lib 
		optimized ${PCL_LIB_DIR}/${_var}release.lib)
ENDFOREACH()

#INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})


