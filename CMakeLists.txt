# use shell : cmake --version
cmake_minimum_required(VERSION 3.22.1)
# your project name
project(main)

include(CTest)
enable_testing()

# use opencv package
find_package(OpenCV 4.7 REQUIRED)

# 查找 NLopt 库
find_package(NLopt REQUIRED)
include_directories(${NLOPT_INCLUDE_DIRS})

# include your headfile
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)
set(INCLUDE_DIR ${CMAKE_SOURCE_DIR}/include)
# package your cppfile
file(GLOB SOURCES_DIR "src/*.cpp")

# include your all file
add_executable(main main.cpp ${SOURCES_DIR})

# 链接 NLopt 库
target_link_libraries(main ${NLOPT_LIBRARIES})
target_link_libraries(main ${OpenCV_LIBS})
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

