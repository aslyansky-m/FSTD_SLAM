cmake_minimum_required(VERSION 3.13)
project(cone_matching)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package( OpenCV REQUIRED )

# darknet
include_directories(darknet/include)
link_directories(${CMAKE_SOURCE_DIR}/darknet/lib)
file(GLOB DARKNET_SRC
        darknet/include/*.h
        darknet/include/*.hpp
        darknet/3rdparty/*.h)

# orbslam
file(GLOB ORBSLAM_SRC
        orbslam2/include/*.h
        orbslam2/src/*.cc)


# compile definitions
add_compile_definitions(OPENCV)
#add_compile_definitions(TRACK_OPTFLOW)

# source
file(GLOB SOURCE
        include/*.h
        src/*.cc)

include_directories(include)

add_executable(cone_matching
        src/yolo_detection.cpp
        ${DARKNET_SRC}
        ${SOURCE} include/Common.h)

target_link_libraries(cone_matching libdarknet.so  ${OpenCV_LIBS} )

add_executable(cone_orbslam_tracking
        src/yolo_orbslam_tracking.cpp
        ${DARKNET_SRC}
        ${SOURCE} include/Common.h include/KLTtracker.h)

target_link_libraries(cone_orbslam_tracking libdarknet.so  ${OpenCV_LIBS} )

add_executable(cone_klt_tracking
        src/yolo_klt_tracking.cpp
        ${DARKNET_SRC}
        include/Common.h include/KLTtracker.h include/Detector.h)

target_link_libraries(cone_klt_tracking libdarknet.so  ${OpenCV_LIBS} )


add_executable(cone_multi_tracking
        src/yolo_multi_tracker.cpp
        ${DARKNET_SRC}
        include/Common.h include/KLTtracker.h include/Detector.h)

target_link_libraries(cone_multi_tracking libdarknet.so  ${OpenCV_LIBS} )


add_executable(cone_sift_matcher
        src/yolo_sift_matcher.cpp
        ${DARKNET_SRC}
        include/Common.h include/KLTtracker.h include/Detector.h)

target_link_libraries(cone_sift_matcher libdarknet.so  ${OpenCV_LIBS} )

#add_executable(yolo_console_dll
#        ${DARKNET_SRC} src/ORBConeExtractor.h)
# target_link_libraries(yolo_console_dll libdarknet.so  ${OpenCV_LIBS} )



