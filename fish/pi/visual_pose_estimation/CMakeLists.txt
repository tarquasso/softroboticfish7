cmake_minimum_required (VERSION 2.8) 
project (visual_pose_estimation)

find_package(OpenCV 3 REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable (line_detection line_detection.cpp)  
target_link_libraries (line_detection ${OpenCV_LIBS})

add_executable (line_detection_histogram line_detection_histogram.cpp)  
target_link_libraries (line_detection_histogram ${OpenCV_LIBS})

add_executable (threshold_tester threshold_tester.cpp)  
target_link_libraries (threshold_tester ${OpenCV_LIBS})