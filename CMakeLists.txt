SET(execName pairwise_incremental)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(WRITE_PCD)
find_package(CUDA 7.0 REQUIRED)
find_package(Boost REQUIRED)
find_package(ZED REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS common io visualization filters)
find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})



add_executable(pairwise_incremental pairwise_incremental.cpp)
add_definitions(-std=c++0x) # -m64)
target_link_libraries(${execName}
			${PCL_LIBRARIES} 
                        ${ZED_LIBRARIES} 
			 ${PCL_COMMON_LIBRARIES} 
			${OpenCV_LIBRARIES} 
			${Boost_LIBRARIES}
			 ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARY} 
			${PCL_VISUALIZATION_LIBRARIES} 
${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})



