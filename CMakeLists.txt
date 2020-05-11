cmake_minimum_required(VERSION 3.5...3.10)
project(mobile_robotics_framework VERSION 1.0.0)

set(package_name mobile_robotics_framework)

find_package(OpenCV REQUIRED COMPONENTS
    core
    imgcodecs
    imgproc
    videoio
)

add_subdirectory(extern)

add_library(${PROJECT_NAME}
    "src/CvImageMsgBuffer.cpp"
)

add_library(${package_name}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    network::network
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_videoio
)

target_compile_features(${PROJECT_NAME}
    PRIVATE
        cxx_auto_type
        cxx_generic_lambdas
        cxx_lambdas
        cxx_lambda_init_captures
        cxx_range_for
)

add_subdirectory(apps)
