find_package(OpenCV REQUIRED COMPONENTS
    core
    imgcodecs
    imgproc
    videoio
)

add_executable(rover "rover.cpp")
target_link_libraries(rover PRIVATE
    mobile_robotics_framework::mobile_robotics_framework
    opencv_core
    opencv_imgcodecs
    opencv_imgproc
    opencv_videoio
)
