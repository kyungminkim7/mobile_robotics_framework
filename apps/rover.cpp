#include <iostream>

#include <network/Node.h>
#include <network/TcpPublisher.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <mobile_robotics_framework/CvImageMsgBuffer.h>

const unsigned short IMG_PORT = 50000;

int main(int argc, char *argv[]) {
    // Initialize ntwk publishers/subscribers
    ntwk::Node node;
    auto imgPublisher = node.advertise(IMG_PORT, 1);

    // Initialize camera
    cv::VideoCapture cam(0);
    if (!cam.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }

    while(true) {
        cv::Mat img;
        if (cam.read(img)) {
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
            cv::flip(img, img, 0);
            imgPublisher->publish(mrf::convertCvImageToMsgBuffer(img));
        }

        node.update();
    }

    return 0;
}
