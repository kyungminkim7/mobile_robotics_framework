#include <iostream>

#include <network/Node.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

const unsigned short IMG_PORT = 10000;

int main(int argc, char *argv[]) {
    // Initialize ntwk publishers/subscribers
    ntwk::Node node;
    auto imgPublisher = node.advertise(IMG_PORT);

    // Initialize camera
    cv::VideoCapture cam(2);
    if (!cam.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }

    while(true) {
        cv::Mat img;
        if (cam.read(img)) {

        }

        node.runOnce();
    }

    return 0;
}
