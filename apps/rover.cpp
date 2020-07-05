#include <iostream>

#include <network/Node.h>
#include <network/Rate.h>
#include <network/TcpPublisher.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

const unsigned short IMG_PORT = 50000;

int main(int argc, char *argv[]) {
    // Initialize ntwk publishers/subscribers
    ntwk::Node node;
    auto imgPublisher = node.advertise(IMG_PORT, ntwk::Compression::JPEG);

    // Initialize camera
    cv::VideoCapture cam(0);
    if (!cam.isOpened()) {
        std::cerr << "Failed to open camera\n";
        return 1;
    }

    while(true) {
        cv::Mat img;
        if (cam.read(img)) {
            cv::resize(img, img, cv::Size(), 0.5f, 0.5f);
            cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
            cv::flip(img, img, 0);
            imgPublisher->publishImage(img.cols, img.rows, 3, img.data);
        }

        node.runOnce();
    }

    return 0;
}
