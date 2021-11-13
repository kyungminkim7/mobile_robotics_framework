#include <cmath>
#include <cstdlib>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <mobile_robotics_framework/BerryImu.h>
#include <network/Node.h>
#include <network/Rate.h>

const unsigned short IMG_PORT = 50000;

int main(int argc, char *argv[]) {
    constexpr float RAD_TO_DEG = 180 / M_PI;

    mrf::BerryImu imu(1);

    ntwk::Rate rate(5);
    while(true) {
        auto linearAccel = imu.getLinearAcceleration();
        auto angularVel = imu.getAngularVelocity() * RAD_TO_DEG;

        std::cout << "Linear accel:\n" << linearAccel << "\n\n";
        std::cout << "Angular vel:\n" << angularVel << "\n\n";

        rate.sleep();
    }


/**
    // Initialize ntwk publishers/subscribers
    ntwk::Node node;

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
        }

        node.runOnce();
    }

**/

    return EXIT_SUCCESS;
}
