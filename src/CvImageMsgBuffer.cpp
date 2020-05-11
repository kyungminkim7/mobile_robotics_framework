#include <mobile_robotics_framework/CvImageMsgBuffer.h>

#include <sensor_msgs/Image_generated.h>

namespace mrf {

std::shared_ptr<flatbuffers::DetachedBuffer> convertCvImageToMsgBuffer(const cv::Mat &img) {
    sensor_msgs::ImageEncoding imgEncoding;
    switch (img.type()) {
    case CV_8UC1:
        imgEncoding = sensor_msgs::ImageEncoding::MONO8;
        break;

    case CV_8UC4:
        imgEncoding = sensor_msgs::ImageEncoding::RGBA8;
        break;

    default:
        imgEncoding = sensor_msgs::ImageEncoding::RGB8;
        break;
    }

    auto size = img.total() * img.elemSize();

    flatbuffers::FlatBufferBuilder imgMsgBuilder(img.total() * img.elemSize() + 100);
    auto imgDataBuffer = imgMsgBuilder.CreateVector(img.data, img.total() * img.channels());
    auto imgMsgBuffer = sensor_msgs::CreateImage(imgMsgBuilder, img.cols, img.rows,
                                                 imgEncoding, imgDataBuffer);
    imgMsgBuilder.Finish(imgMsgBuffer);

    return std::make_shared<flatbuffers::DetachedBuffer>(imgMsgBuilder.Release());
}

} // namespace mrf
