#include <mobile_robotics_framework/CvImageMsgBuffer.h>

namespace mrf {

std::shared_ptr<flatbuffers::DetachedBuffer> convertCvImageToMsgBuffer(cv::Mat img) {
    flatbuffers::FlatBufferBuilder imgMsgBuilder(img.total() * img.elemSize() + 50);
    auto imgDataBuffer = imgMsgBuilder.CreateVector(img.data, img.total() * img.channels());
    auto imgMsgBuffer = sensor_msgs::CreateImage(imgMsgBuilder, img.cols, img.rows,
                                                 , imgDataBuffer);
    imgMsgBuilder.Finish(imgMsgBuffer);
    return std::make_shared<flatbuffers::DetachedBuffer>(imgMsgBuilder.Release());
}

} // namespace mrf
