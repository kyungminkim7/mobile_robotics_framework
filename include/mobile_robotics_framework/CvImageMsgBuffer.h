#include <cstdint>
#include <memory>

#include <flatbuffers/flatbuffers.h>
#include <sensor_msgs/Image_generated.h>
#include <opencv2/core.hpp>

namespace mrf {

std::shared_ptr<flatbuffers::DetachedBuffer> convertCvImageToMsgBuffer(cv::Mat img, sensor_msgs::Encoding);

} // namespace mrf
