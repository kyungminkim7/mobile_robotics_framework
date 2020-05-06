#include <cstdint>
#include <memory>

#include <flatbuffers/flatbuffers.h>
#include <opencv2/core.hpp>

namespace mrf {

std::shared_ptr<flatbuffers::DetachedBuffer> convertCvImageToMsgBuffer(const cv::Mat &img);

} // namespace mrf
