// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_HEADER_STD_MSGS_H_
#define FLATBUFFERS_GENERATED_HEADER_STD_MSGS_H_

#include "flatbuffers/flatbuffers.h"

namespace std_msgs {

struct Header;

FLATBUFFERS_MANUALLY_ALIGNED_STRUCT(4) Header FLATBUFFERS_FINAL_CLASS {
 private:
  uint32_t msgSize_;

 public:
  Header() {
    memset(static_cast<void *>(this), 0, sizeof(Header));
  }
  Header(uint32_t _msgSize)
      : msgSize_(flatbuffers::EndianScalar(_msgSize)) {
  }
  uint32_t msgSize() const {
    return flatbuffers::EndianScalar(msgSize_);
  }
};
FLATBUFFERS_STRUCT_END(Header, 4);

}  // namespace std_msgs

#endif  // FLATBUFFERS_GENERATED_HEADER_STD_MSGS_H_
