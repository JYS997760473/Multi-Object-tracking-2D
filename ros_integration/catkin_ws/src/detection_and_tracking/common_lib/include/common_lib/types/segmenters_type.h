#pragma once
#include <cstdint>

namespace segmenter {

enum class SegmentType : uint8_t {
  RegionEuclideanSegmenter = 0,
  EuclideanSegmenter = 1,
  RegionGrowingSegmenter = 2,
  DoNSegmenter = 3,
  NumOfSegmenters = 4,
};
}