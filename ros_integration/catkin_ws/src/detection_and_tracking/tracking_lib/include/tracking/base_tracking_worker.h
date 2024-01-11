#ifndef TRACKING_BASE_TRACKING_WORKDER_H_
#define TRACKING_BASE_TRACKING_WORKDER_H_

#include <vector>

#include "common_lib/types/object.h"

namespace tracking {
class BaseTrackingWorker {
 public:
  BaseTrackingWorker() {}

  virtual ~BaseTrackingWorker() {}

  // @brief: tracking objects.
  // @param[in] objects_obsved: timestamp.
  // @param[in] options: options.
  // @param[out] objects_tracked: current tracking objects.
  virtual bool track(const std::vector<ObjectPtr>& objects_obsved, double timestamp, const TrackingOptions& options,
                     std::vector<ObjectPtr>* objects_tracked) = 0;
};
}  // namespace tracking

#endif