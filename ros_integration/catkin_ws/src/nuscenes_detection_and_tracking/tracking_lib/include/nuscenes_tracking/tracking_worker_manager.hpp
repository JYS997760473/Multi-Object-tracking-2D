#ifndef TRACKING_TRACKING_WORKER_MANAGER_HPP_
#define TRACKING_TRACKING_WORKER_MANAGER_HPP_

#include "common_lib/types/tracking_options.h"
#include "common_lib/types/type.h"

namespace tracking {
static std::unique_ptr<BaseTrackingWorker> createTrackingWorker(const TrackingWorkerParam& params) {
  std::unique_ptr<BaseTrackingWorker> tracking_worker;
  tracking_worker = std::unique_ptr<BaseTrackingWorker>(new HmTrackingWorker(params));

  return tracking_worker;
}
}  // namespace tracking
#endif