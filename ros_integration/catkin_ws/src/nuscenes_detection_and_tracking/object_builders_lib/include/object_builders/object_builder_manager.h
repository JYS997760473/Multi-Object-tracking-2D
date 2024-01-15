#pragma once

#include "object_builders/base_object_builder.h"
#include "object_builders/min_box_object_builder.h"

static std::unique_ptr<BaseObjectBuilder> createObjectBuilder(const double baselink_fakebaselink_length_along_x_axis) {
  std::unique_ptr<BaseObjectBuilder> builder =
      std::unique_ptr<BaseObjectBuilder>(new MinBoxObjectBuilder(baselink_fakebaselink_length_along_x_axis));
  return builder;
}
