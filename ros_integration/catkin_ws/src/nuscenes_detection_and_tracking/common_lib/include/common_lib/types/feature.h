#pragma once
#ifndef COMMON_LIB_TYPES_FEATURE_H_
#define COMMON_LIB_TYPES_FEATURE_H_
#include <string>
#include <vector>

#include <ros/ros.h>

typedef double FeatureElementType;
typedef double Label;

struct FeatureElement {
  FeatureElement(std::string feature_name, FeatureElementType feature_value)
      : name(feature_name)
      , value(feature_value) {}

  std::string name = "";
  FeatureElementType value = 0.0;
};

/// @brief a feature can be composed of any number of values, each with its own
class Feature {
 public:
  Feature() {}

  ~Feature() {}

  explicit Feature(const FeatureElement& element) { pushBack(element); }

  explicit Feature(const std::string& name, const FeatureElementType& value) { pushBack(FeatureElement(name, value)); }

  void pushBack(const FeatureElement& element) {
    if (findValueByName(element.name, NULL)) {
      ROS_WARN_STREAM(
          "Adding several FeatureValues of same name to Feature is not "
          "recommended.");
    }
    feature_elements_.push_back(element);
  }

  bool setValueById(unsigned int dimension_id, const FeatureElementType& value) {
    feature_elements_.at(dimension_id).value = value;
  }

  size_t size() const { return feature_elements_.size(); }

  bool empty() { return feature_elements_.empty(); }

  const FeatureElement& at(const size_t& index) const { return feature_elements_.at(index); }

  void clear() { feature_elements_.clear(); }

 protected:
  bool findValueByName(const std::string& name, FeatureElement* value) const {
    for (size_t i = 0u; i < feature_elements_.size(); i++) {
      if (feature_elements_.at(i).name == name) {
        if (value != NULL) {
          *value = feature_elements_.at(i);
        }
        return true;
      }
    }
    return false;
  }

 private:
  std::vector<FeatureElement> feature_elements_;
};
#endif