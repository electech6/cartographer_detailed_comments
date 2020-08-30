#include "cartographer/mapping/internal/scan_matching/real_time_correlative_scan_matcher.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {

proto::RealTimeCorrelativeScanMatcherOptions
CreateRealTimeCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* const parameter_dictionary) {
  proto::RealTimeCorrelativeScanMatcherOptions options;
  // 线性搜索窗口
  options.set_linear_search_window(
      parameter_dictionary->GetDouble("linear_search_window"));
  // 角度搜索窗口
  options.set_angular_search_window(
      parameter_dictionary->GetDouble("angular_search_window"));
  //求解得分的两个权重
  options.set_translation_delta_cost_weight(
      parameter_dictionary->GetDouble("translation_delta_cost_weight"));
  options.set_rotation_delta_cost_weight(
      parameter_dictionary->GetDouble("rotation_delta_cost_weight"));
  CHECK_GE(options.translation_delta_cost_weight(), 0.);
  CHECK_GE(options.rotation_delta_cost_weight(), 0.);
  return options;
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
