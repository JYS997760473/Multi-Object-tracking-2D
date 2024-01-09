
namespace common {
namespace geometry {
/**
 * @brief compute velocity's angle change between "v1" and "v2"
 * @note
 *  v1·v2 = |v1|*|v2|cos(theta)
 *  v1xv2 = |v1|*|v2|sin(theta)
 * @param v1
 * @param v2
 * @return
 */
template <typename VectorT>
static double computeTheta2dXyBetweenVectors(const VectorT& v1, const VectorT& v2) {
  double v1_len = sqrt((v1.head(2).cwiseProduct(v1.head(2))).sum());
  double v2_len = sqrt((v2.head(2).cwiseProduct(v2.head(2))).sum());

  // cos(theta) = (v1·v2)/(|v1|*|v2|) = [v1(0)*v2(0)+v1(1)*v2(1)]/(|v1|*|v2|)
  double cos_theta = (v1.head(2).cwiseProduct(v2.head(2))).sum() / (v1_len * v2_len);
  // limit theta to [0, PI] or [-PI, 0]
  if (cos_theta > 1) {
    cos_theta = 1;
  }
  if (cos_theta < -1) {
    cos_theta = -1;
  }

  // sin(theta) = (v1xv2)/(|v1|*|v2|) = [v1(0)*v2(1)-v1(1)*v2(0)]/(|v1|*|v2|)
  double sin_theta = (v1(0) * v2(1) - v1(1) * v2(0)) / (v1_len * v2_len);
  // return [0, PI]
  double theta = acos(cos_theta);
  if (sin_theta < 0) {
    theta = -theta;
  }

  return theta;
}
}  // namespace geometry
}  // namespace common
