/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_MATH_H_
#define CARTOGRAPHER_COMMON_MATH_H_

#include <cmath>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "ceres/ceres.h"
/**
 * @brief common/math.h文件主要实现数学计算，包括：
    区间截断.求n次方.求平方.幅度角度转换.归一化.反正切值
 */
namespace cartographer {
namespace common {

// Clamps 'value' to be in the range ['min', 'max'].
/**
 * @brief 将“值”限制在['min'，'max']范围内。
 * @param[in] value 
 * @param[in] min 
 * @param[in] max 
 * @return T 
 */
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
/**
 * @brief 计算base的exp次方
 * @param[in] base 
 * @param[in] exponent 
 * @return constexpr T 
 */
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
/**
 * @brief 计算a的二次方
 * @param[in] a 
 * @return constexpr T 
 */
template <typename T>
constexpr T Pow2(T a) {
  return Power(a, 2);
}

// Converts from degrees to radians.
/**
 * @brief 角度转弧度
 * @param[in] deg 
 * @return constexpr double 
 */
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
/**
 * @brief 弧度转角度
 * @param[in] rad 
 * @return constexpr double 
 */
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
/**
 * @brief 将角度值限制在[-pi，pi]之间
 * @param[in] difference 
 * @return T 
 */
template <typename T>
T NormalizeAngleDifference(T difference) {
  const T kPi = T(M_PI);
  while (difference > kPi) difference -= 2. * kPi;
  while (difference < -kPi) difference += 2. * kPi;
  return difference;
}
/**
 * @brief atan2 返回原点至点(x,y)的方位角，即与 x 轴的夹角，
          也可以理解为计算复数 x+yi 的辐角,范围是[-pi,pi]
          ATAN2(1,1) -> pi/4:以弧度表示点(1,1)的反正切值，即pi/4=0.785398
 * @param[in] vector 
 * @return T 
 */
template <typename T>
//矩阵模板，创建一个2*1的变量
T atan2(const Eigen::Matrix<T, 2, 1>& vector) {
  return ceres::atan2(vector.y(), vector.x());
}
/**
 * @brief 增加一个内联函数，提高编译效率
 * @param[in] z 
 * @param[in] w 
 * @param[in] zw 
 */
template <typename T>
//  zw[0]  =                           | w[0] -w[1] -w[2]  -w[3]|
//  zw[1]  =                           | w[1]  w[0]  w[3]  -w[2]|
//  zw[2]  = | z[0] z[1] z[2] z[3] | * | w[2] -w[3]  w[0]   w[1]|
//  zw[3]  =                           | w[3]  w[2] -w[1]   w[0]|
inline void QuaternionProduct(const double* const z, const T* const w,
                              T* const zw) {
  zw[0] = z[0] * w[0] - z[1] * w[1] - z[2] * w[2] - z[3] * w[3];
  zw[1] = z[0] * w[1] + z[1] * w[0] + z[2] * w[3] - z[3] * w[2];
  zw[2] = z[0] * w[2] - z[1] * w[3] + z[2] * w[0] + z[3] * w[1];
  zw[3] = z[0] * w[3] + z[1] * w[2] - z[2] * w[1] + z[3] * w[0];
}

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_MATH_H_
