/*
	存储相机的内外参
	完成相机坐标，像素坐标，世界坐标的转换
*/

#ifndef CAMERA_H
#define CAMERA_H

#include "myslam/common_include.h"

namespace myslam {

class Camera {
  public:
	typedef std::shared_ptr<Camera> Ptr;
	float fx_, fy_, cx_, cy_, depth_scale_;  // 相机参数
  public:
	Camera();
	Camera(float fx, float fy, float cx, float cy, float depth_scale):
		fx_(fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale) {}

	// 坐标变换
	cv::Vector3d world2camera(const cv::Vector3d & p_w, const Sophus::SE3 & T_c_w);
	cv::Vector3d camera2world(const cv::Vector3d & p_c, const Sophus::SE3 & T_c_w);
	cv::Vector2d camera2pixel(const cv::Vector3d & p_c);
	cv::Vector3d pixel2camera(const Vector2d & p_p, double depth = 1);
	cv::Vector3d pixel2world(const Vector2d & p_p, const Sophus::SE3 & T_c_w, double depth = 1);
	cv::Vector2d world2pixel(cosnt Vector3d & p_w, const Sophus::SE3 & T_c_w);
};
}

#endif