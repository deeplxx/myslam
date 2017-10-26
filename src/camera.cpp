#include <myslam/camera.h>

namespace myslam {
Camera::Camera() {}

cv::Vector3d Camera::world2camera(const cv::Vector3d& p_w, const Sophus::SE3& T_c_w) {
	return T_c_w * p_w;
}

cv::Vector3d Camera::camera2world(const cv::Vector3d& p_c, const Sophus::SE3& T_c_w) {
	return T_c_w.inverse() * p_c;
}

cv::Vector2d Camera::camera2pixel(const cv::Vector3d& p_c) {
	return cv::Vector2d(
	           fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
	           fy_ * p_c(1, 0) / p_c(2, 0) + cy_
	       );
}

cv::Vector3d Camera::pixel2camera(const cv::Vector2d& p_p, double depth = 1) {
	return cv::Vector3d(
	           (p_p(0, 0) - cx_) * depth / fx_,
	           (p_p(1, 0) - cy_) * depth / fy_,
	           depth
	       );
}

cv::Vector3d Camera::pixel2world(const cv::Vector2d& p_p, cosnt Sophus::SE3& T_c_w, double depth = 1) {
	return camera2world(pixel2camera(p_p, depth), T_c_w);
}

cv::Vector2d Camera::world2pixel(const cv::Vector3d& p_w, const Sophus::SE3& T_c_w) {
	return camera2pixel(world2camera(p_w, T_c_w));
}
}