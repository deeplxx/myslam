/*
	表示路标点
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

/* 路标点 */
class MapPoint {
  public:
	typedef shared_ptr<MapPoint> Ptr;
	unsigned long id_;
	Eigen::Vector3d pos_;
	Eigen::Vector3d norm_;  // 标准观察方向向量
	cv::Mat descriptor_;  // 对应的描述子

	// 评价好坏程度的指标
	int observed_times_;  // 被观察到的次数
	int correct_times_;  // 被匹配的次数

  public:
	MapPoint();
	MapPoint(unsigned long id, Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0),
	         Eigen::Vector3d norm = Eigen::Vector3d(0, 0, 0));

	// factory function
	MapPoint::Ptr createMapPoint();
};
}

#endif