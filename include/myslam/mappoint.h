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
	cv::Vector3d pos_;
	cv::Vector3d norm_;  // 标准观察方向向量
	cv::Mat descriptor_;  // 对应的描述子

	// 评价好坏程度的指标
	int observed_times_;  // 被观察到的次数
	int correct_times_;  // 被匹配的次数

  public:
	MapPoint();
	MapPoint(unsigned long id, cv::Vector3d pos, cv::Vector3d norm);

	// factory function
	MapPoint::Ptr createMapPoint();
};
}

#endif