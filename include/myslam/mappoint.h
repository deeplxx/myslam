/*
	表示路标点
*/

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {

/* 路标点 */
class MapPoint {
  public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;
    Eigen::Vector3d pos_;  // positon in world
    Eigen::Vector3d norm_;  // 标准观察方向向量
    cv::Mat descriptor_;  // 对应的描述子(行向量)
    static unsigned long factory_id_;  // 因为有两个构造函数都需要用到所以定义为成员变量

    // 评价好坏程度的指标
    int observed_times_;  // 被观察到的次数
    int matched_times_;  // 被匹配的次数（某个位姿估计中的内点）
    bool good_;
    vector<Frame::Ptr> observed_frames_;  // 能观察到此点的frame

  public:
    MapPoint();
    MapPoint(
        unsigned long id,
        const Eigen::Vector3d& pos,
        const Eigen::Vector3d& norm,
        Frame::Ptr frame=nullptr,
        const cv::Mat& descriptor=cv::Mat());

    inline cv::Point3d getPositionCV() const {
        return cv::Point3d(pos_(0, 0), pos_(1, 0), pos_(2, 0));
    }

    // factory function
    static MapPoint::Ptr createMapPoint();
    static MapPoint::Ptr createMapPoint(
        const Eigen::Vector3d& pos_world,
        const Eigen::Vector3d& norm,
        const cv::Mat& descriptor,
        Frame::Ptr frame);
};
}

#endif
