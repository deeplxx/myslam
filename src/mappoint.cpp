#include "myslam/mappoint.h"

namespace myslam {

MapPoint::MapPoint(): id_(-1), pos_(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)),
    observed_times_(0), matched_times_(0), good_(true) {

}

MapPoint::MapPoint(
    unsigned long id,
    const Eigen::Vector3d &pos,
    const Eigen::Vector3d &norm,
    Frame::Ptr frame,
    const cv::Mat &descriptor):
    id_(id), pos_(pos), norm_(norm), observed_times_(1), matched_times_(1), descriptor_(descriptor) {
    observed_frames_.push_back(frame);
}

MapPoint::Ptr MapPoint::createMapPoint() {  //用一个函数来创建对象，可以对每次创建新的对象都有所不同
    return MapPoint::Ptr(new MapPoint(factory_id_++, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0)));
}

MapPoint::Ptr MapPoint::createMapPoint(
    const Eigen::Vector3d& pos_world,
    const Eigen::Vector3d& norm,
    const cv::Mat& descriptor,
    Frame::Ptr frame) {
    return MapPoint::Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
}

unsigned long MapPoint::factory_id_ = 0;
}
