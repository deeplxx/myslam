#include "myslam/mappoint.h"

namespace myslam {

MapPoint::MapPoint(): id_(-1), pos_(Eigen::Vector3d(0, 0, 0)), norm_(Eigen::Vector3d(0, 0, 0)),
    observed_times_(0), correct_times_(0) {

}

MapPoint::MapPoint(unsigned long id, Eigen::Vector3d pos, Eigen::Vector3d norm):
    id_(id), pos_(pos), norm_(norm), observed_times_(0), correct_times_(0) {

}

MapPoint::Ptr MapPoint::createMapPoint() {  //用一个函数来创建对象，可以对每次创建新的对象都有所不同
    static unsigned long factory_id = 0;
    return MapPoint::Ptr(new MapPoint(factory_id++));
}
}
