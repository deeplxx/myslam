#include "myslam/frame.h"

namespace myslam {

Frame::Frame(): id_(-1), time_stamp_(-1), camera_(nullptr) {

}

Frame::Frame(unsigned long id, double time_stamp, Sophus::SE3 T_c_w,
             Camera::Ptr camera, cv::Mat color, cv::Mat depth):
    id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color), depth_(depth) {

}

Frame::~Frame() {

}

Frame::Ptr Frame::createFrame() {
    static unsigned long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++, 0));
}

double Frame::findDepth(const cv::KeyPoint &kp) {
    int x = cvRound(kp.pt.x);  // 四舍五入...
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if (d != 0) {
        return double(d) / camera_->depth_scale_;
    } else {  // 插值！！！！
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; i++) {
            d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0) {
                return double(d) / camera_->depth_scale_;
            }
        }
    }

    return -1;
}

Eigen::Vector3d Frame::getCamCenter() const {
    return T_c_w_.inverse().translation();  // ????????????????????????????
}

bool Frame::isInFrame(const Eigen::Vector3d &pt_w) {
    Eigen::Vector3d pt_cam = camera_->world2camera(pt_w, T_c_w_);
    if (pt_cam(2, 0) < 0) {
        return false;
    }

    Eigen::Vector2d pixel = camera_->world2pixel(pt_w, T_c_w_);
    return pixel(0, 0) > 0 &&
           pixel(1, 0) > 0 &&
           pixel(0, 0) < color_.cols &&
           pixel(1, 0) < color_.rows;
}
}
