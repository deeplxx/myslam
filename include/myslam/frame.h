/*
	基本数据单元
*/

#ifndef FRAME_H
#define FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam {

class Frame {
  public:
    typedef shared_ptr<Frame> Ptr;
    unsigned long id_; // frame id
    double time_stamp_;  // frame创建时间
    Sophus::SE3 T_c_w_; // 相机位姿
    Camera::Ptr camera_;  // shared_ptr
    cv::Mat color_, depth_;  // 彩色图与深度图

  public:
    Frame();
    Frame(unsigned long id, double time_stamp,
          Sophus::SE3 T_c_w = Sophus::SE3(), Camera::Ptr camera = nullptr,
          cv::Mat color = cv::Mat(), cv::Mat depth = cv::Mat());
    ~Frame();

    // factory function
    static Frame::Ptr createFrame();

    // 查找给定点对应的深度
    double findDepth(const cv::KeyPoint& kp);

    // 获取相机光心
    Eigen::Vector3d getCamCenter() const;

    // 判断给定点是否在视野内，传入参数是世界坐标
    bool isInFrame(const Eigen::Vector3d& pt_w);
};
}

#endif
