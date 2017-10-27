/*
    两帧间的VO
*/
#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"
#include "myslam/camera.h"
#include "myslam/config.h"


#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

namespace myslam {

class VisualOdometry {
  public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState {  // VO状态：初始化，正常，丢失
        INITIALZING = -1,
        OK = 0,
        LOST = 1
    };

    VOState state_;
    Map::Ptr map_;
    Frame::Ptr ref_frame_;
    Frame::Ptr cur_frame_;

    cv::Ptr<cv::ORB> orb_;  // orb检测和计算器
    vector<cv::Point3d> pts_3d_ref_;  // 参考帧中的3d点
    vector<cv::KeyPoint> kp_cur_;  // 当前帧中的关键点
    cv::Mat descriptor_ref_;
    cv::Mat descriptor_cur_;
    vector<cv::DMatch> feature_matches_;
    Sophus::SE3 T_cur_estimated_;  // 当前帧的位姿估计
    int num_inliers_;  // 在阈值范围内的特征数（内点）
    int num_lost_;  // 丢失次数

    // 参数文件内参数
    int num_features_; // 特征数
    double scale_factor_;  // 图像金字塔尺度
    int level_pyramid_;  // 级数
    float match_ratio_;  // 选择good match的比率
    int max_num_lost_;  // 最大连续丢失次数
    int min_inliers_;  // 最小内点数
    double key_frame_min_rot;  // 两帧间最小旋转量
    double key_frame_min_trans;  // 两帧间最小偏移量

  public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);

  protected:
    // inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatch();
    void poseEstimationPnP();
    void setRef3DPoints();  // 初始化时用
    void addKeyFrame();
    bool checkEstimatedPose();  // 运动变化没有特别大
    bool checkKeyFrame();  // 有足够的偏移或旋转
};
}

#endif
