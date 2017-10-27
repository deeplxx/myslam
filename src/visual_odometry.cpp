#include "myslam/visual_odometry.h"

namespace myslam {

VisualOdometry::VisualOdometry(): state_(INITIALZING), ref_frame_(nullptr), cur_frame_(nullptr),
    map_(new Map), num_lost_(0), num_inliers_(0) {
    num_features_ = Config::get<int>("num_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<int>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_transfer");

    orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry() {

}

bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
    case INITIALZING:  // 初始化
        state_ = OK;
        cur_frame_ = frame;
        map_->insertKeyFrame(frame);  // 插入当前帧作为关键帧
        extractKeyPoints();  // 提取关键点
        computeDescriptors();  // 计算描述子
        setRef3DPoints();  // 计算参考帧3d坐标
        break;
    case OK:  // 正常运行中
        cur_frame_ = frame;
        extractKeyPoints();  // 提取关键点
        computeDescriptors();  // 计算描述子
        featureMatch();  // 特征匹配
        poseEstimationPnP();  // 位姿估计
        if (checkEstimatedPose() == true) {  // 一次好的估计
            cur_frame_->T_c_w_ = T_cur_estimated_ * ref_frame_->T_c_w_;  // 计算当前绝对位姿
            ref_frame_ = cur_frame_;
            setRef3DPoints();
            num_lost_ = 0;  // 重置丢失次数
            if (checkKeyFrame() == true) {
                addKeyFrame();
            }
        } else {  // 不是好的估计
            num_lost_++;
            if (num_lost_ > max_num_lost_) {
                state_ = LOST;
                return false;
            }
        }
        break;
    case LOST:
        cout << "VO has LOST..." << endl;
        break;
    }

    return true;
}

void VisualOdometry::extractKeyPoints() {
    orb_->detect(cur_frame_->color_, kp_cur_);
}

void VisualOdometry::computeDescriptors() {
    orb_->compute(cur_frame_->color_, kp_cur_, descriptor_cur_);
}

void VisualOdometry::featureMatch() {
    // 找到所有匹配
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    matcher.match(descriptor_ref_, descriptor_cur_, matches);

    // 取最佳匹配
    float min_dis = min_element(matches.begin(), matches.end(),  // 令人窒息的操作...
    [](const cv::DMatch& m1, const cv::DMatch& m2) {
        return m1.distance < m2.distance;
    })->distance;
    feature_matches_.clear(); // 先清除先前的数据
    for (cv::DMatch m: matches) {
        if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
            feature_matches_.push_back(m);
        }
    }
    cout << "good matches size: " << feature_matches_.size() << endl;
}

void VisualOdometry::setRef3DPoints() {  // 只有初始化时用
    pts_3d_ref_.clear();
    descriptor_ref_ = cv::Mat();
    for (auto i = 0; i < kp_cur_.size(); i++) {
        double d = ref_frame_->findDepth(kp_cur_[i]);
        if (d > 0) {
            Eigen::Vector3d pt_cam = ref_frame_->camera_->pixel2camera(
                                         Eigen::Vector2d(kp_cur_[i].pt.x, kp_cur_[i].pt.y), d);
            pts_3d_ref_.push_back(cv::Point3d(pt_cam(0, 0), pt_cam(1, 0), pt_cam(2, 0)));  // 获得参考帧的3d坐标
            descriptor_ref_.push_back(descriptor_cur_.row(i));  // 参考帧的描述子
        }
    }
}

void VisualOdometry::poseEstimationPnP() {
    vector<cv::Point3d> pts_3d;
    vector<cv::Point2d> pts_2d;

    for (cv::DMatch m: feature_matches_) {
        pts_3d.push_back(pts_3d_ref_[m.queryIdx]);  // 参考帧的3d坐标（相机坐标）
        pts_2d.push_back(kp_cur_[m.trainIdx].pt);  // 当前帧的关键点像素坐标
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 ref_frame_->camera_->fx_, 0, ref_frame_->camera_->cx_,
                 0, ref_frame_->camera_->fy_, ref_frame_->camera_->cy_,
                 0, 0, 1);
    cv::Mat R, t, inliers;
    cv::solvePnPRansac(pts_3d, pts_2d, K, cv::Mat(), R, t, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_cur_estimated_ = Sophus::SE3(Sophus::SO3(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0)),
                                   Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));
}

bool VisualOdometry::checkEstimatedPose() {
    if (num_inliers_ < min_inliers_) {
        cout << "reject because inlier is too small: " << num_inliers_ << endl;
        return false;
    }

    Sophus::Vector6d d = T_cur_estimated_.log();
    if (d.norm() > 5.0) {
        cout << "reject because motion is too large: " << d.norm() << endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame() {
    Sophus::Vector6d d = T_cur_estimated_.log();
    Eigen::Vector3d trans = d.head(3);
    Eigen::Vector3d rot = d.tail(3);

    if ((rot.norm() > key_frame_min_rot) || (trans.norm() > key_frame_min_trans)) {
        return true;
    }
    return false;
}

void VisualOdometry::addKeyFrame() {
    cout << "add a new key frame: "<< cur_frame_->id_ << endl;
    map_->insertKeyFrame(cur_frame_);
}
}
