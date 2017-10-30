#include "myslam/visual_odometry.h"
#include "myslam/g2o_types.h"

namespace myslam {

VisualOdometry::VisualOdometry():
    state_(INITIALZING),
    ref_frame_(nullptr),
    cur_frame_(nullptr),
    map_(new Map), num_lost_(0),
    num_inliers_(0),
    matcher_flann_(new cv::flann::LshIndexParams(5, 10, 2)) {
    num_features_ = Config::get<int>("num_features");
    scale_factor_ = Config::get<double>("scale_factor");
    level_pyramid_ = Config::get<int>("level_pyramid");
    match_ratio_ = Config::get<float>("match_ratio");
    max_num_lost_ = Config::get<int>("max_num_lost");
    min_inliers_ = Config::get<int>("min_inliers");
    key_frame_min_rot = Config::get<double>("keyframe_min_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_min_transfer");
    mappoint_erase_ratio = Config::get<double>("mappoint_erase_ratio");

    cout << endl << "*************loading config**********" << endl <<
         "num_features: " << num_features_ << endl <<
         "scale_factor: " << scale_factor_ << endl <<
         "level_pyramid:" << level_pyramid_ << endl <<
         "match_ratio:  " << match_ratio_ << endl <<
         "max_num_lost: " << max_num_lost_ << endl <<
         "min_inliers:  " << min_inliers_ << endl <<
         "keyframe_min_rotation: " << key_frame_min_rot << endl <<
         "keyframe_min_transfer: " << key_frame_min_trans << endl <<
         "mappoint_erase_ratio:  " << mappoint_erase_ratio << endl << endl;
    orb_ = cv::ORB::create(num_features_, scale_factor_, level_pyramid_);
}

VisualOdometry::~VisualOdometry() {

}

bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
    case INITIALZING:  // 初始化
        cout << "*******starting init*******" << endl;
        state_ = OK;
        cur_frame_ = frame;
        ref_frame_ = frame;
//        map_->insertKeyFrame(frame);  // 插入当前帧作为关键帧
        extractKeyPoints();  // 提取关键点
        computeDescriptors();  // 计算描述子
//        setRef3DPoints();  // 计算参考帧3d坐标
        addKeyFrame();
        cout << "*******init finish*******\n" << endl;
        break;
    case OK:  // 正常运行中
        cur_frame_ = frame;
        cur_frame_->T_c_w_ = ref_frame_->T_c_w_;  // 初始化一个位姿(为featurmathch函数用)
        extractKeyPoints();  // 提取关键点
        computeDescriptors();  // 计算描述子
        cout << "***1***" << endl;
        featureMatch();  // 特征匹配
        cout << "***2***" << endl;
        poseEstimationPnP();  // 位姿估计
        cout << "***3***" << endl;
        if (checkEstimatedPose() == true) {  // 一次好的估计
            cur_frame_->T_c_w_ = T_cur_estimated_;  // 计算当前绝对位姿
            optimizeMap();
//            ref_frame_ = cur_frame_;
//            setRef3DPoints();
            num_lost_ = 0;  // 重置丢失次数
            if (checkKeyFrame()) {
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
//    cout << "kp_cur size is : " << kp_cur_.size() << endl;
}

void VisualOdometry::computeDescriptors() {
    orb_->compute(cur_frame_->color_, kp_cur_, descriptor_cur_);
//    cout << "descriptor_cur_ size: " << descriptor_cur_.rows << endl;
}

void VisualOdometry::featureMatch() {
    // 找到所有匹配
    boost::timer timer;
    vector<cv::DMatch> matches;
//    cv::BFMatcher matcher(cv::NORM_HAMMING);
//    matcher.match(descriptor_ref_, descriptor_cur_, matches);
////    cout << "all matckes size: " << matches.size() << endl;

//    // 取最佳匹配
//    float min_dis = min_element(matches.begin(), matches.end(),  // 令人窒息的操作...
//    [](const cv::DMatch& m1, const cv::DMatch& m2) {
//        return m1.distance < m2.distance;
//    })->distance;
//    feature_matches_.clear(); // 先清除先前的数据
//    for (cv::DMatch m: matches) {
//        if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
//            feature_matches_.push_back(m);
//        }
//    }
////    cout << "good matches size: " << feature_matches_.size() << endl;
    vector<MapPoint::Ptr> candidate_mappoints;  // 在帧中的关键点
    cv::Mat descriptor_map;  // 候选点的描述子
    for (auto point: map_->map_points_) {
        MapPoint::Ptr p = point.second;
        if (cur_frame_->isInFrame(p->pos_)) {
            p->observed_times_++;
            candidate_mappoints.push_back(p);
            descriptor_map.push_back(p->descriptor_);
        }
    }

    matcher_flann_.match(descriptor_map, descriptor_cur_, matches);
    float min_dis = min_element(matches.begin(), matches.end(),
    [](const cv::DMatch& m1, const cv::DMatch& m2) {
        return m1.distance < m2.distance;
    })->distance;

    matched_3d_pts_.clear();
    matched_2d_kp_index_.clear();
    for (cv::DMatch& m: matches) {
        if (m.distance < max<float>(min_dis * match_ratio_, 30.0)) {
            matched_3d_pts_.push_back(candidate_mappoints[m.queryIdx]);
            matched_2d_kp_index_.push_back(m.trainIdx);
        }
    }
    cout << "good matches size: " << matched_3d_pts_.size() << endl;
    cout << "feature matching cost: " << timer.elapsed() << endl;
}

void VisualOdometry::setRef3DPoints() {  // 只在ref = cur之后用
    pts_3d_ref_.clear();
    descriptor_ref_ = cv::Mat();
    for (auto i = 0; i < kp_cur_.size(); i++) {
        double d = ref_frame_->findDepth(kp_cur_[i]);  // 只在ref = cur之后用
        if (d > 0) {
            Eigen::Vector3d pt_cam = ref_frame_->camera_->pixel2camera(
                                         Eigen::Vector2d(kp_cur_[i].pt.x, kp_cur_[i].pt.y), d);
            pts_3d_ref_.push_back(cv::Point3d(pt_cam(0, 0), pt_cam(1, 0), pt_cam(2, 0)));  // 获得参考帧的3d坐标
            descriptor_ref_.push_back(descriptor_cur_.row(i));  // 参考帧的描述子
        }
    }
}

void VisualOdometry::poseEstimationPnP() {
    vector<cv::Point3d> pts_3d;  // 参考帧中关键点的3d坐标（相机坐标）
    vector<cv::Point2d> pts_2d;  // 当前帧的关键点像素坐标

//    for (cv::DMatch m: feature_matches_) {
//        pts_3d.push_back(pts_3d_ref_[m.queryIdx]);
//        pts_2d.push_back(kp_cur_[m.trainIdx].pt);
//    }
    for (int idx: matched_2d_kp_index_) {
        pts_2d.push_back(kp_cur_[idx].pt);
    }
    for (MapPoint::Ptr point: matched_3d_pts_) {
        pts_3d.push_back(point->getPositionCV());
    }

    cv::Mat K = (cv::Mat_<double>(3, 3) <<
                 ref_frame_->camera_->fx_, 0, ref_frame_->camera_->cx_,
                 0, ref_frame_->camera_->fy_, ref_frame_->camera_->cy_,
                 0, 0, 1);
    cv::Mat R, t, inliers;
    cv::solvePnPRansac(pts_3d, pts_2d, K, cv::Mat(), R, t, false, 100, 4.0, 0.99, inliers);
    num_inliers_ = inliers.rows;
    for (int i = 0; i < num_inliers_; i++) {  // 内点对应的匹配次数+1
        int idx = inliers.at<int>(i, 0);
        matched_3d_pts_[idx]->matched_times_++;
    }
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_cur_estimated_ = Sophus::SE3(Sophus::SO3(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0)),
                                   Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0)));

    // 创建优化
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;  // TODO
    unique_ptr<Block::LinearSolverType> linearSolver(new g2o::LinearSolverDense<Block::PoseMatrixType>());
    unique_ptr<Block> solver_ptr(new Block(move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver_lb = new g2o::OptimizationAlgorithmLevenberg(move(solver_ptr));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver_lb);

    // **加入节点
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
                          T_cur_estimated_.rotation_matrix(), T_cur_estimated_.translation()));
    optimizer.addVertex(pose);

    // **加入边
    for (int i = 0; i < inliers.rows; i++) {
        int idx = inliers.at<int>(i, 0);  // 由此可以看出inliers矩阵每行第一个是id
        EdgeXYZ2UVPoseOnly* edge = new EdgeXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = cur_frame_->camera_;
        edge->point_ = Eigen::Vector3d(pts_3d[idx].x, pts_3d[idx].y, pts_3d[idx].z);
        edge->setMeasurement(Eigen::Vector2d(pts_2d[idx].x, pts_2d[idx].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
    }

    // 迭代优化
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    T_cur_estimated_ = Sophus::SE3(pose->estimate().rotation(), pose->estimate().translation());
}

bool VisualOdometry::checkEstimatedPose() {
    if (num_inliers_ < min_inliers_) {
        cout << "reject because inlier is too small: " << num_inliers_ << endl;
        return false;
    }

    Sophus::SE3 T_r_c = ref_frame_->T_c_w_ * T_cur_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    cout << "motion: " << d.norm() << endl;
    if (d.norm() > 5.0) {
        cout << "reject because motion is too large: " << d.norm() << endl;
        return false;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame() {
    Sophus::SE3 T_r_c = ref_frame_->T_c_w_ * T_cur_estimated_.inverse();
    Sophus::Vector6d d = T_r_c.log();
    Eigen::Vector3d trans = d.head(3);
    Eigen::Vector3d rot = d.tail(3);

    if ((rot.norm() > key_frame_min_rot) || (trans.norm() > key_frame_min_trans)) {
        return true;
    }
    return false;
}

void VisualOdometry::addKeyFrame() {
    cout << "add a new key frame: "<< cur_frame_->id_ << endl;

    // 初始化时将初始化帧的所有关键点加入地图
    if (map_->key_frames_.empty()) {
        for (auto i = 0; i < kp_cur_.size(); i++) {
            double d = cur_frame_->findDepth(kp_cur_[i]);
            if (d < 0) continue;

            Eigen::Vector3d pos_world = ref_frame_->camera_->pixel2world(
                                            Eigen::Vector2d(kp_cur_[i].pt.x, kp_cur_[i].pt.y), cur_frame_->T_c_w_, d);
            Eigen::Vector3d n = pos_world - ref_frame_->getCamCenter();
            n.normalize();
            MapPoint::Ptr mappoint = MapPoint::createMapPoint(
                                         pos_world, n, descriptor_cur_.row(i).clone(), cur_frame_);
            map_->insertMapPoint(mappoint);
        }
    }

    map_->insertKeyFrame(cur_frame_);
    ref_frame_ = cur_frame_;
}

void VisualOdometry::addMapPoints() {
    vector<bool> matched(kp_cur_.size(), false);  // 初始化当前帧中所有关键点的被匹配状态
    for (int idx: matched_2d_kp_index_) {
        matched[idx] = true;
    }
    for (int i = 0; i < kp_cur_.size(); i++) {
        if (matched[i] == true) continue;
        double d = ref_frame_->findDepth(kp_cur_[i]);
        if (d < 0) continue;

        Eigen::Vector3d pts_world = ref_frame_->camera_->pixel2world(
                                        Eigen::Vector2d(kp_cur_[i].pt.x, kp_cur_[i].pt.y), cur_frame_->T_c_w_, d);
        Eigen::Vector3d n = pts_world - ref_frame_->getCamCenter();
        n.normalize();
        MapPoint::Ptr mappoint = MapPoint::createMapPoint(
                                     pts_world, n, descriptor_cur_.row(i).clone(), cur_frame_);
        map_->insertMapPoint(mappoint);
    }
    cout << "after addMapPoint, mappoint size: " << map_->map_points_.size() << endl;
}

void VisualOdometry::optimizeMap() {
    cout << "before optimizeMap, mappoint size: " << map_->map_points_.size() << endl;
    // 移除不合适的点
    for (auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();) {
        // 移除不在当前帧中的点
        if (!cur_frame_->isInFrame(iter->second->pos_)) {
            iter = map_->map_points_.erase(iter);  // iter指向移除元素的后一个元素
            continue;
        }

        // 移除匹配次数/观察到的次数小于一定比例的点
        float match_ratio = float(iter->second->matched_times_) / iter->second->observed_times_;
        if (match_ratio < mappoint_erase_ratio) {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        // 移除难以观察到的点
        double angle = getViewAngel(cur_frame_, iter->second);
        if (angle > M_PI / 6.0) {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        if (iter->second->good_ = false) {
            // TODO
        }

        iter++;
    }
    cout << "after erase, mappoints size: " << map_->map_points_.size() << endl;

    if (matched_2d_kp_index_.size() < 100) {
        addMapPoints();
    }

    if (map_->map_points_.size() > 1000) {
        // TODO: map too large, remove some one
        mappoint_erase_ratio += 0.05;
    } else {
        mappoint_erase_ratio = 0.1;
    }

    cout << "after optimizeMap, mappoints size: " << map_->map_points_.size() << endl;
}

double VisualOdometry::getViewAngel(Frame::Ptr frame, MapPoint::Ptr point) {
    Eigen::Vector3d n = point->pos_ - frame->getCamCenter();
    n.normalize();
    return acos(n.transpose() * point->norm_);
}
}
