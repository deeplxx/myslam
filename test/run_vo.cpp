#include <fstream>
#include <string>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>

#include "myslam/visual_odometry.h"
#include "myslam/config.h"

using namespace std;

void loadImages(ifstream& fin, vector<string>& files, vector<double>& times, const string& dir) {
    string s;
    getline(fin, s);
    getline(fin, s);
    getline(fin, s);
    while (!fin.eof()) {
        string s0;
        getline(fin, s0);
        if (!s0.empty()) {
            stringstream ss;
            ss << s0;
            string f;
            double t;
            ss >> t;
            times.push_back(t);
            ss >> f;
            files.push_back(dir + "/" + f);
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        cerr << "usage: run_vo param_file" << endl;
        return 1;
    }

    myslam::Config::setParamFile(argv[1]);
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

    string dataset_dir = myslam::Config::get<string>("dataset_dir");
    ifstream fin_rgb(dataset_dir + "/rgb.txt");
    ifstream fin_depth(dataset_dir + "/depth.txt");

    if (!(fin_rgb && fin_depth)) {
        cerr << "lack of txt." << endl;
        return 1;
    }

    // 读取文件
    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    loadImages(fin_rgb, rgb_files, rgb_times, dataset_dir);
    loadImages(fin_depth, depth_files, depth_times, dataset_dir);

    myslam::Camera::Ptr camera(new myslam::Camera);

    // viz
    cv::viz::Viz3d vis("Visual Odometry");
    cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
    cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    vis.setViewerPose(cam_pose);
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    vis.showWidget("world", world_coor);
    vis.showWidget("camera", camera_coor);

    // slam
    cout << "read total " << rgb_files.size() << " images" << endl;
    for (int i = 0; i < rgb_files.size(); i++) {
        cout << rgb_files[i] << endl;

        cv::Mat color = cv::imread(rgb_files[i]);
        cv::Mat depth = cv::imread(depth_files[i], -1);
        if (color.data == nullptr || depth.data == nullptr) {
            cerr << "file name error" << endl;
            break;
        }
        myslam::Frame::Ptr frame = myslam::Frame::createFrame();
        frame->camera_ = camera;
        frame->color_ = color;
        frame->depth_ = depth;
        frame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame(frame);
        cout << "VO cost time: " << timer.elapsed() << endl;
        if (vo->state_ == myslam::VisualOdometry::LOST) {
            cerr << "lost motion..." << endl;
            break;
        }
        Sophus::SE3 Tcw = frame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M(
            cv::Affine3d::Mat3(
                Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
            ),
            cv::Affine3d::Vec3(
                Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
            )
        );

        cv::imshow("image", color);
        cv::waitKey(1);
        vis.setWidgetPose("Camera", M);
        vis.spinOnce(1, false);
    }

    return 0;
}
