#include "myslam/config.h"

namespace myslam {

void Config::setParamFile(const string& filename) {
    if (config_ == nullptr) {
        config_ = shared_ptr<Config>(new Config());
    }
    config_->file_ = cv::FileStorage(filename, cv::FileStorage::READ);
    if (config_->file_.isOpened() == false) {
        cerr << "param file: " << filename << " does not exist." << endl;
        config_->file_.release();
        return;
    }
}

Config::~Config() {
    if (file_.isOpened()) {
        file_.release();
    }
}

shared_ptr<Config> Config::config_ = nullptr;

}
