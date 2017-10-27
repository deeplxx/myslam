/*
	负责参数文件的读取
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

class Config {
  private:
	static shared_ptr<Config> config_;
	cv::FileStorage file_;

	Config() {}

  public:
	~Config();

	// 设置新的Config
	static void setParamFile(const string& filename);

	// 获取参数值
	template<typename T>
	static T get(const string& key) {
		return T(config_->file_[key]);
	}
};
}

#endif
