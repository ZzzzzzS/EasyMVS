#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

using JsonNode = nlohmann::json;

namespace MVSConfig
{
	struct GlobalConfig
	{
		static std::string WorkspacePath;
		static std::string RGBMatType;
		static std::string XYZMatType;
	};
}

const int MarkerPointType = 63288;//用来区分特征点和我们的标记点的类型，数字是随便写的，无特殊意义
