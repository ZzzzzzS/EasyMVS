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
