#pragma once
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <sophus/se3.hpp>
#include <Eigen/Dense>

class MapPointObject;
class FrameObject;
class PinholeFrameObject;

/**
 * @brief these functions are used to save and load the data in json format,
 * user do not need to care about the details, unless custom data type is used,
 * for more information, please refer to the official website of nlohmann/json
 * [https://github.com/nlohmann/json](https://github.com/nlohmann/json)
 * 
 */
namespace nlohmann {
	template<>
	struct adl_serializer<cv::Mat>
	{
		static void to_json(json& j, const cv::Mat& mat) 
		{
			try
			{
				j["type_id"] = std::string("opencv-matrix");
				j["rows"] = mat.rows;
				j["cols"] = mat.cols;

				if (mat.empty())
				{
					j["dt"] = "u";
					j["data"] = json::array();
					return;
				}

				switch (mat.type())
				{
				case CV_8UC1:
				{
					j["dt"] = "u";
					std::vector<uint8_t> tmp(mat.begin<uint8_t>(), mat.end<uint8_t>());
					j["data"] = tmp;
					break;
				}
				case CV_8UC2:
				{
					j["dt"] = "2u";
					std::vector<uint8_t> tmp(mat.begin<uint8_t>(), mat.end<uint8_t>());
					j["data"] = tmp;
					break;
				}
				case CV_8UC3:
				{
					j["dt"] = "3u";
					std::vector<uint8_t> tmp(mat.begin<uint8_t>(), mat.end<uint8_t>());
					j["data"] = tmp;
					break;
				}
				case CV_32FC1:
				{
					j["dt"] = "f";
					std::vector<float_t> tmp(mat.begin<float_t>(), mat.end<float_t>());
					j["data"] = tmp;
					break;
				}
				case CV_32FC2:
				{
					j["dt"] = "2f";
					cv::Mat tmpmat = mat.reshape(1);
					std::vector<float_t> tmp(tmpmat.begin<float_t>(), tmpmat.end<float_t>());
					j["data"] = tmp;
					break;
				}
				case CV_32FC3:
				{
					j["dt"] = "3f";
					cv::Mat tmpmat = mat.reshape(1);
					std::vector<float_t> tmp(tmpmat.begin<float_t>(), tmpmat.end<float_t>());
					j["data"] = tmp;
					break;
				}
				case CV_64FC1:
				{
					j["dt"] = "d";
					std::vector<double_t> tmp(mat.begin<double_t>(), mat.end<double_t>());
					j["data"] = tmp;
					break;
				}
				case CV_64FC2:
				{
					j["dt"] = "2d";
					std::vector<double_t> tmp(mat.begin<double_t>(), mat.end<double_t>());
					j["data"] = tmp;
					break;
				}
				case CV_64FC3:
				{
					j["dt"] = "3d";
					std::vector<double_t> tmp(mat.begin<double_t>(), mat.end<double_t>());
					j["data"] = tmp;
					break;
				}
				default:
					j["dt"] = "unknown";
					break;
				}
			}
			catch (const std::exception& e)
			{
				j = json();
				std::cerr << e.what() << std::endl;
			}
		}

		static void from_json(const json& j, cv::Mat& mat) 
		{
			try
			{
				if (j.at("type_id").get<std::string>() != std::string("opencv-matrix"))
				{
					mat = cv::Mat();
					return;
				}

				auto rows = j.at("rows").get<int>();
				auto cols = j.at("cols").get<int>();

				if (rows == 0 || cols == 0)
				{
					mat = cv::Mat();
					return;
				}

				auto typestring = j.at("dt").get<std::string>();
				
				auto type = typestring.back();
				auto chennelstring = typestring.front();
				
				int chennelnumber;
				if (chennelstring > '3' || chennelstring <= '0')
				{
					chennelnumber = 1;
				}
				else
				{
					chennelnumber = chennelstring - '0';
				}

				auto data = j.at("data");
				switch (type)
				{
				case 'u':
				{
					mat = cv::Mat(rows, cols, CV_8UC(chennelnumber));
					uint8_t* ptr = mat.ptr<uint8_t>(0);
					for (size_t i = 0; i < rows*cols*chennelnumber; i++)
					{
						ptr[i] = data[i].get<uint8_t>();
					}
					break;
				}
				case 'f':
				{
					mat = cv::Mat(rows, cols, CV_32FC(chennelnumber));
					float_t* ptr = mat.ptr<float_t>(0);
					for (size_t i = 0; i < rows * cols * chennelnumber; i++)
					{
						ptr[i] = data[i].get<float_t>();
					}
					break;
				}
				case 'd':
				{
					mat = cv::Mat(rows, cols, CV_64FC(chennelnumber));
					double_t* ptr = mat.ptr<double_t>(0);
					for (size_t i = 0; i < rows * cols * chennelnumber; i++)
					{
						ptr[i] = data[i].get<double_t>();
					}
					break;
				}
				default:
					break;
				}
			}
			catch (const std::exception& e)
			{
				mat = cv::Mat();
				std::cerr << e.what() << std::endl;
			}
			
		}
	};

	template<typename T>
	struct adl_serializer<Sophus::SE3<T>>
	{
		static void to_json(json& j, const Sophus::SE3<T>& matrix)
		{
			try
			{
				j["type-id"] = std::string("sophus-se3matrix");
				j["rows"] = 4;
				j["cols"] = 4;
				j["dt"] = typeid(T).name();
				Eigen::Matrix<T, 4, 4> EigenMat = matrix.matrix();
				json datanode;
				for (size_t i = 0; i < 4; i++)
				{
					for (size_t j = 0; j < 4; j++)
					{
						datanode[i * 4 + j] = EigenMat(i, j);
					}
				}
				j["data"] = datanode;
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				j = json();
			}
			
		}

		static void from_json(const json& j, Sophus::SE3<T>& matrix)
		{
			try
			{
				if (j.at("type-id").get<std::string>() != std::string("sophus-se3matrix"))
				{
					matrix = Sophus::SE3<T>();
					return;
				}

				Eigen::Matrix<T, 4, 4> EigenMat;
				auto datanode = j.at("data");
				
				for (size_t i = 0; i < 4; i++)
				{
					for (size_t j = 0; j < 4; j++)
					{
						EigenMat(i, j) = datanode[i * 4 + j];
					}
				}
				matrix = Sophus::SE3<T>(EigenMat);
			}
			catch (const std::exception& e)
			{
				matrix = Sophus::SE3<T>();
				std::cerr << e.what()<<std::endl;
			}
			
		}
	};

	template<typename Scalar,int Rows,int Cols>
	struct adl_serializer<Eigen::Matrix<Scalar, Rows, Cols>>
	{
		static void to_json(nlohmann::json& j, const Eigen::Matrix<Scalar, Rows, Cols>& matrix)
		{
			try
			{
				j["type-id"] = std::string("eigen-matrix");
				j["dt"] = std::string(typeid(Scalar).name());
				j["rows"] = matrix.rows();
				j["cols"] = matrix.cols();
				json DataNode;
				for (size_t i = 0; i < matrix.rows(); i++)
				{
					for (size_t j = 0; j < matrix.cols(); j++)
					{
						DataNode.push_back(matrix(i, j));
					}
				}
				j["data"] = DataNode;
			}
			catch (const std::exception& e)
			{
				j = json();
				std::cerr << e.what() << std::endl;
			}
		}

		static void from_json(const nlohmann::json& j, Eigen::Matrix<Scalar, Rows, Cols>& matrix)
		{
			try
			{
				if (j.at("type-id").get<std::string>() != std::string("eigen-matrix"))
				{
					matrix = Eigen::Matrix<Scalar, Rows, Cols>::Zero();
					return;
				}

				auto rows_ = j.at("rows").get<size_t>();
				auto cols_ = j.at("cols").get<size_t>();
				if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic)
				{
					matrix.resize(rows_, cols_);
				}
				else if (Rows == Eigen::Dynamic && Cols != Eigen::Dynamic)
				{
					matrix.resize(rows_, Cols);
					assert(Cols == j["cols"]);
				}
				else if (Rows != Eigen::Dynamic && Cols == Eigen::Dynamic)
				{
					matrix.resize(Rows, cols_);
					assert(Rows == j["rows"]);
				}
				else
				{
					assert(Rows == j["rows"]);
					assert(Cols == j["cols"]);
				}

				json DataNode = j["data"];
				for (size_t i = 0; i < rows_; i++)
				{
					for (size_t j = 0; j < cols_; j++)
					{
						matrix(i, j) = DataNode[i * Cols + j];
					}
				}
			}
			catch (const std::exception& e)
			{
				matrix = Eigen::Matrix<Scalar, Rows, Cols>::Zero();
				std::cerr << e.what() << std::endl;
			}
		}
	};

	template <>
    struct adl_serializer<cv::DMatch> {
        static void to_json(json& j, const cv::DMatch& opt) {
			j.push_back(opt.queryIdx);
			j.push_back(opt.trainIdx);
			j.push_back(opt.imgIdx);
			j.push_back(opt.distance);
        }

        static void from_json(const json& j, cv::DMatch& opt) {
			opt.queryIdx = j.at(0);
			opt.trainIdx = j.at(1);
			opt.imgIdx = j.at(2);
			opt.distance = j.at(3);
        }
    };

	template<typename T>
	struct adl_serializer<cv::Mat_<T>>
	{
		static void to_json(json& j, const cv::Mat_<T>& mat_) 
		{
			j = cv::Mat(mat_);
		}
		
		static void from_json(const json& j, cv::Mat_<T>& mat_)
		{
			cv::Mat tmp;
			tmp = j;
			mat_ = tmp;
		}
	};

	template<>
	struct adl_serializer<cv::KeyPoint>
	{
		static void to_json(json& j, const cv::KeyPoint& Point)
		{
			try
			{
				j = { Point.pt.x,Point.pt.y,
					Point.size, Point.angle, Point.response,
					Point.octave,Point.class_id
				};
			}
			catch (const std::exception& e)
			{
				j = json();
				std::cerr << e.what() << std::endl;
			}
		}

		static void from_json(const json& j, cv::KeyPoint& Point)
		{
			try
			{
				Point.pt.x = j[0];
				Point.pt.y = j[1];
				Point.size = j[2];
				Point.angle = j[3];
				Point.response = j[4];
				Point.octave = j[5];
				Point.class_id = j[6];
			}
			catch (const std::exception& e)
			{
				Point = cv::KeyPoint();
				std::cerr << e.what() << std::endl;
			}
		}
	};

	template<typename T>
	struct adl_serializer<cv::Size_<T>>
	{
		static void to_json(json& j, const cv::Size_<T>& Size)
		{
			try
			{
				j = { Size.width,Size.height };
			}
			catch (const std::exception& e)
			{
				j = json();
				std::cerr << e.what() << std::endl;
			}
		}

		static void from_json(const json& j, cv::Size_<T>& Size)
		{
			try
			{
				Size.width=j.at(0);
				Size.height=j.at(1);
			}
			catch(const std::exception& e)
			{
				Size = cv::Size_<T>();
				std::cerr << e.what() << '\n';
			}
			
		}
	};
}
