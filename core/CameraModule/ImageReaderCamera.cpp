#include "ImageReaderCamera.h"
#include <opencv2/core/utils/filesystem.hpp>
#include <filesystem>

PinholeImageReader::Ptr PinholeImageReader::Create(const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff)
{
	auto ptr = std::make_shared<PinholeImageReader>();
	ptr->updateCameraMatrix(CameraMatrix);
	ptr->updateDistCoeff(DistCoeff);
	return ptr;
}

std::string PinholeImageReader::type_name()
{
	return std::string("pinhole-image-reader");
}

bool PinholeImageReader::save(JsonNode& fs)
{
	if(!PinholeCamera::save(fs))
		return false;
	try
	{
		fs["type-id"]		= this->type_name();
		fs["recursive"]		= this->recursive;
		fs["order"]			= this->order;
		fs["file-prefix"]	= this->FilePrefix;
		fs["file-postfix"]	= this->FilePostfix;
		fs["file-type"]		= this->FileType;
		fs["begin-number"]	= this->BeginNumber;
		fs["end-number"]	= this->EndNumber;
		fs["imread-flag"]	= this->ImreadFlag;
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	return false;
}

bool PinholeImageReader::load(JsonNode& fs)
{
	if (!PinholeCamera::load(fs))
		return false;
	try
	{
		this->recursive		= fs.at("recursive");
		this->order			= fs.at("order");
		this->FilePrefix	= fs.at("file-prefix");
		this->FilePostfix	= fs.at("file-postfix");
		this->FileType		= fs.at("file-type");
		this->BeginNumber	= fs.at("begin-number");
		this->EndNumber		= fs.at("end-number");
		this->ImreadFlag	= fs.at("imread-flag");
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	return false;
}

bool PinholeImageReader::open()
{
	using namespace cv::utils::fs;
	if (this->order)
	{
		for (size_t i = this->BeginNumber; i <= this->EndNumber; i++)
		{
			auto string = this->CameraName + "/" + this->FilePrefix + std::to_string(i) + this->FilePostfix + this->FileType;
			if(std::filesystem::exists(string))
				this->FileList.push_back(string);
			else
			{
				std::cout << this->type_name() << ": file does NOT exist, name=" << std::endl << string << std::endl;
			}
		}
	}
	else
	{
		std::vector<cv::String> files;
		glob(this->CameraName, cv::String(), files, this->recursive);
		for (auto&& file : files)
		{
			if (file.rfind(this->FileType) != std::string::npos)
			{
				this->FileList.push_back(file);
			}
		}
	}

	return this->isOpened();
}

bool PinholeImageReader::open(const cv::String& filename, int apiPreference)
{
	this->CameraName = filename;
	return this->open();
}

bool PinholeImageReader::open(const cv::String& filename, int apiPreference, const std::vector<int>& params)
{
	this->CameraName = filename;
	return this->open();
}

bool PinholeImageReader::open(int index, int apiPreference)
{
	return this->isOpened();
}

bool PinholeImageReader::open(int index, int apiPreference, const std::vector<int>& params)
{
	return this->isOpened();
}

bool PinholeImageReader::read(cv::OutputArray image)
{
	return this->retrieve(image, this->ImreadFlag);
}

void PinholeImageReader::release()
{
	this->FileList.clear();
}

bool PinholeImageReader::grab()
{
	//nothing to grab
	return true;
}

bool PinholeImageReader::isOpened() const
{
	return !this->FileList.empty(); //if it's opened, the file list should not be empty.
}

bool PinholeImageReader::retrieve(cv::OutputArray image, int flag)
{
	if (!this->isOpened())
		return false;

	auto name = this->FileList.front();
	this->FileList.pop_front();
	cv::Mat tmp = cv::imread(name, flag);
	image.assign(tmp);
	return !tmp.empty();
}

