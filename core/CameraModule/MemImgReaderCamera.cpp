#include "MemImgReaderCamera.h"

PinholeMemImgReader::Ptr PinholeMemImgReader::Create(const cv::Mat1d& CameraMatrix, const cv::Mat1d& DistCoeff)
{
	auto ptr = std::make_shared<PinholeMemImgReader>();
	ptr->updateCameraMatrix(CameraMatrix);
	ptr->updateDistCoeff(DistCoeff);
	
	return ptr;
}

std::string PinholeMemImgReader::type_name()
{
	return std::string("pinhole-memory-image-reader");
}

bool PinholeMemImgReader::save(JsonNode& fs)
{
	return PinholeCamera::save(fs);
}

bool PinholeMemImgReader::load(JsonNode& fs)
{
	return PinholeCamera::save(fs);
}

bool PinholeMemImgReader::open()
{
	return this->isOpened();
}

bool PinholeMemImgReader::open(const cv::String& filename, int apiPreference)
{
	std::cout << this->type_name() << ":open with filename is not supported." << std::endl;
	return false;
}

bool PinholeMemImgReader::open(const cv::String& filename, int apiPreference, const std::vector<int>& params)
{
	std::cout << this->type_name() << ":open with filename is not supported." << std::endl;
	return false;
}

bool PinholeMemImgReader::open(int index, int apiPreference)
{
	std::cout << this->type_name() << ":open with camera index is not supported." << std::endl;
	return false;
}

bool PinholeMemImgReader::open(int index, int apiPreference, const std::vector<int>& params)
{
	std::cout << this->type_name() << ":open with camera index is not supported." << std::endl;
	return false;
}

bool PinholeMemImgReader::read(cv::OutputArray image)
{
	return this->retrieve(image);
}

void PinholeMemImgReader::release()
{
	while (!imgQueue.empty())
	{
		imgQueue.pop();
	}
}

bool PinholeMemImgReader::grab()
{
	return true;
}

bool PinholeMemImgReader::isOpened() const
{
	return !this->imgQueue.empty();
}

bool PinholeMemImgReader::retrieve(cv::OutputArray image, int flag)
{
	if (this->isOpened())
	{
		image.assign(this->imgQueue.front());
		this->imgQueue.pop();
		return true;
	}
	else
	{
		return false;
	}
}

bool PinholeMemImgReader::addImage(cv::InputArray Img)
{
	auto img = Img.getMat();
	if (img.empty())
	{
		return false;
	}
	else
	{
		this->imgQueue.push(img);
		return true;
	}
}

bool PinholeMemImgReader::addImages(cv::InputArrayOfArrays Imgs)
{
	std::vector<cv::Mat> imgs;
	Imgs.getMatVector(imgs);
	if (imgs.empty())
	{
		return false;
	}
	else
	{
		for (auto& i : imgs)
		{
			this->imgQueue.push(i);
		}
		return true;
	}
}

int PinholeMemImgReader::getBufferSize()
{
	return this->imgQueue.size();
}
