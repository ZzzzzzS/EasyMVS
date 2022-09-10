#pragma once
#include "CameraObject.h"

/**
 * @brief read images from folder.
 */
class PinholeImageReader : public PinholeCamera
{
public:
	using Ptr = std::shared_ptr<PinholeImageReader>;

	/**
	 * @brief Creates PinholeImageReader object.
	 *
	 * @param CameraName the unique name of Camera
	 * @param CameraMatrix the 3x3 camera intrinsic matrix
	 * @param DistCoeff the camera distortion parameters
	 * @return Ptr the shared pointer of PinholeCamera class
	 */
	static PinholeImageReader::Ptr Create(const cv::Mat1d& CameraMatrix = cv::Mat1d(), const cv::Mat1d& DistCoeff = cv::Mat1d());
public:

	virtual std::string type_name() override;

	virtual bool save(JsonNode& fs) override;

	virtual bool load(JsonNode& fs) override;

	virtual bool open() override;
	virtual bool open(const cv::String& filename, int apiPreference = cv::CAP_ANY) override;
	virtual bool open(const cv::String& filename, int apiPreference, const std::vector< int >& params) override;
	virtual bool open(int index, int apiPreference = cv::CAP_ANY);
	virtual bool open(int index, int apiPreference, const std::vector< int >& params);
	virtual bool read(cv::OutputArray image);
	virtual void release();
	virtual bool grab();
	virtual bool isOpened() const;
	virtual bool retrieve(cv::OutputArray image, int flag = 0);


private:
	bool recursive=false;
	bool order=false;
	std::string FilePrefix;
	std::string FilePostfix;
	std::string FileType = ".bmp";
	int BeginNumber = 0;
	int EndNumber = 0;
	int ImreadFlag = cv::IMREAD_COLOR;

	std::list<std::string> FileList;
};
