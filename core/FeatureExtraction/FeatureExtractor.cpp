#include "FeatureExtractor.h"
#include <opencv2/features2d.hpp>

FeatureExtractor::FeatureExtractor()
{
}

FeatureExtractor::~FeatureExtractor()
{
}

std::string FeatureExtractor::type_name()
{
	return std::string("Workflow FeatureExtractor");
}

bool FeatureExtractor::clear()
{
	this->detector->clear();
	this->detector.reset();
	this->m_isInit=false;
	return true;
}

bool FeatureExtractor::Compute(FrameObject::Ptr frame)
{
	cv::Mat rgb=frame->RGBMat;
	this->detector->detectAndCompute(frame->RGBMat,cv::noArray(),frame->KeyPoints,frame->KeyPointsDescriptors);
	return !frame->KeyPoints.empty();
}

void FeatureExtractor::Trigger(DataQueue data)
{
	DataQueue DataOut;
	while (!data.empty())
	{
		auto ptr=std::dynamic_pointer_cast<FrameObject>(data.front());
		if(ptr==nullptr)
		{
			emit this->Warning(this->type_name()+": failed to dynamic cast input data");
		}
		else
		{
			this->Compute(ptr);
		}
		data.pop();
		DataOut.push(ptr);
	}
	emit this->Finished(DataOut);
}

void FeatureExtractor::Trigger()
{
	emit this->Error("Can NOT triggered without input data");
}

CVFeatureExtractor::Ptr CVFeatureExtractor::Create() 
{
	return std::make_shared<CVFeatureExtractor>();
}

CVFeatureExtractor::CVFeatureExtractor() 
{
}

CVFeatureExtractor::~CVFeatureExtractor() 
{
}

bool CVFeatureExtractor::load(JsonNode& fs) 
{
	try
	{
		//initialize with different algorithm
		std::string algorithm = fs.at("algorithm-name");
		auto parameters = fs.at("parameters");
		if (algorithm == "opencv-orb")
		{
			try
			{
				int nfeatures		= parameters.at("nfeatures");
				float scaleFactor	= parameters.at("scale-factor");
				int nlevel			= parameters.at("nlevel");
				int edgeThreshold	= parameters.at("edge-threshold");
				int firstlevel		= parameters.at("first-level");
				int WTA_K			= parameters.at("wta-k");
				int scoretype		= parameters.at("score-type"); //TODO: fix with enum type
				int patchsize		= parameters.at("patch-size");
				int fastthres		= parameters.at("fast-threshold");

				this->detector = cv::ORB::create(nfeatures, scaleFactor,
					nlevel, edgeThreshold, firstlevel,
					WTA_K, static_cast<cv::ORB::ScoreType>(scoretype),
					patchsize, fastthres);
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				std::cout << this->type_name() << ": failed to load parameters, default parameters will be used!" << std::endl;
				this->detector = cv::ORB::create();
			}
		}
		else if(algorithm=="opencv-sift")
		{
			try
			{
				int nfeature		 = parameters.at("nfeatures");
				int noctavelayers	 = parameters.at("n-octave-layers");
				double contrastThres = parameters.at("contrast-threshold");
				double edgeThres	 = parameters.at("edge-threshold");
				double sigma		 = parameters.at("sigma");
				//int desctype		 = parameters.at("descriptor-type");

				this->detector = cv::SIFT::create(nfeature, noctavelayers, 
					contrastThres, edgeThres, sigma);
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
				std::cout << this->type_name() << ": failed to load parameters, default parameters will be used!" << std::endl;
				this->detector = cv::SIFT::create();
			}
		}
		else
		{
			auto message = this->type_name() + ": failed to initialize, bad algorithm type!";
			throw std::exception(message.c_str());
		}

		this->m_isInit = (this->detector == nullptr) ? false : true;
		return this->m_isInit;
	}
	catch (const std::exception& e)
	{
		this->m_isInit = false;
		std::cerr << e.what() << std::endl;
	}
	return false;
}

bool CVFeatureExtractor::save(JsonNode& fs) 
{
	try
	{
		JsonNode parameters;
		
		if (this->detector->getDefaultName() == std::string("Feature2D.ORB"))
		{
			fs["algorithm-name"] = std::string("opencv-orb");
			auto ptr = this->detector.staticCast<cv::ORB>();

			parameters["nfeatures"]= ptr->getMaxFeatures();
			parameters["scale-factor"] = ptr->getScaleFactor();
			parameters["nlevel"] = ptr->getNLevels();
			parameters["edge-threshold"] = ptr->getEdgeThreshold();
			parameters["first-level"] = ptr->getFirstLevel();
			parameters["wta-k"] = ptr->getWTA_K();
			parameters["score-type"] = static_cast<int>(ptr->getScoreType()); //TODO: fix with enum type
			parameters["patch-size"] = ptr->getPatchSize();
			parameters["fast-threshold"] = ptr->getFastThreshold();

			fs.at("parameters") = parameters;
		}
		else if (this->detector->getDefaultName() == std::string("Feature2D.SIFT"))
		{
			fs["algorithm-name"] = std::string("opencv-sift");

			//auto ptr = this->detector.staticCast<cv::SIFT>();
			fs["paramers"] = JsonNode::value_t::null;
			std::cout << "save SIFT parameters is not supported currently!" << std::endl;
			//TODO: fix sift save
		}
		else
		{
			std::cout << this->type_name() <<
				": failed to save algorithm parameters, type " <<
				typeid(this->detector).name() << " is unknown" << std::endl;
			fs = JsonNode::value_t::null;
			return false;
		}
		return true;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		fs = JsonNode::value_t::null;
	}
	return false;
}

std::string CVFeatureExtractor::type_name()
{
	return std::string("Workflow CVFeatureExtractor");
}
