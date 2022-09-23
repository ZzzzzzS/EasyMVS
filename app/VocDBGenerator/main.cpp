#include <iostream>
#include <vector>

#include <DBoW2/DBoW2.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/utils/filesystem.hpp>

#include <argparse/argparse.hpp>

#include "FeatureExtraction/FSift128.h"

using Sift128Vocabulary = DBoW2::TemplatedVocabulary<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;
using Sift128Database = DBoW2::TemplatedDatabase<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;


enum class KeyPointType_e
{
	ORB=0,
	SIFT = 1
};


bool LoadFeature(std::string Path, KeyPointType_e type, std::vector<cv::Mat>& Feature);
bool BuildTreeSift(std::string OutPath, std::vector<cv::Mat>& Features, Sift128Vocabulary& Sifttree);
bool BuildTreeORB(std::string OutPath, std::vector<cv::Mat>& Features, OrbVocabulary& ORBtree);
void showProcess(double current, double all_block_num);

int main(int argc, char* argv[])
{
	argparse::ArgumentParser program("VocDB Generator","1.0");
	//Input: input image folder, KeyPoint Type, K, L, ScoringType, WeightingType
	//output: file path
	program.add_argument("-i", "--input").help("Input image folder").required();
	program.add_argument("-k", "--keypoint")
		.help("KeyPoint Type <ORB SIFT>")
		.required().default_value(std::string("SIFT"));
	
	program.add_argument("-K", "--K").help("K").required().default_value(9);
	program.add_argument("-L", "--L").help("L").required().default_value(3);
	
	program.add_argument("-s", "--scoring")
		.help("ScoringType <  L1_NORM, L2_NORM, CHI_SQUARE,KL,BHATTACHARYYA,DOT_PRODUCT >")
		.required().default_value(std::string("L2_NORM"));
	program.add_argument("-w", "--weighting")
		.help("WeightingType <  TF_IDF,TF,IDF,BINARY >")
		.required().default_value(std::string("TF_IDF"));
	
	program.add_argument("-o", "--output").help("Output file path").required();
	
	try
	{
		program.parse_args(argc, argv);
	}
	catch (const std::runtime_error& err)
	{
		std::cout << err.what() << std::endl;
		std::cout << program;
		exit(1);
	}
	
	std::string input = program.get<std::string>("-i");
	std::string output = program.get<std::string>("-o");
	std::string keypoint = program.get<std::string>("--keypoint");
	
	KeyPointType_e KeyPointType;
	
	if (keypoint == std::string("ORB"))
	{
		KeyPointType = KeyPointType_e::ORB;
	}
	else if (keypoint == std::string("SIFT"))
	{
		KeyPointType = KeyPointType_e::SIFT;
	}
	else
	{
		std::cout << "KeyPoint Type Error" << std::endl;
		exit(1);
	}

	int K = program.get<int>("-K");
	int L = program.get<int>("-L");
	
	std::string ScoringTypeStr = program.get<std::string>("-s");
	DBoW2::ScoringType ScoringType;
	if (ScoringTypeStr == "L1_NORM")
	{
		ScoringType = DBoW2::ScoringType::L1_NORM;
	}
	else if (ScoringTypeStr == "L2_NORM")
	{
		ScoringType = DBoW2::ScoringType::L2_NORM;
	}
	else if (ScoringTypeStr == "CHI_SQUARE")
	{
		ScoringType = DBoW2::ScoringType::CHI_SQUARE;
	}
	else if (ScoringTypeStr == "KL")
	{
		ScoringType = DBoW2::ScoringType::KL;
	}
	else if (ScoringTypeStr == "BHATTACHARYYA")
	{
		ScoringType = DBoW2::ScoringType::BHATTACHARYYA;
	}
	else if (ScoringTypeStr == "DOT_PRODUCT")
	{
		ScoringType = DBoW2::ScoringType::DOT_PRODUCT;
	}
	else
	{
		std::cout << "Scoring Type Error" << std::endl;
		exit(1);
	}
	
	std::string WeightingTypeStr = program.get<std::string>("-w");
	DBoW2::WeightingType WeightingType;

	if(WeightingTypeStr =="TF_IDF")
	{
		WeightingType = DBoW2::WeightingType::TF_IDF;
	}
	else if (WeightingTypeStr == "TF")
	{
		WeightingType = DBoW2::WeightingType::TF;
	}
	else if (WeightingTypeStr == "IDF")
	{
		WeightingType = DBoW2::WeightingType::IDF;
	}
	else if (WeightingTypeStr == "BINARY")
	{
		WeightingType = DBoW2::WeightingType::BINARY;
	}
	else
	{
		std::cout << "Weighting Type Error" << std::endl;
		exit(1);
	}

	std::vector<cv::Mat> Feature;
	if (!LoadFeature(input, KeyPointType, Feature))
	{
		std::cout << "failed to load features" << std::endl;
		exit(1);
	}
	
	if (KeyPointType == KeyPointType_e::ORB)
	{
		std::cout << "Creating a small " << K << "^" << L << " vocabulary..." << std::endl;
		OrbVocabulary voc(K, L, WeightingType, ScoringType);
		BuildTreeORB(output, Feature, voc);
	}
	else if (KeyPointType == KeyPointType_e::SIFT)
	{
		std::cout << "Creating a small " << K << "^" << L << " vocabulary..." << std::endl;
		Sift128Vocabulary voc(K, L, WeightingType, ScoringType);
		BuildTreeSift(output, Feature, voc);
	}
	else
	{
		std::cout << "Keypoint Type Error" << std::endl;
		exit(1);
	}
	
	return 0;
}

void showProcess(double current, double all_block_num)
{
	if (current < all_block_num - 1)
	{
		char x[300];
		memset(x, 0, 300);
		sprintf(&x[0], "detecting[%.2lf%%]", current * 100.0 / (all_block_num - 1));
		printf("\r %s", x);
	}
	else
	{
		char x[300];
		memset(x, 0, 300);
		sprintf(&x[0], "detecting Finished[%.2lf%%]\n", current * 100.0 / (all_block_num - 1));
		printf("\r %s", x);
	}
	fflush(stdout);
}

bool LoadFeature(std::string Path, KeyPointType_e type, std::vector<cv::Mat>& Feature)
{
	
	std::vector<cv::String> result;
	cv::utils::fs::glob(Path, cv::String(), result, true);
	if (result.empty()) return false;
	
	cv::Ptr<cv::Feature2D> detector;
	switch (type)
	{
	case KeyPointType_e::ORB:
		detector = cv::ORB::create();
		break;
	case KeyPointType_e::SIFT:
		detector = cv::SIFT::create();
		break;
	default:
		return false;
		break;
	}

	std::cout << "Extracting" << " features..." << std::endl;
	
	Feature.reserve(result.size());
	int count = 0;
	for (auto& item : result)
	{
		cv::Mat img = cv::imread(item);
		if (img.empty()) continue;
		
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
		detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
		if (descriptors.empty()) continue;
		
		Feature.push_back(descriptors);
		
		showProcess(count++, result.size());
	}
	return !Feature.empty();
}


bool BuildTreeORB(std::string OutPath, std::vector<cv::Mat>& Features, OrbVocabulary& ORBtree)
{
	std::vector<std::vector<cv::Mat>> FeaturesVec;
	FeaturesVec.reserve(Features.size());
	for (auto& plain : Features)
	{
		std::vector<cv::Mat> out;
		out.resize(plain.rows);
		for (int i = 0; i < plain.rows; ++i)
		{
			out[i] = plain.row(i);
		}
		FeaturesVec.push_back(out);
	}

	ORBtree.create(FeaturesVec);
	std::cout << "... done!" << std::endl;
	std::cout << "Vocabulary information: " << std::endl
		<< ORBtree << std::endl << std::endl;
	
	std::cout << "Saving vocabulary..." << std::endl;
	OrbDatabase db(ORBtree, false, 0);
	for (int i = 0; i < FeaturesVec.size(); i++)
	{
		db.add(FeaturesVec[i]);
	}
	db.save(OutPath);
	std::cout << "... done!" << std::endl;
	
	std::cout << "Database information: " << std::endl << db << std::endl;
	DBoW2::QueryResults ret;
	for (int i = 0; i < FeaturesVec.size(); i++)
	{
		db.query(FeaturesVec[i], ret, 4);

		// ret[0] is always the same image in this case, because we added it to the 
		// database. ret[1] is the second best match.

		std::cout << "Searching for Image " << i << ". " << ret << std::endl;
	}

	std::cout << std::endl;

	return true;
}


bool BuildTreeSift(std::string OutPath, std::vector<cv::Mat>& Features, Sift128Vocabulary& Sifttree)
{ 
	std::vector<std::vector<DBoW2::FSift128::TDescriptor>> FeaturesVec;
	FeaturesVec.reserve(Features.size());
	
	for (auto& item : Features)
	{
		std::vector<DBoW2::FSift128::TDescriptor> out;
		DBoW2::FSift128::fromMat32F(item, out);
		FeaturesVec.push_back(out);
	}
	
	Sifttree.create(FeaturesVec);
	std::cout << "... done!" << std::endl;
	std::cout << "Vocabulary information: " << std::endl
		<< Sifttree << std::endl << std::endl;

	std::cout << "Saving vocabulary..." << std::endl;
	Sift128Database db(Sifttree, false, 0);
	for (int i = 0; i < FeaturesVec.size(); i++)
	{
		db.add(FeaturesVec[i]);
	}
	db.save(OutPath);
	std::cout << "... done!" << std::endl;

	std::cout << "Database information: " << std::endl << db << std::endl;
	DBoW2::QueryResults ret;
	for (int i = 0; i < FeaturesVec.size(); i++)
	{
		db.query(FeaturesVec[i], ret, 4);

		// ret[0] is always the same image in this case, because we added it to the 
		// database. ret[1] is the second best match.

		std::cout << "Searching for Image " << i << ". " << ret << std::endl;
	}

	std::cout << std::endl;

	return true;
}
