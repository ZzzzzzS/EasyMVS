
#include <iostream>
#include <vector>

// DBoW2
//#include "DBoW2.h" // defines OrbVocabulary and OrbDatabase
#include <DBoW2/DBoW2.h>
//#include <DBoW2/FSurf64.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>

#include "FeatureExtraction/FSift128.h"
using Sift128Vocabulary = DBoW2::TemplatedVocabulary<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;
using Sift128Database = DBoW2::TemplatedDatabase<DBoW2::FSift128::TDescriptor, DBoW2::FSift128>;

using namespace DBoW2;
using namespace std;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void loadFeatures(vector<vector<vector<float>>>& features);
void changeStructure(const cv::Mat& plain, vector<vector<float>>& out);
void testVocCreation(const vector<vector<vector<float> > >& features);
void testDatabase(const vector<vector<vector<float> > >& features);


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// number of training images
const int NIMAGES = 10;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
    cout << endl << "Press enter to continue" << endl;
    getchar();
}

// ----------------------------------------------------------------------------

int main()
{
    vector<vector<vector<float>>> features;
    loadFeatures(features); // 提取SIFT特征

    testVocCreation(features); // 特征转换为词袋树

    wait();

    testDatabase(features); //词袋树转换为数据库

    return 0;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

void loadFeatures(vector<vector<vector<float>>>& features)
{
    features.clear();
    features.reserve(NIMAGES);

    cv::Ptr<cv::SIFT> orb = cv::SIFT::create();

    cout << "Extracting SIFT features..." << endl;
    for (int i = 0; i < NIMAGES; ++i)
    {
        stringstream ss;
        if (i < 4)
            ss << "images/image" << i << ".png";
        else
            ss << "images/IMG_0" << i + 557 << " (small).JPG";

        cv::Mat image = cv::imread(ss.str(), 0);
        cv::Mat mask;
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        orb->detectAndCompute(image, mask, keypoints, descriptors);

        features.push_back(vector<vector<float> >());
        FSift128::fromMat32F(descriptors, features.back());
    }
}

void changeStructure(const cv::Mat& plain, vector<vector<float>>& out)
{
    out.resize(plain.rows);
    for (size_t i = 0; i < plain.rows; i++)
    {
        out[i].reserve(plain.cols);
        for (size_t j = 0; j < plain.cols; j++)
        {
            out[i].push_back(plain.at<float>(i, j));
        }
    }
}

// ----------------------------------------------------------------------------

void testVocCreation(const vector<vector<vector<float>> >& features)
{
    // branching factor and depth levels 
    const int k = 9;
    const int L = 3;
    const WeightingType weight = TF_IDF;
    const ScoringType scoring = L2_NORM;

    Sift128Vocabulary voc(k, L, weight, scoring);

    cout << "Creating a small " << k << "^" << L << " vocabulary..." << endl;
    voc.create(features);
    cout << "... done!" << endl;

    cout << "Vocabulary information: " << endl
        << voc << endl << endl;

    // lets do something with this vocabulary
    cout << "Matching images against themselves (0 low, 1 high): " << endl;
    BowVector v1, v2;
    for (int i = 0; i < NIMAGES; i++)
    {
        voc.transform(features[i], v1);
        for (int j = 0; j < NIMAGES; j++)
        {
            voc.transform(features[j], v2);

            double score = voc.score(v1, v2);
            cout << "Image " << i << " vs Image " << j << ": " << score << endl;
        }
    }

    // save the vocabulary to disk
    cout << endl << "Saving vocabulary..." << endl;
    voc.save("small_voc.yml.gz");
    cout << "Done" << endl;
}

// ----------------------------------------------------------------------------

void testDatabase(const vector<vector<vector<float>> >& features)
{
    cout << "Creating a small database..." << endl;

    // load the vocabulary from disk
    Sift128Vocabulary voc("small_voc.yml.gz");

    Sift128Database db(voc, false, 0); // false = do not use direct index
    // (so ignore the last param)
    // The direct index is useful if we want to retrieve the features that 
    // belong to some vocabulary node.
    // db creates a copy of the vocabulary, we may get rid of "voc" now

    // add images to the database
    for (int i = 0; i < NIMAGES; i++)
    {
        db.add(features[i]);
    }

    cout << "... done!" << endl;

    cout << "Database information: " << endl << db << endl;

    // and query the database
    cout << "Querying the database: " << endl;

    QueryResults ret;
    for (int i = 0; i < NIMAGES; i++)
    {
        db.query(features[i], ret, 4);

        // ret[0] is always the same image in this case, because we added it to the 
        // database. ret[1] is the second best match.

        cout << "Searching for Image " << i << ". " << ret << endl;
    }

    cout << endl;

    // we can save the database. The created file includes the vocabulary
    // and the entries added
    cout << "Saving database..." << endl;
    db.save("small_db.yml.gz");
    cout << "... done!" << endl;

    // once saved, we can load it again  
    cout << "Retrieving database once again..." << endl;
    Sift128Database db2("small_db.yml.gz");
    cout << "... done! This is: " << endl << db2 << endl;
}

// ----------------------------------------------------------------------------

