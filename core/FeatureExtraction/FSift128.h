#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <DBoW2/FClass.h>

/**
 * @brief DBoW2 namespace.
 */
namespace DBoW2 {

    /// Functions to manipulate SIFT descriptors
    class FSift128 : protected FClass
    {
    public:

        /// Descriptor type
        using TDescriptor = std::vector<float>;
        /// Pointer to a single descriptor
        using pDescriptor = const TDescriptor*;
        /// Descriptor length
        static constexpr int L = 128;

        /**
         * @brief Returns the number of dimensions of the descriptor space
         * @return dimensions
         */
        constexpr static int dimensions()
        {
            return L;
        }

        /**
         * @brief Calculates the mean value of a set of descriptors
         * @param descriptors vector of pointers to descriptors
         * @param mean mean descriptor
         */
        static void meanValue(const std::vector<pDescriptor>& descriptors,
            TDescriptor& mean);

        /**
         * @brief Calculates the (squared) distance between two descriptors
         * @param a
         * @param b
         * @return (squared) distance
         */
        static double distance(const TDescriptor& a, const TDescriptor& b);

        /**
         * @brief Returns a string version of the descriptor
         * @param a descriptor
         * @return string version
         */
        static std::string toString(const TDescriptor& a);

        /**
         * @brief Returns a descriptor from a string
         * @param a descriptor
         * @param s string version
         */
        static void fromString(TDescriptor& a, const std::string& s);

        /**
         * @brief Returns a mat with the descriptors in float format
         * @param descriptors
         * @param mat (out) NxL 32F matrix
         */
        static void toMat32F(const std::vector<TDescriptor>& descriptors,
            cv::Mat& mat);

        /**
         * @brief construct descriptors with the mat in float format.
         * @param mat (in) NxL 32F matrix
         * @param descriptors
         */
        static void fromMat32F(const cv::Mat& mat, std::vector<TDescriptor>& descriptors);

        /**
         * @brief This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.
         */
        static std::vector<TDescriptor> fromMat32F(const cv::Mat& mat);
    };

} // namespace DBoW2

