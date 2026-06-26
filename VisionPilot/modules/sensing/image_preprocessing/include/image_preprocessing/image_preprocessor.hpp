#ifndef VISIONPILOT_IMAGE_PREPROCESSOR_HPP
#define VISIONPILOT_IMAGE_PREPROCESSOR_HPP
#include <string>
#include <opencv2/opencv.hpp>

class ImagePreprocessor {
public:
    ImagePreprocessor();

    ~ImagePreprocessor() = default;

    void preprocess(const cv::Mat &image, cv::Mat &warped_image, cv::Mat &resized_image, const cv::Size &size);

    // C maps raw camera pixel → warped 1024×512 BEV (perspective transform).
    const cv::Mat& C_mat()   const { return C; }
    // Raw frame dimensions seen on the last preprocess() call.
    cv::Size       raw_size() const { return raw_size_; }

private:
    std::string homography_C_matrix_path;
    cv::Mat     C;
    cv::Size    raw_size_;
};

#endif //VISIONPILOT_IMAGE_PREPROCESSOR_HPP
