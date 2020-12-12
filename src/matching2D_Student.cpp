#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {

        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF matching cross-check=" << crossCheck;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        cout << "FLANN matching";
    }
    else{
        throw invalid_argument(matcherType + " not among options-[MAT_BF, MAT_FLANN]");
    }

    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches; 

        // TODO : implement k-nearest-neighbor matching
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (kNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;

    }
    else{
        throw invalid_argument(selectorType+ " not among options-[SEL_KNN, SEL_NN]");
    }

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("ORB") == 0){
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("AKAZE") == 0){
        extractor = cv::AKAZE::create();
    }

    else if (descriptorType.compare("BRIEF") == 0){
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }

    else if (descriptorType.compare("FREAK") == 0){
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("SIFT") == 0){
       // extractor = cv::xfeatures2d::SIFT::create();
    }    
    
    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){
    // Detector parameters for harris
    int blockSize = 2; // neighbourhood for every pixel considered
    int apertureSize = 3; // for sobel gradient 
    int minResponse = 100; // minium cornerness response for flitering
    double k = 0.04; // Harris parameter

    // Detect Harris corners and normalize the output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    
    // Initialize storage
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    double t = (double)cv::getTickCount();

    // call on the harris function
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

    // normalize the response between [0, 255]
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

    // calculate absolute vaules and convert to 8bit
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // permissible overlap between features
    double max_overlap = 0.0; 

    for (size_t j = 0; j < img.rows; j++){
        for (size_t i = 0; i < img.cols; i++){
            // obtain response and see if above threshold
            int response = (int) dst_norm.at<float>(j, i);

            if (response > minResponse){
                // store the keypoint
                cv::KeyPoint newKeyPoint;

                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;

                // non maxima suppression
                bool overlap = false;

                // loop through all stored keypoints
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it){
                    // calculate overlap between key points
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);

                    if (kptOverlap > max_overlap){
                        // we have overlapping keypoints
                        overlap = true;

                        // pick the stronger intensity
                        if (newKeyPoint.response > (*it).response){
                            *it = newKeyPoint;
                            break;
                        }
                    }
                }

                // if no overlap push current point to they keypoint
                if(!overlap){
                    keypoints.push_back(newKeyPoint);

                }
            }

        }

    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }

}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis){
    //initialize pointer for detector types
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType.compare("FAST") == 0){
    	int threshold = 30; 
        detector = cv::FastFeatureDetector::create(threshold=threshold);
    }

    else if (detectorType.compare("ORB") == 0){
        detector = cv::ORB::create();
    }

    else if (detectorType.compare("BRISK") == 0){
        detector = cv::BRISK::create();
    }

    else if (detectorType.compare("AKAZE") == 0){
        detector = cv::AKAZE::create();
    }

    else if (detectorType.compare("SIFT") == 0){
       // detector = cv::xfeatures2d::SIFT::create();
    }

    else{
        throw invalid_argument(detectorType + " not among options-[FAST, ORB, BRISK, AKAZE, SIFT]");
    }

    double t = (double)cv::getTickCount();
    // extract keypoints
    detector->detect(img, keypoints);
    t=((double)cv::getTickCount()-t)/cv::getTickFrequency();
    cout << detectorType<<" detection with n =" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    if(bVis)
    {
        cv::Mat visImage=img.clone();
        cv::drawKeypoints(img,keypoints,visImage,cv::Scalar::all(-1),cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName=detectorType+" keypoint detection results";
        cv::namedWindow(windowName,6);
        cv::imshow(windowName,visImage);
        cv::waitKey(0);
    }

}