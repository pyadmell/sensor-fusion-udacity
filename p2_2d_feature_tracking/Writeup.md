# Camera Based 2D Feature Tracking

<img src="images/matched_keypoints.png" width="820" height="248" />

## [Rubric Points](https://review.udacity.com/#!/rubrics/2549/view)
---
### 1. Data Buffer

#### MP.1 Data Buffer Optimization
- Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

```cpp
// push image into data frame buffer
DataFrame frame;
frame.cameraImg = imgGray;
if(dataBuffer.size() == dataBufferSize)
{ 
  dataBuffer.erase(dataBuffer.begin());
}
dataBuffer.push_back(frame);
```

### 2. Keypoints

#### MP.2 Keypoint Detection
- Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

```cpp
if (detectorType.compare("SHITOMASI") == 0)
{
    // SHITOMASI
    detKeypointsShiTomasi(keypoints, imgGray, false);
}
else if (detectorType.compare("HARRIS") == 0)
{
   // HARRIS
   detKeypointsHarris(keypoints, imgGray, false);
}
else
{
   // Other methods: FAST, BRISK, ORB, AKAZE, SIFT
   detKeypointsModern(keypoints, imgGray, detectorType, false);
}
```

```cpp
// Detect keypoints in image using FAST, BRISK, ORB, AKAZE, and SIFT
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;
    if (detectorType.compare("FAST") == 0)
    {
        detector = cv::FastFeatureDetector::create(30, true, cv::FastFeatureDetector::TYPE_9_16);
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        detector = cv::xfeatures2d::SIFT::create();
    }
    else
    {
        std::cout << "Invalid keypoints detector" << std::endl;
    }

    double t = (double) cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << detectorType << " detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;

    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + "Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using Harris
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, const bool bVis)
{
    int blockSize = 2;
    int apertureSize = 3;
    int minResponse = 100;
    double k = 0.04;
    double overlapThreshold = 0.0;
    int scaledApertureSize = 2*apertureSize;
    cv::Mat dst, dstNorm, dstNormScaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);

    double t = (double) cv::getTickCount();
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dstNorm, dstNormScaled);

    for (size_t i=0; i<dstNorm.rows; ++i) 
    {
        for (size_t j=0; j<dstNorm.cols; ++j) 
        {
            int response = (int)dstNorm.at<float>(i,j);
            if (response > minResponse) 
            {
                cv::KeyPoint point;
                point.pt = cv::Point2f(i, j);
                point.size = scaledApertureSize;
                point.response = response;
                point.class_id = 0;
                bool isOverlapping = false;
                for (auto &keypoint: keypoints) 
                {
                    if (cv::KeyPoint::overlap(point, keypoint) > overlapThreshold) 
                    {
                        isOverlapping = true;
                        if (point.response > keypoint.response) 
                        {
                            keypoint = point;
                            break;
                        }
                    }
                }
                if (!isOverlapping) 
                { 
                  keypoints.push_back(point); 
                }
            }
        }
    }

    t = ((static_cast<double>(cv::getTickCount())) - t) / cv::getTickFrequency();
    std::cout << "Harris detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << std::endl;
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
```

#### MP.3 Keypoint Removal
- Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

```cpp
// only keep keypoints on the preceding vehicle
bool bFocusOnVehicle = true;
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle)
{
	vector<cv::KeyPoint> roiKeypoints;
	for (const auto &point: keypoints)
	{
		if (vehicleRect.contains(point.pt))
		{
			roiKeypoints.push_back(point);
		}
	}
	keypoints = roiKeypoints;
}
```

### 3. Descriptors

#### MP.4 Keypoint Descriptors
- Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

```cpp
cv::Mat descriptors;
string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT
descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
```

```cpp
else if (descriptorType.compare("BRIEF") == 0)
{
	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
}
else if (descriptorType.compare("ORB") == 0)
{
	extractor = cv::ORB::create();
}
else if (descriptorType.compare("FREAK") == 0)
{
	extractor = cv::xfeatures2d::FREAK::create();
}
else if (descriptorType.compare("AKAZE") == 0)
{
	extractor = cv::AKAZE::create();
}
else if (descriptorType.compare("SIFT") == 0)
{
	extractor = cv::xfeatures2d::SIFT::create();
}
else
{
	std::cout<<"Invalid descriptor!"<<std::endl;
}
```

#### MP.5 Descriptor Matching
- Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

```cpp
/ Convert binary descriptors to floating point
if (descSource.type() != CV_32F)
{ 
	descSource.convertTo(descSource, CV_32F);
}

if(descRef.type() != CV_32F)
{
	descRef.convertTo(descRef, CV_32F);
}

matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

...

std::vector<std::vector<cv::DMatch>> knnMatches;
matcher->knnMatch(descSource, descRef, knnMatches, 2);
```
#### MP.6 Descriptor Distance Ratio
- Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

```cpp
float threshold = 0.8;
for (const auto match: knnMatches) 
{
	if (match[0].distance < (threshold * match[1].distance))
	{
		matches.push_back(match[0]);
	}
}
```

### 4. Performance
---
#### MP.7 Performance Evaluation 1
- Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
- Number of all keypoints and keypoints in ROI are provided in [MP_7_ALL.csv](MP_7_ALL.csv) and [MP_7_ROI.csv](MP_7_ROI.csv) files for each image. The following table summarize the results in terms of rounded average number ROI keypoints and standard deviation.

Detector  | AVG. number of keypoints | Standard deviation
--------  | ------------------------ | -------------------
SHITOMASI | 118                      | 5
HARRIS    | 25                       | 9
FAST      | 149                      | 5
BRISK     | 276                      | 13
ORB       | 116                      | 14
AKAZE     | 167                      | 8
SIFT      | 139                      | 9

#### MP.8 Performance Evaluation 2
- Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
- Number of matched keypoints for all images are provided in [MP_8.csv](MP_8.csv) files for each image. The following table summarize the results in terms of rounded average number matched keypoints and standard deviation.

Detector  | Descriptor  | Avg. number of matched keypoints | Standard deviation
--------  | ----------- | -------------------------------- | ------------------
SHITOMASI | BRISK       | 85                               | 5
SHITOMASI | BRIEF       | 105                              | 5
SHITOMASI | ORB         | 101                              | 3
SHITOMASI | FREAK       | 85                               | 3
SHITOMASI | SIFT        | 88                               | 5
HARRIS    | BRISK       | 13                               | 4
HARRIS    | BRIEF       | 16                               | 3
HARRIS    | ORB         | 16                               | 4
HARRIS    | FREAK       | 14                               | 3
HARRIS    | SIFT        | 18                               | 5
FAST      | BRISK       | 100                              | 7
FAST      | BRIEF       | 122                              | 7
FAST      | ORB         | 119                              | 6
FAST      | FREAK       | 98                               | 6
FAST      | SIFT        | 247                              | 10
BRISK     | BRISK       | 174                              | 9
BRISK     | BRIEF       | 189                              | 11
BRISK     | ORB         | 168                              | 8
BRISK     | FREAK       | 169                              | 9
BRISK     | SIFT        | 151                              | 10
ORB       | BRISK       | 83                               | 7
ORB       | BRIEF       | 61                               | 14
ORB       | ORB         | 85                               | 12
ORB       | FREAK       | 47                               | 6
ORB       | SIFT        | 57                               | 10
AKAZE     | BRISK       | 135                              | 8
AKAZE     | BRIEF       | 141                              | 9
AKAZE     | ORB         | 131                              | 8
AKAZE     | FREAK       | 132                              | 9
AKAZE     | SIFT        | 120                              | 11
SIFT      | BRISK       | 66                               | 6
SIFT      | BRIEF       | 78                               | 7
SIFT      | ORB         | 61                               | 6
SIFT      | FREAK       | 66                               | 6
SIFT      | SIFT        | 69                               | 7

#### MP.9 Performance Evaluation 3
- Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
- The time log for all combinations and all images are provided in [MP_9.csv](MP_9.csv) files. The following table summarize the results in terms of average elapsed time and standard deviation.

Detector | Descriptor | Average (ms) | STD (ms)
-------- | ---------- | ------------ | --------
SHITOMASI | BRISK | 209.71 | 2.03
SHITOMASI | BRIEF | 17.63 | 0.93
SHITOMASI | ORB | 17.87 | 1.07
SHITOMASI | FREAK | 42.71 | 0.70
SHITOMASI | SIFT | 23.63 | 0.48
HARRIS | BRISK | 238.04 | 6.48
HARRIS | BRIEF | 15.08 | 2.38
HARRIS | ORB | 15.53 | 2.42
HARRIS | FREAK | 42.22 | 3.04
HARRIS | SIFT | 25.05 | 2.68
FAST | BRISK | 199.64 | 0.63
FAST | BRIEF | 6.55 | 0.30
FAST | ORB | 6.52 | 0.22
FAST | FREAK | 34.44 | 0.58
FAST | SIFT | 17.73 | 0.48
BRISK | BRISK | 425.15 | 5.47
BRISK | BRIEF | 228.94 | 3.27
BRISK | ORB | 233.87 | 3.39
BRISK | FREAK | 260.44 | 11.45
BRISK | SIFT | 262.15 | 0.74
ORB | BRISK | 204.25 | 0.69
ORB | BRIEF | 10.96 | 0.25
ORB | ORB | 14.75 | 0.42
ORB | FREAK | 38.62 | 0.54
ORB | SIFT | 20.30 | 0.25
AKAZE | BRISK | 254.58 | 3.80
AKAZE | BRIEF | 58.82 | 1.34
AKAZE | ORB | 63.23 | 1.25
AKAZE | FREAK | 85.04 | 2.38
AKAZE | SIFT | 69.44 | 1.18
SIFT | BRISK | 252.77 | 6.52
SIFT | BRIEF | 69.74 | 4.83
SIFT | ORB | 82.05 | 1.00
SIFT | FREAK | 97.50 | 3.78
SIFT | SIFT | 92.53 | 1.32

Rank  |  Detector-Descriptor | Avg. Number of Keypoints | Avg. Time (ms)
----- | -------------------- | ------------------------ | --------
1     | x-x                  | x                        | x
2     | x-x                  | x                        | x 
3     | x-x                  | x                        | x
