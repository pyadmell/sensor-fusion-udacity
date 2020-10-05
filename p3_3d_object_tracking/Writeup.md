# 3D Object Tracking

## [Rubric Points](https://review.udacity.com/#!/rubrics/2550/view)
---
### FP.1 Match 3D Objects
> Implement the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

```cpp
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //// STUDENT ASSIGNMENT: FP.1
    size_t prevBoxSize = prevFrame.boundingBoxes.size();
    size_t currBoxSize = currFrame.boundingBoxes.size();
    int matchingBoxScore[prevBoxSize][currBoxSize];
    for(size_t i=0;i<prevBoxSize;++i)
    {
      for(size_t j=0; j<currBoxSize;++j)
      {
        matchingBoxScore[i][j]=0;
      }
    }

    // iterate pnt matchs, cnt box-box match score
    for(const auto &match: matches)
    {
        cv::KeyPoint prevKpnt = prevFrame.keypoints[match.queryIdx];
        cv::Point prevPoint = cv::Point(prevKpnt.pt.x, prevKpnt.pt.y);
        
        // filter points out of roi
        std::vector<size_t> prevBoxIds;
        for(size_t i=0; i<prevBoxSize; ++i)
        {
            if(prevFrame.boundingBoxes[i].roi.contains(prevPoint))
            {
                prevBoxIds.push_back(i);
            }
        }

        cv::KeyPoint currKpnt = currFrame.keypoints[match.trainIdx];
        cv::Point currPoint = cv::Point(currKpnt.pt.x, currKpnt.pt.y);
        
        // filter points out of roi
        std::vector<size_t> currBoxIds;
        for(size_t i=0; i<currBoxSize; ++i)
        {
            if (currFrame.boundingBoxes[i].roi.contains(currPoint))
            {
                currBoxIds.push_back(i);
            }
        }

        // find matching score between prev and curr
        for(auto const &i: prevBoxIds)
        {
            for(auto const &j: currBoxIds)
            {
                matchingBoxScore[i][j] += 1;
            }
        }
    }

    for(size_t i=0; i<prevBoxSize; ++i) 
    {
        auto maxIt = std::max_element(matchingBoxScore[i], matchingBoxScore[i] + currBoxSize);
        bbBestMatches[i] = std::distance(matchingBoxScore[i], maxIt);
    }
   //// EOF STUDENT ASSIGNMENT
}
```

### FP.2 Compute Lidar-based TTC
> Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

```cpp
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //// STUDENT ASSIGNMENT: FP.2
    size_t numInLane = 0;
    auto sumPointsInLane = [&](double sum, const LidarPoint &point)
    {
      double laneWidth = 3.5;
      if(std::fabs(point.y) <= laneWidth/2.0 && point.x>0.0)
      {
        numInLane++;
        return sum+point.x;
      }
      return sum;
    };
    
    numInLane = 0;
    double averageXPrev= std::accumulate(lidarPointsPrev.begin(),lidarPointsPrev.end(), 0.0, sumPointsInLane);
    if(numInLane>0)
    {
        averageXPrev /= static_cast<double>(numInLane);
    }

    numInLane = 0;
    double averageXCurr= std::accumulate(lidarPointsCurr.begin(),lidarPointsCurr.end(), 0.0,
        sumPointsInLane);
    if(numInLane>0)
    {
        averageXCurr /= static_cast<double>(numInLane);
    }

    double dT= 1.0/frameRate;
    TTC = (averageXCurr * dT) / (averageXPrev - averageXCurr);
    //// EOF STUDENT ASSIGNMENT
}
```

### FP.3 Associate Keypoint Correspondences with Bounding Boxes
> Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

```cpp
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //// STUDENT ASSIGMENT: FP.3
    std::vector<double> dist;
    for(const auto &point: kptMatches)
    {
        auto kptCurr = kptsCurr[point.trainIdx];
        if (boundingBox.roi.contains(kptCurr.pt))
        {
            auto &kptPrev = kptsPrev[point.queryIdx];
            dist.push_back(cv::norm(kptCurr.pt - kptPrev.pt));
        }
	  }

    double meanDistance = std::accumulate(dist.begin(), dist.end(), 0.0);
    meanDistance /= static_cast<double>(dist.size());

    for (const auto &point: kptMatches)
    {
        auto kptCurr = kptsCurr[point.trainIdx];
        if (boundingBox.roi.contains(kptCurr.pt))
        { 
            int kptPrevIdx = point.queryIdx;
            auto &kptPrev = kptsPrev[kptPrevIdx];
            if (cv::norm(kptCurr.pt - kptPrev.pt) < 1.3*meanDistance)
            {
                boundingBox.keypoints.push_back(kptCurr);
                boundingBox.kptMatches.push_back(point);
            }
        }
    }
    //// EOF STUDENT ASSIGMENT
}
```

### FP.4 Compute Camera-based TTC
> Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

```cpp
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    //// STUDENT ASSIGNMENT: FP.4
    std::vector<double> distanceRatios;
    double minDistance = 100.0; // Minimum required distance
    for (auto itOut=kptMatches.begin(); itOut!=kptMatches.end()-1; ++itOut)
    {
        cv::KeyPoint kpOuterCurr = kptsCurr[itOut->trainIdx];
        cv::KeyPoint kpOuterPrev = kptsPrev[itOut->queryIdx];
        for (auto itIn=kptMatches.begin()+1; itIn!=kptMatches.end(); ++itIn)
        {
            cv::KeyPoint kpInnerCurr = kptsCurr[itIn->trainIdx];
            cv::KeyPoint kpInnerPrev = kptsPrev[itIn->queryIdx];
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDistance)
            {
                distanceRatios.push_back(distCurr/distPrev);
            }
        }
    }

    if (distanceRatios.empty()) 
    {
        TTC = NAN;
        return;
    }

    std::sort(distanceRatios.begin(), distanceRatios.end());
    size_t medianIndex = distanceRatios.size()/2;
    double medianDistRatio;
    if (distanceRatios.size()%2 == 0)
    {
        medianDistRatio = (distanceRatios[medianIndex - 1] + distanceRatios[medianIndex])/2.0;
    } 
    else
    {
        medianDistRatio = distanceRatios[medianIndex];
    }

    double dT = 1.0/frameRate;
    TTC = -dT/(1.0 - medianDistRatio);
    //// EOF STUDENT ASSIGNMENT
}
```

### FP.5 Performance Evaluation 1

> Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.
- Initially, some TTC estimations were way off, and TTC could go up to 40 s. But the result became more consistent after filtering out the out of lane points. However, some jumps in TTC estimations are still present, for example, in frames (1-4) and (15-18), where TTC changes about 3 seconds between two consecutive frames, increasing from 13 s to 16 s and returning to 14 s, while visually the distance seems to be decreasing. This is more apparent in frames 15-18. The difference might have resulted from the frame rate, which is assumed to be constant.

| Frame 1-2 | Frame 2-3 | Frame 3-4|
|----|----|----|
| ![](./images/performance_evaluation_1/lidar_1.png) | ![](./images/performance_evaluation_1/lidar_2.png) | ![](./images/performance_evaluation_1/lidar_3.png) |

| Frame 15-16 | Frame 16-17 | Frame 17-18|
|----|----|----|
| ![](./images/performance_evaluation_1/lidar_4.png) | ![](./images/performance_evaluation_1/lidar_5.png) | ![](./images/performance_evaluation_1/lidar_6.png) |

### FP.6 Performance Evaluation 2

> Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.
- The results for all detector/descriptor combinations are reported in the [FP_6_Performance_Evaluation_2.csv](./FP_6_Performance_Evaluation_2.csv) spreadsheet.
- In the midterm project, the top 3 detector/descriptor combinations were seletected as follows based on their accuracy and speed. 

| Rank | Detector | Descriptor |
| ---- | -------- | ---------- |
| 1    | FAST     | BRIEF      |
| 2    | FAST     | ORB        |
| 3    | FAST     | SIFT       |

- Comparing the lidar TTC and camera TTC, the above combinations result in the same level of accuracy and consistency. The difference between the camera and lidar estimations is also less compared to other combinations.
- The camera TTC estimation is less consistent and less accurate compared to lidar TTC. It seems that this is related to finding some matched points on the ground or on other vehicles.

| Frame 7-8 | Frame 8-9 | Frame 11-12|
|----|----|----|
| ![](./images/performance_evaluation_2/0000000008.png) | ![](./images/performance_evaluation_2/0000000009.png) | ![](./images/performance_evaluation_2/0000000012.png) |

