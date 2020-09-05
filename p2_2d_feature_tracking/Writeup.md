# Camera Based 2D Feature Tracking

<img src="imgages/matched_keypoints.png" width="820" height="248" />

## [Rubric Points](https://review.udacity.com/#!/rubrics/2549/view)
---
### 1. Data Buffer

#### MP.1 Data Buffer Optimization
* Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

```c++

```

```c++

```
### 2. Keypoints

#### MP.2 Keypoint Detection
* Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

```c++

```

```c++

```

#### MP.3 Keypoint Removal
* Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

```c++

```

### 3. Descriptors

#### MP.4 Keypoint Descriptors
* Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

```c++
 
```

```c++

```

#### MP.5 Descriptor Matching
* Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

```c++

```
#### MP.6 Descriptor Distance Ratio
* Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

```c++
```

```c++

```

### 4. Performance
---
#### MP.7 Performance Evaluation 1
* Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

Detector  | Number of keypoints
--------  | -------------------
SHITOMASI | x-x
HARRIS    | x-x
FAST      | x-x
BRISK     | x-x
ORB       | x-x
AKAZE     | x-x
SIFT      | x-x

#### MP.8 Performance Evaluation 2
* Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

Detector  | Descriptor  | Number of matched keypoints
--------  | ----------- | ---------------------------
SHITOMASI | BRISK       | x-x
HARRIS    | BRISK       | x-x
FAST      | BRISK       | x-x
BRISK     | BRISK       | x-x
ORB       | BRISK       | x-x
AKAZE     | BRISK       | x-x
SIFT      | BRISK       | x-x
SHITOMASI | BRIEF       | x-x
HARRIS    | BRIEF       | x-x
FAST      | BRIEF       | x-x
BRISK     | BRIEF       | x-x
ORB       | BRIEF       | x-x
AKAZE     | BRIEF       | x-x
SIFT      | BRIEF       | x-x
SHITOMASI | ORB         | x-x
HARRIS    | ORB         | x-x
FAST      | ORB         | x-x
BRISK     | ORB         | x-x
ORB       | ORB         | x-x
AKAZE     | ORB         | x-x
SIFT      | ORB         | x-x
SHITOMASI | FREAK       | x-x
HARRIS    | FREAK       | x-x
FAST      | FREAK       | x-x
BRISK     | FREAK       | x-x
ORB       | FREAK       | x-x
AKAZE     | FREAK       | x-x
SIFT      | FREAK       | x-x
SHITOMASI | SIFT        | x-x
HARRIS    | SIFT        | x-x
FAST      | SIFT        | x-x
BRISK     | SIFT        | x-x
ORB       | SIFT        | x-x
AKAZE     | SIFT        | x-x
SIFT      | SIFT        | x-x

#### MP.9 Performance Evaluation 3
* Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

Detector     | Descriptor  | Avg. Number of Keypoints | Avg. Time (ms)
------------ | ----------- | ------------------------ | --------
FAST-BRIEF   |             | x                        |  
FAST-ORB     |             | x                        |  
FAST-SIFT    |             | x                        | 

Rank  |  Detector-Descriptor | Avg. Number of Keypoints | Avg. Time (ms)
----- | -------------------- | ------------------------ | --------
1     | x-x                  | x                        | x
2     | x-x                  | x                        | x 
3     | x-x                  | x                        | x
