/*
*  flow_opencv.hpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#pragma once

#include <iostream>
#include <cv.h>
#include <cmath>

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/video/tracking.hpp"

#include "optical_flow.hpp"
#include "trackFeatures.h"

class OpticalFlowOpenCV : public OpticalFlow {

  private:
    //params which can be set
    int num_features;
    float confidence_multiplier;
    //general
    std::vector<int> updateVector;
    std::vector<cv::Point2f> features_current, features_previous, useless;

  public:

    inline void setNumFeatures( int n_feat ) { num_features = n_feat; };
    inline void setConfMultiplier( float conf_multi ) { confidence_multiplier = conf_multi; };

    inline int getNumFeatures() { return num_features; };
    inline int getConfMultiplier() { return confidence_multiplier; };

    OpticalFlowOpenCV( float f_length_x, float f_length_y, int num_feat = 30, float conf_multi = 1.645f );
    ~OpticalFlowOpenCV();

    int calcFlow( const cv::Mat &img_current, float &flow_x, float &flow_y );

};
