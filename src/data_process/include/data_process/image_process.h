#ifndef IMAGE_PROCESS_H__
#define IMAGE_PROCESS_H__

#include <iostream>  
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common_func/algorithm.h"

cv::Mat Vec2Mat(uchar array[][MAP_COL_NUM], int row, int col);

//uchar** Mat2Vec(cv::Mat mat);


#endif
