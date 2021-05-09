#include "data_process/image_process.h"


using namespace cv;

Mat Vec2Mat(uchar array[][MAP_COL_NUM], int row, int col)
{
 
    Mat img(row ,col,  CV_8UC1);
    uchar *ptmp = NULL;
    for (int i = 0; i <row; ++i)
    {
        ptmp = img.ptr<uchar>(i);
 
        for (int j = 0; j < col; ++j)
        {
            ptmp[j] = array[i][j];
        }
    }
    
    return img;
}
/*
uchar** Mat2Vec(Mat mat)
{
    
    uchar **array = new uchar*[mat.rows];
    for (int i = 0; i<mat.rows; ++i)
        array[i] = new uchar[mat.cols];
    for (int i = 0; i < mat.rows; ++i)
    {
        for (int j = 0; j < mat.cols; ++j)
        {
            array[i][j] = mat.at<uchar>(i, j);
        }
    }
 
    return array;
}*/