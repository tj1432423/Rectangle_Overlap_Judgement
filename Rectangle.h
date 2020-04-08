#ifndef RECTANGLE_H
#define RECTANGLE_H


#include <math.h>
#include "opencv2/core.hpp"

using namespace cv;


class Rectangle{
public:
    Rectangle(double _center_x,double _center_y,double _center_phi,double _length,double _width);

    bool Overlap(const Rectangle& obj);

    double length;
    double width;
    double head;

    Mat center_point;   //[center_x,center_y]
    Mat unit_axis_vector; // [first_axis_vector;second_axis_vector]  //副轴与主轴垂直，且符合右手定则
    Mat vertex; // 4个顶点,逆时针


private:
    bool Atleft(const Mat& _p_start_point,const Mat& _p_end_point,const Mat& _point);

};


#endif // RECTANGLE_OVERLAP_H
