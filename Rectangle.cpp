#include "Rectangle.h"

Rectangle::Rectangle(double _center_x,double _center_y,double _center_phi,double _length,double _width){
    //******************************//
    //_center_x:矩形中心点x坐标
    //_center_y:矩形中心点y坐标
    //_center_phi:矩形主轴与x轴正向的航向角，单位:rad
    //_length:矩形主轴方向的边长
    //_width:矩形副轴方向的边长
    //******************************//
    // 矩形基本信息
    length=_length>0 ? _length:0;
    width=_width>0 ? _width:0;
    head=_center_phi;

    // 矩形中心点位姿
    center_point=Mat::zeros(1,2,CV_64FC1);
    center_point.at<double>(0,0)=_center_x;
    center_point.at<double>(0,1)=_center_y;

    // 主轴单位向量
    unit_axis_vector=Mat::zeros(2,2,CV_64FC1);
    unit_axis_vector.at<double>(0,0)=cos(head);
    unit_axis_vector.at<double>(0,1)=sin(head);

    // 副轴单位向量(且负轴正方向由主轴逆时针旋转度而来)
    unit_axis_vector.at<double>(1,0)=unit_axis_vector.at<double>(0,1);
    unit_axis_vector.at<double>(1,1)=-unit_axis_vector.at<double>(0,0);
    normalize(unit_axis_vector.row(1),unit_axis_vector.row(1),1,0,NORM_L2);

    Mat tmp_first_unit_axis_vector=unit_axis_vector.row(0).clone();
    hconcat(tmp_first_unit_axis_vector,Mat::zeros(1,1,CV_64FC1),tmp_first_unit_axis_vector);
    Mat tmp_second_unit_axis_vector=unit_axis_vector.row(1).clone();
    hconcat(tmp_second_unit_axis_vector,Mat::zeros(1,1,CV_64FC1),tmp_second_unit_axis_vector);

    if(tmp_first_unit_axis_vector.cross(tmp_second_unit_axis_vector).at<double>(0,2)<0){
        unit_axis_vector.row(1)=-unit_axis_vector.row(1);
    }

    //求顶点坐标（逆时针）
    vertex=Mat::zeros(4,2,CV_64FC1);
    vertex.row(0)=center_point+0.5*length*unit_axis_vector.row(0)+0.5*width*unit_axis_vector.row(1);
    vertex.row(1)=center_point-0.5*length*unit_axis_vector.row(0)+0.5*width*unit_axis_vector.row(1);
    vertex.row(2)=center_point-0.5*length*unit_axis_vector.row(0)-0.5*width*unit_axis_vector.row(1);
    vertex.row(3)=center_point+0.5*length*unit_axis_vector.row(0)-0.5*width*unit_axis_vector.row(1);

}

bool Rectangle::Overlap(const Rectangle& obj){      //由超平面分割定理得出
    for(int i=0;i<4;i++){
        bool flag=true;
        for(int j=0;j<4;j++){
            if (Atleft(vertex.row(i),vertex.row((i+1)%4),obj.vertex.row(j))){
                flag=false;
                break;
            }
        }
        if (flag==true) return false;
    }

    for(int i=0;i<4;i++){
        bool flag=true;
        for(int j=0;j<4;j++){
            if (Atleft(obj.vertex.row(i),obj.vertex.row((i+1)%4),vertex.row(j))){
                flag=false;
                break;
            }
        }
        if (flag==true) return false;
    }
    return true;
}

bool Rectangle::Atleft(const Mat& _p_start_point,const Mat& _p_end_point,const Mat& _point){       //判断点在向量的左侧,或位于向量上
    Mat m;
    vconcat(_p_start_point,_p_end_point,m);
    vconcat(m,_point,m);
    hconcat(m,Mat::ones(3,1,CV_64FC1),m);
    return determinant(m)>=0 ? true:false;
}
