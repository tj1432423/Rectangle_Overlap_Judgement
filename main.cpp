# include <iostream>
# include "Rectangle.h"
//# include <math.h>


using namespace std;


int main(){
    Rectangle rec(2,2,0,4,4);
    Rectangle A(4,4,-0.1,0,0);
    //Rectangle B(18,18,0,2,2);

    cout<<rec.Overlap(A)<<endl;
    cout<<A.Overlap(rec)<<endl;

//    cout<<rec.unit_axis_vector<<endl;
//    cout<<"-----------"<<endl;
//    cout<<rec.center_point<<endl;
//    cout<<"-----------"<<endl;
//    cout<<rec.vertex<<endl;
//    cout<<"-----------"<<endl;

//    cout<<M_PI<<endl;
    return 0;
}


