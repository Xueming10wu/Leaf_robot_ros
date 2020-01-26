
#include <iostream>
#include <math.h>

using namespace std;

int main(int argc, char *argv[])
{
    double ex = 0;
    double ey = 0;
    double ez = 0;

    double x=0;
    double y=0;
    double z=0;
    double w=0;

    cout << "Roll Pitch Taw (Z Y X) 弧度制 :\n";
    cin >> ez >> ey >> ex;


    double cy = cos(ex * 0.5);
    double sy = sin(ex * 0.5);

    double cp = cos(ey * 0.5);
    double sp = sin(ey * 0.5);
    
    double cr = cos(ez * 0.5);
    double sr = sin(ez * 0.5);

    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cr * sp * cr;
    z = sy * cp * cr  - cy * sp * sr;

    cout << "x  y  z  w    "<< x << "  " << y << "  " << z << "  " << w << endl;


    return 0;
}