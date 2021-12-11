#include <iostream>

#include "quaternion.hpp"

using namespace std;
using namespace transforms;




int main(){
    Quaternion test_q1;
    
    // Test quaternion
    cout << "q1 --  " << test_q1 << endl;

    Eigen::Vector4d v2;
    v2 << 10, 3, 4, 1; 

    Quaternion test_q2(v2); // expected output will be a normalized vector
    cout << "q2 --  " << test_q2 << endl; 

    Eigen::Vector3d v3;
    v3 << 10, 3, 4; 
    Quaternion test_q3(v3);// expected output will be value as it is a point
    cout << "q3 --  " << test_q3 << endl;
    

    Quaternion test_q4(test_q3);
    cout << "q4 --  " << test_q4 << endl;

    Quaternion test_q5(test_q2);
    cout << "q5 --  " << test_q5 << endl;

    // Test rvec 

    // Test euler angles

    // Test Rotqtion matrix

    return 0;

}