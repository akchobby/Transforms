/** @file quaternion_main.cpp
 * @brief A test program to check the various functionalities of the quaternion class
 *
 * @author Anil Kumar Chavali
 * 
 */

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
    Quaternion test_q3(v3, true);// expected output will be value as it is a point
    cout << "q3 --  " << test_q3 << endl;
    

    Quaternion test_q4(test_q3);
    cout << "q4 --  " << test_q4 << endl; // copy point

    Quaternion test_q5(test_q2);
    cout << "q5 --  " << test_q5 << endl; // copy rotation

    // ----- operations
    Quaternion test_q6 = test_q1 + test_q3 ;
    cout << "q6 --  " << test_q6 << endl; // addition

    Quaternion test_q7 = test_q1 - test_q3 ;
    cout << "q7 --  " << test_q7 << endl; // subtraction

    Quaternion test_q8 = test_q1 * test_q3 ;
    cout << "q8 --  " << test_q8 << endl; // multiplication

    Quaternion test_q9 = test_q3 / 2.0 ;
    cout << "q9 --  " << test_q9 << endl; // scalar division

    // Test Rotation matrix
    Eigen::Matrix3d m1;
    m1 << 0.7071, 0.5,     0.5,
          0.0,    0.7071, -0.7071,
         -0.7071, 0.5,     0.5;

    Quaternion test_q10(m1);
    cout << "q10 --  " << test_q10 << endl; // x:0.3535533905932738, y:0.3535533905932738, z:-0.14644660940672624, w:0.8535533905932737 , 
   
   // Test euler angles
    Eigen::Vector3d v4;
    v4 << 0.0, M_PI/2, M_PI/2;

    Quaternion test_q11(v4);
    cout << "q11 --  " << test_q11 << endl;

    cout << "q13 --  \n" << test_q10.getEulerAngles() << endl; //euler x45,y45,z0

    Eigen::Vector3d v5;
    v5 << 0.0, 45.5,  45.0;

    Quaternion test_q14(v5, false, true);
    cout << "q14 --  \n" << test_q14.getEulerAngles(true) << endl; //euler x45,y45.5,z0


    // Test rvec 


    Quaternion test_q12 = test_q11.inv();
    cout << "q12 --  " << test_q12 << endl;

    

    return 0;

}