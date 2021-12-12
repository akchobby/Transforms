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

    // ----- operations
    Quaternion test_q6 = test_q1 + test_q3 ;
    cout << "q6 --  " << test_q6 << endl;

    Quaternion test_q7 = test_q1 - test_q3 ;
    cout << "q7 --  " << test_q7 << endl;

    Quaternion test_q8 = test_q1 * test_q3 ;
    cout << "q8 --  " << test_q8 << endl;

    Quaternion test_q9 = test_q3 / 2.0 ;
    cout << "q9 --  " << test_q9 << endl;


    // Test rvec 

    // Test euler angles

    // Test Rotqtion matrix

    return 0;

}