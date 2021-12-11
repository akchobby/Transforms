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
    Quaternion test_q2(v2);
    cout << "q2 --  " << test_q2 << endl;

    // Test rvec 

    // Test euler angles

    // Test Rotqtion matrix

    return 0;

}