#include <iostream>
#include <Eigen/Core>

using namespace transforms;

// Quaternion 
Quaternion::Quaternion(Eigen::Vector4d& v, bool point = false):point(point), q(v){
   
}

// Euler angles or Rvec
Quaternion::Quaternion(Eigen::Vector3d& v, bool deg = false, bool rvec = false, bool point = false):point(point){

    if (rvec){

    }else{
        
    }

}

// Rotation matrix
Quaternion::Quaternion(Eigen::Matrix3d& m):point(false){

    double trace = m.trace(); 

    if (trace > 0){

    }else{
        
    }

}

// Getters ---------------

void Quaternion::getRotationMatrix(Eigen::Matrix3d& m){


}

void Quaternion::getQuaternion(Eigen::Vector4d& v){
    v=q;
}

