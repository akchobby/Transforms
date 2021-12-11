#pragma once

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <Eigen/Core>

using namespace std;


namespace transforms {

    class Quaternion{
        public:
            
            // create quaternion --- a vector case

            //default
            Quaternion();

            // quaternion vector 
            Quaternion(Eigen::Vector4d& v, bool point);
            Quaternion(Eigen::Vector4d& v);

            // euler angles or rvec
            Quaternion(Eigen::Vector3d& v, bool deg, bool rvec, bool point);


            // create quaternion --- a matrix case
            Quaternion(Eigen::Matrix3d& m);


            ~Quaternion();


            // Operaters ---------

            friend ostream& operator<<(ostream& os, const Quaternion& q);


            // Getters -----------
            void getRotationMatrix(Eigen::Matrix3d& m);

            // conversions
            void getQuaternion(Eigen::Vector4d& v);
            void getEulerAngles(Eigen::Vector3d& v);


        private:
            // quaternion array
            Eigen::Vector4d q;

            // properties
            double magnitude; 
            bool point = false;

    };


}