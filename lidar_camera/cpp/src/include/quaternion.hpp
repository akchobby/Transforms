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
            ~Quaternion();

            //copy constructor
            Quaternion(const Quaternion& q);

            // quaternion vector 
            Quaternion(const Eigen::Vector4d& v,  bool point = false);
            Quaternion(const Eigen::Vector3d& v); // for a point

            // euler angles or rvec
            Quaternion(const Eigen::Vector3d& v, bool deg, bool rvec);


            // create quaternion --- a matrix case
            Quaternion(const Eigen::Matrix3d& m);


            


            // Operaters ---------

            friend ostream& operator<<(ostream& os, const Quaternion& q);

            Quaternion operator+(const Quaternion& rhs_q);
            Quaternion operator-(const Quaternion& rhs_q);
            Quaternion operator*(const Quaternion& rhs_q);
            Quaternion operator/(const double& scalar);


            // Getters -----------
            void getRotationMatrix(Eigen::Matrix3d& m);

            // conversions
            Eigen::Vector4d getQuaternion() const;
            bool getPoint() const;

            void getEulerAngles(Eigen::Vector3d& v);


        private:
            // quaternion array
            Eigen::Vector4d q;

            // properties
            double magnitude; 
            bool point = false;

    };


}