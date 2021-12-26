/** @file quaternion.hpp
 * @brief A class under the transforms namespace that allows you to generate a quaternion and 
 * perform all various operations on it.
 *
 * @author Anil Kumar Chavali
 * 
 */
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

            // point, euler angles(ZYX order or ypr) or rvec
            Quaternion(const Eigen::Vector3d& v, bool point = false, bool deg = false, bool rvec = false);


            // create quaternion --- a matrix case

            // rotation matrix
            Quaternion(const Eigen::Matrix3d& m);


            // Operators ---------

            friend ostream& operator<<(ostream& os, const Quaternion& q);

            Quaternion operator+(const Quaternion& rhs_q);
            Quaternion operator-(const Quaternion& rhs_q);
            Quaternion operator*(const Quaternion& rhs_q);
            Quaternion operator/(const double& scalar);

            Quaternion conjugate();
            Quaternion inv();


            // Getters -----------
            Eigen::Vector4d getQuaternion() const;
            bool getPoint() const;

            // conversions
            void getRotationMatrix(Eigen::Matrix3d& m);
            Eigen::Vector3d getEulerAngles(bool deg=false);


        private:
            // quaternion array
            Eigen::Vector4d q;

            // properties
            double magnitude; 
            bool point = false;

    };


}