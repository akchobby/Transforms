#include <iostream>
#include <Eigen/Core>

namespace transforms {

    class Quaternion{
        public:
            
            // create quaternion --- a vector case

            // quaternion vector 
            Quaternion(Eigen::Vector4d& v, bool point);

            // euler angles or rvec
            Quaternion(Eigen::Vector3d& v, bool deg, bool rvec, bool point);


            // create quaternion --- a matrix case
            Quaternion(Eigen::Matrix3d& m);


            ~Quaternion();


            // Getters -----------
            void getRotationMatrix(Eigen::Matrix3d& m);

            // conversions
            void getQuaternion(Eigen::Vector4d& v);


        private:
            // quaternion array
            Eigen::Vector4d q;

            // properties
            double magnitude; 
            bool point = false;

    }


}