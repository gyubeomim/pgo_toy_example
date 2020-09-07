#ifndef ALIAS_TYPES_H_
#define ALIAS_TYPES_H_

#include <Eigen/Dense>

typedef Eigen::Matrix<double,2,2> Mat22;
typedef Eigen::Matrix<double,3,3> Mat33;
typedef Eigen::Matrix<double,4,4> Mat44;

typedef Eigen::Matrix<double,2,3> Mat23;
typedef Eigen::Matrix<double,2,4> Mat24;

typedef Eigen::Matrix<double,3,4> Mat34;
typedef Eigen::Matrix<double,3,5> Mat35;

typedef Eigen::Matrix<double,2,1> Vec2;
typedef Eigen::Matrix<double,3,1> Vec3;
typedef Eigen::Matrix<double,4,1> Vec4;
typedef Eigen::Matrix<double,5,1> Vec5;
typedef Eigen::Matrix<double,6,1> Vec6;
typedef Eigen::Matrix<double,7,1> Vec7;
typedef Eigen::Matrix<double,8,1> Vec8;
typedef Eigen::Matrix<double,9,1> Vec9;
typedef Eigen::Matrix<double,10,1> Vec10;
typedef Eigen::Matrix<double,11,1> Vec11;
typedef Eigen::Matrix<double,12,1> Vec12;

typedef Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> MatXX;

typedef Eigen::Isometry3d Isometry;

typedef Eigen::Quaterniond Quaternion;


#endif /* ALIAS_TYPES_H_ */
