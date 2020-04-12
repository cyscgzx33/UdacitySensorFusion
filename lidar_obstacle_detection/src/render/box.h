#ifndef BOX_H
#define BOX_H
#include <Eigen/Geometry> 

// An advanced Box struct:
// Aiming at rotation along Z axis using quaternion expression
struct BoxQ
{
	Eigen::Vector3f bboxTransform;
	Eigen::Quaternionf bboxQuaternion;
	float cube_length;
    float cube_width;
    float cube_height;
};

struct Box
{
	float x_min;
	float y_min;
	float z_min;
	float x_max;
	float y_max;
	float z_max;
};
#endif