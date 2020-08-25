// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "AtomicTransform.h"

#include <cgv/math/transformations.h>
#include <libs/cgv_gl/gl/gl.h>

#include "math_helper.h"

void AtomicTransform::set_limits(double lower, double upper)
{
	this->lower_limit = lower;
	this->upper_limit = upper;
	this->value = 0;
}

void AtomicTransform::set_value(const double& value, void*)
{
	this->value = value;
	changed_signal(value);
}

std::string AtomicTransform::get_name() const { return title; }
const double AtomicTransform::get_value(void*) const { return value; }
const double AtomicTransform::get_lower_limit() const { return lower_limit; }
const double AtomicTransform::get_upper_limit() const { return upper_limit; }

AtomicRotationTransform::AtomicRotationTransform(Vec3 axis)
	: axis(axis)
{
	lower_limit = -180;
	upper_limit = 180;
	value = 0;
}

Mat4 AtomicRotationTransform::calculate_matrix()
{
	return rotate(axis, (float)value);	
}

void AtomicRotationTransform::drawIndicator(float size)
{
	if (lower_limit == upper_limit)
		return;
	//Calculate rotation matrix from x-axis to current axis
	Vec3 rot_axis = cgv::math::cross(Vec3(1, 0, 0), axis);
	float angle = acosf(axis.x() / axis.length());
	Mat4 t;
	if (angle == 0)
		t.identity();
	else
		t = rotate(rot_axis, angle * 180.0f / 3.1415926f);
	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(0, 0, 0);
	for (float v = (float)lower_limit; v <= upper_limit; v += 5)
	{
		auto p = t * size * Vec4(0, -cosf(v * PI / 180.0f), -sinf(v * PI / 180.0f), 0);
		glVertex3f(p.x(), p.y(), p.z());
	}
	auto end = t * size * Vec4(0, -cosf((float)upper_limit * PI / 180.0f), -sinf((float)upper_limit * PI / 180.0f), 0);
	glVertex3f(end.x(), end.y(), end.z());
	glEnd();
}
void AtomicRotationTransform::drawActualIndicator(float size)
{
	//Calculate rotation matrix from x-axis to current axis
	Vec3 rot_axis = cgv::math::cross(Vec3(1, 0, 0), axis);
	float angle = acosf(axis.x() / axis.length());
	Mat4 t;
	if (angle == 0)
		t.identity();
	else
		t = rotate(rot_axis, angle * 180.0f / 3.1415926f);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	auto p = t * size * Vec4(0, -cosf((float)value * PI / 180.0f), -sinf((float)value * PI / 180.0f), 0);
	glVertex3f(p.x(), p.y(), p.z());
	glEnd();
}

void AtomicRotationTransform::optimize_value(const Vec3& local_vector, const Vec3& target, bool inverse)
{
	//project vectors in the tangent plane of the axis and normalize
	Vec3 local_in_tangent_plane = local_vector - cgv::math::dot(local_vector, axis) * axis;
	auto length = local_in_tangent_plane.length();
	if (length < 0.01)
		return; //the local vector is mostly parallel to the axis; there is not much we can do
	local_in_tangent_plane.normalize();

	Vec3 target_in_tangent_plane = target - cgv::math::dot(target, axis) * axis;
	length = target_in_tangent_plane.length();
	if (length < 0.01)
		return; //the target vector is mostly parallel to the axis; there is not much we can do
	target_in_tangent_plane.normalize();
	
	double optimal_value = atan2(
		cgv::math::dot(axis, cgv::math::cross(local_in_tangent_plane, target_in_tangent_plane)), 
		cgv::math::dot(local_in_tangent_plane, target_in_tangent_plane)) * 180.0f / 3.1415926f;
	if (inverse)
		optimal_value *= -1;
	if (optimal_value < lower_limit || optimal_value > upper_limit)
	{
		//check which limit is closer
		while (abs(upper_limit - optimal_value) > 180.0f)
			optimal_value += 360.0f * (optimal_value < upper_limit ? 1.0f : -1.0f);
		double distance_to_upper = abs(upper_limit - optimal_value);
		while (abs(lower_limit - optimal_value) > 180.0f)
			optimal_value += 360.0f * (optimal_value < lower_limit ? 1.0f : -1.0f);
		double distance_to_lower = abs(lower_limit - optimal_value);
		if (distance_to_lower < distance_to_upper)
			optimal_value = lower_limit;
		else
			optimal_value = upper_limit;
	}
	set_value(optimal_value);
}

AtomicTranslationTransform::AtomicTranslationTransform(int dim) 
	: dim(dim) 
{
	lower_limit = -10;
	upper_limit = 10;
}

Mat4 AtomicTranslationTransform::calculate_matrix()
{	
	Mat4 result;
	result.identity();
	result(dim, 3) = (float)value;	
	return result;
}

void AtomicTranslationTransform::optimize_value(const Vec3& local_vector, const Vec3& target, bool inverse)
{
}
