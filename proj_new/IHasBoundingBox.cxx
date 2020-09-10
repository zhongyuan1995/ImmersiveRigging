// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "IHasBoundingBox.h"


IHasBoundingBox::IHasBoundingBox(void)
{
	reset_bounding_box();
}

void IHasBoundingBox::reset_bounding_box()
{
	max = Vec3(-std::numeric_limits<float>::infinity());
	min = Vec3(std::numeric_limits<float>::infinity());
}


IHasBoundingBox::~IHasBoundingBox(void)
{
}

Vec3 IHasBoundingBox::getMin()
{
	return min;
}

Vec3 IHasBoundingBox::getMax()
{
	return max;
}

void IHasBoundingBox::add_point(Vec3 p)
{
	if (p.x() < min.x())
		min.x() = p.x();
	if (p.x() > max.x())
		max.x() = p.x();
	if (p.y() < min.y())
		min.y() = p.y();
	if (p.y()> max.y())
		max.y() = p.y();
	if (p.z() < min.z())
		min.z() = p.z();
	if (p.z() > max.z())
		max.z() = p.z();
}