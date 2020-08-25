// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"
#include <cgv/math/transformations.h>

///creates a 4x4 rotation matrix
template <typename T>
const cgv::math::fmat<T, 4, 4> rotate(const cgv::math::fvec<T, 3> axis, T angle_degrees)
{
	auto matrix = cgv::math::rotate_44(axis.x(), axis.y(), axis.z(), angle_degrees);
	Mat4 result;
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			result(i, j) = matrix(i, j);
	return result;
}

///creates a 4x4 translation matrix
template <typename T>
const cgv::math::fmat<T, 4, 4> translate(const T&x, const T &y, const T&z)
{
	cgv::math::fmat<T, 4, 4> m;
	m.identity();
	m(0, 3) = x;
	m(1, 3) = y;
	m(2, 3) = z;
	return m;
}

///creates a 4x4 translation matrix
template <typename T>
const cgv::math::fmat<T, 4, 4> translate(const cgv::math::fvec<T, 3> v)
{
	cgv::math::fmat<T, 4, 4> m;
	m.identity();
	m(0, 3) = v.x();
	m(1, 3) = v.y();
	m(2, 3) = v.z();
	return m;
}

///creates a 4x4 translation matrix
template <typename T>
const cgv::math::fmat<T, 4, 4> translate(const cgv::math::fvec<T, 4> v)
{
	cgv::math::fmat<T, 4, 4> m;
	m.identity();
	m(0, 3) = v.x();
	m(1, 3) = v.y();
	m(2, 3) = v.z();
	return m;
}