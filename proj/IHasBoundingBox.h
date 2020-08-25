// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"

//Represents an object that contains a bounding box
class IHasBoundingBox
{
public:
	IHasBoundingBox(void);
	~IHasBoundingBox(void);

	//Gets the bounding box' minimum corner coordinate.
	Vec3 getMin();

	//Gets the bounding box' maximum corner coordinate.
	Vec3 getMax();

private:
	Vec3 min, max;

protected:
	//Adds a point to the bounding box. If the point lies outside of the bounding box,
	//the bounding box is expanded accordingly.
	void add_point(Vec3 position);

	//
	void reset_bounding_box();
};

