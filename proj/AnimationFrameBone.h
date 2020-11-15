// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "Bone.h"

#include <vector>

//Represents animation data for a single bone
struct AnimationFrameBone
{
	//The bone, for which animation data is specified
	Bone* bone;

	//Values for every bone dof in the same order as specified in the bone
	std::vector<double> dof_values;
};