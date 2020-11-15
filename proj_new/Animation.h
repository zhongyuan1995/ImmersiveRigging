// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"
#include "AnimationFrameBone.h"
#include "Skeleton.h"

#include <string>

class Animation
{
public:
	/// load animation from exteral file 
	bool read_amc_file(std::string filename, SkinningSkeleton*, bool myanim);
	/// add the current frame to an array, which will be written to an external file
	void saveframe_to_vector(int frame);
	/// save current animation 
	void saveframe_to_file(std::string filename);
	///
	bool ready_to_record(SkinningSkeleton* s) { frames_record.clear(); skel = s; last_frame = -1; return true; }
	///
	int frame_count() const;
	/// apply the dofs to current skeleton, a new pose should be shown after calling this function 
	void apply_frame(int frame) const;

private:
	// Contains a std::vector<AnimationFrameBone> for each frame, which contains animation data for a set of bones.
	std::vector<std::vector<AnimationFrameBone>> frames;
	std::vector<std::vector<AnimationFrameBone>> frames_record;
	// used to store the basic position at the beginning, to avoid artifact, we just apply the diff. val. to animations 
	float skel_base_x; // internal varibles, we do not have to know them 
	float skel_base_y;
	float skel_base_z;
	bool is_firstroot = true;
	SkinningSkeleton* skel;
	int last_frame = -1;
};
