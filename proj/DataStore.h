// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "Skeleton.h"
#include "Mesh.h"
#include "Animation.h"

#include <cgv/signal/signal.h>
#include <memory>

class DataStore
{
public:

	// This signal is used to report a change of the skeleton to the viewer.
	// Skeleton* is the only parameter of this signal. There are no signals
	// with no parameters.
	// Use connect(...) and connect_copy(...) to register a listener.
	// Use operator() to call a signal.
	cgv::signal::signal<std::shared_ptr<SkinningSkeleton>> skeleton_changed;

	cgv::signal::signal<std::shared_ptr<SkinningMesh>> mesh_changed;

	cgv::signal::signal<Bone*, int> endeffector_changed;
	cgv::signal::signal<Bone*> base_changed;

	DataStore();

	~DataStore();

	//Used for Skeleton Visualization
	void set_skeleton(std::shared_ptr<SkinningSkeleton>);
	std::shared_ptr<SkinningSkeleton> get_skeleton() const;
	void del_skeleton();

	//Used for Skinning
	void set_mesh(std::shared_ptr<SkinningMesh>);
	std::shared_ptr<SkinningMesh> get_mesh() const;

	//Used for IK, we have to set them first, and then, give target posi.
	void set_endeffector(Bone*, int which);
	Bone* get_endeffector() const;

	void set_base(Bone*);
	Bone* get_base() const;

	//Specifies if the next dof change is caused by the Inverse Kinematics module
	bool dof_changed_by_ik;

private:
	std::shared_ptr<SkinningSkeleton> skeleton;
	std::shared_ptr<SkinningMesh> mesh;
	Bone* endeffector, *base;
};