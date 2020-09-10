// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "DataStore.h"

DataStore::DataStore()
	: skeleton(nullptr), mesh(nullptr), endeffector(nullptr), base(nullptr), dof_changed_by_ik(false)
{ }

DataStore::~DataStore()
{
}

void DataStore::set_skeleton(std::shared_ptr<SkinningSkeleton> s)
{
	skeleton = s;
	skeleton_changed(skeleton);
}

void DataStore::del_skeleton()
{
	skeleton = nullptr;
	skeleton_changed(skeleton);
}

std::shared_ptr<SkinningSkeleton> DataStore::get_skeleton() const
{
	return skeleton;
}

void DataStore::set_mesh(std::shared_ptr<SkinningMesh> m)
{
	mesh = m;
	mesh_changed(mesh);
}

std::shared_ptr<SkinningMesh> DataStore::get_mesh() const
{
	return mesh;
}

void DataStore::set_endeffector(Bone* b, int which)
{
	endeffector = b;
	endeffector_changed(b, which);
}

Bone* DataStore::get_endeffector() const
{
	return endeffector;
}

void DataStore::set_base(Bone* b)
{
	base = b;
	base_changed(b);
}

Bone* DataStore::get_base() const
{
	return base;
}