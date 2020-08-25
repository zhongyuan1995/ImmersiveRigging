//
// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) 2016 CGV TU Dresden - All Rights Reserved
//

#pragma once

#include "common.h"
#include "DataStore.h"
#include "Mesh.h"

#include <cgv/gui/trigger.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/context.h>
#include <cgv_gl/gl/gl.h>

using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::gui;
using namespace cgv::math;
using namespace cgv::render;
using namespace cgv::utils;

class SkinnedMeshViewer : public node, public drawable, public provider
{
private:
	DataStore* data;	

protected:
	virtual bool init(context&);

	void mesh_changed(std::shared_ptr<SkinningMesh>);



public:	
	void load_mesh();
	void load_attachment();
	// The constructor of this class
	SkinnedMeshViewer(DataStore*);

	// Create the gui elements
	void create_gui();
	// Draw the scene
	void draw(context& c);
};

