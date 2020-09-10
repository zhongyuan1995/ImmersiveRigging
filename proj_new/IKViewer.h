// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"
#include "DataStore.h"

#include <list>

#include <cgv/gui/trigger.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv/render/drawable.h>
#include <cgv/render/context.h>
#include <cgv_gl/gl/gl.h>

using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::gui;
using namespace cgv::math;
using namespace cgv::render;
using namespace cgv::utils;

class IKViewer : public node, public drawable, public provider, public event_handler
{
private:
	DataStore* data;
	
	void set_target_position_2d(int x, int y);

	void base_changed(Bone*);

	void calculate_kinematic_chain(Bone* base, Bone* endeffector, int which);

	bool modifying; //Specifies if the user is currently modifying the IK target position	

	Vec4 target_position;
	Mat4 current_endeffector_matrix; //transform from base to endeffector
	Mat4 current_base_matrix; //transform from global origin to base

	
	Bone* common_ancestor; //lowest common ancestor of base and endeffector
	Mat4 current_ancestor_matrix; //transform from root to common_ancestor

	unsigned int max_iterations;

	std::list<std::shared_ptr<BaseTransform>> kinematic_chain;
	std::list<std::shared_ptr<BaseTransform>> kinematic_chain_1;
	std::list<std::shared_ptr<BaseTransform>> kinematic_chain_2;


public:

	void set_max_iter(int iter) {max_iterations = iter;}
	void optimize(int which);
	void endeffector_changed(Bone*, int which);
	void set_target_position_vr(Vec4 posi) { target_position = posi; }
	// The constructor of this class
	IKViewer(DataStore*);

	bool handle(cgv::gui::event& e);
	void stream_help(std::ostream& os);

	// Create the gui elements
	void create_gui();
	// Draw the scene
	void draw(context& c);	
};

