// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"
#include "DataStore.h"

#include <cgv/gui/trigger.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv/media/illum/surface_material.h>
#include <cgv/render/drawable.h>
#include <cgv/render/context.h>
#include <cgv_gl/gl/gl.h>

using namespace cgv::base;
using namespace cgv::signal;
using namespace cgv::gui;
using namespace cgv::math;
using namespace cgv::render;
using namespace cgv::utils;

class SkeletonViewer : public node, public drawable, public provider
{
private:
	// slot for the signal
	void timer_event(double, double dt);
	void generate_tree_view_nodes();
	void generate_tree_view_nodes(gui_group_ptr parent, Bone* bone);
	void tree_selection_changed(base_ptr p, bool select);
	void generate_bone_gui(Bone* bone);
	void write_pinocchio();
	void load_pinocchio();
	void load_animation();
	void start_choose_base();
	void dof_changed(double new_value);
	void recursive_connect_signals(Bone* b);
	void compute_posi_jointlist_recur(Bone* node, const Mat4& global_to_parent_local, int level);

	DataStore* data;
	gui_group_ptr tree_view;
	gui_group_ptr bone_group;
	cgv::media::illum::surface_material material;
	cgv::media::illum::surface_material material_x;
	cgv::media::illum::surface_material material_y;
	cgv::media::illum::surface_material material_z;
	//float scale_factor_skel = 0.1;
	// Maps gui elements in the tree view to a specific bone
	std::map<base_ptr, Bone*> gui_to_bone;
	Animation* animation;
	Animation* anim_to_record;
	double animationTime;
	double recordingTime;
	bool ianim = false;
	std::vector<box3> jointlist; // in world coordi. 
	std::vector<rgb> jointlist_color;
public:
	void start_animation();
	void stop_animation();
	void start_record_anim() {
		recording = true;
		std::cout << "recording to list" << std::endl;
	}
	bool prepare_record_anim() {
		/*if (anim_to_record)
		{
			delete anim_to_record;
			anim_to_record = nullptr;
		}*/
		auto a = new Animation();
		if (a->ready_to_record(data->get_skeleton().get())) {
			anim_to_record = a;
			recordingTime = 0;
			//recording = true;
		}
		else
		{
			delete a;
		}
		return true;
	}
	bool stop_record_anim(std::string filename) {
		recording = false;
		anim_to_record->saveframe_to_file(filename);
		std::cout << "succ. writtren to file" << std::endl;
		return true;
	}
	void draw_skeleton_subtree(Bone* node, const Mat4& global_to_parent_local, context& ctx, int level, bool arrows, bool indicators);
	void set_skel_origin_ori_translation(const Vec3 rot_axis, float angle_degrees, Vec3 translation) {// rotate first
		data->get_skeleton()->set_origin_rotation(rot_axis, angle_degrees);
		data->get_skeleton()->set_origin_translation(translation);
		// after this, origin will be changed (it is a matrix)

		// we have to re-compute jointlist
		data->get_skeleton()->clear_bone_list();
		jointlist.clear();
		jointlist_color.clear();
		compute_posi_jointlist_recur(data->get_skeleton()->get_root(), data->get_skeleton()->get_origin(), 0);
	}
	void skeleton_changed(std::shared_ptr<SkinningSkeleton>);
	SkeletonViewer(DataStore* data, std::string skelname);
	void load_skeleton();
	void load_skeleton_given_name(std::string f);
	void load_animation_given_name(std::string f, bool myanim);
	void create_gui();
	void draw(context& c); 
	std::vector<box3> get_jointlist();
	std::vector<rgb> get_jointlistcolor() { return jointlist_color; }
	std::string get_parent_type() const;

	float cubesize = 0.05;
	bool playing = false;
	bool recording = false;
	bool render_axis_arrow = false;
	bool should_apply_dofs_to_others_for_imitating = false;
};

