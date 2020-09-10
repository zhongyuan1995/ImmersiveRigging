// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include <cgv/base/base.h>
#include "IKViewer.h"

#include "math_helper.h"

#include <unordered_set>

#include <cgv/gui/mouse_event.h>
#include <cgv/math/inv.h>
#include <cgv/math/mat.h>

IKViewer::IKViewer(DataStore* data)
	: node("IK Viewer"), data(data), modifying(false), target_position(0, 0, 0, 1), max_iterations(15)
{
	connect(data->endeffector_changed, this, &IKViewer::endeffector_changed);
	connect(data->base_changed, this, &IKViewer::base_changed);
}

void IKViewer::endeffector_changed(Bone* b, int which)
{
	calculate_kinematic_chain(data->get_base(), b, which);

	post_redraw();
}

void IKViewer::base_changed(Bone* b)
{
	calculate_kinematic_chain(data->get_base(), data->get_endeffector(), 0);
	calculate_kinematic_chain(data->get_base(), data->get_endeffector(), 1);
	calculate_kinematic_chain(data->get_base(), data->get_endeffector(), 2);

	post_redraw();
}

void IKViewer::calculate_kinematic_chain(Bone* base, Bone* endeffector, int which)
{
	if (!base || !endeffector)
		return;
	Bone *b;
	Mat4 transform_root_to_base;
	transform_root_to_base.identity();
	std::unordered_set<Bone*> base_path_to_root;
	if (base)
	{
		//traverse from base to root
		b = base;
		while (b)
		{
			if(b->childCount()>0)
			transform_root_to_base = b->get_translation_transform_current_joint_to_next() * transform_root_to_base;
			for (int i = b->dof_count() - 1; i >= 0; --i)
				transform_root_to_base = b->get_dof(i)->calculate_matrix() * transform_root_to_base;
			transform_root_to_base = b->get_orientation_transform_prev_joint_to_current() * transform_root_to_base;
			base_path_to_root.insert(b);
			b = b->get_parent();
		}
		current_base_matrix = data->get_skeleton()->get_origin() * transform_root_to_base;
	}
	if (!base || !endeffector)
		return;
	Mat4 t;
	current_endeffector_matrix.identity();
	if(which == 0)
		kinematic_chain.clear();
	else if(which == 1)
		kinematic_chain_1.clear();
	else if(which == 2)
		kinematic_chain_2.clear();
	common_ancestor = nullptr;
	//traverse from endeffector to lowest common ancestor
	b = endeffector;
	while (b)
	{
		if (base_path_to_root.find(b) != base_path_to_root.end())
		{
			common_ancestor = b;
			break;
		}
		if (b->childCount() > 0)
		t = b->get_translation_transform_current_joint_to_next();
		if (which == 0)
			kinematic_chain.push_front(std::make_shared<StaticTransform>(t));
		else if (which == 1)
			kinematic_chain_1.push_front(std::make_shared<StaticTransform>(t));
		else if (which == 2)
			kinematic_chain_2.push_front(std::make_shared<StaticTransform>(t));
		current_endeffector_matrix = t * current_endeffector_matrix;
		for (int current_dof_id = b->dof_count() - 1; current_dof_id >= 0; --current_dof_id)
		{
			auto dof = b->get_dof(current_dof_id);
			t = dof->calculate_matrix();
			if (which == 0)
				kinematic_chain.push_front(dof);
			else if (which == 1)
				kinematic_chain_1.push_front(dof);
			else if (which == 2)
				kinematic_chain_2.push_front(dof);
			current_endeffector_matrix = t * current_endeffector_matrix;
		}
		t = b->get_orientation_transform_prev_joint_to_current();
		if (which == 0)
			kinematic_chain.push_front(std::make_shared<StaticTransform>(t));
		else if (which == 1)
			kinematic_chain_1.push_front(std::make_shared<StaticTransform>(t));
		else if (which == 2)
			kinematic_chain_2.push_front(std::make_shared<StaticTransform>(t));
		current_endeffector_matrix = t * current_endeffector_matrix;
		b = b->get_parent();
	}
	//traverse from base to lowest common ancestor
	std::deque<std::shared_ptr<BaseTransform>> chain_base_to_ancestor;
	Mat4 matrix_base_to_ancestor; matrix_base_to_ancestor.identity();
	b = base;
	while (b)
	{
		if (b == common_ancestor)
			break;
		if (b->childCount() > 0)
		t = cgv::math::inv(b->get_translation_transform_current_joint_to_next());
		chain_base_to_ancestor.push_back(std::make_shared<StaticTransform>(t));
		matrix_base_to_ancestor = matrix_base_to_ancestor * t;
		for (int current_dof_id = b->dof_count() - 1; current_dof_id >= 0; --current_dof_id)
		{
			auto dof = b->get_dof(current_dof_id);
			t = cgv::math::inv(dof->calculate_matrix());
			chain_base_to_ancestor.push_back(std::make_shared<InverseTransform>(dof));
			matrix_base_to_ancestor = matrix_base_to_ancestor * t;
		}
		t = cgv::math::inv(b->get_orientation_transform_prev_joint_to_current());
		chain_base_to_ancestor.push_back(std::make_shared<StaticTransform>(t));
		matrix_base_to_ancestor = matrix_base_to_ancestor * t;
		b = b->get_parent();
	}
	current_endeffector_matrix = matrix_base_to_ancestor * current_endeffector_matrix;
	if (which == 0)
		kinematic_chain.insert(kinematic_chain.begin(), chain_base_to_ancestor.begin(), chain_base_to_ancestor.end());
	else if (which == 1)
		kinematic_chain_1.insert(kinematic_chain_1.begin(), chain_base_to_ancestor.begin(), chain_base_to_ancestor.end());
	else if (which == 2)
		kinematic_chain_2.insert(kinematic_chain_2.begin(), chain_base_to_ancestor.begin(), chain_base_to_ancestor.end());
	//traverse from ancestor to root
	current_ancestor_matrix.identity();
	b = common_ancestor;
	while (b)
	{
		if (b->childCount() > 0)
		current_ancestor_matrix = b->get_translation_transform_current_joint_to_next() * current_ancestor_matrix;
		for (int i = b->dof_count() - 1; i >= 0; --i)
			current_ancestor_matrix = b->get_dof(i)->calculate_matrix() * current_ancestor_matrix;
		current_ancestor_matrix = b->get_orientation_transform_prev_joint_to_current() * current_ancestor_matrix;
		b = b->get_parent();
	}
	t = data->get_skeleton()->get_origin() * transform_root_to_base * current_endeffector_matrix;
	target_position.x() = t(0, 3);
	target_position.y() = t(1, 3);
	target_position.z() = t(2, 3);
	// just initialize 
}

void IKViewer::optimize(int which)
{
	//used for correct GUI behavior
	data->dof_changed_by_ik = true;

	auto skeleton_size = (data->get_skeleton()->getMax() - data->get_skeleton()->getMin());
	float distance_threshold = 0.0001f * std::max({ skeleton_size.x(), skeleton_size.y(), skeleton_size.z() });

	//split the current matrix in:
	//  before_dof -> dof -> after_dof

	for (unsigned int iteration = 0; iteration < max_iterations; ++iteration)
	{
		Mat4 after_dof;
		after_dof.identity();
		Mat4 before_dof = current_endeffector_matrix;
		Vec4 target_base_local = cgv::math::inv(current_base_matrix) * target_position;
		//std::cout << "posi: "<<target_position << std::endl;
		//start from the last bone, yzy, traversal along the chain from beginning to the end 
		//and modify the dofs at the same time. 
		if (which == 0)
			for (auto it = kinematic_chain.rbegin(); it != kinematic_chain.rend(); ++it)
			{
				auto t = *it;
				before_dof = before_dof * inv(t->calculate_matrix());
				Vec4 target_dof_local = inv(before_dof) * target_base_local;
				t->optimize_value(Vec3(after_dof(0, 3), after_dof(1, 3), after_dof(2, 3)), Vec3(3, &target_dof_local(0)));
				after_dof = t->calculate_matrix() * after_dof;
			}
		else if (which == 1)
			for (auto it = kinematic_chain_1.rbegin(); it != kinematic_chain_1.rend(); ++it)
			{
				auto t = *it;
				before_dof = before_dof * inv(t->calculate_matrix());
				Vec4 target_dof_local = inv(before_dof) * target_base_local;
				t->optimize_value(Vec3(after_dof(0, 3), after_dof(1, 3), after_dof(2, 3)), Vec3(3, &target_dof_local(0)));
				after_dof = t->calculate_matrix() * after_dof;
			}
		else if (which == 2)
			for (auto it = kinematic_chain_2.rbegin(); it != kinematic_chain_2.rend(); ++it)
			{
				auto t = *it;
				before_dof = before_dof * inv(t->calculate_matrix());
				Vec4 target_dof_local = inv(before_dof) * target_base_local;
				t->optimize_value(Vec3(after_dof(0, 3), after_dof(1, 3), after_dof(2, 3)), Vec3(3, &target_dof_local(0)));
				after_dof = t->calculate_matrix() * after_dof;
			}
		current_endeffector_matrix = before_dof * after_dof;
		float error = Vec3(current_endeffector_matrix(0, 3) - target_base_local.x(),
			current_endeffector_matrix(1, 3) - target_base_local.y(),
			current_endeffector_matrix(2, 3) - target_base_local.z()).length();
		if (error <= distance_threshold)
			break;
	}
	//update the origin
	//traverse from base to ancestor
	Mat4 transform_ancestor_to_base;
	transform_ancestor_to_base.identity();
	Bone* b = data->get_base();
	while (b)
	{
		if (b == common_ancestor)
			break;
		if (b->childCount() > 0)
		transform_ancestor_to_base = b->get_translation_transform_current_joint_to_next() * transform_ancestor_to_base;
		for (int i = b->dof_count() - 1; i >= 0; --i)
			transform_ancestor_to_base = b->get_dof(i)->calculate_matrix() * transform_ancestor_to_base;
		transform_ancestor_to_base = b->get_orientation_transform_prev_joint_to_current() * transform_ancestor_to_base;
		b = b->get_parent();
	}
	Mat4 origin_to_base = current_ancestor_matrix * transform_ancestor_to_base;
	Mat4 global_to_origin = current_base_matrix * cgv::math::inv(origin_to_base);
	data->get_skeleton()->set_origin(global_to_origin);

	//used for correct GUI behavior
	data->dof_changed_by_ik = false;
}



void IKViewer::set_target_position_2d(int x, int y)
{
	auto ctx = get_context();
	if (!ctx)
		return;

	//Get the model-view-projection matrix
	GLfloat mv[16];
	GLfloat proj[16];
	glGetFloatv(GL_MODELVIEW_MATRIX, mv);
	glGetFloatv(GL_PROJECTION_MATRIX, proj);

	cgv::math::mat<float> modelview(4, 4, mv);
	cgv::math::mat<float> projm(4, 4, proj);

	cgv::math::mat<float> mvp = projm * modelview;
	cgv::math::mat<float> inv_mvp = cgv::math::inv_44(mvp);

	//Get the z-value of the current target
	auto projected_target = mvp * cgv::math::vec<float>(4, &target_position[0]);
	float z = projected_target.z() / projected_target.w();

	//Unproject the screen position into the scene
	int width = ctx->get_width();
	int height = ctx->get_height();

	float proj_position[4] = { (2.0f * x) / width - 1.0f, (-2.0f * y) / height + 1.0f, z, 1.0f };
	auto unprojected = inv_mvp * cgv::math::vec<float>(4, proj_position);
	unprojected *= 1.0f / unprojected.w();

	target_position.x() = unprojected.x();
	target_position.y() = unprojected.y();
	target_position.z() = unprojected.z();

	if (data->get_endeffector())
		optimize(0);

	post_redraw();
}


bool IKViewer::handle(event& e)
{
	if (e.get_kind() == EID_MOUSE) {			
		cgv::gui::mouse_event me = (cgv::gui::mouse_event&) e;

		switch (me.get_action()) {
		case MA_PRESS:
			if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
				modifying = true;
				set_target_position_2d(me.get_x(), me.get_y());
				return true;
			}			
			break;
		case MA_RELEASE:
			modifying = false;
			break;
		case MA_DRAG:
			if (modifying)
			{
				set_target_position_2d(me.get_x(), me.get_y());
				return true;
			}
			break;		
		default: break;
		}
	}
	return false;
}

void IKViewer::stream_help(std::ostream& os)
{
}

void IKViewer::draw(cgv::render::context& ctx)
{
	if (!data->get_skeleton())
		return;

	auto skeleton_size = (data->get_skeleton()->getMax() - data->get_skeleton()->getMin());
	float scale = 0.2f * std::max({ skeleton_size.x(), skeleton_size.y(), skeleton_size.z() });

	glBegin(GL_LINES);

	if (data->get_endeffector())
	{
		glColor3f(1, 1, 1);

		glVertex3f(target_position.x() - scale, target_position.y(), target_position.z());
		glVertex3f(target_position.x() + scale, target_position.y(), target_position.z());

		glVertex3f(target_position.x(), target_position.y() - scale, target_position.z());
		glVertex3f(target_position.x(), target_position.y() + scale, target_position.z());

		glVertex3f(target_position.x(), target_position.y(), target_position.z() - scale);
		glVertex3f(target_position.x(), target_position.y(), target_position.z() + scale);
	}

	if (data->get_base())
	{
		glColor3f(0.5f, 1, 0.5f);

		glVertex3f(current_base_matrix(0, 3) - scale, current_base_matrix(1, 3), current_base_matrix(2, 3));
		glVertex3f(current_base_matrix(0, 3) + scale, current_base_matrix(1, 3), current_base_matrix(2, 3));

		glVertex3f(current_base_matrix(0, 3), current_base_matrix(1, 3) - scale, current_base_matrix(2, 3));
		glVertex3f(current_base_matrix(0, 3), current_base_matrix(1, 3) + scale, current_base_matrix(2, 3));

		glVertex3f(current_base_matrix(0, 3), current_base_matrix(1, 3), current_base_matrix(2, 3) - scale);
		glVertex3f(current_base_matrix(0, 3), current_base_matrix(1, 3), current_base_matrix(2, 3) + scale);
	}

	glEnd();
}

void IKViewer::create_gui()
{
	add_member_control(this, "Max Iterations", max_iterations, "value_slider", "min=1;max=100");
}