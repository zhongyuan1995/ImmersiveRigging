// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "SkeletonViewer.h"

#include <cgv/utils/ostream_printf.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/key_event.h>
#include <cgv/render/view.h>
#include <cgv/render/render_types.h>
#include <cgv/base/find_action.h>

#include "math_helper.h"
#include <cgv\math\ftransform.h>
#include <cgv\media\illum\surface_material.cxx>

using namespace cgv::utils;

cgv::render::shader_program SkinningMesh::prog;

// The constructor of this class
SkeletonViewer::SkeletonViewer(DataStore* data, std::string skelname)
	: node(skelname), data(data)
	, animation(nullptr), animationTime(0)
{	
	connect(data->skeleton_changed, this, &SkeletonViewer::skeleton_changed);
	connect(get_animation_trigger().shoot, this, &SkeletonViewer::timer_event);
	material_x.set_diffuse_reflectance({ 1, 0, 0 });
	material_y.set_diffuse_reflectance({ 0, 1, 0 });
	material_z.set_diffuse_reflectance({ 0, 0, 1 });
}

// jointlist used for intesection computing only, not for rendering
void SkeletonViewer::compute_posi_jointlist_recur(Bone* node, const Mat4& global_to_parent_local, int level) {
	auto global_to_current_local = global_to_parent_local * node->calculate_transform_prev_to_current_with_dofs();
	auto my_root_position = global_to_current_local * node->get_bone_local_root_position();
	auto my_tip_position = global_to_current_local * node->get_bone_local_tip_position();

	static const cgv::media::illum::surface_material::color_type colors[] =
	{
		{  27.0 / 256.0, 158.0 / 256.0, 119.0 / 256.0 },
		{ 217.0 / 256.0,  95.0 / 256.0,   2.0 / 256.0 },
		{ 117.0 / 256.0, 112.0 / 256.0, 179.0 / 256.0 },
		{ 231.0 / 256.0,  41.0 / 256.0, 138.0 / 256.0 },
		{ 102.0 / 256.0, 166.0 / 256.0,  30.0 / 256.0 },
		{ 230.0 / 256.0, 171.0 / 256.0,   2.0 / 256.0 },
		{ 166.0 / 256.0, 118.0 / 256.0,  29.0 / 256.0 },
	};

	cgv::render::render_types::dvec3
		aRoot(my_root_position.x(), my_root_position.y(), my_root_position.z()),
		aTip(my_tip_position.x(), my_tip_position.y(), my_tip_position.z());

	// use new size for added bones // tobetested 
	if (node->jointsize_stored_as_bone_parameter > 0)
		cubesize = node->jointsize_stored_as_bone_parameter;
	if (node->get_name()._Equal("root")) {
		jointlist.push_back(box3(vec3(aRoot.x() - cubesize / 2, aRoot.y() - cubesize / 2, aRoot.z() - cubesize / 2),
			vec3(aRoot.x() + cubesize / 2, aRoot.y() + cubesize / 2, aRoot.z() + cubesize / 2)));
		jointlist_color.push_back(colors[level % 7]);
	}else 
	{
		//aTip can be converted to box3 
		jointlist.push_back(box3(vec3(aTip.x() - cubesize / 2, aTip.y() - cubesize / 2, aTip.z() - cubesize / 2),
			vec3(aTip.x() + cubesize / 2, aTip.y() + cubesize / 2, aTip.z() + cubesize / 2)));
		jointlist_color.push_back(colors[level % 7]);
	}
	cubesize = 0.05f; // reset the cubesize value

	data->get_skeleton()->push_back_to_bone_list(node);// bone list will be updated, can be used to "look up" bones and add child
	
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		auto child = node->child_at(i);
		compute_posi_jointlist_recur(child, global_to_current_local, level+1);
	}

}

//draws a part of a skeleton, represented by the given root node. used for rendering.
void SkeletonViewer::draw_skeleton_subtree(Bone* node, const Mat4& global_to_parent_local, context& ctx, int level, bool arrows, bool indicators)
{
	/*if (!playing)
		return;*/
	auto global_to_current_local = global_to_parent_local * node->calculate_transform_prev_to_current_with_dofs();
	auto my_root_position = global_to_current_local * node->get_bone_local_root_position();
	auto my_tip_position = global_to_current_local * node->get_bone_local_tip_position();
	if (arrows)
	{
		static const cgv::media::illum::surface_material::color_type colors[] =
		{
			{  27.0/256.0, 158.0/256.0, 119.0/256.0 },
			{ 217.0/256.0,  95.0/256.0,   2.0/256.0 },
			{ 117.0/256.0, 112.0/256.0, 179.0/256.0 },
			{ 231.0/256.0,  41.0/256.0, 138.0/256.0 },
			{ 102.0/256.0, 166.0/256.0,  30.0/256.0 },
			{ 230.0/256.0, 171.0/256.0,   2.0/256.0 },
			{ 166.0/256.0, 118.0/256.0,  29.0/256.0 },
		};

		material.set_diffuse_reflectance(colors[level%7]);
		ctx.set_material(material);

		ctx.ref_surface_shader_program().enable(ctx);
		cgv::render::render_types::dvec3
			aRoot(my_root_position.x(), my_root_position.y(), my_root_position.z()),
			aTip(my_tip_position.x(), my_tip_position.y(), my_tip_position.z());

		if (playing)
		if (node->jointsize_stored_as_bone_parameter > 0)
			cubesize = node->jointsize_stored_as_bone_parameter;
		if (node->get_name()._Equal("root")) {
			ctx.tesselate_box(box3(vec3(aRoot.x() - cubesize / 2, aRoot.y() - cubesize / 2, aRoot.z() - cubesize / 2),
				vec3(aRoot.x() + cubesize / 2, aRoot.y() + cubesize / 2, aRoot.z() + cubesize / 2)), false, false);
		}

		if ((aTip - aRoot).length() > std::numeric_limits<double>::epsilon()) 
			{
				// ref. vr_view_ptr rewrite todo 
				glLineWidth(5);
				glBegin(GL_LINES);
				glColor3f(0, 0, 0);
				glVertex3f(aRoot.x(), aRoot.y(), aRoot.z());
				glVertex3f(aTip.x(), aTip.y(), aTip.z());
				glEnd();
				glLineWidth(1);
				//ctx.tesselate_unit_sphere_given_posi(aRoot,0.02,3);

				if (playing)
					ctx.tesselate_box(box3(vec3(aTip.x() - cubesize / 2, aTip.y() - cubesize / 2, aTip.z() - cubesize / 2),
						vec3(aTip.x() + cubesize / 2, aTip.y() + cubesize / 2, aTip.z() + cubesize / 2)), false, false);
				
			}
		cubesize = 0.05f; // reset the cubesize value 
		//ctx.tesselate_arrow(aRoot, aTip, 0.1, 2.0, 0.5);
		ctx.ref_surface_shader_program().disable(ctx);
	}
	Mat4 dof_matrix = global_to_parent_local * node->calculate_transform_prev_to_current_without_dofs();
	float indicatorSize = (data->get_skeleton()->getMax()-data->get_skeleton()->getMin()).length() * 0.03125f;
	//draw indicators for dofs
	if (indicators)
	{
		glDepthMask(false);
		int i = 0;
		// draw for each dof.
		for (int i = 0; i < node->dof_count(); i++)
		{
			auto t = node->get_dof(i);
			glPushMatrix();
			glMultMatrixf(dof_matrix);
			glColor4f(0, 1, 0, 0.2f);
			t->drawIndicator(indicatorSize);
			glColor4f(0.5f, 1, 0.5f, 1);
			t->drawActualIndicator(indicatorSize);
			glPopMatrix();
			dof_matrix = dof_matrix * node->get_dof(i)->calculate_matrix();
		}
		glDepthMask(true);
	}
	// draw local axis 
	if (render_axis_arrow && false) {
		dvec3 axis_root_vec3(my_root_position.x(), my_root_position.y(), my_root_position.z());
		vec4 local_axis_arrow_x = vec4(0.06, 0, 0, 1);
		vec4 local_axis_arrow_y = vec4(0, 0.06, 0, 1);
		vec4 local_axis_arrow_z = vec4(0, 0, 0.06, 1);
		//glDepthMask(false);

		// yzy. or, use axes_mat
		/*cgv::media::illum::surface_material axes_mat = 
			cgv::media::illum::surface_material(cgv::media::illum::BT_OREN_NAYAR, 
				cgv::media::illum::surface_material::color_type(0, 0, 0), 0.2f);*/
		local_axis_arrow_x = node->get_orientationTransformGlobalToLocal() * local_axis_arrow_x;
		local_axis_arrow_y = node->get_orientationTransformGlobalToLocal() * local_axis_arrow_y;
		local_axis_arrow_z = node->get_orientationTransformGlobalToLocal() * local_axis_arrow_z;

		dvec3 translated_local_axis_x = vec3(local_axis_arrow_x.x(), local_axis_arrow_x.y(), local_axis_arrow_x.z());
		dvec3 translated_local_axis_y = vec3(local_axis_arrow_y.x(), local_axis_arrow_y.y(), local_axis_arrow_y.z());
		dvec3 translated_local_axis_z = vec3(local_axis_arrow_z.x(), local_axis_arrow_z.y(), local_axis_arrow_z.z());

		ctx.ref_surface_shader_program().enable(ctx);
		//ctx.ref_surface_shader_program().set_uniform(ctx, "map_color_to_material", 3);
		ctx.set_material(material_x);
		//ctx.set_color(rgb(1, 0, 0));
		ctx.tesselate_arrow(axis_root_vec3, axis_root_vec3 + translated_local_axis_x, 0.1, 2.0, 0.5);
		ctx.set_material(material_y);
		//ctx.set_color(rgb(0, 1, 0));
		ctx.tesselate_arrow(axis_root_vec3, axis_root_vec3 + translated_local_axis_y, 0.1, 2.0, 0.5);
		ctx.set_material(material_z);
		//ctx.set_color(rgb(0, 0, 1));
		ctx.tesselate_arrow(axis_root_vec3, axis_root_vec3 + translated_local_axis_z, 0.1, 2.0, 0.5);
		ctx.ref_surface_shader_program().disable(ctx);
		
		//glDepthMask(true);
	}
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		auto child = node->child_at(i);
		draw_skeleton_subtree(child, global_to_current_local, ctx, level+1, arrows, indicators);
	}
}

void SkeletonViewer::timer_event(double, double dt)
{
	if (animation && playing)
	{
		int fr;
		if (ianim)
			fr = 60;
		else
			fr = 120;
		animationTime += dt;
		int frame = (int)std::round(animationTime * fr) % animation->frame_count();
		animation->apply_frame(frame);
	}

	if (anim_to_record && recording) { // may be a problem
		recordingTime += dt;
		int frame = (int)std::round(recordingTime * 60) % 65535; // 
		anim_to_record->saveframe_to_vector(frame);
	}
}

void SkeletonViewer::start_animation() { playing = true; }
void SkeletonViewer::stop_animation() { playing = false; }

void SkeletonViewer::skeleton_changed(std::shared_ptr<SkinningSkeleton> s)
{
	// This function is called whenever the according signal of the
	// data store has been called.

	//Rebuild the tree-view
	generate_tree_view_nodes();

	//Fit view to skeleton
	//std::vector<cgv::render::view*> view_ptrs;
	//cgv::base::find_interface<cgv::render::view>(get_node(), view_ptrs);
	//if (view_ptrs.empty()) {
	//	// If there is no view, we cannot update it
	//	cgv::gui::message("could not find a view to adjust!!");
	//}
	//else
	//{
	//	Vec3 center = (s->getMin() + s->getMax()) * 0.5;
	//	view_ptrs[0]->set_focus(center.x(), center.y(), center.z());
	//	// Set the scene's size at the focus point
	//	view_ptrs[0]->set_y_extent_at_focus(s->getMax().y() - s->getMin().y());
	//}	

	data->get_skeleton()->clear_bone_list();
	jointlist.clear();
	jointlist_color.clear();
	compute_posi_jointlist_recur(s->get_root(), s->get_origin(), 0);// for intersection computation 


	//connect signals	
	recursive_connect_signals(s->get_root());	

	post_redraw();
}

void SkeletonViewer::recursive_connect_signals(Bone* b)
{
	for (int i = 0; i < b->dof_count(); ++i)
		connect(b->get_dof(i)->changed_signal, this, &SkeletonViewer::dof_changed);
	for (int i = 0; i < b->childCount(); ++i)
		recursive_connect_signals(b->child_at(i));
}

void SkeletonViewer::dof_changed(double)
{
	if (!data->dof_changed_by_ik) {
		data->set_endeffector(nullptr, 0);
		data->set_endeffector(nullptr, 1);
		data->set_endeffector(nullptr, 2);
	}
	if (!playing) {
		// this actually works! emulator can not simulate this....
		data->get_skeleton()->clear_bone_list();
		jointlist.clear();
		jointlist_color.clear();
		compute_posi_jointlist_recur(data->get_skeleton()->get_root(), data->get_skeleton()->get_origin(), 0);
	}
	should_apply_dofs_to_others_for_imitating = true;
	post_redraw();
}

void SkeletonViewer::generate_tree_view_nodes()
{
	tree_view->remove_all_children();
	gui_to_bone.clear();

	if (!data->get_skeleton() || !data->get_skeleton()->get_root())
		return;
	generate_tree_view_nodes(tree_view, data->get_skeleton()->get_root());
}

void SkeletonViewer::generate_tree_view_nodes(gui_group_ptr parent, Bone* bone)
{
	if (bone->childCount() == 0)
	{
		//If this is a leaf, use a button
		auto button = parent->add_button(bone->get_name(), "", "");
		gui_to_bone[button] = bone;
	}
	else
	{
		//If this is not a leaf, use a group
		auto g = parent->add_group(bone->get_name(), "", "", "");
		gui_to_bone[g] = bone;
		for (int i = 0; i < bone->childCount(); ++i)
			generate_tree_view_nodes(g, bone->child_at(i));
	}
}

void SkeletonViewer::start_choose_base()
{
	Bone* b = data->get_endeffector();
	data->set_endeffector(nullptr, 0);
	data->set_endeffector(nullptr, 1);
	data->set_endeffector(nullptr, 2);
	data->set_base(b);
}

void SkeletonViewer::tree_selection_changed(base_ptr p, bool select)
{
	bone_group->remove_all_children();

	if (select)		
	{
		Bone* bone = gui_to_bone.at(p);
		generate_bone_gui(bone);
		data->set_endeffector(bone,0);
	}
	else
	{
		data->set_endeffector(nullptr, 0);
		data->set_endeffector(nullptr, 1);
		data->set_endeffector(nullptr, 2);
	}
}

std::string SkeletonViewer::get_parent_type() const
{
	return "layout_group";
}

void SkeletonViewer::load_skeleton_given_name(std::string f) {
	if (!f.empty())
	{
		auto s = std::make_shared<SkinningSkeleton>();
		if (s->fromASFFile(f))
		{
			//s->set_origin_rotation(Vec3(0, 1, 0), 180);
			//s->set_origin_translation(Vec3(-1.5, 1, -2.8));// 1 meter high by default
			//s->set_scalefactor(scale_factor_skel); // can not change the scale here.
			data->set_skeleton(s);
			data->set_endeffector(nullptr, 0);
			data->set_endeffector(nullptr, 1);
			data->set_endeffector(nullptr, 2);
			data->set_base(s->get_root());

			/*data->get_skeleton()->clear_bone_list();
			jointlist.clear();
			jointlist_color.clear();
			compute_posi_jointlist_recur(s->get_root(), s->get_origin(), 0);*/// for intersection computation 
		}
		else
		{
			cgv::gui::message("Could not load specified ASF file.");
		}
	}
}

void SkeletonViewer::load_skeleton()
{
	std::string filename = cgv::gui::file_open_dialog("Open", "Skeleton Files (*.asf):*.asf");
	if (!filename.empty())
	{
		auto s = std::make_shared<SkinningSkeleton>();
		if (s->fromASFFile(filename))
		{
			//s->set_origin_rotation(Vec3(0, 1, 0), 180);
			//s->set_origin_translation(Vec3(-1.5, 1, -2.8));// 1 meter high by default

			//s->set_scalefactor(scale_factor_skel); // can not change the scale here.
			data->set_skeleton(s);
			data->set_endeffector(nullptr, 0);
			data->set_endeffector(nullptr, 1);
			data->set_endeffector(nullptr, 2);
			data->set_base(s->get_root());

			/*data->get_skeleton()->clear_bone_list();
			jointlist.clear();
			jointlist_color.clear();
			compute_posi_jointlist_recur(s->get_root(), s->get_origin(), 0);*/// for intersection computation 
		}
		else
		{
			cgv::gui::message("Could not load specified ASF file.");
		}
	}
}

void SkeletonViewer::write_pinocchio()
{
	if (!data->get_skeleton())
	{
		cgv::gui::message("An ASF skeleton has to be loaded first.");
		return;
	}

	std::string filename = cgv::gui::file_save_dialog("Save", "Pinocchio Skeleton (*.txt):*.txt");
	if (!filename.empty())
	{
		data->get_skeleton()->write_pinocchio_file(filename);
	}
}

void SkeletonViewer::load_pinocchio()
{
	if (!data->get_skeleton())
	{
		cgv::gui::message("An ASF skeleton has to be loaded first.");
		return;
	}

	std::string filename = cgv::gui::file_open_dialog("Open", "Pinocchio skeleton (*.out):*.out");
	if (!filename.empty())
	{
		data->get_skeleton()->read_pinocchio_file(filename);		
		skeleton_changed(data->get_skeleton());
	}
}

void SkeletonViewer::load_animation_given_name(std::string f,bool myanim)
{
	if (!data->get_skeleton())
	{
		cgv::gui::message("An ASF skeleton has to be loaded first.");
		return;
	}

	std::string filename = f;
	if (!filename.empty())
	{
		if (animation)
		{
			delete animation;
			animation = nullptr;
		}
		auto a = new Animation();
		if (a->read_amc_file(filename, data->get_skeleton().get(),myanim))
		{
			ianim = myanim;
			animationTime = 0;
			animation = a;
			playing = true;
		}
		else
		{
			delete a;
			cgv::gui::message("Could not load specified AMC file.");
		}
	}
}

void SkeletonViewer::load_animation()
{
	if (!data->get_skeleton())
	{
		cgv::gui::message("An ASF skeleton has to be loaded first.");
		return;
	}

	std::string filename = cgv::gui::file_open_dialog("Open", "Animation File (*.amc):*.amc");
	if (!filename.empty())
	{
		if (animation)
		{
			delete animation;
			animation = nullptr;
		}
		auto a = new Animation();
		if (a->read_amc_file(filename, data->get_skeleton().get(),false))
		{
			animationTime = 0;
			animation = a;
			playing = true;
		}
		else
		{
			delete a;
			cgv::gui::message("Could not load specified AMC file.");
		}
	}
}

// Create the gui elements
void SkeletonViewer::create_gui()
{	
	//Bone tree view
	parent_group->multi_set("layout='table';rows=3;spacings='normal';");	

	tree_view = add_group("", "tree_group", "h=300;column_heading_0='Bones';column_width_0=-1", "f");	
	bone_group = add_group("", "align_group", "h=150", "f");
	
	auto dock_group = add_group("", "dockable_group", "", "fF");

	connect(tree_view->on_selection_change, this, &SkeletonViewer::tree_selection_changed);

	generate_tree_view_nodes();

	//Other GUI elements
	auto gui_group = dock_group->add_group("", "align_group", "", "f");

	connect_copy(gui_group->add_button("Load ASF skeleton", "", "\n")->click,
		rebind(this, &SkeletonViewer::load_skeleton));	

	connect_copy(gui_group->add_button("Load Animation", "", "\n")->click,
		rebind(this, &SkeletonViewer::load_animation));

	connect_copy(gui_group->add_button("Start Animation", "", "\n")->click,
		rebind(this, &SkeletonViewer::start_animation));

	connect_copy(gui_group->add_button("Stop Animation", "", "\n")->click,
		rebind(this, &SkeletonViewer::stop_animation));

	connect_copy(gui_group->add_button("Choose IK Base", "", "\n")->click,
		rebind(this, &SkeletonViewer::start_choose_base));
		
	connect_copy(gui_group->add_button("Write Pinocchio skeleton", "", "\n")->click,
		rebind(this, &SkeletonViewer::write_pinocchio));

	connect_copy(gui_group->add_button("Load Pinocchio skeleton", "", "\n")->click,
		rebind(this, &SkeletonViewer::load_pinocchio));
}

void SkeletonViewer::generate_bone_gui(Bone* bone)
{
	// Add bone-specific gui elements to bone_group.
	// Use the layout "\n" to specify vertical alignment

	bone_group->add_view("Selected Bone", bone->get_name());

	for (int i = 0; i < bone->dof_count(); ++i)
	{
		auto dof = bone->get_dof(i);
		auto slider = bone_group->add_control<double>(dof->get_name(), dof.get(), "value_slider");
		slider->set("min", dof->get_lower_limit());
		slider->set("max", dof->get_upper_limit());
	}
}


// a better way: use an other func. to ruduce the work with recur. in draw call todo
// recur. func will be used to calcu. boxes as the skel. is defined
// will be re-computed when skel. changed. we can only do the calcu. after loading + skel. changed 
// boxes are stored in a static way
std::vector<cgv::media::axis_aligned_box<float, 3>> SkeletonViewer::get_jointlist() { // box3 not indexed pp-
	return jointlist;
}

void SkeletonViewer::draw(context& ctx)
{
	// moved to vr_test.cxx, beacause of the rendering of skybox
}
