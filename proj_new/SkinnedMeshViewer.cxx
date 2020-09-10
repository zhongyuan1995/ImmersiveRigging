// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include <cgv/base/base.h>
#include "SkinnedMeshViewer.h"

#include <cgv/gui/file_dialog.h>
#include <cgv/gui/dialog.h>
#include <cgv/render/view.h>
#include <cgv/base/find_action.h>

SkinnedMeshViewer::SkinnedMeshViewer(DataStore* data)
	: node("Mesh Viewer"), data(data)
{
	connect(data->mesh_changed, this, &SkinnedMeshViewer::mesh_changed);
}

bool SkinnedMeshViewer::init(context& ctx)
{
	cgv::render::gl::ensure_glew_initialized();

	if(!SkinningMesh::init_shaders(ctx))
		cgv::gui::message("Could not init shaders.");
	return true;
}

void SkinnedMeshViewer::mesh_changed(std::shared_ptr<SkinningMesh> m)
{
	////Fit view to skeleton
	//cgv::render::view* view_ptr = find_view_as_node();
	//if (view_ptr) {
	//	Vec3 center = (m->getMin() + m->getMax()) * 0.5;
	//	view_ptr->set_focus(center.x(), center.y(), center.z());
	//	// Set the scene's size at the focus point
	//	view_ptr->set_y_extent_at_focus(m->getMax().y() - m->getMin().y());
	//}
	//else
	//	// If there is no view, we cannot update it
	//	cgv::gui::message("could not find a view to adjust!!");

	post_redraw();
}

void SkinnedMeshViewer::create_gui()
{
	connect_copy(add_button("Load OBJ mesh", "", "\n")->click,
		rebind(this, &SkinnedMeshViewer::load_mesh));

	connect_copy(add_button("Load Pinocchio attachment", "", "\n")->click,
		rebind(this, &SkinnedMeshViewer::load_attachment));
}

void SkinnedMeshViewer::load_mesh()
{
	std::string filename = cgv::gui::file_open_dialog("Open", "OBJ Files (*.obj):*.obj");
	if (!filename.empty())
	{
		auto m = std::make_shared<SkinningMesh>();
		if (m->read_obj(filename.c_str()))
			data->set_mesh(m);
		else
		{
			cgv::gui::message("Could not load specified OBJ file.");
		}
	}
}

void SkinnedMeshViewer::load_attachment()
{
	if (!data->get_mesh())
	{
		cgv::gui::message("A mesh has to be loaded first.");
		return;
	}

	std::string filename = cgv::gui::file_open_dialog("Open", "Pinocchio attachment (*.out):*.out");
	if (!filename.empty())
	{
		data->get_mesh()->read_attachment(filename);
		post_redraw();
	}
}


void SkinnedMeshViewer::draw(context& c)
{
	glEnable(GL_CULL_FACE);
	glDisable(GL_BLEND);

	if (data->get_mesh())
	{
		if (data->get_skeleton())
		{
			std::vector<Mat4> skinning_matrices;
			data->get_skeleton()->get_skinning_matrices(skinning_matrices);
			data->get_mesh()->set_skinning_matrices(skinning_matrices);
		}
		data->get_mesh()->draw(c);
	}
}