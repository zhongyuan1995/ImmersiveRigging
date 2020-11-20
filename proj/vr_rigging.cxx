#include "vr_rigging.h" 
 
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/dialog.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cg_vr/vr_events.h>

#include <random>

#include "intersection.h"
#include <math_helper.h>


void vr_rigging::load_mesh_1() {
	mesh_dir = data_dir + "/gen_dataset/speider_simple0/spiderman.obj";

	// clear varibles 
	start_point_list.clear();
	end_point_list.clear();
	mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	mmesh->set_mesh_scale(0.019f);

	// set mesh 
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::load_mesh_2() {
	mesh_dir = data_dir + "/gen_dataset/" + "horse_simple0/horse.obj";

	// clear varibles 
	start_point_list.clear();
	end_point_list.clear();
	mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	mmesh->set_mesh_scale(1);

	// set mesh 
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::load_mesh_3() {
	mesh_dir = data_dir + "/gen_dataset/pinocchio_model1_0/mesh.obj";

	// clear varibles 
	start_point_list.clear();
	end_point_list.clear();
	mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	mmesh->set_mesh_scale(1);

	// set mesh 
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::load_mesh_4() {
	mesh_dir = data_dir + "/gen_dataset/pinocchio_model6_0/mesh.obj";

	// clear varibles 
	start_point_list.clear();
	end_point_list.clear();
	mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	mmesh->set_mesh_scale(1);

	// set mesh 
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::load_mesh_5() {
	mesh_dir = data_dir + "/gen_dataset/robot_0/mesh.obj";

	// clear varibles 
	start_point_list.clear();
	end_point_list.clear();
	mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	mmesh->set_mesh_scale(1);

	// set mesh 
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
/// 
void vr_rigging::translation_to_desired_posi() {
	mmesh->set_mesh_translation(cur_left_hand_posi);
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);
}
/// 
void vr_rigging::orient_to_desired_ori() {
	mat3 cur_ori = mmesh->get_mesh_orientation();
	mat3 new_rot = cgv::math::rotate3<double>(90, vec3(0, 1, 0));
	mmesh->set_mesh_orientation(new_rot * cur_ori);
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);
}
///
void vr_rigging::toggle_mesh_render() {
	b_render_mesh = !b_render_mesh;
	if (b_render_mesh)
		label_content = "[INFO] show mesh successfully\n" + label_content;
	else
		label_content = "[INFO] hide mesh successfully\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::toggle_mesh_transparent() {
	ds->get_mesh()->b_set_transparent = !ds->get_mesh()->b_set_transparent;
	if (ds->get_mesh()->b_set_transparent)
		label_content = "[INFO] transparent on\n" + label_content;
	else
		label_content = "[INFO] transparent off\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::toggle_mesh_wireframe() {
	ds->get_mesh()->b_set_polygonmode = !ds->get_mesh()->b_set_polygonmode;
	if (ds->get_mesh()->b_set_polygonmode)
		label_content = "[INFO] wireframe button on!\n" + label_content;
	else
		label_content = "[INFO] wireframe button off!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::toggle_face_culling() {
	ds->get_mesh()->b_face_culling = !ds->get_mesh()->b_face_culling;
	post_redraw();
}
///
void vr_rigging::toggle_other_twoskel() {
	b_toggle_show_imitating_skel = !b_toggle_show_imitating_skel;
}
///
void vr_rigging::toggle_imitating() {
	b_toggle_imitating = !b_toggle_imitating;
}
///
void vr_rigging::del_skel() {
	// del. bones 
	for (auto& b : ds->get_skeleton()->get_bone_list()) {
		if (!b->get_name()._Equal("root")) { // del all bones except root bone, length 0, degenerated 
			b->get_parent()->remove_a_child(b);
		}
	}
	jointlist.clear(); // tobetested 

	skel_view->skeleton_changed(ds->get_skeleton());

	label_content = "[INFO] bones deleted\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::load_skel_with_dofs() { // those three skel. should be added at the same time 
	// editable one 
	if (skel_view) {
		skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/jump.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/jump.asf");
		post_redraw();

		label_content = "[INFO] demo skel. loaded\n" + label_content;
		label_outofdate = true;
	}
}
///
void vr_rigging::load_demo_skel1() {
	from_jump_asf = false;
	left_ee = right_ee = hmd_ee = nullptr;
	start_point_list.clear();
	end_point_list.clear();

	skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.asf");
	skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
	load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.asf");
	post_redraw();

	label_content = "[INFO] button clicked!\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::load_demo_skel2() {
	from_jump_asf = false;
	left_ee = right_ee = hmd_ee = nullptr;
	start_point_list.clear();
	end_point_list.clear();

	skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.asf");
	skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
	load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.asf");
	post_redraw();

	label_content = "[INFO] button clicked!\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::load_demo_skel3() {
	from_jump_asf = false;
	left_ee = right_ee = hmd_ee = nullptr;
	start_point_list.clear();
	end_point_list.clear();

	skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.asf");
	skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
	load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.asf");
	post_redraw();

	label_content = "[INFO] button clicked!\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::take_screen_capture() {
	context* ctx = get_context();
	if (ctx == 0)
		return;
	ctx->write_frame_buffer_to_image(data_dir + "/gen_dataset/" + "speider_simple0/screen_capture.png");
}
///
void vr_rigging::load_demo_animation() {
	skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/jump.amc", false);
}
///
void vr_rigging::load_stored_anim1() {
	skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_1.amc", true);
}
///
void vr_rigging::load_stored_anim2() {
	skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_2.amc", true);
}
///
void vr_rigging::load_stored_anim3() {
	skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_3.amc", true);
}
///
void vr_rigging::stop_anim() {

}
///
void vr_rigging::start_record() {
	skel_view->prepare_record_anim();
	skel_view->start_record_anim();

	label_content = "[INFO] recording...\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::stop_record_and_save() {
	skel_view->stop_record_anim(data_dir + "/gen_dataset/" + "speider_simple0/test.amc");
	label_content = "[INFO] animation has been exported as 'test.amc' \n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::save_curskel_to_file() {
	if (ds->get_skeleton()) {
		ds->get_skeleton()->write_pinocchio_file(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel.txt");
		//ds->get_skeleton()->set_origin_rotation();
		// problem occour when rigging with pinocchio, write pinocchio with an other coordi.
		if (!from_jump_asf)
			ds->get_skeleton()->writeASFFile(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel.asf");
	}
	label_content = "[INFO] created skel. has been saved \nto tmp. file as 'tmpskel.asf'\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::apply_rigged_skel()
{
	if (from_jump_asf) { // do not have to adjest skel. in vr 
		ds->get_skeleton()->read_pinocchio_file(data_dir + "/gen_dataset/" + "speider_simple0/adjested_skeleton.out");
	}
	skel_view->skeleton_changed(ds->get_skeleton());
	mmesh->read_attachment(data_dir + "/gen_dataset/" + "speider_simple0/skinned_attachment.out");
	post_redraw();
	label_content = "[INFO] attachment has been loaded! mesh skinned\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::apply_dofs() {
	if (ds->get_skeleton() && tmpdata_1->get_skeleton() && tmpdata_2->get_skeleton()) {
		tmpdata_1->get_skeleton()->apply_dofs_given_skel_pointer(ds->get_skeleton().get());
		tmpdata_2->get_skeleton()->apply_dofs_given_skel_pointer(ds->get_skeleton().get());
	}
}
///
void vr_rigging::build_skel() {
	// for the skel. we created
}
///
void vr_rigging::gen_asf_skel_file() {
	ds->get_skeleton()->writeASFFile(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel.asf");
}
///
void vr_rigging::start_autorigging_pinoccio() {
	label_content = "[INFO] autorig started! This may take a while...\n" + label_content;
	label_outofdate = true;
	int i;
	ArgData a;

	a.filename = mesh_dir;
	string skelfile = data_dir + "/gen_dataset/" + "speider_simple0/tmpskel.txt";
	a.skeleton = FileSkeleton(skelfile);
	a.skeletonname = skelfile;

	Mesh m(a.filename);
	if (m.vertices.size() == 0) {
		cout << "Error reading file.  Aborting." << endl;
		exit(0);
		return;
	}

	for (i = 0; i < (int)m.vertices.size(); ++i)
		m.vertices[i].pos = a.meshTransform * m.vertices[i].pos;
	m.normalizeBoundingBox();
	m.computeVertexNormals();

	Skeleton given = a.skeleton;
	given.scale(a.skelScale * 0.7);

	//if (a.stopAtMesh) { //if early bailout
	//	addMesh(new StaticDisplayMesh(m));
	//	return;
	//}

	PinocchioOutput o;
	if (!a.noFit) { //do everything
		o = autorig(given, m);
	}
	else { //skip the fitting step--assume the skeleton is already correct for the mesh
		TreeType* distanceField = constructDistanceField(m);
		VisTester<TreeType>* tester = new VisTester<TreeType>(distanceField);

		o.embedding = a.skeleton.fGraph().verts;
		for (i = 0; i < (int)o.embedding.size(); ++i)
			o.embedding[i] = m.toAdd + o.embedding[i] * m.scale;

		o.attachment = new Attachment(m, a.skeleton, o.embedding, tester);

		delete tester;
		delete distanceField;
	}

	if (o.embedding.size() == 0) {
		cout << "Error embedding" << endl;
		exit(0);
	}

	/*if (a.motionname.size() > 0) {
		addMesh(new DefMesh(m, given, o.embedding, *(o.attachment), new Motion(a.motionname)));
	}
	else {
		addMesh(new StaticDisplayMesh(m));

		for (i = 1; i < (int)o.embedding.size(); ++i)
		{
			addLine(LineSegment(o.embedding[i], o.embedding[given.fPrev()[i]], Vector3(.5, .5, 0), 4.));
		}
	}*/

	//output skeleton embedding
	for (int i = 0; i < (int)o.embedding.size(); ++i)
		o.embedding[i] = (o.embedding[i] - m.toAdd) / m.scale;
	ofstream oss(data_dir + "/gen_dataset/" + "speider_simple0/adjested_skeleton.out");
	for (i = 0; i < (int)o.embedding.size(); ++i) {
		oss << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] <<
			" " << o.embedding[i][2] << " " << a.skeleton.fPrev()[i] << endl;
	}

	//output attachment
	ofstream astrm(data_dir + "/gen_dataset/" + "speider_simple0/skinned_attachment.out");
	for (i = 0; i < (int)m.vertices.size(); ++i) {
		Vector<double, -1> v = o.attachment->getWeights(i);
		for (int j = 0; j < v.size(); ++j) {
			double d = std::floor(0.5 + v[j] * 10000.) / 10000.;
			astrm << d << " ";
		}
		astrm << endl;
	}

	delete o.attachment;

	label_content = "[INFO] autorig finished! attachments are exported!\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::adjest_mesh_scale() {
	float height_of_the_mesh = ds->get_mesh()->getMax().y() - ds->get_mesh()->getMin().y();
	float height_of_the_avater = hmd_origin.y();
	float new_factor = height_of_the_avater / height_of_the_mesh;
	cout << "height_of_the_mesh: " << height_of_the_mesh << endl;
	cout << "height_of_the_avater: " << height_of_the_avater << endl;
	cout << "new_factor: " << new_factor << endl;
	if (new_factor > 0.0001f)
		ds->get_mesh()->set_mesh_scale(new_factor);

	// re-load mesh
	mmesh->read_obj(mesh_dir.c_str());
	ds->set_mesh(mmesh);

	// update info board 
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::shuffle_frame() {
	//cur_local_frame_rot_rel_XYZ
	vec3 bonedir_inworldspace = vec3(1, 1, 1);
	vec3 x_asix_dir = vec3(1, 0, 0);
	bonedir_inworldspace.normalize();
	x_asix_dir.normalize();
	vec3 rot_axis = cross(x_asix_dir, bonedir_inworldspace);
	rot_axis.normalize();
	float rot_degree = acos(dot(bonedir_inworldspace, x_asix_dir)) * 180 / PI;
	mat3 rot_mat = rotate3(rot_degree, rot_axis);
	// from http://planning.cs.uiuc.edu/node103.html
	float yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
	float pitch = atan2(-rot_mat(2, 0), sqrt(pow(rot_mat(2, 1), 2) + pow(rot_mat(2, 2), 2)));
	float roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
	cur_local_frame_rot_rel_XYZ[0] = roll * 180 / PI;
	cur_local_frame_rot_rel_XYZ[1] = pitch * 180 / PI;
	cur_local_frame_rot_rel_XYZ[2] = yaw * 180 / PI;

	post_redraw();
}
///
void vr_rigging::remove_pg1() {
	pg1->elements.clear();
	pg1->boxvector.clear();
	pg1->colorvector.clear();
	post_redraw();
}
///
void vr_rigging::translate_model_in_y_dir_upwards() {
	ds->get_mesh()->set_orientation_translation(
		cgv::math::rotate3<double>(180.0f, vec3(0, 1, 0)),
		vec3(1.5, (ds->get_mesh()->getMax().y() - ds->get_mesh()->getMin().y()) / 2.0f, 0));
	load_mesh();

	label_content = "[INFO] mesh position adjested!\n" + label_content;
	label_outofdate = true;
}
///
void vr_rigging::load_addi_two_guys(string f) {
	tmpskel_view_1->load_skeleton_given_name(f);
	tmpskel_view_1->set_skel_origin_ori_translation(Vec3(0, 1, 0), 90, Vec3(-2, 1, -1.2));

	tmpskel_view_2->load_skeleton_given_name(f);
	tmpskel_view_2->set_skel_origin_ori_translation(Vec3(0, 1, 0), 45, Vec3(-2, 1, -2.8));
}
///
void vr_rigging::load_mesh() {
	g_mesh_filename = cgv::gui::file_open_dialog("Open", "OBJ Files (*.obj):*.obj");
	mmesh->read_obj(g_mesh_filename.c_str());
	ds->set_mesh(mmesh);
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
void vr_rigging::load_mesh_with_gui() {
	mesh_dir = data_dir + "/gen_dataset/" + "speider_simple0/spiderman.obj";
	mmesh->read_obj(mesh_dir.c_str());
	//mmesh->read_obj(g_mesh_filename.c_str());
	ds->set_mesh(mmesh);
	label_content = "[INFO] mesh loaded!\n" + label_content;
	label_outofdate = true;
	post_redraw();
}
///
cgv::render::render_types::mat3 vr_rigging::from_global_roll_yaw_pitch_vec_to_matrix() {
	vec3 tmp_roll_yaw_pitch_vec = vec3(
		cur_local_frame_rot_rel_XYZ[0],
		cur_local_frame_rot_rel_XYZ[1],
		cur_local_frame_rot_rel_XYZ[2]
	);
	return rotate3(tmp_roll_yaw_pitch_vec);
}
///
cgv::render::render_types::mat3 vr_rigging::compute_matrix_from_two_dirs(vec3 a, vec3 b) {
	a.normalize();
	b.normalize();
	vec3 rot_axis = cross(a, b);
	rot_axis.normalize();
	float rot_degree = acos(dot(a, b)) * 180 / PI;
	return rotate3(rot_degree, rot_axis);
}
///
void vr_rigging::from_matrix_to_euler_angle_as_global_var(mat3 rot_mat) {
	// from http://planning.cs.uiuc.edu/node103.html
	float yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
	float pitch = atan2(-rot_mat(2, 0), sqrt(pow(rot_mat(2, 1), 2) + pow(rot_mat(2, 2), 2)));
	float roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
	cur_local_frame_rot_rel_XYZ[0] = roll * 180 / PI;
	cur_local_frame_rot_rel_XYZ[1] = pitch * 180 / PI;
	cur_local_frame_rot_rel_XYZ[2] = yaw * 180 / PI;
}
///
void vr_rigging::init_cameras(vr::vr_kit* kit_ptr)
{
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	nr_cameras = camera_ptr->get_nr_cameras();
	frame_split = camera_ptr->get_frame_split();
	for (int i = 0; i < nr_cameras; ++i) {
		std::cout << "camera " << i << "(" << nr_cameras << "):" << std::endl;
		camera_ptr->put_camera_intrinsics(i, false, &focal_lengths[i](0), &camera_centers[i](0));
		camera_ptr->put_camera_intrinsics(i, true, &focal_lengths[2 + i](0), &camera_centers[2 + i](0));
		std::cout << "  fx=" << focal_lengths[i][0] << ", fy=" << focal_lengths[i][1] << ", center=[" << camera_centers[i] << "]" << std::endl;
		std::cout << "  fx=" << focal_lengths[2+i][0] << ", fy=" << focal_lengths[2+i][1] << ", center=[" << camera_centers[2+i] << "]" << std::endl;
		float camera_to_head[12];
		camera_ptr->put_camera_to_head_matrix(i, camera_to_head);
		kit_ptr->put_eye_to_head_matrix(i, camera_to_head);
		camera_to_head_matrix[i] = vr::get_mat4_from_pose(camera_to_head);
		std::cout << "  C2H=" << camera_to_head_matrix[i] << std::endl;
		camera_ptr->put_projection_matrix(i, false, 0.001f, 10.0f, &camera_projection_matrix[i](0, 0));
		camera_ptr->put_projection_matrix(i, true, 0.001f, 10.0f, &camera_projection_matrix[2+i](0, 0));
		std::cout << "  dP=" << camera_projection_matrix[i] << std::endl;
		std::cout << "  uP=" << camera_projection_matrix[2+i] << std::endl;
	}
	post_recreate_gui();
}
///
void vr_rigging::start_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->start())
		cgv::gui::message(camera_ptr->get_last_error());
}
///
void vr_rigging::stop_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->stop())
		cgv::gui::message(camera_ptr->get_last_error());
}
/// compute intersection points of controller ray with movable boxes
void vr_rigging::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		vec3 origin_box_i = origin - movable_box_translations[i];
		movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			movable_boxes[i],
			t_result, p_result, n_result, 0.000001f)) {

			// transform result back to world coordinates
			movable_box_rotations[i].rotate(p_result);
			p_result += movable_box_translations[i];
			movable_box_rotations[i].rotate(n_result);

			// store intersection information
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}
///
void vr_rigging::gui_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < pg1->boxvector.size(); ++i) {
		vec3 origin_box_i = origin;
		//movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		//movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			pg1->boxvector.at(i),
			t_result, p_result, n_result, 0.000001f)) {

			// store intersection information
			gui_intersection_points.push_back(p_result);
			gui_intersection_colors.push_back(color);
			gui_intersection_box_indices.push_back((int)i);
			gui_intersection_controller_indices.push_back(ci);
		}
	}
}
void vr_rigging::reset_jointlist_color() { // reset after loading from skel_view 
	jointlist_colors = jointlist_colors_standard;
}
///
void vr_rigging::skel_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color) {
	// put all boxes into a vector<box3> list, similar to the draw call in skelviewer 
	// it was "static" var. getted from skel_view obj. so this will not take too much effort
	if (skel_view) { // new added: stack-overflow
		jointlist = skel_view->get_jointlist();
		jointlist_colors = skel_view->get_jointlistcolor();
		for (size_t i = 0; i < jointlist.size(); ++i) {
			vec3 origin_box_i = origin;
			//movable_box_rotations[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			//movable_box_rotations[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3  p_result;
			vec3  n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(
				origin_box_i, direction_box_i,
				jointlist.at(i),
				t_result, p_result, n_result, 0.000001f)) {
				skel_intersection_points.push_back(p_result);
				skel_intersection_box_indices.push_back((int)i);
			}
		}
	}
}
///
void vr_rigging::fast_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color) {
	for (size_t i = 0; i < fast_jointlist.size(); ++i) {
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin, direction,
			fast_jointlist.at(i),
			t_result, p_result, n_result, 0.000001f)) {
			fast_intersection_points.push_back(p_result);
			fast_intersection_box_indices.push_back((int)i);
		}
	}
}
/// keep track of status changes
void vr_rigging::on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status)
{
	// ignore all but left controller changes
	if (ci != 0)
		return;
	vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
	// check for attaching of controller
	if (old_status == vr::VRS_DETACHED) {
		left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
		post_recreate_gui();
	}
	// check for attaching of controller
	if (new_status == vr::VRS_DETACHED) {
		left_inp_cfg.clear();
		post_recreate_gui();
	}
}
/// register on device change events
void vr_rigging::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			init_cameras(kit_ptr);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				// copy left controller input configurations from new device in order to make it adjustable
				left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
				for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
					left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}
/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_rigging::construct_table(float tw, float td, float th, float tW) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	// construct table
	rgb table_clr = rgb(// 26, 82, 45
		26 / 255.0f,
		82 / 255.0f,
		45 / 255.0f
	);
	float zdiff = -2.8f;
	float xdiff = -0.5f;
	boxes.push_back(box3(
		vec3(-0.5f * tw - 2 * tW + xdiff, th, -0.5f * td - 2 * tW + zdiff),
		vec3(0.5f * tw + 2 * tW + xdiff, th + tW, 0.5f * td + 2 * tW + zdiff)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f * tw + xdiff, 0, -0.5f * td + zdiff), vec3(-0.5f * tw - tW + xdiff, th, -0.5f * td - tW + zdiff)));
	boxes.push_back(box3(vec3(-0.5f * tw + xdiff, 0, 0.5f * td + zdiff), vec3(-0.5f * tw - tW + xdiff, th, 0.5f * td + tW + zdiff)));
	boxes.push_back(box3(vec3(0.5f * tw + xdiff, 0, -0.5f * td + zdiff), vec3(0.5f * tw + tW + xdiff, th, -0.5f * td - tW + zdiff)));
	boxes.push_back(box3(vec3(0.5f * tw + xdiff, 0, 0.5f * td + zdiff), vec3(0.5f * tw + tW + xdiff, th, 0.5f * td + tW + zdiff)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_rigging::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if(walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if(ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}
/// construct boxes for environment
void vr_rigging::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f * ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f * ed;
			/*if ( (x + 0.5f*s > -0.5f*w && x < 0.5f*w + 1) && (z + 0.5f*s > -0.5f*d && z < 0.5f*d) )
				continue;*/
			if ((x + 0.5f * s > -0.5f * w && x < 0.5f * w) && (z + 0.5f * s > -0.5f * d && z < 0.5f * d))
				continue;
			float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x + s, h, z + s)));
			box_colors.push_back(
				rgb(
					0.4f * distribution(generator) + 0.1f,
					0.4f * distribution(generator) + 0.3f,
					0.4f * distribution(generator) + 0.1f
				)
			);
		}
	}
}
/// construct boxes that can be moved around
void vr_rigging::construct_left_hand_box()
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);

	vec3 extent(tmpboxsize);
	vec3 center(0);
	vec3 demoposi = vec3(0, 0, -0.2f);
	vec3 posi1 = vec3(-0.2, 0.2f, 0);
	vec3 posi2 = vec3(0.2, 0.2f, 0); // z can not be positive??

	/*vec3 pos = cur_left_hand_posi;
	mat3 rotation = cur_left_hand_rot;*/

	// box1 
	movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
	movable_box_colors.push_back(rgb(distribution(generator),
		distribution(generator),
		distribution(generator)));
	movable_box_translations.push_back(cur_left_hand_posi + demoposi);
	quat rot(cur_left_hand_rot);
	rot.normalize();
	movable_box_rotations.push_back(rot);
}
/// construct boxes that can be moved around
void vr_rigging::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for(size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.01f;
		extent *= std::min(tw, td)*0.1f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}
/// construct a scene with a table
void vr_rigging::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	construct_table(tw, td, th, tW);
	construct_environment(0.2f, 1.3 * w, 1.3 * d, h, w, d, h); // performance issue
	construct_movable_boxes(tw, td, th, tW, 50);
	construct_boxgui();
}
///
void vr_rigging::construct_boxgui() {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	rgb writecol = rgb(1, 1, 1);
	rgb bkgcol = rgb(0.7, 0.6, 0.7);
	rgb col1 = rgb(100 / 255.0f, 46 / 255.0f, 128 / 255.0f);
	float font_size = 100; // pp- yzy font will become small if all are 150. 2h
	float smallbox_font_size = 100;
	float smaller_f = 25;

	boxgui_button first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.8f, -3.0f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "<", 120, "xxx", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.8f, -2.5f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), ">", 120, "xxx", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Change Skybox", font_size, "xxx", true);
	pg1->elements.push_back(first_btn);

	string image2dir = data_dir + "/skybox/cm_xp.jpg";
	string image0dir = data_dir + "/skybox/BluePinkNebular_xp.jpg";
	string image1dir = data_dir + "/skybox/igen_2/xp.jpg";

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"xxx", 0, image0dir, false);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.5f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"xxx", 0, image1dir, false);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.25f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"xxx", 0, image2dir, false);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Save/Load Skel.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"demoskel", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_skel1", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_skel2", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_skel3", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 2, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save Skel.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_skel1", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_skel2", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_skel3", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Save/Load Animation", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"start_rec", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"pause_rec", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"stop_rec", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_anim1", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_anim2", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_anim3", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.5f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save Animation", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_anim1", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_anim2", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 8), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_anim3", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 9), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"stop_anim", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Bone Edit", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	//-----------------------------add to existing skel.-------------------//
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"add_bone_\nexist", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"add_\nroot", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"del_bone_\nexist", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"edit_bone_\nexist", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"backward", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"del_all", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"f_del_\nall", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"f_back", smallbox_font_size, "D:/icon_res/default.png", true);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Mode for Left Control.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"adjest_\ncoordi.", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"reset_\nall_\nadjestment", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"scale_\njointbox", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"build_bone", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"shuffle_\nlocal_frame", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn); 
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"def_local\n_frame", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"def_min\n_dof", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"def_max\n_dof", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 8), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"shuffle_dof\n_def", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	//adjest_bone\n_exist
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 9), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"adjest_bone\n_exist", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f+ 0.5f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
		"s_anim3", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Mesh Style", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_demo1", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_demo2", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_demo3", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_demo4", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"l_demo5", smallbox_font_size - 20, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"scale_\nadjestment", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"reset_\nmesh", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
		"demo8", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f+ 1, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Mesh Style", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f+ 1, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"show_\nwireframe", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f+ 1, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"show_\nvertices", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"trans.", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	//vec3 center_gan, extend_gan, center_slider, extend_slider; box3 gan; box3 slider;
	//center_gan = vec3(2.45f - 0.3 - 0.6, 2.0f, -1.75f + 0.25f * 2);
	//extend_gan = vec3(0.6, 0.01, 0.01);
	//gan = box3(vec3(center_gan - extend_gan), vec3(center_gan + extend_gan));
	//boxes.push_back(gan);
	//box_colors.push_back(rgb(1, 0, 0));

	//center_slider = vec3(2.45f - 0.3 - 0.6, 2.0f, -1.75f + 0.25f * 2); // adjestable x 
	//extend_slider = vec3(0.05);
	//slider = box3(vec3(center_slider - extend_slider), vec3(center_slider + extend_slider));
	//boxes.push_back(slider);
	//box_colors.push_back(rgb(1, 0, 0)); 

	////---------- y
	//center_gan = vec3(2.45f - 0.3 - 0.6 - 1.5 * 1, 2.0f, -1.75f + 0.25f * 2);
	//extend_gan = vec3(0.6, 0.01, 0.01);
	//gan = box3(vec3(center_gan - extend_gan), vec3(center_gan + extend_gan));
	//boxes.push_back(gan);
	//box_colors.push_back(rgb(0, 1, 0));

	//center_slider = vec3(2.45f - 0.3 - 0.6 - 1.5 * 1, 2.0f, -1.75f + 0.25f * 2); // adjestable x 
	//extend_slider = vec3(0.05);
	//slider = box3(vec3(center_slider - extend_slider), vec3(center_slider + extend_slider));
	//boxes.push_back(slider);
	//box_colors.push_back(rgb(0, 1, 0));

	////------------- z
	//center_gan = vec3(2.45f - 0.3 - 0.6 - 1.5 * 2 , 2.0f, -1.75f + 0.25f * 2);
	//extend_gan = vec3(0.6, 0.01, 0.01);
	//gan = box3(vec3(center_gan - extend_gan), vec3(center_gan + extend_gan));
	//boxes.push_back(gan);
	//box_colors.push_back(rgb(0, 0, 1));

	//center_slider = vec3(2.45f - 0.3 - 0.6 - 1.5 * 2, 2.0f, -1.75f + 0.25f * 2); // adjestable x 
	//extend_slider = vec3(0.05);
	//slider = box3(vec3(center_slider - extend_slider), vec3(center_slider + extend_slider));
	//boxes.push_back(slider);
	//box_colors.push_back(rgb(0, 0, 1));
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"rotation.", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	rgb cur_color = rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f);
	first_btn = boxgui_button(vec3(2.45f, 2.0f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, cur_color,
		"scale", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"toggle\nmesh", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"toggle\ntransparent", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"toggle\nwireframe", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 1, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"toggle\nface_culling", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Skinning", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"write_\npinocchio\n_skel.", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"autorig_\nwith_\npinocchio", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"autorig_\nwith_heat_\ndiffusion", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"apply_\nattachments", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);


	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f, -2.5f), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), 
		"IK", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_base", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_ee_left", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_ee_right", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
		"s_ee_head", smallbox_font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);

	//---------------------------------------------------------------second part of gui------------------------//
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "Load Scene", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f, 2.5, 0.5f));
	//pg1->elements.push_back(first_btn);
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"l_scene1", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 0, 2.5, 0.5f));
	//pg1->elements.push_back(first_btn);

	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"toggle_addi\n_skel", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 1, 2.5, 0.5f));
	//pg1->elements.push_back(first_btn);

	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"toggle_\nimitating", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 2, 2.5, 0.5f));
	//pg1->elements.push_back(first_btn);
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f), "IK", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"ccd", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 0, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"opti. ccd", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 1, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"posing", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 2, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	////
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"s_base", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 3, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	////
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"s_ee_left", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 4, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	////
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"s_ee_right", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 5, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	////
	//first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
	//	"s_ee_head", smallbox_font_size, "D:/icon_res/default.png", true);
	//first_btn.do_transform = true;
	//first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	//first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 6, 2.5 - 0.25 * 1, 0.5f));
	//pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Skel. Retargeting", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	first_btn.do_transform = true;
	first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	first_btn.set_trans(vec3(2.25f - 0.5f, 2.5 - 0.25 * 2, 0.5f));
	pg1->elements.push_back(first_btn);*/

	pg1->push_to_render_vector();
}
///
void vr_rigging::render_confirm_panel() {
		int smaller_f = 25;
		int smallbox_font_size = 50;

		boxgui_button first_btn = boxgui_button(vec3(confirm_gui_posi.x() - 0.2f, confirm_gui_posi.y(), confirm_gui_posi.z()), 0.1, 0.4, 0.4, rgb(0, 1, 0),
			"This will overwrite \nthe existing file!\nContinue?", smaller_f, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		/*first_btn = boxgui_button(vec3(confirm_gui_posi.x() - 0.2f, confirm_gui_posi.y(), confirm_gui_posi.z() - 0.35f), 0.1, 0.2, 0.2, rgb(0, 1, 0),
			"yes", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(confirm_gui_posi.x() - 0.2f, confirm_gui_posi.y(), confirm_gui_posi.z() + 0.35f), 0.1, 0.2, 0.2, rgb(0, 1, 0),
			"no", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
		pg1->push_to_render_vector();
		post_redraw();
}
///
cgv::render::render_types::vec3 vr_rigging::compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction)
{
	float t_result;
	vec3  p_result = vec3(0);
	vec3  n_result;
	box3 floor = box3(vec3(-3, -1, -4), vec3(3, 0, 4));
	if (cgv::media::ray_axis_aligned_box_intersection(
		origin, direction,
		floor,
		t_result, p_result, n_result, 0.000001f)) {
		return p_result;
	}

	return p_result;
}
///
vr_rigging::vr_rigging()
{
	frame_split = 0;
	extent_texcrd = vec2(0.5f, 0.5f);
	center_left  = vec2(0.5f,0.25f);
	center_right = vec2(0.5f,0.25f);
	seethrough_gamma = 0.33f;
	frame_width = frame_height = 0;
	background_distance = 2;
	background_extent = 2;
	undistorted = true;
	shared_texture = true;
	max_rectangle = false;
	nr_cameras = 0;
	camera_tex_id = -1;
	camera_aspect = 1;
	use_matrix = true;
	show_seethrough = false;
	set_name("vr_test");
	build_scene(5, 7, 3, 0.2f, 0.8f, 0.8f, 0.72f, 0.03f);
	vr_view_ptr = 0;
	ray_length = 2;
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_rigging::on_device_change);
	connect(cgv::gui::ref_vr_server().on_status_change, this, &vr_rigging::on_status_change);

	srs.radius = 0.005f;

	label_outofdate = true;
	label_text = "Info Board"; // change the content with this varible! 
	label_font_idx = 0;
	label_upright = true;
	label_face_type = cgv::media::font::FFA_BOLD;
	label_resolution = 512;
	label_size = 30.0f;
	label_color = rgb(1, 1, 1);

	ulabel_outofdate = true;
	ulabel_text =
		"Usage Description:\n"
		"    grasp button on left controller -> move around the scene\n"
		"    grasp button on right controller -> interact with buttons\n"
		"    touch on left controller -> main operations(eg. edit bones)\n"
		"    stick events on right controller -> adjest the joint size\n"
		"        +y -> enlarge, -y -> reduce\n"
		"    please change the operation mode first by clicking the btns!\n"
		"Key Functionalities:\n"
		"    1)load mesh\n"
		"        locate the obj file with explorer, the working folder is\n"
		"        defined in this way.\n"
		"    2)define skeleton\n"
		"        first, click on the 'add_root' button, touch to add the \n"
		"        'root' bone. Then, click on the 'add_bone_exist' button,\n"
		"        touch once on left controller to set the parent bone, \n"
		"        touch twice to add the new bone to the skeleton.\n"
		"    3)autorig with pinocchio(integrated within the framework)\n"
		"        after a skeleton is defined, we export the skel. to disk\n"
		"        such that we can use it for autorig with pinocchio.\n"
		"    4)motion capture\n"
		"        first, we have to define the position of end effectors.\n"
		"        when base/left_ee/right_ee are all set, we touch on both\n"
		"        controller to start IK process (with ccd or etc.)\n"
		"    5)record/load animation\n"
		"        animation can be recorded by clicking 'start_rec' button \n"
		"        and s_animX to save it to corresponding file in the folder\n"
		"About: designed for immersive rigging, skinning and animation.\n"
		; // change the content with this varible! 
	ulabel_font_idx = 0;
	ulabel_upright = true;
	ulabel_face_type = cgv::media::font::FFA_BOLD;
	ulabel_resolution = 512;
	ulabel_size = 15.0f;
	ulabel_color = rgb(1, 1, 1);

	cgv::media::font::enumerate_font_names(ufont_names);
	ufont_enum_decl = "enums='";
	ufont_enum_decl += "'";
	ulabel_face_type = cgv::media::font::FFA_BOLD;
	ulabel_font_face = cgv::media::font::find_font(ufont_names[0])->get_font_face(ulabel_face_type);

	cgv::media::font::enumerate_font_names(font_names);
	font_enum_decl = "enums='";
	for (unsigned i = 0; i < font_names.size(); ++i) {
		if (i > 0)
			font_enum_decl += ";";
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == "calibri") {
			label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
			//label_font_idx = i;
			if(pg1)
				for (int i = 0; i < pg1->elements.size(); i++) {
					if (pg1->elements.at(i).flag_use_label)
						pg1->elements.at(i).gui_label_texture->label_font_idx = i;
				}
		}
		font_enum_decl += std::string(fn);
	}
	font_enum_decl += "'";
	label_face_type = cgv::media::font::FFA_BOLD;
	label_font_face = cgv::media::font::find_font(font_names[0])->get_font_face(label_face_type);
	state[0] = state[1] = state[2] = state[3] = IS_NONE;

	end_point_size_list.clear();
	end_point_list.clear();

	ds = new DataStore();
	tmpdata_1 = new DataStore();
	tmpdata_2 = new DataStore();
	skel_view = new SkeletonViewer(ds, "editable");
	tmpskel_view_1 = new SkeletonViewer(tmpdata_1, "newskel_1");
	tmpskel_view_2 = new SkeletonViewer(tmpdata_2, "newskel_2");
	ik_view = new IKViewer(ds);
}
///	
void vr_rigging::stream_help(std::ostream& os) {
	os << "vr_test: no shortcuts defined" << std::endl;
}
///
void vr_rigging::on_set(void* member_ptr)
{
	if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
		label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
		label_outofdate = true;
	}
	if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
		member_ptr == &label_size || member_ptr == &label_text) {
		label_outofdate = true;
	}

	vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
	if (kit_ptr) {
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			if (member_ptr >= &left_inp_cfg[ii] && member_ptr < &left_inp_cfg[ii] + 1)
				kit_ptr->set_controller_input_config(0, ii, left_inp_cfg[ii]);
	}
	update_member(member_ptr);
	post_redraw();
}
///
bool vr_rigging::handle(cgv::gui::event& e)
{
	// check if vr event flag is not set and don't process events in this case
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;
	// check event id
	switch (e.get_kind()) {
	case cgv::gui::EID_KEY:
	{
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
		if (vrke.get_action() != cgv::gui::KA_RELEASE) {
			switch (vrke.get_key()) {
			case vr::VR_GRIP:
				std::cout << "grip button " << (vrke.get_controller_index() == 0 ? "left":"right") << " controller pressed" << std::endl;
				if (vrke.get_controller_index() == 0) {
					if (toggle_posing) {
						if (left_ee) {
							vec3 origin, direction;
							// ccd calcu. skel. based on LEFT hand  
							ds->set_endeffector(left_ee, 0);
							vrke.get_state().controller[0].put_ray(&origin(0), &direction(0));

							ik_view->set_target_position_vr(
								Vec4(
									origin.x(),
									origin.y(),
									origin.z()));
							//ik_view->set_max_iter(30);
							ik_view->optimize(0);
						}
						else {
							std::cout << "setup left_ee first!" << std::endl;
						}
					}
					else {
						vec3 origin, direction;
						vrke.get_state().controller[0].put_ray(&origin(0), &direction(0)); // attention! [0/1]
						// compute intersection point 
						vec3 posi = compute_ray_plane_intersection_point(origin, direction);
						// set the point 
						vr_view_ptr->set_tracking_origin(Vec3(posi.x(), vr_view_ptr->get_tracking_origin().y(), posi.z()));
					}
				}
				else 
					btn_keydown_boxgui = true;
				return true;
			case vr::VR_DPAD_RIGHT:
				std::cout << "touch pad of " << (vrke.get_controller_index() == 0 ? "left" : "right") << " controller pressed at right direction" << std::endl;
				return true;
			}
		}
		break;
	}
	case cgv::gui::EID_THROTTLE:
	{
		cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
		std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
			<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;
		return true;
	}
	case cgv::gui::EID_STICK:
	{
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		switch (vrse.get_action()) {
		case cgv::gui::SA_TOUCH:
			if (vrse.get_controller_index() == 0) {// left hand touch to perform actions  
				std::cout << vrse.get_state().controller[0].button_flags << std::endl;
				std::cout << vrse.get_state().controller[1].button_flags << std::endl;
				if (lefthandmode._Equal("adjest_bone_exist")) {
					keydown_adjest_bone_exist = !keydown_adjest_bone_exist;
				}
				if (lefthandmode._Equal("def_max_dof")) {
					toggle_def_max_dof = !toggle_def_max_dof;
					if (toggle_def_max_dof) {
						//cur_rot_mat = from_global_roll_yaw_pitch_vec_to_matrix();
						temp_rot = cur_rot_mat; // cur_rot_mat should be a fixed frame
					}
					else {
						from_matrix_to_euler_angle_as_global_var(temp_rot);
					}
				}
				if (lefthandmode._Equal("def_min_dof")) {
					toggle_def_min_dof = !toggle_def_min_dof;
					if (toggle_def_min_dof) {
						//cur_rot_mat = from_global_roll_yaw_pitch_vec_to_matrix();
						temp_rot = cur_rot_mat;
					}
					else {
						from_matrix_to_euler_angle_as_global_var(temp_rot);
					}
				}
				if (lefthandmode._Equal("def local frame")) {
					toggle_local_dofs_def = !toggle_local_dofs_def;
					if (toggle_local_dofs_def) {
						//cur_rot_mat = from_global_roll_yaw_pitch_vec_to_matrix();
						temp_rot = cur_rot_mat;
					}
					else {
						if (bone_tobeadjested_idx != -1) {
							Bone* cur_bone_to_be_adjested =
								ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeadjested_idx);
							from_matrix_to_euler_angle_as_global_var(cur_rot_mat);
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(2)->set_value(cur_local_frame_rot_rel_XYZ[0]);
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(1)->set_value(cur_local_frame_rot_rel_XYZ[1]);
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(0)->set_value(cur_local_frame_rot_rel_XYZ[2]);

							// perform post process, bounding box will be re-calculated! we need it to write pinocchio file 
							ds->get_skeleton()->postprocess(ds->get_skeleton()->get_root(), Vec3(0, 0, 0));
							// std::cout << skel_view->get_jointlist().size();
							// update skel. the tree view will be updated at the sametime 
							skel_view->skeleton_changed(ds->get_skeleton());
						}
						//from_matrix_to_euler_angle_as_global_var(temp_rot);
					}
					std::cout << "toggle_local_dofs_def: " << toggle_local_dofs_def << std::endl;
				}
				if (lefthandmode._Equal("add_bone")) {
					keydown = true;
					is_even_point = !is_even_point;
				}
				if (lefthandmode._Equal("move bone"))
					state[vrse.get_controller_index()] = IS_GRAB;
				if (lefthandmode._Equal("fast_add_root")) {
					b_fast_add_root = true;
				}
				if (lefthandmode._Equal("del bone")) {// del by touching left controller, tobetested
					del_keydown = true;
				}
				if (lefthandmode._Equal("ccd")) {
					do_ik_ccd = true;
				}
				if (lefthandmode._Equal("select base")) {
					select_base = true;
				}
				if (lefthandmode._Equal("select ee left")) {
					select_endeffector = true;
				}
				if (lefthandmode._Equal("select ee right")) {
					select_endeffector_1 = true;
				}
				if (lefthandmode._Equal("select ee head")) {
					select_endeffector_2 = true;
				}
			}
			break;
		case cgv::gui::SA_RELEASE:
			if (state[vrse.get_controller_index()] == IS_GRAB)
				state[vrse.get_controller_index()] = IS_OVER;
			break;
		case cgv::gui::SA_PRESS: 
			// press at y dir on left controller to start ccd 
			if (vrse.get_controller_index() == 0) { //&& (vrse.get_y() > 0.9)
				toggle_ccd = !toggle_ccd;
				// set to the desired position in front of mirror 
				// pp x component of the position can not be setted correctly, due to calibration?
				// z component must be set to this value! 
				vr_view_ptr->set_tracking_origin(Vec3(
					vr_view_ptr->get_tracking_origin().x(),
					vr_view_ptr->get_tracking_origin().y(),
					-1.2f));
				if (toggle_ccd)
					toggle_chain = !toggle_chain;
				if (toggle_ccd)
					label_content = "[INFO] ccd on!\n" + label_content;
				else
					label_content = "[INFO] ccd off!\n" + label_content;
				label_outofdate = true;
			}
			break;
		case cgv::gui::SA_UNPRESS:
			std::cout << "stick " << vrse.get_stick_index()
				<< " of controller " << vrse.get_controller_index()
				<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
				<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
			return true;
		case cgv::gui::SA_MOVE:
		case cgv::gui::SA_DRAG:
			if ((vrse.get_controller_index() == 1) && tmpboxsize < 0.5f && (vrse.get_x() > 0.9)) {
				tmpboxsize += enlargestep;
			}
			else if ((vrse.get_controller_index() == 1) && tmpboxsize > 0.04f && (vrse.get_x() < -0.9)) {
				tmpboxsize -= enlargestep;
			}

			// left block of left controller 
			if (vrse.get_controller_index() == 0 && (vrse.get_x() < -0.9) && (vrse.get_y() < 0.1f) && (vrse.get_y() > -0.1f)) {
				vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() + 10);
			}
			// right block of left controller 
			if (vrse.get_controller_index() == 0 && (vrse.get_x() > 0.9) && (vrse.get_y() < 0.1f) && (vrse.get_y() > -0.1f)) {
				vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() - 10);
			}
			if (vrse.get_controller_index() == 1 && mesh_scale_mode) {
			}
			return true;
		}
		return true;
	}
	case cgv::gui::EID_POSE:
		cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
		// check for controller pose events
		int ci = vrpe.get_trackable_index();
		if (ci != -1) {

			// computing intersections with right hand 
			if (btn_keydown_boxgui) {
				// clear all intersections, gui
				gui_intersection_points.clear();
				gui_intersection_colors.clear();
				gui_intersection_box_indices.clear();
				gui_intersection_controller_indices.clear();

				// compute intersec. with main gui  
				vec3 origin, direction;
				vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));

				// compute intersections
				gui_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				if (pg1->elements.size() > 0 && gui_intersection_points.size() > 0) {
					// button clicked! button handlers!
					// std::cout << "gui button " << gui_intersection_box_indices.front() << " clicked!" << std::endl;
					// call back func.!
					int cur_btn_idx = gui_intersection_box_indices.front();

					// load diff. skybox 
					if (cur_btn_idx == 3) {
						which_skybox = 0;
						post_redraw();
					}

					//
					if (cur_btn_idx == 4) {
						which_skybox = 1;
						post_redraw();
					}

					//
					if (cur_btn_idx == 5) {
						which_skybox = 2;
						post_redraw();
					}

					// toggle_addi\n_skel 
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle_addi\n_skel")) {
						toggle_other_twoskel();
					}

					// toggle_\nimitating
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle_\nimitating")) {
						toggle_imitating();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("scale_\nadjestment")) {
						adjest_mesh_scale();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("scale")) {
						//mesh_scale_mode = !mesh_scale_mode;
						//adjest_mesh_scale();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("reset_\nmesh")) {
						// clean varibles 
						start_point_list.clear();
						end_point_list.clear();
						mmesh->set_orientation_translation(rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
						mmesh->set_mesh_scale(1);

						// re-load mesh
						mmesh->read_obj(mesh_dir.c_str());
						ds->set_mesh(mmesh);

						// update info board 
						label_content = "[INFO] mesh loaded!\n" + label_content;
						label_outofdate = true;
						post_redraw();
					}

					// load demo mesh 
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo1")) {
						load_mesh_1();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo2")) {
						load_mesh_2();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo3")) {
						load_mesh_3();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo4")) {
						load_mesh_4();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo5")) {
						load_mesh_5();
					}

					// load demo skel. 
					if (pg1->elements.at(cur_btn_idx).label._Equal("demoskel")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						start_point_list.clear();
						end_point_list.clear();

						from_jump_asf = true;
						left_ee = right_ee = hmd_ee = nullptr;
						skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/jump.asf");
						skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
						
						// set global varibles for rendering 
						jointlist = skel_view->get_jointlist();
						jointlist_colors_standard = skel_view->get_jointlistcolor();
						jointlist_colors = skel_view->get_jointlistcolor();

						// load additional two guys 
						load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/jump.asf");
						post_redraw();

						label_content = "[INFO] demo skel. loaded\n" + label_content;
						label_outofdate = true;
					}

					//l_skel1
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel1")) {
						from_jump_asf = false;
						left_ee = right_ee = hmd_ee = nullptr;
						start_point_list.clear();
						end_point_list.clear();

						skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.asf");
						skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
						
						// set global varibles for rendering 
						jointlist = skel_view->get_jointlist();
						jointlist_colors_standard = skel_view->get_jointlistcolor();
						jointlist_colors = skel_view->get_jointlistcolor();
						
						load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.asf");
						post_redraw();

						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel2")) {
						from_jump_asf = false;
						left_ee = right_ee = hmd_ee = nullptr;
						start_point_list.clear();
						end_point_list.clear();

						skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.asf");
						skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));

						// set global varibles for rendering 
						jointlist = skel_view->get_jointlist();
						jointlist_colors_standard = skel_view->get_jointlistcolor();
						jointlist_colors = skel_view->get_jointlistcolor();
						
						load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.asf");
						post_redraw();

						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel3")) {
						from_jump_asf = false;
						left_ee = right_ee = hmd_ee = nullptr;
						start_point_list.clear();
						end_point_list.clear();

						skel_view->load_skeleton_given_name(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.asf");
						skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));

						// set global varibles for rendering, load from skel_view 
						jointlist = skel_view->get_jointlist();
						jointlist_colors_standard = skel_view->get_jointlistcolor();
						jointlist_colors = skel_view->get_jointlistcolor();
						
						load_addi_two_guys(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.asf");
						post_redraw();

						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
					}

					// save skel. 
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel1")) {
						ds->get_skeleton()->write_pinocchio_file(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.txt");
						ds->get_skeleton()->writeASFFile(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_1.asf");
						label_content = "[INFO] skel. saved!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel2")) {
						ds->get_skeleton()->write_pinocchio_file(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.txt");
						ds->get_skeleton()->writeASFFile(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_2.asf");
						label_content = "[INFO] skel. saved!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel3")) {
						ds->get_skeleton()->write_pinocchio_file(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.txt");
						ds->get_skeleton()->writeASFFile(data_dir + "/gen_dataset/" + "speider_simple0/tmpskel_3.asf");
						label_content = "[INFO] skel. saved!\n" + label_content;
						label_outofdate = true;
					}

					// load demo animation 
					/*if (pg1->elements.at(cur_btn_idx).label._Equal("jump")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->load_animation_given_name(projdir + "_workdemo1_spiderman/jump.amc",false);
					}*/

					// save cur. skel. to file
					if (pg1->elements.at(cur_btn_idx).label._Equal("write_\npinocchio\n_skel.")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						save_curskel_to_file();
					}

					// autorig with pinocchio 
					if (pg1->elements.at(cur_btn_idx).label._Equal("autorig_\nwith_\npinocchio")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						start_autorigging_pinoccio();
						//will be modified to multithread 
					}

					// apply_\nattachments
					if (pg1->elements.at(cur_btn_idx).label._Equal("apply_\nattachments")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						apply_rigged_skel();
					}

					// motion capture rel.
					if (pg1->elements.at(cur_btn_idx).label._Equal("ccd")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						toggle_ccd = !toggle_ccd;

						// set to the desired position in front of mirror 
						vr_view_ptr->set_tracking_origin(Vec3(1.5f, vr_view_ptr->get_tracking_origin().y(), -1.0f));

					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("posing")) {
						toggle_posing = !toggle_posing;
						label_content = "[INFO] posing!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_base")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "select base";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_ee_left")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "select ee left";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_ee_right")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "select ee right";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_ee_head")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "select ee head";
					}

					// bone skel. editing rel. 
					if (pg1->elements.at(cur_btn_idx).label._Equal("add_\nroot")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "fast_add_root";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("add_bone_\nexist")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "add_bone";
					}

					//del_bone_\nexist
					if (pg1->elements.at(cur_btn_idx).label._Equal("del_bone_\nexist")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "del bone";
					}

					//edit_bone_\nexist ,
					if (pg1->elements.at(cur_btn_idx).label._Equal("edit_bone_\nexist")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "move bone";
					}

					//backward
					if (pg1->elements.at(cur_btn_idx).label._Equal("backward")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						// use a stack to record user operations 
					}

					//del_all
					if (pg1->elements.at(cur_btn_idx).label._Equal("del_all")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						del_skel();
					}

					// useless 
					if (pg1->elements.at(cur_btn_idx).label._Equal("walk around")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						lefthandmode = "walk around";
					}

					// toggle mesh renderer 
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle\nmesh")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						toggle_mesh_render();
					}
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle\ntransparent")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						toggle_mesh_transparent();
					}
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle\nwireframe")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						toggle_mesh_wireframe();
					}
					if (pg1->elements.at(cur_btn_idx).label._Equal("toggle\nface_culling")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						toggle_face_culling();
					}

					// start_rec
					if (pg1->elements.at(cur_btn_idx).label._Equal("start_rec")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						start_record();
					}

					//skel_view->stop_record_anim("test.amc");
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_anim1")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->stop_record_anim(data_dir + "/gen_dataset/" + "speider_simple0/anim_1.amc");
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_anim2")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->stop_record_anim(data_dir + "/gen_dataset/" + "speider_simple0/anim_2.amc");
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("s_anim3")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->stop_record_anim(data_dir + "/gen_dataset/" + "speider_simple0/anim_3.amc");
					}

					// load animation 
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim1")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_1.amc", true);
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim2")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_2.amc", true);
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim3")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
						skel_view->load_animation_given_name(data_dir + "/gen_dataset/" + "speider_simple0/anim_3.amc", true);
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("stop_anim")) {
						skel_view->stop_animation();
						label_content = "[INFO] animation stopped!\n" + label_content;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("l_scene1")) {
						label_content = "[INFO] button clicked!\n" + label_content;
						label_outofdate = true;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("def_min\n_dof")) {
						lefthandmode = "def_min_dof";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("def_max\n_dof")) {
						lefthandmode = "def_max_dof";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("def_local\n_frame")) {
						lefthandmode = "def local frame";
					}

					// click twice 
					if (pg1->elements.at(cur_btn_idx).label._Equal("adjest_bone\n_exist")) {
						lefthandmode = "adjest_bone_exist";
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("shuffle_dof\n_def")) {
						shuffle_dof_def_xyz++;
						if (shuffle_dof_def_xyz > 2) // only xyz three objs
							num_of_all_choices = 0;
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("shuffle_\nlocal_frame")) {
						// compute cur_local_frame_rot_rel_XYZ in degrees
						vec3 asix_dir = vec3(1, 0, 0);
						shuffle_local_frame_dir_num++;
						if (shuffle_local_frame_dir_num > num_of_all_choices - 1) {
							shuffle_local_frame_dir_num = 0;
						}
						///
						if (shuffle_local_frame_dir_num == 0) {
							asix_dir = vec3(1, 0, 0);
						}
						else if (shuffle_local_frame_dir_num == 1) {
							asix_dir = vec3(0, 1, 0);
						}
						else if (shuffle_local_frame_dir_num == 2) {
							asix_dir = vec3(0, 0, 1);
						}
						else if (shuffle_local_frame_dir_num == 3) {
							asix_dir = vec3(-1, 0, 0);
						}
						else if (shuffle_local_frame_dir_num == 4) {
							asix_dir = vec3(0, -1, 0);
						}
						else if (shuffle_local_frame_dir_num == 5) {
							asix_dir = vec3(0, 0, -1);
						}

						if (lefthandmode == "adjest_bone_exist" && bone_tobeadjested_idx != -1) {
							// compute the selected bone and reset global varibles  
							Bone* cur_bone_to_be_adjested =
								ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeadjested_idx);
							vec3 bonedir_inworldspace = cur_bone_to_be_adjested->get_direction_in_world_space();
							mat3 rot_mat = compute_matrix_from_two_dirs(asix_dir, bonedir_inworldspace);
							from_matrix_to_euler_angle_as_global_var(rot_mat);
							cur_rot_mat = rot_mat;
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(2)->set_value(cur_local_frame_rot_rel_XYZ[0]);
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(1)->set_value(cur_local_frame_rot_rel_XYZ[1]);
							cur_bone_to_be_adjested->get_asix_as_orientation_list().at(0)->set_value(cur_local_frame_rot_rel_XYZ[2]);

							// perform post process, bounding box will be re-calculated! we need it to write pinocchio file 
							// update skel. the tree view will be updated at the sametime 
							ds->get_skeleton()->postprocess(ds->get_skeleton()->get_root(), Vec3(0, 0, 0));
							skel_view->skeleton_changed(ds->get_skeleton());
						}
						else if (lefthandmode == "add_bone" && end_point_list.size()) {
							// when adding bone to existing skeleton
							vec3 bonedir_inworldspace = end_point_list.at(end_point_list.size() - 1)
								- start_point_list.at(start_point_list.size() - 1);
							mat3 rot_mat = compute_matrix_from_two_dirs(asix_dir, bonedir_inworldspace);
							from_matrix_to_euler_angle_as_global_var(rot_mat);
							cur_rot_mat = rot_mat;
						}
						post_redraw();
					}

					//
					if (pg1->elements.at(cur_btn_idx).label._Equal("build_bone")) {
						// add bone here. addchild method will be called, ref. to the bone reading process 
						Bone* parent_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeaddednext_idx);
						std::cout << parent_bone->get_name() << std::endl;
						Bone* current_node = new Bone();
						string addi_str = "";
						if (ds->get_skeleton())
							if (ds->get_skeleton()->get_size_bone_list() > 0)
								addi_str = "new_";
						current_node->set_name(addi_str + "new_bone_" + to_string(newbone_idx++)); // "new_bone_0, new_bone_1..."
						ds->get_skeleton()->add_new_bone_to_map(addi_str + "new_bone_" + to_string(newbone_idx++), current_node);

						// adjest bone para. that we have drawn 
						Vec3 bonedir_inworldspace = end_point_list.at(end_point_list.size() - 1)
							- start_point_list.at(start_point_list.size() - 1);
						current_node->set_direction_in_world_space(bonedir_inworldspace); // length already included 
						current_node->set_length(1);

						// local coordi. the same as global one 
						/*axis 0 0 -20   XYZ*/
						//float a[3] = { 0, 0, 0 };// will be adjested later. todo
						std::string order = "XYZ";
						for (int i = 0; i < 3; ++i)
						{
							AtomicRotationTransform* t;
							if (order.at(i) == 'X')
								t = new AtomicXRotationTransform();
							else if (order.at(i) == 'Y')
								t = new AtomicYRotationTransform();
							else if (order.at(i) == 'Z')
								t = new AtomicZRotationTransform();
							t->set_value(cur_local_frame_rot_rel_XYZ[i]);
							current_node->add_axis_rotation(t);
						}

						// apply full dof currently 
						/*dof rx ry rz*/
						int n_dofs = 3; // to be adjested in vr
						AtomicTransform* dof;
						dof = new AtomicXRotationTransform();
						current_node->add_dof(dof);
						dof = new AtomicYRotationTransform();
						current_node->add_dof(dof);
						dof = new AtomicZRotationTransform();
						current_node->add_dof(dof);

						/*limits(-160.0 20.0)
							(-70.0 70.0)
							(-70.0 60.0)*/
							/*for (int i = 0; i < n_dofs; ++i)
								current_node->get_dof(n_dofs - i - 1)->set_limits(-180.0, 180.0);*/
						current_node->get_dof(2)->set_limits(def_dof_x_min, def_dof_x_max);
						current_node->get_dof(1)->set_limits(def_dof_y_min, def_dof_y_max);
						current_node->get_dof(0)->set_limits(def_dof_z_min, def_dof_z_max);
						current_node->jointsize_stored_as_bone_parameter = tmpboxsize;

						// define the relation between bones 
						parent_bone->add_child(current_node);

						// perform post process, bounding box will be re-calculated! we need it to write pinocchio file 
						ds->get_skeleton()->postprocess(ds->get_skeleton()->get_root(), Vec3(0, 0, 0));
						// std::cout << skel_view->get_jointlist().size();
						// update skel. the tree view will be updated at the sametime 
						skel_view->skeleton_changed(ds->get_skeleton()); // jointlist updated inside 

						label_content = "[INFO] a new bone has been built!\n" + label_content;
						label_outofdate = true;
					}

					// transform to the position of the left hand controller 
					if (pg1->elements.at(cur_btn_idx).label._Equal("trans.")) {
						translation_to_desired_posi();
					}

					// rotate 90 degree each time 
					if (pg1->elements.at(cur_btn_idx).label._Equal("rotation.")) {
						orient_to_desired_ori();
					}

					// add further button callbacks functions here
				}

				// compute intersec. with sub menu, not used currently 
				vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
				compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				if (intersection_points.size() > 0) {
					int cur_btn_idx = intersection_box_indices.front();
					if (cur_btn_idx == 0) {
						// make cur box smaller 
					}

					if (cur_btn_idx == 1) {
						// make it lager 
					}
				}

				btn_keydown_boxgui = false;
			}
			else {
				// no button, usually. 
				// animation, when intersection! anim only works for right controller?
				// clean vectors used for intersection calculation 
				// (may not need a vector at all, we typically need the first intersec.)

				// clear the boxgui intersection list
				gui_intersection_points.clear();
				gui_intersection_colors.clear();
				gui_intersection_box_indices.clear();
				gui_intersection_controller_indices.clear();

				// clear the skeleton intersection list 
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();

				// update the global varible, the position of the headset 
				hmd_origin.x() = vrpe.get_state().hmd.pose[9];
				hmd_origin.y() = vrpe.get_state().hmd.pose[10];
				hmd_origin.z() = vrpe.get_state().hmd.pose[11];

				// compute intersections 
				vec3 origin, direction;

				// computing intersection with skeleton 
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				// mark cur. posi as global var.
				cur_left_hand_posi = origin;
				cur_left_hand_dir = direction;
				if (!skel_view->playing)
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

				// computing intersection with boxgui buttons 
				vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
				gui_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

				// has intersec. with skel. joint box 
				if (skel_intersection_points.size() > 0) {
					// front means the first intersection box idx 
					// render yellow for intersected joint boxes 
					jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 1, 102.0f / 255.0f);
				}
				else {

					// reset color array to standard color array 
					reset_jointlist_color();
				}

				// has intersec. with boxgui
				if (pg1->elements.size() > 0 && gui_intersection_points.size() > 0) {
					pg1->elements.at(gui_intersection_box_indices.front()).has_intersec = true;

					// rendered box list will be changed after this call 
					pg1->push_to_render_vector();
				}
				else {

					// no intersection with gui
					bool redrawneeded = false;
					for (int i = 0; i < pg1->elements.size(); i++) {
						if (pg1->elements.at(i).has_intersec) {
							// if there is any intersection in last frame 
							// re draw needed 
							redrawneeded = true;
						}
					}
					if (redrawneeded) {
						for (int i = 0; i < pg1->elements.size(); i++) { // set all flags to false 
							pg1->elements.at(i).has_intersec = false;
						}
						pg1->push_to_render_vector();// re-gen the vec. for rendering 
						//post_redraw();
					}
				}
			}

			if (toggle_def_min_dof) {
				if (ci == 0) {
					//mat3 t = vrpe.get_orientation();
						//float angle = acos((t(0,0) + t(1,1) + t(2,2) - 1) / 2) * 180 / PI;
					vec3 p = vrpe.get_position();
					/*vec3 last_p = vrpe.get_last_position();
					float diff_p = (p - last_p).length();
					float projected_angle = asin(diff_p) * 180 / PI;*/
					float projected_angle = 360 * (p.y() - 1) - 180;
					if (projected_angle < -180)
						projected_angle = -180;
					if (projected_angle > 180)
						projected_angle = 180;
					vec3 asix_dir;
					if (shuffle_dof_def_xyz == 0) {
						def_dof_x_min = projected_angle;
						asix_dir = vec3(1, 0, 0);
					}
					else if (shuffle_dof_def_xyz == 1) {
						def_dof_y_min = projected_angle;
						asix_dir = vec3(0, 1, 0);
					}
					else if (shuffle_dof_def_xyz == 2) {
						def_dof_z_min = projected_angle;
						asix_dir = vec3(0, 0, 1);
					}
					vec3 cur_asix_dir = temp_rot * asix_dir;
					if (true) {
						mat3 rot_with_fixed_asix = rotate3(projected_angle, cur_asix_dir);
						cur_min_rot_mat = rot_with_fixed_asix * temp_rot;
					}
				}
			}

			if (toggle_def_max_dof) {
				if (ci == 0) {
					//mat3 t = vrpe.get_orientation();
						//float angle = acos((t(0,0) + t(1,1) + t(2,2) - 1) / 2) * 180 / PI;
					vec3 p = vrpe.get_position();
					/*vec3 last_p = vrpe.get_last_position();
					float diff_p = (p - last_p).length();
					float projected_angle = asin(diff_p) * 180 / PI;*/
					float projected_angle = 360 * (p.y() - 1) - 180;
					if (projected_angle < -180)
						projected_angle = -180;
					if (projected_angle > 180)
						projected_angle = 180;
					vec3 asix_dir;
					if (shuffle_dof_def_xyz == 0) {
						def_dof_x_max = projected_angle;
						asix_dir = vec3(1, 0, 0);
					}
					else if (shuffle_dof_def_xyz == 1) {
						def_dof_y_max = projected_angle;
						asix_dir = vec3(0, 1, 0);
					}
					else if (shuffle_dof_def_xyz == 2) {
						def_dof_z_max = projected_angle;
						asix_dir = vec3(0, 0, 1);
					}
					vec3 cur_asix_dir = temp_rot * asix_dir;
					if (true) {
						mat3 rot_with_fixed_asix = rotate3(projected_angle, cur_asix_dir);
						cur_max_rot_mat = rot_with_fixed_asix * temp_rot;
					}
				}
			}

			// define local dofs 
			if (toggle_local_dofs_def) {
				/*vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				cur_rot_mat = compute_matrix_from_two_dirs(vec3(1, 0, 0), -direction);*/

				if (ci == 0) {
					//mat3 t = vrpe.get_orientation();
					//float angle = acos((t(0,0) + t(1,1) + t(2,2) - 1) / 2) * 180 / PI;
					vec3 p = vrpe.get_position();
					/*vec3 last_p = vrpe.get_last_position();
					float diff_p = (p - last_p).length();
					float projected_angle = asin(diff_p) * 180 / PI;*/
					float projected_angle = 360 * (p.y() - 1) - 180;
					if (projected_angle < -180)
						projected_angle = -180;
					if (projected_angle > 180)
						projected_angle = 180;
					vec3 asix_dir;
					if (shuffle_local_frame_dir_num == 0) {
						asix_dir = vec3(1, 0, 0);
					}
					else if (shuffle_local_frame_dir_num == 1) {
						asix_dir = vec3(0, 1, 0);
					}
					else if (shuffle_local_frame_dir_num == 2) {
						asix_dir = vec3(0, 0, 1);
					}
					else if (shuffle_local_frame_dir_num == 3) {
						asix_dir = vec3(-1, 0, 0);
					}
					else if (shuffle_local_frame_dir_num == 4) {
						asix_dir = vec3(0, -1, 0);
					}
					else if (shuffle_local_frame_dir_num == 5) {
						asix_dir = vec3(0, 0, -1);
					}
					vec3 cur_asix_dir = temp_rot * asix_dir;
					mat3 rot_with_fixed_asix = rotate3(projected_angle, cur_asix_dir);
					cur_rot_mat = rot_with_fixed_asix * temp_rot;
				}
			}

			if (object_teleport) { // tobetested 
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				vec3 posi = compute_ray_plane_intersection_point(origin, direction);
				vec3 ori_trans = ds->get_mesh()->get_mesh_translation();
				vec3 updated_trans = vec3(posi.x(), ori_trans.y(), posi.z());
				mmesh->set_mesh_translation(updated_trans);
				mmesh->read_obj(g_mesh_filename.c_str()); // reload the mesh file 
				ds->set_mesh(mmesh);

				object_teleport = false;
			}

			if (select_endeffector) {
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				// clear skel. intersection list, tmp usage
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				// has intersec. with skel. joint box 
				if (skel_intersection_points.size() > 0) {
					left_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
					ds->set_endeffector(left_ee, 0);
					jointlist_colors_standard.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
					//post_redraw();
					// ik_view->endeffector_changed(cur_bone, 0);
					// must include this to calcu. the kinematics chain!
					// provide info. later todo. 
					// set up the endeffector of chain0, and calculate chain auto.
					// we have to calculate chain1 and 2 with other two buttons 
				}
				select_endeffector = false;
			}

			if (select_endeffector_1) {
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				// clear skel. intersection list, tmp usage
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				// has intersec. with skel. joint box 
				if (skel_intersection_points.size() > 0) {
					right_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
					ds->set_endeffector(right_ee, 0);
					jointlist_colors_standard.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
					//ik_view->endeffector_changed(cur_bone, 0);// must include this to calcu. the kinematics chain!
					// provide info. later todo. 
					// set up the endeffector of chain0, and calculate chain auto.
					// we have to calculate chain1 and 2 with other two buttons 
				}
				select_endeffector_1 = false;
			}

			if (select_endeffector_2) {
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				// clear skel. intersection list, tmp usage
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				// has intersec. with skel. joint box 
				if (skel_intersection_points.size() > 0) {
					hmd_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
					ds->set_endeffector(hmd_ee, 0);
					jointlist_colors_standard.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
					// set up the endeffector of chain2, and calculate chain auto.
					// we have to calculate chain1 and 2 with other two buttons 
				}
				//post_redraw();
				select_endeffector_2 = false;
			}

			if (select_base) {
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				// clear skel. intersection list, tmp usage
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				// has intersec. with skel. joint box 
				if (skel_intersection_points.size() > 0) {
					base_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
					ds->set_base(base_bone);
					jointlist_colors_standard.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
				}
				//post_redraw();
				select_base = false;
			}

			if (toggle_ccd) {
				if (ds->get_base() && (right_ee || left_ee)) { // ds ee not defined now 
					vec3 origin, direction;
					if (left_ee) {
						// ccd calcu. skel. based on LEFT hand  
						ds->set_endeffector(left_ee, 0);
						vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
						left_hand_target_posi = vec3(origin.x(), origin.y(), 2 * mirror_plane_z - origin.z());
						ik_view->set_target_position_vr(
							Vec4(
								left_hand_target_posi.x(),
								left_hand_target_posi.y(),
								left_hand_target_posi.z(), 1));
						
						//ik_view->set_max_iter(30);
						ik_view->optimize(0);

						// orientation correction 
						//mat3 ori_mat = compute_matrix_from_two_dirs(vec3(0, -1, 0), direction);
						vec3 mirror_direction = vec3(direction.x(), direction.y(), -direction.z());
						left_ee->set_direction_in_world_space(mirror_direction);
					}

					if (right_ee) {
						// ccd calcu. skel. based on RIGHT hand
						ds->set_endeffector(right_ee, 0);
						vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
						right_hand_target_posi = vec3(origin.x(), origin.y(), 2 * mirror_plane_z - origin.z());
						ik_view->set_target_position_vr(
							Vec4(
								origin.x(),
								origin.y(),
								2 * mirror_plane_z - origin.z(), 1));
						//ik_view->set_max_iter(30);
						ik_view->optimize(0);

						vec3 mirror_direction = vec3(direction.x(), direction.y(), -direction.z());
						left_ee->set_direction_in_world_space(mirror_direction);
					}

					if (hmd_ee) {
						// ccd calcu. skel. based on head position 
						ds->set_endeffector(hmd_ee, 0);
						hmd_origin.x() = vrpe.get_state().hmd.pose[9];
						hmd_origin.y() = vrpe.get_state().hmd.pose[10];
						hmd_origin.z() = vrpe.get_state().hmd.pose[11];
						head_target_posi = vec3(
							hmd_origin.x(),
							hmd_origin.y(),
							2 * mirror_plane_z - hmd_origin.z());
						ik_view->set_target_position_vr(
							Vec4(
								hmd_origin.x(),
								hmd_origin.y(),
								2 * mirror_plane_z - hmd_origin.z(), 1));
						//ik_view->set_max_iter(2);
						ik_view->optimize(0); // optimize the chain2, head chain 
					}

					//cur_bone->get_dof(i)->set_value(masterbone->get_dof(i)->get_value());
					/*for (int i = 0; i < skel_head_current->get_bone_list().size(); i++) {
						int num_of_dofs_at_i = skel_head_current->get_bone_list().at(i)->dof_count();
						for (int j = 0; j < num_of_dofs_at_i; j++) {
							double left_hand_val =
								skel_main_hand->get_bone_list().at(i)->get_dof(j)->get_value();
							double right_hand_val =
								skel_mirror_hand->get_bone_list().at(i)->get_dof(j)->get_value();
							double hmd_val =
								skel_head_current->get_bone_list().at(i)->get_dof(j)->get_value();
							double mean_val = (left_hand_val + right_hand_val + hmd_val) / 3.0f;
							skel_head_current->get_bone_list().at(i)->get_dof(j)->set_value(mean_val);
						}
					}*/
					//post_redraw();
				}
				//do_ik_ccd = false;
			}

			if (del_keydown) {
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				if (skel_intersection_points.size() > 0) {
					int bone_tobedel_idx = skel_intersection_box_indices.front();
					Bone* del_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobedel_idx);
					if (del_bone->get_parent())// if not root bone 
						del_bone->get_parent()->remove_a_child(del_bone); 
					else // if a root bone has been found
						ds->get_skeleton()->clear_bones();  

					// annonce that the skeleton has been changed 
					skel_view->skeleton_changed(ds->get_skeleton());
				}
				del_keydown = false;
			}

			if (keydown_adjest_bone_exist) {
				if (!skel_view->playing && ci == 0) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					skel_intersection_points.clear();
					skel_intersection_box_indices.clear();

					// compute intersections 
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					
					// has intersec. with skel. joint box 
					if (skel_intersection_points.size() > 0) {
						bone_tobeadjested_idx = skel_intersection_box_indices.front();
						jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 1, 102.0f / 255.0f);
						
						// update global vars
						Bone* cur_bone_to_be_adjested =
							ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeadjested_idx);
						cur_local_frame_rot_rel_XYZ[0] = cur_bone_to_be_adjested->get_dof(2)->get_value();
						cur_local_frame_rot_rel_XYZ[1] = cur_bone_to_be_adjested->get_dof(1)->get_value();
						cur_local_frame_rot_rel_XYZ[2] = cur_bone_to_be_adjested->get_dof(0)->get_value();
						cur_rot_mat = from_global_roll_yaw_pitch_vec_to_matrix();
						
						// update information board 
						label_content = "cur_rot_mat computed acc. bone!\n" + label_content;
						label_outofdate = true;
					}
				}
				keydown_adjest_bone_exist = false;
			}

			if (keydown) {
				if (!skel_view->playing) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					if (is_even_point) {// an intersection is required for even points, but not for odd points
						// clear skel. intersection list, tmp usage
						skel_intersection_points.clear();
						skel_intersection_box_indices.clear();

						// compiute intersections
						skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
						
						// has intersec. with skel. joint box 
						if (skel_intersection_points.size() > 0) {
							// front means the first intersection box idx 
							start_point_list.push_back(jointlist.at(skel_intersection_box_indices.front()).get_center());
							jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 1, 102.0f / 255.0f);
							
							// compute cur. bone ref. here 
							bone_tobeaddednext_idx = skel_intersection_box_indices.front();// the same order as jointist, and, bone list!
							drawingbone = true;

							// reset rotations 
							cur_local_frame_rot_rel_XYZ[0] = 0;
							cur_local_frame_rot_rel_XYZ[1] = 0;
							cur_local_frame_rot_rel_XYZ[2] = 0;
						}
					}
					else if (drawingbone) { // is even point and are drawing bones, has intersection before 
						end_point_list.push_back(cur_left_hand_posi + vec3(cur_left_hand_dir * 0.2f)); // tobetested
						//end_point_size_list.push_back(tmpboxsize);

						drawingbone = false;
					}
					post_redraw();
				}
				keydown = false;
			}

			if (state[0] == IS_GRAB) {
				// for left hand , translation of the selected bone node
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));

				// clear skel. intersection list, tmp usage
				skel_intersection_points.clear();
				skel_intersection_box_indices.clear();
				skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				
				// get previous and current controller position
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				mat3 rotation = vrpe.get_rotation_matrix();

				// there is an intersection with skeleton 
				if (skel_intersection_points.size() > 0) {
					jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
					int cur_bone_idx = skel_intersection_box_indices.front();
					Bone* cur_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(cur_bone_idx);
					vec3 oldcenter = jointlist.at(cur_bone_idx).get_center();
					vec3 newcenter = rotation * (oldcenter - last_pos) + pos;
					vec3 tmppoint = newcenter;
					
					// just for intersection test, not for rendering 
					jointlist.at(cur_bone_idx) = box3( 
						vec3(tmppoint.x() - cur_bone->jointsize_stored_as_bone_parameter / 2, //tmpboxsize
							tmppoint.y() - cur_bone->jointsize_stored_as_bone_parameter / 2,
							tmppoint.z() - cur_bone->jointsize_stored_as_bone_parameter / 2),
						vec3(tmppoint.x() + cur_bone->jointsize_stored_as_bone_parameter / 2,
							tmppoint.y() + cur_bone->jointsize_stored_as_bone_parameter / 2,
							tmppoint.z() + cur_bone->jointsize_stored_as_bone_parameter / 2));// setup new posi. if has intersection 
																  
					// to setup the bone , we have to change 
					// translationTransformCurrentJointToNext of the choosen bone
					Mat4 oldt = cur_bone->get_translation_transform_current_joint_to_next();
					Mat4 curt = translate(newcenter - oldcenter);
					Mat4 newt = curt * oldt;
					cur_bone->set_translation_transform_current_joint_to_next(newt);

					// update bone
					skel_view->skeleton_changed(ds->get_skeleton());
				}
			}
			else { state[0] = IS_OVER; }

			if (b_fast_add_root) { 
				// add joints as boxes 
				// click to add bones 
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));

				// add to skel.
				Bone* current_node = new Bone();
				current_node->set_name("root"); 

				/*dof rx ry rz*/
				int n_dofs = 3; // to be adjested in vr
				AtomicTransform* dof;
				dof = new AtomicXRotationTransform();
				current_node->add_dof(dof);
				dof = new AtomicYRotationTransform();
				current_node->add_dof(dof);
				dof = new AtomicZRotationTransform();
				current_node->add_dof(dof);
				dof = new AtomicXTranslationTransform();
				current_node->add_dof(dof);
				dof = new AtomicYTranslationTransform();
				current_node->add_dof(dof);
				dof = new AtomicZTranslationTransform();
				current_node->add_dof(dof);

				/*limits(-160.0 20.0)
				(-70.0 70.0)
				(-70.0 60.0)*/
				for (int i = 0; i < n_dofs; ++i)
					current_node->get_dof(n_dofs - i - 1)->set_limits(-180.0, 180.0);

				// adjest the size of the root bone
				current_node->jointsize_stored_as_bone_parameter = tmpboxsize;
				current_node->calculate_matrices();

				// must create a new one 
				auto s = std::make_shared<SkinningSkeleton>();

				// modify the ik 
				ds->set_endeffector(nullptr, 0);
				ds->set_endeffector(nullptr, 1);
				ds->set_endeffector(nullptr, 2);
				ds->set_base(current_node);

				// modify the skel.
				s->set_root(current_node);
				s->add_new_bone_to_map("root", current_node);

				ds->set_skeleton(s);
				skel_view->skeleton_changed(ds->get_skeleton());
				skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, origin + vec3(direction * 0.2f));
				post_redraw();

				b_fast_add_root = false;
			}

			if (b_fast_add_skel) { 
				// add skel as lines 
				// you need to follow the chains! 
				// clean
				fast_intersection_points.clear();
				fast_intersection_box_indices.clear();

				// conpute intersections 
				vec3 origin, direction;
				vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
				fast_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				
				// clicked, and has intersec. with skel. joint box 
				if (fast_intersection_points.size() > 0) {
					fast_is_reset = false; // for rendering 
					fast_jointlist_colors.at(fast_intersection_box_indices.front()) = rgb(1, 1, 102.0f / 255.0f);
					
					// for bone building, most imp. stru.: fast_bone_posi_vec_as_chain
					fast_bone_posi_vec_as_chain.push_back(fast_jointlist.at(fast_intersection_box_indices.front()).get_center());
				}
				else {

				}
				b_fast_add_skel = false;
			}

			// may do not have to call it explicitly 
			if (b_fast_reset) {
				// reset by insearting a mark in vector 
				// and stop drawing the line 
				// fast_bone_posi_vec_as_chain.push_back(vec3(-100, -100, -100)); 
				// reset mark for bone building 
				// fast_is_reset = true; // for rendering 
				b_fast_reset = false;
			}

			if (vrpe.get_trackable_index() == 0) { 
				// move acco. to left hand 
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();

				//cur_left_hand_posi = vrpe.get_position();
				cur_left_hand_rot = vrpe.get_rotation_matrix();

				for (auto& tran : movable_box_translations) {
					tran = cur_left_hand_rot * (tran - last_pos) + pos;
				}
				for (auto& rot : movable_box_rotations) {
					rot = quat(cur_left_hand_rot) * rot;
				}
			}

			post_redraw();
		}
		return true;
	}
	return false;
}
///
bool vr_rigging::init(cgv::render::context& ctx)
{
	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(2.2f);

	cgv::gui::connect_vr_server(true);
	fast_bone_posi_vec_as_chain.clear();

	auto view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_DEVICE +
					cgv::gui::VRE_STATUS +
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}

	skyprog.build_program(ctx, "skycube.glpr");
	img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");
	tmp_tex.create_from_images(ctx, data_dir + "/skybox/BluePinkNebular_{xp,xn,yp,yn,zp,zn}.jpg");
	test_tex.create_from_images(ctx, data_dir + "/skybox/igen_2/{xp,xn,yp,yn,zp,zn}.jpg");

	if(pg1)
		pg1->icon_shader_prog.build_program(ctx, "image.glpr");

	cgv::render::gl::ensure_glew_initialized();
	mmesh = std::make_shared<SkinningMesh>();
	if (mmesh) {
		mmesh->init_shaders(ctx);
		mmesh->set_orientation_translation(cgv::math::rotate3<double>(0, vec3(0, 1, 0)), vec3(1.2, 0, -2.8));
	}
	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);

	toggle_usage_description = false;
	cur_rot_mat.identity();
	temp_rot.identity();
	cur_min_rot_mat.identity();
	cur_max_rot_mat.identity();

	return true;
}
///
void vr_rigging::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
}
///
void vr_rigging::init_frame(cgv::render::context& ctx)
{
	for (int i = 0; i < pg1->elements.size(); i++) {
		if (pg1->elements.at(i).flag_use_label) {
			if (pg1->elements.at(i).gui_label_texture->label_fbo.get_width() != pg1->elements.at(i).gui_label_texture->label_resolution) {
				pg1->elements.at(i).gui_label_texture->label_tex.destruct(ctx);
				pg1->elements.at(i).gui_label_texture->label_fbo.destruct(ctx);
			}
			if (!pg1->elements.at(i).gui_label_texture->label_fbo.is_created()) {
				pg1->elements.at(i).gui_label_texture->label_tex.create(ctx, cgv::render::TT_2D, pg1->elements.at(i).gui_label_texture->label_resolution * pg1->elements.at(i).radio, pg1->elements.at(i).gui_label_texture->label_resolution);
				pg1->elements.at(i).gui_label_texture->label_fbo.create(ctx, pg1->elements.at(i).gui_label_texture->label_resolution, pg1->elements.at(i).gui_label_texture->label_resolution);
				pg1->elements.at(i).gui_label_texture->label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
				pg1->elements.at(i).gui_label_texture->label_tex.set_mag_filter(cgv::render::TF_LINEAR);
				pg1->elements.at(i).gui_label_texture->label_fbo.attach(ctx, pg1->elements.at(i).gui_label_texture->label_tex);
				pg1->elements.at(i).gui_label_texture->label_outofdate = true;
			}
			if (pg1->elements.at(i).gui_label_texture->label_outofdate && pg1->elements.at(i).gui_label_texture->label_fbo.is_complete(ctx)) {
				glPushAttrib(GL_COLOR_BUFFER_BIT);
				pg1->elements.at(i).gui_label_texture->label_fbo.enable(ctx);
				pg1->elements.at(i).gui_label_texture->label_fbo.push_viewport(ctx);
				ctx.push_pixel_coords();
				glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
				glClear(GL_COLOR_BUFFER_BIT);
				glColor4f(pg1->elements.at(i).gui_label_texture->label_color[0], pg1->elements.at(i).gui_label_texture->label_color[1], pg1->elements.at(i).gui_label_texture->label_color[2], 1);
				ctx.set_cursor(20, (int)ceil(pg1->elements.at(i).gui_label_texture->label_size) + 20);
				ctx.enable_font_face(label_font_face, pg1->elements.at(i).gui_label_texture->label_size);
				ctx.output_stream() << pg1->elements.at(i).gui_label_texture->label_text << "\n";
				ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face
				ctx.enable_font_face(label_font_face, 0.7f * pg1->elements.at(i).gui_label_texture->label_size);
				ctx.output_stream().flush();
				ctx.pop_pixel_coords();
				pg1->elements.at(i).gui_label_texture->label_fbo.pop_viewport(ctx);
				pg1->elements.at(i).gui_label_texture->label_fbo.disable(ctx);
				glPopAttrib();
				pg1->elements.at(i).gui_label_texture->label_outofdate = false;

				pg1->elements.at(i).gui_label_texture->label_tex.generate_mipmaps(ctx);
			}
		}
	}
	if (label_fbo.get_width() != label_resolution) {
		label_tex.destruct(ctx);
		label_fbo.destruct(ctx);
	}
	if (!label_fbo.is_created()) {
		label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
		label_fbo.create(ctx, label_resolution, label_resolution);
		label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_tex.set_mag_filter(cgv::render::TF_LINEAR);
		label_fbo.attach(ctx, label_tex);
		label_outofdate = true;
	}
	if (label_outofdate && label_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		label_fbo.enable(ctx);
		label_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor4f(label_color[0], label_color[1], label_color[2], 1);
		ctx.set_cursor(20, (int)ceil(label_size) + 20);
		ctx.enable_font_face(label_font_face, label_size);
		ctx.output_stream() << label_text << "\n";
		ctx.output_stream().flush(); 
		ctx.enable_font_face(label_font_face, 0.7f * label_size);
		ctx.output_stream() << label_content;
		ctx.output_stream().flush();
		ctx.pop_pixel_coords();
		label_fbo.pop_viewport(ctx);
		label_fbo.disable(ctx);
		glPopAttrib();
		label_outofdate = false;
		label_tex.generate_mipmaps(ctx);
	}
	if (ulabel_fbo.get_width() != ulabel_resolution) {
		ulabel_tex.destruct(ctx);
		ulabel_fbo.destruct(ctx);
	}
	if (!ulabel_fbo.is_created()) {
		ulabel_tex.create(ctx, cgv::render::TT_2D, ulabel_resolution, ulabel_resolution);
		ulabel_fbo.create(ctx, ulabel_resolution, ulabel_resolution);
		ulabel_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		ulabel_tex.set_mag_filter(cgv::render::TF_LINEAR);
		ulabel_fbo.attach(ctx, ulabel_tex);
		ulabel_outofdate = true;
	}
	if (ulabel_outofdate && ulabel_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		ulabel_fbo.enable(ctx);
		ulabel_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
		glClearColor(0.4f, 0.4f, 0.4f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);
		glColor4f(ulabel_color[0], ulabel_color[1], ulabel_color[2], 1);
		ctx.set_cursor(20, (int)ceil(ulabel_size) + 20);
		ctx.enable_font_face(ulabel_font_face, ulabel_size);
		ctx.output_stream() << ulabel_text << "\n";
		ctx.output_stream().flush(); 
		ctx.enable_font_face(ulabel_font_face, 0.7f * ulabel_size);
		ctx.output_stream() << ulabel_content;
		ctx.output_stream().flush();
		ctx.pop_pixel_coords();
		ulabel_fbo.pop_viewport(ctx);
		ulabel_fbo.disable(ctx);
		glPopAttrib();
		ulabel_outofdate = false;
		ulabel_tex.generate_mipmaps(ctx);
	}
	if (vr_view_ptr && vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_eye() == 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
		vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
		if (kit_ptr) {
			vr::vr_camera* camera_ptr = kit_ptr->get_camera();
			if (camera_ptr && camera_ptr->get_state() == vr::CS_STARTED) {
				uint32_t width = frame_width, height = frame_height, split = frame_split;
				if (shared_texture) {
					box2 tex_range;
					if (camera_ptr->get_gl_texture_id(camera_tex_id, width, height, undistorted, &tex_range.ref_min_pnt()(0))) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
					}
					else
						camera_tex_id = -1;
				}
				else {
					std::vector<uint8_t> frame_data;
					if (camera_ptr->get_frame(frame_data, width, height, undistorted, max_rectangle)) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
						cgv::data::data_format df(width, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGBA);
						cgv::data::data_view dv(&df, frame_data.data());
						if (camera_tex.is_created()) {
							if (camera_tex.get_width() != width || camera_tex.get_height() != height)
								camera_tex.destruct(ctx);
							else
								camera_tex.replace(ctx, 0, 0, dv);
						}
						if (!camera_tex.is_created())
							camera_tex.create(ctx, dv);
					}
					else if (camera_ptr->has_error())
						cgv::gui::message(camera_ptr->get_last_error());
				}
				if (frame_width != width || frame_height != height) {
					frame_width = width;
					frame_height = height;

					center_left(0) = camera_centers[2](0) / frame_width;
					center_left(1) = camera_centers[2](1) / frame_height;
					center_right(0) = camera_centers[3](0) / frame_width;
					center_right(1) = camera_centers[3](1) / frame_height;

					update_member(&frame_width);
					update_member(&frame_height);
					update_member(&center_left(0));
					update_member(&center_left(1));
					update_member(&center_right(0));
					update_member(&center_right(1));
				}
				if (split != frame_split) {
					frame_split = split;
					update_member(&frame_split);
				}
			}
		}
	}
	
}
///
void vr_rigging::draw(cgv::render::context& ctx)
{
	

	// rendering headset and controller
	if (vr_view_ptr) {
		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;
		const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
		if (state_ptr) {
			for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				vec3 ray_origin, ray_direction;
				state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
				P.push_back(ray_origin);
				R.push_back(0.002f);
				P.push_back(ray_origin + ray_length * ray_direction);
				R.push_back(0.003f);
				rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
				C.push_back(c);
				C.push_back(c);
			}
		}
		if (P.size() > 0) {
			auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
			cr.set_render_style(cone_style);
			//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
			cr.set_position_array(ctx, P);
			cr.set_color_array(ctx, C);
			cr.set_radius_array(ctx, R);
			if (!cr.render(ctx, 0, P.size())) {
				cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				int pi = prog.get_position_index();
				int ci = prog.get_color_index();
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				glLineWidth(3);
				prog.enable(ctx);
				glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				glLineWidth(1);
			}
		}
	}

	// initialize a box render 
	cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);

	// draw static boxes
	renderer.set_render_style(style);
	renderer.set_box_array(ctx, boxes);
	renderer.set_color_array(ctx, box_colors);
	if (renderer.validate_and_enable(ctx)) {
		renderer.draw(ctx, 0, boxes.size());
	}
	renderer.disable(ctx);

	// rendering the skybox 
	// large enough to contain the whole scene
	float max_scene_extent = 100;
	switch (which_skybox) {
	case 0:
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		test_tex.enable(ctx, 1);
		skyprog.enable(ctx);
		skyprog.set_uniform(ctx, "img_tex", 1);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4<double>(
			max_scene_extent, max_scene_extent, max_scene_extent));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		skyprog.disable(ctx);
		test_tex.disable(ctx);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
		break;
	case 1:
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		img_tex.enable(ctx, 1);
		skyprog.enable(ctx);
		skyprog.set_uniform(ctx, "img_tex", 1);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4<double>(
			max_scene_extent, max_scene_extent, max_scene_extent));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		skyprog.disable(ctx);
		img_tex.disable(ctx);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
		break;
	case 2:
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		tmp_tex.enable(ctx, 1);
		skyprog.enable(ctx);
		skyprog.set_uniform(ctx, "img_tex", 1);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4<double>(
			max_scene_extent, max_scene_extent, max_scene_extent));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		skyprog.disable(ctx);
		tmp_tex.disable(ctx);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
		break;
	}

	// draw info label
	if (vr_view_ptr && label_tex.is_created()) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		int pi = prog.get_position_index();
		int ti = prog.get_texcoord_index();
		vec3 p(-0.5f, hmd_origin.y(), -2.8f); // change the position here 
		vec3 y = label_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
		vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));
		float w = 1.0f, h = 1.0f;
		std::vector<vec3> P;
		std::vector<vec2> T;
		P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
		P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		label_tex.enable(ctx);
		ctx.set_color(rgb(1, 1, 1));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		label_tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}

	// render a board for usage description
	if (vr_view_ptr && ulabel_tex.is_created() && toggle_usage_description) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		int pi = prog.get_position_index();
		int ti = prog.get_texcoord_index();
		vec3 p(1.2, 1.5, -4.0f); // change the position here 
		/*vec3 y = ulabel_upright ? vec3(0, 1.0f, 0) : normalize(vr_view_ptr->get_view_up_dir_of_kit());
		vec3 x = normalize(cross(vec3(vr_view_ptr->get_view_dir_of_kit()), y));*/
		vec3 x = vec3(1, 0, 0);
		vec3 y = vec3(0, 1, 0);
		float w = 2.5f, h = 2.5f;
		std::vector<vec3> P;
		std::vector<vec2> T;
		P.push_back(p - 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(0.0f, 0.0f));
		P.push_back(p + 0.5f * w * x - 0.5f * h * y); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p - 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p + 0.5f * w * x + 0.5f * h * y); T.push_back(vec2(1.0f, 1.0f));
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		ulabel_tex.enable(ctx);
		ctx.set_color(rgb(1, 1, 1));
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		ulabel_tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}

	// draw quads for boxgui 
	if (toggle_boxgui)
		for (int i = 0; i < pg1->elements.size(); i++) {
			vec3 center = pg1->elements.at(i).center_of_quad; // (2.39, 2.8f, -3.0f); is the posi. of the first 
			vec2 ext_vec_quad = pg1->elements.at(i).ext_of_quad; // half 
			//cgv::render::shader_program& quad_prog = ctx.ref_default_shader_program(true);
			int pi = pg1->icon_shader_prog.get_position_index();
			int ti = pg1->icon_shader_prog.get_texcoord_index();
			std::vector<vec3> P;
			std::vector<vec2> T;
			vec3 l_p1 = center + vec3(0, ext_vec_quad.x(), ext_vec_quad.y());
			vec3 l_p2 = center + vec3(0, ext_vec_quad.x(), -ext_vec_quad.y());
			vec3 l_p3 = center + vec3(0, -ext_vec_quad.x(), ext_vec_quad.y());
			vec3 l_p4 = center + vec3(0, -ext_vec_quad.x(), -ext_vec_quad.y());

			vec3 p_p1 = center + vec3(0, -ext_vec_quad.x(), ext_vec_quad.y());
			vec3 p_p2 = center + vec3(0, -ext_vec_quad.x(), -ext_vec_quad.y());
			vec3 p_p3 = center + vec3(0, ext_vec_quad.x(), ext_vec_quad.y());
			vec3 p_p4 = center + vec3(0, ext_vec_quad.x(), -ext_vec_quad.y());
			//if(pg1->elements.at(i).do_transform)
			if (pg1->elements.at(i).flag_use_label) {
				P.push_back(pg1->elements.at(i).rot * (l_p1 - center) + pg1->elements.at(i).trans + center); T.push_back(vec2(1.0f, 1.0f));// set ds pp-
				P.push_back(pg1->elements.at(i).rot * (l_p2 - center) + pg1->elements.at(i).trans + center); T.push_back(vec2(0.0f, 1.0f));
				P.push_back(pg1->elements.at(i).rot * (l_p3 - center) + pg1->elements.at(i).trans + center); T.push_back(vec2(1.0f, 0.0f));
				P.push_back(pg1->elements.at(i).rot * (l_p4 - center) + pg1->elements.at(i).trans + center); T.push_back(vec2(0.0f, 0.0f));
			}
			else {
				P.push_back(pg1->elements.at(i).rot * p_p1 + pg1->elements.at(i).trans); T.push_back(vec2(1.0f, 1.0f));// set ds pp-
				P.push_back(pg1->elements.at(i).rot * p_p2 + pg1->elements.at(i).trans); T.push_back(vec2(0.0f, 1.0f));
				P.push_back(pg1->elements.at(i).rot * p_p3 + pg1->elements.at(i).trans); T.push_back(vec2(1.0f, 0.0f));
				P.push_back(pg1->elements.at(i).rot * p_p4 + pg1->elements.at(i).trans); T.push_back(vec2(0.0f, 0.0f));
			}
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
			cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
			pg1->icon_shader_prog.enable(ctx);// enable shader 
			if (pg1->elements.at(i).flag_use_label) {
				pg1->elements.at(i).gui_label_texture->label_tex.enable(ctx);
				glDisable(GL_CULL_FACE);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
				pg1->elements.at(i).gui_label_texture->label_tex.disable(ctx);
				/*label_tex.enable(ctx);
				glDisable(GL_CULL_FACE);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
				label_tex.disable(ctx);*/
			}
			else {
				glActiveTexture(GL_TEXTURE0);
				glEnable(GL_TEXTURE_2D);
				glBindTexture(GL_TEXTURE_2D, pg1->elements.at(i).icon_texture);
				glDisable(GL_CULL_FACE);
				glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
				glDisable(GL_TEXTURE_2D);
			}
			pg1->icon_shader_prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
		}

	// draw buttons for boxgui
	if (toggle_boxgui && pg1->elements.size() > 0) {
		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, pg1->boxvector);
		renderer.set_color_array(ctx, pg1->colorvector);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)pg1->boxvector.size());
		}
		renderer.disable(ctx);
	}

	// render dynamic mesh 
	if (ds->get_mesh() && b_render_mesh)
	{
		if (ds->get_skeleton())
		{
			std::vector<Mat4> skinning_matrices;
			ds->get_skeleton()->get_skinning_matrices(skinning_matrices);
			ds->get_mesh()->set_skinning_matrices(skinning_matrices);
		}
		ds->get_mesh()->draw(ctx);
	}

	// render editable skeleton 
	if ((ds->get_skeleton() != nullptr) && skel_view)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_CULL_FACE);
		skel_view->draw_skeleton_subtree(ds->get_skeleton()->get_root(),
			ds->get_skeleton()->get_origin(), ctx, 0, true, true);
		glDisable(GL_CULL_FACE);
		glDisable(GL_BLEND);
	}

	// rendering joint points at static state 
	if (!jointlist.empty() && (!skel_view->playing)) {
		renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, jointlist);
		renderer.set_color_array(ctx, jointlist_colors);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)jointlist.size());
		}
		renderer.disable(ctx);
	}

	// render fast_jointlist for static skeleton rendering 
	if (!skel_view->playing) {
		if (fast_jointlist.size() > 0) {
			renderer = cgv::render::ref_box_renderer(ctx);
			renderer.set_render_style(style);
			renderer.set_box_array(ctx, fast_jointlist);
			renderer.set_color_array(ctx, fast_jointlist_colors);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)fast_jointlist.size());
			}
			renderer.disable(ctx);
		}
	}

	// for local frame rendering 
	if (skel_view) {
		skel_view->render_axis_arrow = toggle_render_local_frame;
	}

	// imitating only when dofs are changed 
	if (b_toggle_imitating && skel_view->should_apply_dofs_to_others_for_imitating) {
		apply_dofs();
		skel_view->should_apply_dofs_to_others_for_imitating = false;
	}

	// draw the other two guys
	if (b_toggle_show_imitating_skel) {
		if (tmpdata_1->get_skeleton() != nullptr)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glEnable(GL_CULL_FACE);
			tmpskel_view_1->draw_skeleton_subtree(tmpdata_1->get_skeleton()->get_root(),
				tmpdata_1->get_skeleton()->get_origin(), ctx, 0, true, true);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
		}
		if (tmpdata_2->get_skeleton() != nullptr)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glEnable(GL_CULL_FACE);
			tmpskel_view_2->draw_skeleton_subtree(tmpdata_2->get_skeleton()->get_root(),
				tmpdata_2->get_skeleton()->get_origin(), ctx, 0, true, true);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
		}
	}


	// drawing local frame according to cur_rot_mat, adjesting local frame 
	if (lefthandmode == "def local frame" && bone_tobeadjested_idx > 0) { // if already selected a bone
		std::vector<vec3> p_list;
		std::vector<rgb> color_list;
		/*Bone* cur_bone_to_be_adjested =
			ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeadjested_idx);*/
		vec3 cur_posi_vec3 = jointlist.at(bone_tobeadjested_idx - 1).get_center();

		p_list.push_back(cur_posi_vec3);
		p_list.push_back(cur_posi_vec3 + cur_rot_mat * vec3(0.2, 0, 0));
		color_list.push_back(rgb(1, 0, 0));
		color_list.push_back(rgb(1, 1, 1));

		p_list.push_back(cur_posi_vec3);
		p_list.push_back(cur_posi_vec3 + cur_rot_mat * vec3(0, 0.2, 0));
		color_list.push_back(rgb(0, 1, 0));
		color_list.push_back(rgb(1, 1, 1));

		p_list.push_back(cur_posi_vec3);
		p_list.push_back(cur_posi_vec3 + cur_rot_mat * vec3(0, 0, 0.2));
		color_list.push_back(rgb(0, 0, 1));
		color_list.push_back(rgb(1, 1, 1));

		if (p_list.size() > 0) {
			cgv::render::shader_program& prog = ctx.ref_default_shader_program();
			int pi = prog.get_position_index();
			int ci = prog.get_color_index();
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, p_list);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, color_list);
			cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
			glLineWidth(3);
			prog.enable(ctx);
			glDrawArrays(GL_LINES, 0, (GLsizei)p_list.size());
			prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
			glLineWidth(1);
		}
	}

	// draw skel 
	if (!skel_view->playing) {
		// render lines
		std::vector<vec3> vertex_array_in_point_list;
		std::vector<rgb> colorarray;
		if ((lefthandmode == "def local frame" || lefthandmode == "add_bone") && start_point_list.size() > 0) {
			// render a local frame, according to cur_local_frame_rot_rel_XYZ
			vec3 last_point_posi = start_point_list.at(start_point_list.size() - 1);
			vertex_array_in_point_list.push_back(last_point_posi);
			vertex_array_in_point_list.push_back(last_point_posi + cur_rot_mat * vec3(0.2, 0, 0));
			colorarray.push_back(rgb(1, 0, 0));
			colorarray.push_back(rgb(1, 0, 0));

			vertex_array_in_point_list.push_back(last_point_posi);
			vertex_array_in_point_list.push_back(last_point_posi + cur_rot_mat * vec3(0, 0.2, 0));
			colorarray.push_back(rgb(0, 1, 0));
			colorarray.push_back(rgb(0, 1, 0));

			vertex_array_in_point_list.push_back(last_point_posi);
			vertex_array_in_point_list.push_back(last_point_posi + cur_rot_mat * vec3(0, 0, 0.2));
			colorarray.push_back(rgb(0, 0, 1));
			colorarray.push_back(rgb(0, 0, 1));

			if (cur_min_rot_mat(0, 0) != 1) {
				// render an other frame to def. min vals
				vec3 last_point_posi = start_point_list.at(start_point_list.size() - 1);
				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_min_rot_mat * vec3(0.2, 0, 0));
				colorarray.push_back(rgb(1, 0, 0));
				colorarray.push_back(rgb(0, 0, 0));

				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_min_rot_mat * vec3(0, 0.2, 0));
				colorarray.push_back(rgb(0, 1, 0));
				colorarray.push_back(rgb(0, 0, 0));

				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_min_rot_mat * vec3(0, 0, 0.2));
				colorarray.push_back(rgb(0, 0, 1));
				colorarray.push_back(rgb(0, 0, 0));
			}

			if (cur_max_rot_mat(0, 0) != 1) {
				// render an other frame to def. min vals
				vec3 last_point_posi = start_point_list.at(start_point_list.size() - 1);
				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_max_rot_mat * vec3(0.2, 0, 0));
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(1, 0, 0));

				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_max_rot_mat * vec3(0, 0.2, 0));
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(0, 1, 0));

				vertex_array_in_point_list.push_back(last_point_posi);
				vertex_array_in_point_list.push_back(last_point_posi + cur_max_rot_mat * vec3(0, 0, 0.2));
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(0, 0, 1));
			}
		}
		if (end_point_list.size() > 0) {
			for (int i = 0; i < start_point_list.size() - 1; i++) {
				vertex_array_in_point_list.push_back(start_point_list.at(i));
				vertex_array_in_point_list.push_back(end_point_list.at(i));
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(0, 0, 0));
			}
		}
		// special care 
		if (start_point_list.size() > 0) {
			if (is_even_point && drawingbone) {
				vertex_array_in_point_list.push_back(start_point_list.at(start_point_list.size() - 1));
				vertex_array_in_point_list.push_back(
					cur_left_hand_posi + vec3(cur_left_hand_dir * 0.2f)); // shall be modified. todo
					// pointing to submenu box // correct
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(0, 0, 0));
				// an addi. box should be rendered 
			}
			else if (end_point_list.size() > 0) {
				vertex_array_in_point_list.push_back(start_point_list.at(start_point_list.size() - 1));
				vertex_array_in_point_list.push_back(end_point_list.at(end_point_list.size() - 1));
				colorarray.push_back(rgb(0, 0, 0));
				colorarray.push_back(rgb(0, 0, 0));
			}
		}
		if (vertex_array_in_point_list.size() > 0) {
			cgv::render::shader_program& prog = ctx.ref_default_shader_program();
			int pi = prog.get_position_index();
			int ci = prog.get_color_index();
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, vertex_array_in_point_list);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, colorarray);
			cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
			glLineWidth(3);
			prog.enable(ctx);
			glDrawArrays(GL_LINES, 0, (GLsizei)vertex_array_in_point_list.size());
			prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
			cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
			glLineWidth(1);
		}
	}

	// render the yellow box 
	if (lefthandmode == "fast_add_root" || lefthandmode == "add_bone") {
		vec3 boxcenter = cur_left_hand_posi + vec3(cur_left_hand_dir * 0.2f);
		box3 demobox = box3(vec3(boxcenter - tmpboxsize / 2.0f),
			vec3(boxcenter + tmpboxsize / 2.0f));
		vector<box3> boxarray;
		vector<rgb> boxcolorarray;
		boxarray.push_back(demobox);
		boxcolorarray.push_back(rgb(1, 1, 0));

		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, boxarray);
		renderer.set_color_array(ctx, boxcolorarray);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)boxarray.size());
		}
		renderer.disable(ctx);
	}

	// visual feedback for skeleton rendering 

	// render target position 
	if (toggle_ccd) {
		// prepare position information for rendering 
		std::vector<vec3> vertex_array_in_point_list;
		std::vector<rgb> colorarray;
		if (left_ee) {
			vec3 cur_posi = left_hand_target_posi;
			vertex_array_in_point_list.push_back(cur_posi - vec3(0.3, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0.3, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0.3, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0.3, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0, 0.3));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0, 0.3));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
		}
		if (right_ee) {
			vec3 cur_posi = right_hand_target_posi;
			vertex_array_in_point_list.push_back(cur_posi - vec3(0.3, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0.3, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0.3, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0.3, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0, 0.3));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0, 0.3));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
		}
		if (hmd_ee) {
			vec3 cur_posi = head_target_posi;
			vertex_array_in_point_list.push_back(cur_posi - vec3(0.3, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0.3, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0.3, 0));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0.3, 0));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
			vertex_array_in_point_list.push_back(cur_posi - vec3(0, 0, 0.3));
			vertex_array_in_point_list.push_back(cur_posi + vec3(0, 0, 0.3));
			colorarray.push_back(rgb(0, 0, 0));
			colorarray.push_back(rgb(0, 0, 0));
		}

		// render cursors for target positions 
		cgv::render::shader_program& prog = ctx.ref_default_shader_program();
		int pi = prog.get_position_index();
		int ci = prog.get_color_index();
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, vertex_array_in_point_list);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, colorarray);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
		glLineWidth(3);
		prog.enable(ctx);
		glDrawArrays(GL_LINES, 0, (GLsizei)vertex_array_in_point_list.size());
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
		glLineWidth(1);
	}


}
///
void vr_rigging::finish_draw(cgv::render::context& ctx)
{
	return;
	if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		GLint active_texture, texture_binding;
		if (shared_texture) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, camera_tex_id);
		}
		else
			camera_tex.enable(ctx, 0);

		prog.set_uniform(ctx, "texture", 0);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::translate4<double>(0, 3, 0));
		prog.enable(ctx);
		ctx.set_color(rgba(1, 1, 1, 0.8f));
		ctx.tesselate_unit_square();
		prog.disable(ctx);
		if (shared_texture) {
			glActiveTexture(active_texture);
			glBindTexture(GL_TEXTURE_2D, texture_binding);
		}
		else
			camera_tex.disable(ctx);
		ctx.pop_modelview_matrix();
		glDisable(GL_BLEND);
	}
}
///
void vr_rigging::create_gui() {
	add_member_control(this, "toggle_usage_description", toggle_usage_description, "check");
	add_member_control(this, "toggle_render_local_frame", toggle_render_local_frame, "check");
	add_member_control(this, "toggle_boxgui", toggle_boxgui, "check");

	if (begin_tree_node("mesh related", show_mesh_related, true, "level=2")) {
		align("\a");
		connect_copy(add_button("translation_to_desired_posi")->click, cgv::signal::rebind(this, &vr_rigging::translation_to_desired_posi));
		connect_copy(add_button("orient_to_desired_ori")->click, cgv::signal::rebind(this, &vr_rigging::orient_to_desired_ori));
		connect_copy(add_button("adjest_mesh_scale")->click, cgv::signal::rebind(this, &vr_rigging::adjest_mesh_scale));
		connect_copy(add_button("load_mesh_1")->click, cgv::signal::rebind(this, &vr_rigging::load_mesh_1));
		connect_copy(add_button("load_mesh_2")->click, cgv::signal::rebind(this, &vr_rigging::load_mesh_2));
		connect_copy(add_button("load_mesh_3")->click, cgv::signal::rebind(this, &vr_rigging::load_mesh_3));
		connect_copy(add_button("load_mesh_4")->click, cgv::signal::rebind(this, &vr_rigging::load_mesh_4));
		connect_copy(add_button("load_mesh_5")->click, cgv::signal::rebind(this, &vr_rigging::load_mesh_5));

		align("\b");
		end_tree_node(show_mesh_related);
	}

	if (begin_tree_node("skeleton related", show_skel_related, true, "level=2")) {
		align("\a");
		connect_copy(add_button("load_skel_with_dofs")->click, cgv::signal::rebind(this, &vr_rigging::load_skel_with_dofs));
		connect_copy(add_button("load_demo_skel1")->click, cgv::signal::rebind(this, &vr_rigging::load_demo_skel1));
		connect_copy(add_button("load_demo_skel2")->click, cgv::signal::rebind(this, &vr_rigging::load_demo_skel2));
		connect_copy(add_button("load_demo_skel3")->click, cgv::signal::rebind(this, &vr_rigging::load_demo_skel3));
		align("\b");
		end_tree_node(show_skel_related);
	}

}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_rigging> vr_rigging_reg("vr_rigging");
