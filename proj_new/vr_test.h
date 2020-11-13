#pragma once
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/frame_buffer.h>

#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <fstream>
#include <cgv/base/register.h>
#include "DataStore.h"
#include "SkeletonViewer.h"
#include "Mesh.h"
#include "IKViewer.h"
#include "SkinnedMeshViewer.h"
#include "BoxGui.h"

#include "../proj_pinocchio/skeleton.h"
#include "../proj_pinocchio/attachment.h"
#include "../proj_pinocchio/pinocchioApi.h"

class ArgData
{
public:
	ArgData()
	{
		stopAtMesh = false;
		stopAfterCircles = false;
		skelScale = 1.0f;
		noFit = false;
		skeleton = HumanSkeleton();
	}

	bool stopAtMesh;
	bool stopAfterCircles;
	string filename;
	string motionname;
	Quaternion<> meshTransform;
	double skelScale;
	bool noFit;
	Skeleton skeleton;
	string skeletonname;
};

using namespace cgv::base;

DataStore* ds;
SkeletonViewer* skel_view;
DataStore* tmpdata_1;
SkeletonViewer* tmpskel_view_1;
DataStore* tmpdata_2;
SkeletonViewer* tmpskel_view_2;
IKViewer* ik_view;

struct Initializer
{
	Initializer()
	{
		ds = new DataStore();
		tmpdata_1 = new DataStore();
		tmpdata_2 = new DataStore();
		skel_view = new SkeletonViewer(ds, "editable");
		tmpskel_view_1 = new SkeletonViewer(tmpdata_1, "newskel_1");
		tmpskel_view_2 = new SkeletonViewer(tmpdata_2, "newskel_2");
		ik_view = new IKViewer(ds);

		/*register_object(base_ptr(skel_view), "");
		register_object(base_ptr(tmpskel_view_1), "");
		register_object(base_ptr(tmpskel_view_2), "");
		register_object(base_ptr(ik_view), "");*/
	}

	~Initializer()
	{
		delete ds;
		delete tmpdata_1;
		delete tmpdata_2;
		delete skel_view;
		delete tmpskel_view_1;
		delete tmpskel_view_2;
		delete ik_view;
	}

};
///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cgv\gui\file_dialog.h>

class vr_test :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
protected:
	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};

	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering styles
	cgv::render::box_render_style style;
	cgv::render::rounded_cone_render_style cone_style;

	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// render information for mesh
	cgv::render::mesh_render_info MI;

protected:

	// manage controller input configuration for left controller
	std::vector<vr::controller_input_config> left_inp_cfg;

	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for all controllers
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	mat4 camera_to_head_matrix[2];
	vec2 focal_lengths[4];
	vec2 camera_centers[4];
	GLuint camera_tex_id;
	cgv::math::fmat<float, 4, 4> camera_projection_matrix[4];
	cgv::render::texture camera_tex;
	cgv::render::shader_program seethrough;
	int nr_cameras;
	int frame_width, frame_height;
	int frame_split;
	float seethrough_gamma;
	bool undistorted;
	bool shared_texture;
	bool max_rectangle;
	float camera_aspect;
	bool show_seethrough;
	bool use_matrix;
	float background_distance;
	float background_extent;
	vec2 extent_texcrd;
	vec2 center_left;
	vec2 center_right;

	// skybox
	cgv::render::shader_program skyprog;	
	cgv::render::texture img_tex;
	cgv::render::texture tmp_tex;
	cgv::render::texture test_tex;
	std::string projdir;

	int which_skybox = 1; 
	float tmpboxsize = 0.03f;
	float enlargestep = 0.0025f;
	bool b_render_mesh = true;
	bool b_toggle_show_imitating_skel = true;
	bool b_toggle_imitating = true;
	float mirror_plane_z = -2;
	float mirror_plane_x = 1.2;

	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	std::vector<box3> jointlist;
	std::vector<rgb> jointlist_colors;
	std::vector<box3> fast_jointlist;
	std::vector<rgb> fast_jointlist_colors;
	std::shared_ptr<SkinningMesh> mmesh;
	vec3 hmd_origin;
	string g_mesh_filename = "";
	Bone* left_ee;
	Bone* right_ee;
	Bone* hmd_ee;

	bool keydown = false;
	bool del_keydown = false;
	bool b_fast_add_root = false;
	bool b_fast_add_skel = false;
	bool b_fast_reset = false;
	bool do_ik_ccd = false;
	bool toggle_ccd = false; // for ccd we can not select end effector // toggle ccd button to start/stop 
	bool select_endeffector = false;
	bool select_endeffector_1 = false;
	bool select_endeffector_2 = false;
	bool select_base = false;
	bool btn_keydown_boxgui = false;
	bool toggle_chain = true;
	bool show_my_box = false;
	bool teleport = false;
	bool object_teleport = false;
	bool drawingbone = false;
	bool toggle_posing = false;
	bool toggle_local_dofs_def = false;
	bool toggle_def_min_dof = false;
	bool toggle_def_max_dof = false;
	bool toggle_render_local_frame = true;
	bool keydown_adjest_bone_exist = false;

	float def_dof_x_min = -180;
	float def_dof_x_max = 180;
	float def_dof_y_min = -180;
	float def_dof_y_max = 180;
	float def_dof_z_min = -180;
	float def_dof_z_max = 180;

	boxgui_page* pg1 = new boxgui_page();

	// intersection points gui
	std::vector<vec3> gui_intersection_points;
	std::vector<rgb>  gui_intersection_colors;
	std::vector<int>  gui_intersection_box_indices;
	std::vector<int>  gui_intersection_controller_indices;

	// intersections with skel joint box 
	std::vector<vec3> skel_intersection_points;
	std::vector<int>  skel_intersection_box_indices;

	// intersections with skel joint box in fast mode
	std::vector<vec3> fast_intersection_points;
	std::vector<int>  fast_intersection_box_indices;

	// sample for rendering text labels
	string label_text;
	string label_content;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;
	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label
	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;

	// sample for rendering text labels
	string ulabel_text;
	string ulabel_content;
	int ulabel_font_idx;
	bool ulabel_upright;
	float ulabel_size;
	rgb ulabel_color;
	bool ulabel_outofdate; // whether label texture is out of date
	unsigned ulabel_resolution; // resolution of label texture
	cgv::render::texture ulabel_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer ulabel_fbo; // fbo used for offline rendering of label
	// general font information
	std::vector<const char*> ufont_names;
	std::string ufont_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr ulabel_font_face;
	cgv::media::font::FontFaceAttributes ulabel_face_type;
	bool toggle_usage_description;

	std::vector<vec3> start_point_list;
	std::vector<vec3> end_point_list;
	std::vector<float> end_point_size_list; // no size for root node 
	bool is_even_point = false; // start from -1
	bool fast_is_reset = true;
	bool mesh_scale_mode = false;
	std::vector<vec3> fast_bone_posi_vec_as_chain;

	int bone_tobeaddednext_idx = -1;
	int bone_tobeadjested_idx = -1;
	int newbone_idx = 0;
	std::string lefthandmode;

	std::string working_dir = "../../../plugins/vr_rigging_pub/gen_dataset/";
	std::string mesh_dir = "";
	bool confirm_needed = false;
	bool confirmed = false;
	vec3 confirm_gui_posi = 0;

	bool from_jump_asf = true;

	float cur_local_frame_rot_rel_XYZ[3] = { 0, 0, 0 };
	int shuffle_local_frame_dir_num = 0;
	int shuffle_dof_def_xyz = 0;
	int num_of_all_choices = 6; 
	vec3 roll_yaw_pitch_vec;
	mat3 cur_rot_mat;
	mat3 temp_rot;
	mat3 cur_min_rot_mat;
	mat3 cur_max_rot_mat;
	vec3 left_hand_target_posi;
	vec3 right_hand_target_posi;
	vec3 head_target_posi;

public:
	void init_cameras(vr::vr_kit* kit_ptr);

	void start_camera();

	void stop_camera();

	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// keep track of status changes
	void on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes(float tw, float td, float th, float tW, size_t nr);
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW);
	///
	mat3 from_global_roll_yaw_pitch_vec_to_matrix() {
		vec3 tmp_roll_yaw_pitch_vec = vec3(
			cur_local_frame_rot_rel_XYZ[0],
			cur_local_frame_rot_rel_XYZ[1],
			cur_local_frame_rot_rel_XYZ[2]
		);
		return rotate3(tmp_roll_yaw_pitch_vec);
	}
	mat3 compute_matrix_from_two_dirs(vec3 a, vec3 b) {
		a.normalize();
		b.normalize();
		vec3 rot_axis = cross(a, b);
		rot_axis.normalize();
		float rot_degree = acos(dot(a, b)) * 180 / PI;
		return rotate3(rot_degree, rot_axis);
	}
	void from_matrix_to_euler_angle_as_global_var(mat3 rot_mat) {
		// from http://planning.cs.uiuc.edu/node103.html
		float yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
		float pitch = atan2(-rot_mat(2, 0), sqrt(pow(rot_mat(2, 1), 2) + pow(rot_mat(2, 2), 2)));
		float roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
		cur_local_frame_rot_rel_XYZ[0] = roll * 180 / PI;
		cur_local_frame_rot_rel_XYZ[1] = pitch * 180 / PI;
		cur_local_frame_rot_rel_XYZ[2] = yaw * 180 / PI;
	}
	void construct_left_hand_box(); 
	void gui_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	void skel_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	void fast_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	void construct_boxgui();
	void render_confirm_panel();
	vec3 compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction);
	void load_mesh() {
		g_mesh_filename = cgv::gui::file_open_dialog("Open", "OBJ Files (*.obj):*.obj");
		mmesh->read_obj(g_mesh_filename.c_str());
		ds->set_mesh(mmesh);
		label_content = "[INFO] mesh loaded!\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void load_mesh_with_gui() {
		mesh_dir = working_dir + "speider_simple0/spiderman.obj";
		mmesh->read_obj(mesh_dir.c_str());
		//mmesh->read_obj(g_mesh_filename.c_str());
		ds->set_mesh(mmesh);
		label_content = "[INFO] mesh loaded!\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void skel_auto_complete() { // todo 

	}
	void toggle_mesh_render() {
		b_render_mesh = !b_render_mesh;
		if (b_render_mesh)
			label_content = "[INFO] show mesh successfully\n" + label_content;
		else
			label_content = "[INFO] hide mesh successfully\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void toggle_mesh_transparent() {
		ds->get_mesh()->b_set_transparent = !ds->get_mesh()->b_set_transparent;
		if (ds->get_mesh()->b_set_transparent)
			label_content = "[INFO] transparent on\n" + label_content;
		else
			label_content = "[INFO] transparent off\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void toggle_mesh_wireframe() {
		ds->get_mesh()->b_set_polygonmode = !ds->get_mesh()->b_set_polygonmode;
		if (ds->get_mesh()->b_set_polygonmode)
			label_content = "[INFO] wireframe button on!\n" + label_content;
		else
			label_content = "[INFO] wireframe button off!\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void toggle_face_culling() {
		ds->get_mesh()->b_face_culling = !ds->get_mesh()->b_face_culling;
		post_redraw();
	}
	void toggle_other_twoskel() {
		b_toggle_show_imitating_skel = !b_toggle_show_imitating_skel;
	}
	void toggle_imitating() {
		b_toggle_imitating = !b_toggle_imitating;
	}
	void del_skel() {
		// del. bones 
		for (auto& b : ds->get_skeleton()->get_bone_list()) {
			if (!b->get_name()._Equal("root")) { // del all bones except root bone, length 0, degenerated 
				b->get_parent()->remove_a_child(b);
			}
		}
		jointlist.clear(); // tobetested 
		label_content = "[INFO] bones deleted\n" + label_content;
		label_outofdate = true;
	}
	void load_skel_with_dofs() { // those three skel. should be added at the same time 
		// editable one 
		skel_view->load_skeleton_given_name(working_dir + "speider_simple0/jump.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys(working_dir + "speider_simple0/jump.asf");
		post_redraw();

		label_content = "[INFO] demo skel. loaded\n" + label_content;
		label_outofdate = true;
	}
	void load_demo_skel1() {
		from_jump_asf = false;
		left_ee = right_ee = hmd_ee = nullptr;
		start_point_list.clear();
		end_point_list.clear();

		skel_view->load_skeleton_given_name(working_dir + "speider_simple0/tmpskel_1.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys(working_dir + "speider_simple0/tmpskel_1.asf");
		post_redraw();

		label_content = "[INFO] button clicked!\n" + label_content;
		label_outofdate = true;
	}
	void load_demo_skel2() {
		from_jump_asf = false;
		left_ee = right_ee = hmd_ee = nullptr;
		start_point_list.clear();
		end_point_list.clear();

		skel_view->load_skeleton_given_name(working_dir + "speider_simple0/tmpskel_2.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys(working_dir + "speider_simple0/tmpskel_2.asf");
		post_redraw();

		label_content = "[INFO] button clicked!\n" + label_content;
		label_outofdate = true;
	}
	void load_demo_skel3() {
		from_jump_asf = false;
		left_ee = right_ee = hmd_ee = nullptr;
		start_point_list.clear();
		end_point_list.clear();

		skel_view->load_skeleton_given_name(working_dir + "speider_simple0/tmpskel_3.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys(working_dir + "speider_simple0/tmpskel_3.asf");
		post_redraw();

		label_content = "[INFO] button clicked!\n" + label_content;
		label_outofdate = true;
	}
	void take_screen_capture() {
		context* ctx = get_context();
		if (ctx == 0)
			return;
		ctx->write_frame_buffer_to_image(working_dir + "speider_simple0/screen_capture.png");
	}
	void load_demo_animation() {
		skel_view->load_animation_given_name(working_dir + "speider_simple0/jump.amc", false);
	}
	void load_stored_anim1() {
		skel_view->load_animation_given_name(working_dir + "speider_simple0/anim_1.amc", true);
	}
	void load_stored_anim2() {
		skel_view->load_animation_given_name(working_dir + "speider_simple0/anim_2.amc", true);
	}
	void load_stored_anim3() {
		skel_view->load_animation_given_name(working_dir + "speider_simple0/anim_3.amc", true);
	}
	void stop_anim() {
	
	}
	void start_record() {
		skel_view->prepare_record_anim();
		skel_view->start_record_anim();

		label_content = "[INFO] recording...\n" + label_content;
		label_outofdate = true;
	}
	void stop_record_and_save() {
		skel_view->stop_record_anim(working_dir + "speider_simple0/test.amc");
		label_content = "[INFO] animation has been exported as 'test.amc' \n" + label_content;
		label_outofdate = true;
	}
	void save_curskel_to_file() {
		if (ds->get_skeleton()) {
			ds->get_skeleton()->write_pinocchio_file(working_dir + "speider_simple0/tmpskel.txt");
			//ds->get_skeleton()->set_origin_rotation();
			// problem occour when rigging with pinocchio, write pinocchio with an other coordi.
			if(!from_jump_asf)
				ds->get_skeleton()->writeASFFile(working_dir + "speider_simple0/tmpskel.asf");
		}
		label_content = "[INFO] created skel. has been saved \nto tmp. file as 'tmpskel.asf'\n" + label_content;
		label_outofdate = true;
	}
	void apply_rigged_skel()
	{
		if (from_jump_asf) { // do not have to adjest skel. in vr 
			ds->get_skeleton()->read_pinocchio_file(working_dir + "speider_simple0/adjested_skeleton.out"); 	
		}
		skel_view->skeleton_changed(ds->get_skeleton());
		mmesh->read_attachment(working_dir + "speider_simple0/skinned_attachment.out");
		post_redraw();
		label_content = "[INFO] attachment has been loaded! mesh skinned\n" + label_content;
		label_outofdate = true;
	}
	void apply_dofs() {
		if (ds->get_skeleton() && tmpdata_1->get_skeleton() && tmpdata_2->get_skeleton()) {
			tmpdata_1->get_skeleton()->apply_dofs_given_skel_pointer(ds->get_skeleton().get());
			tmpdata_2->get_skeleton()->apply_dofs_given_skel_pointer(ds->get_skeleton().get());
		}
	}
	void build_skel() {
		// for the skel. we created
	}
	void gen_asf_skel_file() {
		ds->get_skeleton()->writeASFFile(working_dir + "speider_simple0/tmpskel.asf");
	}
	void start_autorigging_pinoccio() {
		label_content = "[INFO] autorig started! This may take a while...\n" + label_content;
		label_outofdate = true;
		int i;
		ArgData a;

		a.filename = mesh_dir;
		string skelfile = working_dir + "speider_simple0/tmpskel.txt";
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
		ofstream oss(working_dir + "speider_simple0/adjested_skeleton.out");
		for (i = 0; i < (int)o.embedding.size(); ++i) {
			oss << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] <<
				" " << o.embedding[i][2] << " " << a.skeleton.fPrev()[i] << endl;
		}

		//output attachment
		ofstream astrm(working_dir + "speider_simple0/skinned_attachment.out");
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
	void adjest_mesh() {
		float new_factor = hmd_origin.y() / (ds->get_mesh()->getMax().y() - ds->get_mesh()->getMin().y());
		cout << "new_factor: " << new_factor << endl;
		ds->get_mesh()->set_mesh_scale(mesh_scale * new_factor);

		// re-load mesh
		mmesh->read_obj(mesh_dir.c_str());
		ds->set_mesh(mmesh);
		label_content = "[INFO] mesh loaded!\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void shuffle_frame() {
		//cur_local_frame_rot_rel_XYZ
		vec3 bonedir_inworldspace = vec3(1, 1, 1);
		vec3 x_asix_dir = vec3(1, 0, 0);
		bonedir_inworldspace.normalize();
		x_asix_dir.normalize();
		vec3 rot_axis = cross(x_asix_dir, bonedir_inworldspace);
		rot_axis.normalize();
		float rot_degree = acos(dot(bonedir_inworldspace, x_asix_dir)) * 180 / PI;
		mat3 rot_mat = rotate3( rot_degree, rot_axis);
		// from http://planning.cs.uiuc.edu/node103.html
		float yaw = atan2(rot_mat(1, 0), rot_mat(0, 0));
		float pitch = atan2(-rot_mat(2, 0), sqrt(pow(rot_mat(2, 1), 2) + pow(rot_mat(2, 2), 2)));
		float roll = atan2(rot_mat(2, 1), rot_mat(2, 2));
		cur_local_frame_rot_rel_XYZ[0] = roll * 180 / PI;
		cur_local_frame_rot_rel_XYZ[1] = pitch * 180 / PI;
		cur_local_frame_rot_rel_XYZ[2] = yaw * 180 / PI;

		/*cur_local_frame_rot_rel_XYZ[0] = 90;
		cur_local_frame_rot_rel_XYZ[1] = 0;
		cur_local_frame_rot_rel_XYZ[2] = 0;*/
		post_redraw();
	}
	void remove_pg1() {
		/*pg1->elements.pop_back();
		pg1->boxvector.pop_back();
		pg1->colorvector.pop_back();*/
		pg1->elements.clear();
		pg1->boxvector.clear();
		pg1->colorvector.clear();
		post_redraw();
	}
	void translate_model_in_y_dir_upwards() {
		ds->get_mesh()->set_rotation_translation(
			cgv::math::rotate3<double>(180.0f, vec3(0, 1, 0)),
			vec3(1.5, (ds->get_mesh()->getMax().y() - ds->get_mesh()->getMin().y()) / 2.0f, 0));
		load_mesh();

		label_content = "[INFO] mesh position adjested!\n" + label_content;
		label_outofdate = true;
	}
	void load_addi_two_guys(string f) {
		tmpskel_view_1->load_skeleton_given_name(f);
		tmpskel_view_1->set_skel_origin_ori_translation(Vec3(0, 1, 0), 90, Vec3(-2, 1, -1.2));

		tmpskel_view_2->load_skeleton_given_name(f);
		tmpskel_view_2->set_skel_origin_ori_translation(Vec3(0, 1, 0), 45, Vec3(-2, 1, -2.8));
	}
	/*void toggle_usage_description_func() {
		toggle_usage_description = !toggle_usage_description;
	}*/
public:
	vr_test();

	std::string get_type_name() { return "vr_test"; }

	void stream_help(std::ostream& os);

	void on_set(void* member_ptr);

	bool handle(cgv::gui::event& e);
	
	bool init(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void finish_draw(cgv::render::context& ctx);

	void create_gui();
};

///@}
