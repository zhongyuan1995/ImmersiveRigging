#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
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


#include "../proj_pinocchio/skeleton.h"
#include "../proj_pinocchio/attachment.h"
#include "../proj_pinocchio/pinocchioApi.h"
struct ArgData
{
	ArgData() :
		stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
		skeleton(HumanSkeleton())
	{
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
//SkinnedMeshViewer* mesh_view;

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
		//mesh_view = new SkinnedMeshViewer(ds);

		register_object(base_ptr(skel_view), "");
		register_object(base_ptr(tmpskel_view_1), "");
		register_object(base_ptr(tmpskel_view_2), "");
		register_object(base_ptr(ik_view), "");
		//register_object(base_ptr(mesh_view), "");
	}

	~Initializer()
	{
		delete tmpdata_2;
		delete tmpdata_1;
		delete ds;
	}

} global_initializer;
///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include "intersection.h"
//#include <cgv\media\illum\surface_material.cxx>

using namespace cgv::math;
using namespace cgv::media::illum;

// different interaction states for the controllers
enum InteractionState
{
	IS_NONE,
	IS_OVER,
	IS_GRAB
};

#include "BoxGui.h"
#include <cgv/base/register.h>
#include <math_helper.h>

#include <cgv/gui/file_dialog.h>
#include <cgv/gui/dialog.h>

/// the plugin class vr_test inherits like other plugins from node, drawable and provider
class vr_test :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider
{
protected:
	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering style for boxes
	cgv::render::box_render_style style;
	//surface_material democube_mat;

	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// render information for mesh
	cgv::render::mesh_render_info MI;


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


	// keep deadzone and precision vector for left controller
	cgv::gui::vr_server::vec_flt_flt left_deadzone_and_precision;
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

	// state of current interaction with boxes for each controller
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	//std::vector<vec3> pointlist;
	std::vector<vec3> start_point_list;
	std::vector<vec3> end_point_list;
	std::vector<float> end_point_size_list; // no size for root node 
	bool is_even_point = false; // start from -1
	bool fast_is_reset = true;
	std::vector<vec3> fast_bone_posi_vec_as_chain;

	cgv::render::shader_program skyprog;
	cgv::render::texture img_tex;
	cgv::render::texture tmp_tex;
	cgv::render::texture test_tex;
	std::vector<cgv::render::texture> img_tex_list;// does not work

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

	int which_skybox = 1;

	int bone_tobeaddednext_idx = -1;
	int newbone_idx = 0;
	std::string lefthandmode;

	
	std::vector<box3> jointlist;
	std::vector<rgb> jointlist_colors;

	std::vector<box3> fast_jointlist;
	std::vector<rgb> fast_jointlist_colors;

	std::shared_ptr<SkinningMesh> mmesh;
	
	float tmpboxsize = 0.05f;
	float enlargestep = 0.0025f;

	bool b_render_mesh = true;
	bool b_toggle_show_imitating_skel = true;
	bool b_toggle_imitating = true;
	vec3 hmd_origin;
	float mirror_plane_z = -2;
	float mirror_plane_x = 1.2;

	string g_mesh_filename = "";

	Bone* left_ee;
	Bone* right_ee;
	Bone* hmd_ee;

	std::string projdir = "../../../plugins/vr_rigging_pub/";
	std::string skyboxdir = projdir + "skybox/";
	std::string meshdir = projdir + "gen_dataset/";
	// add def.


	// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
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
	vec3 compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction)
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

	// yzy 
	void skel_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color) {
		// put all boxes into a vector<box3> list, similar to the draw call in skelviewer 
		// skel_view->get_jointlist();
		// it was "static" var. getted from skel_view obj. so this will not take too much effort 
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

	// yzy
	void fast_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color) {
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

	void gui_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
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

				// transform result back to world coordinates
				//movable_box_rotations[i].rotate(p_result);
				//p_result += pg1->boxvector.at(i).get_center();
				//movable_box_rotations[i].rotate(n_result);

				// store intersection information
				gui_intersection_points.push_back(p_result);
				gui_intersection_colors.push_back(color);
				gui_intersection_box_indices.push_back((int)i);
				gui_intersection_controller_indices.push_back(ci);
			}
		}
	}
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach)
	{
		if (attach) {
			if (last_kit_handle == 0) {
				vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
				if (kit_ptr) {
					last_kit_handle = kit_handle;
					left_deadzone_and_precision = kit_ptr->get_controller_throttles_and_sticks_deadzone_and_precision(0);
					cgv::gui::ref_vr_server().provide_controller_throttles_and_sticks_deadzone_and_precision(kit_handle, 0, &left_deadzone_and_precision);
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
	void construct_table(float tw, float td, float th, float tW);
	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling);
	/// construct boxes for environment
	void construct_environment(float s, float ew, float ed, float eh, float w, float d, float h);
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_movable_boxes();
	void construct_boxgui();
	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W,
		float tw, float td, float th, float tW)
	{
		construct_room(w, d, h, W, false, false);
		construct_table(tw, td, th, tW);
		construct_environment(0.2f, 1.3 * w, 1.3 * d, h, w, d, h); // performance issue
		//construct_movable_boxes(tw, td, th, tW, 20);
		//construct_movable_boxes();
		construct_boxgui();
	}

	CullingMode cull_mode;
	ColorMapping color_mapping;
	rgb  surface_color;
	IlluminationMode illumination_mode;

	bool have_new_mesh = true;	
	cgv::media::mesh::simple_mesh<> M;

	
public:
	vr_test()
	{
		set_name("vr_rigging");
		build_scene(5, 7, 3, 0.2f, 1.0f, 1.0f, 0.8f, 0.1f);
		vr_view_ptr = 0;
		ray_length = 3;
		last_kit_handle = 0;
		connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_test::on_device_change);

		mesh_scale = 0.019f;
		mesh_location = dvec3(-1.5f, 0, -2.8f); // not used 
		mesh_orientation = dquat(1, 0, 0, 0);

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

		// add- init 
	}
	std::string get_type_name() const
	{
		return "vr_rigging";
	}
	float var1 = -1.03762f, var2 = 0.166515f, var3 = -2.77964f;
	float scale_factor = 1;
	void load_mesh() {
		g_mesh_filename = cgv::gui::file_open_dialog("Open", "OBJ Files (*.obj):*.obj");
		//mmesh->read_obj("D:\zrdevpacks\zract_easyRigging\data_attached\gen_dataset\speider_simple1/spiderman.obj");
		mmesh->read_obj(g_mesh_filename.c_str());
		ds->set_mesh(mmesh);
		label_content = "[INFO] mesh loaded!\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void skel_auto_complete() { // todo 

	}
	void toggle_mesh_render() {
		b_render_mesh = !b_render_mesh;
		if(b_render_mesh)
			label_content = "[INFO] show mesh successfully\n" + label_content;
		else
			label_content = "[INFO] hide mesh successfully\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void toggle_mesh_transparent() {
		ds->get_mesh()->b_set_transparent = !ds->get_mesh()->b_set_transparent;
		if(ds->get_mesh()->b_set_transparent)
			label_content = "[INFO] transparent on\n" + label_content;
		else 
			label_content = "[INFO] transparent off\n" + label_content;
		label_outofdate = true;
		post_redraw();
	}
	void toggle_mesh_wireframe() {
		ds->get_mesh()->b_set_polygonmode = !ds->get_mesh()->b_set_polygonmode;
		if(ds->get_mesh()->b_set_polygonmode)
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
	void load_skel() { // those three skel. should be added at the same time 
		// editable one 
		skel_view->load_skeleton_given_name("tmpskel.asf");
		skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
		load_addi_two_guys("tmpskel.asf");
		post_redraw();

		label_content = "[INFO] demo skel. loaded\n" + label_content;
		label_outofdate = true;
	}
	void take_screen_capture() {
		context* ctx = get_context();
		if (ctx == 0)
			return;
		ctx->write_frame_buffer_to_image("screen_capture.png");
	}
	void load_demo_animation() {
		skel_view->load_animation_given_name("FFA_REGULAR/zrdevpacks/zract_easyRigging/data_attached/_workdemo1_spiderman/jump.amc", false);
	}
	void load_stored_anim1() {
		skel_view->load_animation_given_name("anim_1.amc", true);
	}
	void load_stored_anim2() {
		skel_view->load_animation_given_name("anim_2.amc", true);
	}
	void start_record() {
		skel_view->prepare_record_anim();
		skel_view->start_record_anim();

		label_content = "[INFO] recording...\n" + label_content; 
		label_outofdate = true;
	}
	void stop_record_and_save() {
		skel_view->stop_record_anim("test.amc");
		label_content = "[INFO] animation has been exported as 'test.amc' \n" + label_content; 
		label_outofdate = true;
	}
	void save_curskel_to_file(){
		if (ds->get_skeleton()) {
			ds->get_skeleton()->write_pinocchio_file("tmpskel.txt");
			//ds->get_skeleton()->set_origin_rotation();
			// problem occour when rigging with pinocchio, write pinocchio with an other coordi.
			ds->get_skeleton()->writeASFFile("tmpskel.asf");
		}
		label_content = "[INFO] created skel. has been saved \nto tmp. file as 'tmpskel.asf'\n" + label_content; 
		label_outofdate = true;
	}
	void apply_rigged_skel() 
	{
		//ds->get_skeleton()->read_pinocchio_file("adjested_skeleton.out"); // do not have to adjest skel. in vr 
		mmesh->read_attachment("skinned_attachment.out");
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
		ds->get_skeleton()->writeASFFile("tmpskel.asf");
	}
	void start_autorigging_pinoccio() {
		label_content = "[INFO] autorig started! This may take a while...\n" + label_content;
		label_outofdate = true;
		int i;
		ArgData a;

		a.filename = g_mesh_filename;
		string skelfile = "tmpskel.txt";
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
		ofstream oss("adjested_skeleton.out");
		for (i = 0; i < (int)o.embedding.size(); ++i) {
			oss << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] <<
				" " << o.embedding[i][2] << " " << a.skeleton.fPrev()[i] << endl;
		}

		//output attachment
		ofstream astrm("skinned_attachment.out");
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
		ds->get_mesh()->set_mesh_scale(new_factor * mesh_scale);
		load_mesh();

		label_content = "[INFO] mesh position changed!\n" + label_content;
		label_outofdate = true;
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
		tmpskel_view_1->set_skel_origin_ori_translation(Vec3(0,1,0), 90, Vec3(-2, 1, -1.2));

		tmpskel_view_2->load_skeleton_given_name(f);
		tmpskel_view_2->set_skel_origin_ori_translation(Vec3(0, 1, 0), 45, Vec3(-2, 1, -2.8));
	}
	void create_gui()
	{
		add_decorator("vr_test", "heading", "level=2");
		// take_screen_capture
		connect_copy(add_button("take_screen_capture")->click, cgv::signal::rebind(this, &vr_test::take_screen_capture));
		// load demo 
		connect_copy(add_button("load mesh.")->click, cgv::signal::rebind(this, &vr_test::load_mesh));
		connect_copy(add_button("load skel.")->click, cgv::signal::rebind(this, &vr_test::load_skel));
		connect_copy(add_button("load_demo_animation")->click, cgv::signal::rebind(this, &vr_test::load_demo_animation));
		// animation rel.
		connect_copy(add_button("start_record")->click, cgv::signal::rebind(this, &vr_test::start_record));
		connect_copy(add_button("stop_record_and_save")->click, cgv::signal::rebind(this, &vr_test::stop_record_and_save));
		connect_copy(add_button("load_stored_anim1")->click, cgv::signal::rebind(this, &vr_test::load_stored_anim1));
		connect_copy(add_button("load_stored_anim2")->click, cgv::signal::rebind(this, &vr_test::load_stored_anim2));
		connect_copy(add_button("apply_dofs_to_multiview")->click, cgv::signal::rebind(this, &vr_test::apply_dofs));
		
		// skel. rel.
		connect_copy(add_button("save_skel")->click, cgv::signal::rebind(this, &vr_test::save_curskel_to_file));
		connect_copy(add_button("start_autorig")->click, cgv::signal::rebind(this, &vr_test::start_autorigging_pinoccio));
		connect_copy(add_button("apply_rigged_skel")->click, cgv::signal::rebind(this, &vr_test::apply_rigged_skel));
		connect_copy(add_button("del_skel")->click, cgv::signal::rebind(this, &vr_test::del_skel));
		connect_copy(add_button("toggle_show_other_twoskel")->click, cgv::signal::rebind(this, &vr_test::toggle_other_twoskel));
		connect_copy(add_button("toggle_imitating")->click, cgv::signal::rebind(this, &vr_test::toggle_imitating));

		// mesh rel. 
		connect_copy(add_button("toggle_mesh_render")->click, cgv::signal::rebind(this, &vr_test::toggle_mesh_render));
		connect_copy(add_button("toggle_mesh_transparent")->click, cgv::signal::rebind(this, &vr_test::toggle_mesh_transparent));
		connect_copy(add_button("toggle_mesh_wireframe")->click, cgv::signal::rebind(this, &vr_test::toggle_mesh_wireframe));
		connect_copy(add_button("toggle_face_culling")->click, cgv::signal::rebind(this, &vr_test::toggle_face_culling));
		connect_copy(add_button("adjest mesh scale", "", "\n")->click,cgv::signal::rebind(this, &vr_test::adjest_mesh));
		//translate_model_in_y_dir_upwards
		connect_copy(add_button("upward_translation", "", "\n")->click, cgv::signal::rebind(this, &vr_test::translate_model_in_y_dir_upwards));
		

		//add_member_control(this, "scale factor", scale_factor_skel, "value_slider", "min=0.01;max=10");
		add_member_control(this, "skel_posi_x", var1, "value_slider", "min=-10;max=10;log=true;ticks=true");
		add_member_control(this, "skel_posi_y", var2, "value_slider", "min=-10;max=10;log=true;ticks=true");
		add_member_control(this, "skel_posi_z", var3, "value_slider", "min=-10;max=10;log=true;ticks=true");
		add_member_control(this, "skel_scale", scale_factor, "value_slider", "min=0.01;max=10;log=true;ticks=true");
		add_member_control(this, "mesh_scale", mesh_scale, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		add_gui("mesh_location", mesh_location, "vector", "options='min=-3;max=3;ticks=true");
		add_gui("mesh_orientation", static_cast<dvec4&>(mesh_orientation), "direction", "options='min=-1;max=1;ticks=true");
		add_member_control(this, "ray_length", ray_length, "value_slider", "min=0.1;max=10;log=true;ticks=true");
		if (last_kit_handle) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
			const std::vector<std::pair<int, int> >* t_and_s_ptr = 0;
			if (kit_ptr)
				t_and_s_ptr = &kit_ptr->get_controller_throttles_and_sticks(0);
			add_decorator("deadzone and precisions", "heading", "level=3");
			int ti = 0;
			int si = 0;
			for (unsigned i = 0; i < left_deadzone_and_precision.size(); ++i) {
				std::string prefix = std::string("unknown[") + cgv::utils::to_string(i) + "]";
				if (t_and_s_ptr) {
					if (t_and_s_ptr->at(i).second == -1)
						prefix = std::string("throttle[") + cgv::utils::to_string(ti++) + "]";
					else
						prefix = std::string("stick[") + cgv::utils::to_string(si++) + "]";
				}
				add_member_control(this, prefix + ".deadzone", left_deadzone_and_precision[i].first, "value_slider", "min=0;max=1;ticks=true;log=true");
				add_member_control(this, prefix + ".precision", left_deadzone_and_precision[i].second, "value_slider", "min=0;max=1;ticks=true;log=true");
			}
		}
		if (begin_tree_node("box style", style)) {
			align("\a");
			add_gui("box style", style);
			align("\b");
			end_tree_node(style);
		}
		if (begin_tree_node("movable box style", movable_style)) {
			align("\a");
			add_gui("movable box style", movable_style);
			align("\b");
			end_tree_node(movable_style);
		}
		if (begin_tree_node("intersections", srs)) {
			align("\a");
			add_gui("sphere style", srs);
			align("\b");
			end_tree_node(srs);
		}
		if (begin_tree_node("mesh", mesh_scale)) {
			align("\a");
			add_member_control(this, "scale", mesh_scale, "value_slider", "min=0.0001;step=0.0000001;max=100;log=true;ticks=true");
			add_gui("location", mesh_location, "", "main_label='';long_label=true;gui_type='value_slider';options='min=-2;max=2;step=0.001;ticks=true'");
			add_gui("orientation", static_cast<dvec4&>(mesh_orientation), "direction", "main_label='';long_label=true;gui_type='value_slider';options='min=-1;max=1;step=0.001;ticks=true'");
			align("\b");
			end_tree_node(mesh_scale);
		}

		if (begin_tree_node("label", label_size)) {
			align("\a");
			add_member_control(this, "text", label_text);
			add_member_control(this, "upright", label_upright);
			add_member_control(this, "font", (cgv::type::DummyEnum&)label_font_idx, "dropdown", font_enum_decl);
			add_member_control(this, "face", (cgv::type::DummyEnum&)label_face_type, "dropdown", "enums='regular,bold,italics,bold+italics'");
			add_member_control(this, "size", label_size, "value_slider", "min=8;max=64;ticks=true");
			add_member_control(this, "color", label_color);
			add_member_control(this, "resolution", (cgv::type::DummyEnum&)label_resolution, "dropdown", "enums='256=256,512=512,1024=1024,2048=2048'");
			align("\b");
			end_tree_node(label_size);
		}
	}
	void on_set(void* member_ptr)
	{
		if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
			label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
			label_outofdate = true;
		}
		if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
			member_ptr == &label_size || member_ptr == &label_text) {
			label_outofdate = true;
		}
		update_member(member_ptr);
		post_redraw();
	}
	void stream_help(std::ostream& os)
	{
		os << "vr_test: no shortcuts defined" << std::endl;
	}

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
	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	bool show_my_box = false;
	bool teleport = false;
	bool object_teleport = false;
	bool drawingbone = false;
	bool handle(cgv::gui::event& e)
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
				/*std::cout << vrke.get_state().controller[0].button_flags << std::endl;
				std::cout << vrke.get_state().controller[1].button_flags << std::endl;*/
				switch (vrke.get_key()) {
					case vr::VR_LEFT_BUTTON0:
						std::cout << "button 0 of left controller pressed" << std::endl;
						teleport = true;
						return true;
					case vr::VR_LEFT_MENU:
						std::cout << "button VR_LEFT_MENU of left controller pressed" << std::endl;
						/*if (lefthandmode._Equal("add bone")) {
							keydown = true;
							is_even_point = !is_even_point;
						}*/
						object_teleport = true;
						
						return true;
					case vr::VR_RIGHT_BUTTON0:
						std::cout << "button 0 of right controller pressed" << std::endl;
						btn_keydown_boxgui = true;
						return true;
					case vr::VR_RIGHT_STICK_RIGHT:
						std::cout << "touch pad of right controller pressed at right direction" << std::endl;
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
			case cgv::gui::SA_TOUCH: // all operations will be done with left hand, just a touch 
				// if (state[vrse.get_controller_index()] == IS_OVER)
				// if both controllers are touched, start ccd. tobetested 
				if ((vrse.get_state().controller[0].button_flags == 0x0020)
					&&(vrse.get_state().controller[1].button_flags == 0x0020)) {
					toggle_ccd = !toggle_ccd;
					// set to the desired position in front of mirror 
					// pp x component of the position can not be setted correctly, due to calibration?
					// z component must be set to this value! 
					vr_view_ptr->set_tracking_origin(Vec3(
						vr_view_ptr->get_tracking_origin().x(),
						vr_view_ptr->get_tracking_origin().y(), 
						-1.2f));
					if(toggle_ccd)
						toggle_chain = !toggle_chain;
					if(toggle_ccd)
						label_content = "[INFO] ccd on!\n" + label_content;
					else
						label_content = "[INFO] ccd off!\n" + label_content;
					label_outofdate = true;

				}
				if (vrse.get_state().controller[0].button_flags == 0x0020) {// for left hand only 
					std::cout << vrse.get_state().controller[0].button_flags << std::endl;
					std::cout << vrse.get_state().controller[1].button_flags << std::endl;
					if (lefthandmode._Equal("add bone")) {
						keydown = true;
						is_even_point = !is_even_point;
					}
					if (lefthandmode._Equal("move bone"))
						state[vrse.get_controller_index()] = IS_GRAB;
					if (lefthandmode._Equal("fast_add_root")) {
						b_fast_add_root = true;
					}
					//if (lefthandmode._Equal("fast_add_skel")) {
					//	b_fast_add_skel = true;
					//	//fast_is_even_point = !fast_is_even_point;
					//}
					/*if (lefthandmode._Equal("fast_reset")) { // do not use this
						b_fast_reset = true;
					}*/
					/*if (lefthandmode._Equal("add bone")) {
						keydown = true;
						is_even_point = !is_even_point;
					}*/
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
				
			case cgv::gui::SA_UNPRESS:
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
				return true;
			case cgv::gui::SA_MOVE:
			case cgv::gui::SA_DRAG: 
				std::cout << "stick " << vrse.get_stick_index()
					<< " of controller " << vrse.get_controller_index()
					<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
					<< " from " << vrse.get_last_x() << ", " << vrse.get_last_y()
					<< " to " << vrse.get_x() << ", " << vrse.get_y() << std::endl;


				if (vrse.get_state().controller[1].axes[1] > 0 && tmpboxsize < 0.5f) {
					tmpboxsize += enlargestep;
				}
				else if(tmpboxsize > 0.04f){
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
				return true;
			}
			return true;
		}

		case cgv::gui::EID_POSE:
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (ci != -1) {
				// teleport 
				if (teleport) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0)); // attention! [0/1]
					// compute intersection point 
					vec3 posi = compute_ray_plane_intersection_point(origin, direction);
					// set the point 
					vr_view_ptr->set_tracking_origin(Vec3(posi.x(), vr_view_ptr->get_tracking_origin().y(), posi.z()));

					teleport = false;
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
						for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					// has intersec. with skel. joint box 
					if (skel_intersection_points.size() > 0) {
						left_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
						ds->set_endeffector(left_ee, 0);
						// ik_view->endeffector_changed(cur_bone, 0);// must include this to calcu. the kinematics chain!
						// provide info. later todo. 
						// set up the endeffector of chain0, and calculate chain auto.
						// we have to calculate chain1 and 2 with other two buttons 
					}
					post_redraw();
					select_endeffector = false;
				}
				if (select_endeffector_1) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					// clear skel. intersection list, tmp usage
					skel_intersection_points.clear();
					skel_intersection_box_indices.clear();
					for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					// has intersec. with skel. joint box 
					if (skel_intersection_points.size() > 0) {
						right_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
						ds->set_endeffector(right_ee, 0);
						//ik_view->endeffector_changed(cur_bone, 0);// must include this to calcu. the kinematics chain!
						// provide info. later todo. 
						// set up the endeffector of chain0, and calculate chain auto.
						// we have to calculate chain1 and 2 with other two buttons 
					}
					post_redraw();
					select_endeffector_1 = false;
				}
				if (select_endeffector_2) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					// clear skel. intersection list, tmp usage
					skel_intersection_points.clear();
					skel_intersection_box_indices.clear();
					for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					// has intersec. with skel. joint box 
					if (skel_intersection_points.size() > 0) {
						hmd_ee = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
						ds->set_endeffector(hmd_ee, 0);
						// set up the endeffector of chain2, and calculate chain auto.
						// we have to calculate chain1 and 2 with other two buttons 
					}
					post_redraw();
					select_endeffector_2 = false;
				}

				if (select_base) {
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					// clear skel. intersection list, tmp usage
					skel_intersection_points.clear();
					skel_intersection_box_indices.clear();
					for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					// has intersec. with skel. joint box 
					if (skel_intersection_points.size() > 0) {
						Bone* cur_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(skel_intersection_box_indices.front());
						ds->set_base(cur_bone);
					}
					post_redraw();
					select_base = false;
				}

				if (toggle_ccd) {
					if (ds->get_base() && right_ee && left_ee ) { // ds ee not defined now 
						vec3 origin, direction;

						// ccd calcu. skel. based on LEFT hand  
						ds->set_endeffector(left_ee, 0);
						vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0)); 
						ik_view->set_target_position_vr(
							Vec4(
								origin.x(), 
								origin.y(), 
								2 * mirror_plane_z - origin.z(), 1));
						//ik_view->set_max_iter(30);
						ik_view->optimize(0);

						// ccd calcu. skel. based on RIGHT hand
						ds->set_endeffector(right_ee, 0);
						vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
						ik_view->set_target_position_vr(
							Vec4(
								origin.x(), 
								origin.y(), 
								2 * mirror_plane_z - origin.z(), 1));
						//ik_view->set_max_iter(30);
						ik_view->optimize(0);

						//// ccd calcu. skel. based on head position 
						//ds->set_endeffector(hmd_ee, 0);
						//hmd_origin.x() = vrpe.get_state().hmd.pose[9];
						//hmd_origin.y() = vrpe.get_state().hmd.pose[10];
						//hmd_origin.z() = vrpe.get_state().hmd.pose[11];
						//ik_view->set_target_position_vr(
						//	Vec4(
						//		hmd_origin.x(),
						//		hmd_origin.y(),
						//		2 * mirror_plane_z - hmd_origin.z(), 1));
						//ik_view->set_max_iter(2);
						//ik_view->optimize(0); // optimize the chain2, head chain 

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
						del_bone->get_parent()->remove_a_child(del_bone);
						skel_view->skeleton_changed(ds->get_skeleton());
					}
					del_keydown = false;
				}

				if (keydown) {
					if (!skel_view->playing) {
						vec3 origin, direction;
						vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
						if (is_even_point) {// an intersection is required for even points, but not for odd points
							// clear skel. intersection list, tmp usage
								skel_intersection_points.clear();
								skel_intersection_box_indices.clear();
								for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
							skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
							// has intersec. with skel. joint box 
							if (skel_intersection_points.size() > 0) {
								// front means the first intersection box idx 
								start_point_list.push_back(jointlist.at(skel_intersection_box_indices.front()).get_center());
								jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 1, 102.0f / 255.0f);
								// compute cur. bone ref. here 
								bone_tobeaddednext_idx = skel_intersection_box_indices.front();// the same order as jointist, and, bone list!
								drawingbone = true;
							}
						}
						else if(drawingbone){ // is even point and are drawing bones, has intersection before 
							end_point_list.push_back(cur_left_hand_posi + vec3(cur_left_hand_dir * 0.2f)); // tobetested
							//end_point_size_list.push_back(tmpboxsize);

							// add bone here. addchild method will be called, ref. to the bone reading process 
							Bone* parent_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(bone_tobeaddednext_idx);
							std::cout << parent_bone->get_name() << std::endl;
							Bone* current_node = new Bone();
							current_node->set_name("new_bone_" + to_string(newbone_idx++)); // "new_bone_0, new_bone_1..."
							ds->get_skeleton()->add_new_bone_to_map("new_bone_" + to_string(newbone_idx++), current_node);

							// adjest bone para. that we have drawn 
							Vec3 bonedir_inworldspace = end_point_list.at(end_point_list.size() - 1)
								- start_point_list.at(start_point_list.size() - 1);
							current_node->set_direction_in_world_space(bonedir_inworldspace); // length already included 
							current_node->set_length(1);

							// local coordi. the same as global one 
							/*axis 0 0 -20   XYZ*/
							float a[3] = { 0, 0, 0 };// will be adjested later. todo
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
								t->set_value(a[i]);
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
							for (int i = 0; i < n_dofs; ++i)
								current_node->get_dof(n_dofs - i - 1)->set_limits(-180.0, 180.0);

							current_node->jointsize_stored_as_bone_parameter = tmpboxsize;

							// pf 1h
							// this will be calcu. in the post process step
							//current_node->calculate_matrices();
							
							parent_bone->add_child(current_node);
							// perform post process, bounding box will be re-calculated! we need it to write pinocchio file 
							ds->get_skeleton()->postprocess(ds->get_skeleton()->get_root(), Vec3(0, 0, 0));
							// std::cout << skel_view->get_jointlist().size();
							// update skel. the tree view will be updated at the sametime 
							skel_view->skeleton_changed(ds->get_skeleton()); // jointlist updated inside 
							drawingbone = false;
						}
						post_redraw();
					}
					keydown = false;
				}

				// only this for right hand 
				if (btn_keydown_boxgui) {
					// clear all intersections, gui
					gui_intersection_points.clear();
					gui_intersection_colors.clear();
					gui_intersection_box_indices.clear();
					gui_intersection_controller_indices.clear();

					// compute intersec. with main gui  
					vec3 origin, direction;
					vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
					gui_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					//std::cout << "entering button  :" << std::endl;
					if (gui_intersection_points.size() > 0) {
						// button clicked! button handlers!
						// std::cout << "gui button " << gui_intersection_box_indices.front() << " clicked!" << std::endl;
						// call back func.!
						int cur_btn_idx = gui_intersection_box_indices.front();
						// load diff. skybox 
						if (cur_btn_idx == 3) {
							which_skybox = 0;
							post_redraw();
						}
						if (cur_btn_idx == 4) {
							which_skybox = 1;
							post_redraw();
						}
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
						// load demo mesh 
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_demo1")) {
							mmesh->read_obj("FFA_REGULAR/zrdevpacks/zract_easyRigging/data_attached/gen_dataset/speider_simple1/spiderman.obj");
							//mmesh->read_obj(g_mesh_filename.c_str());
							ds->set_mesh(mmesh);
							label_content = "[INFO] mesh loaded!\n" + label_content;
							label_outofdate = true;
							post_redraw();
						}
						// load demo skel. 
						if (pg1->elements.at(cur_btn_idx).label._Equal("demoskel")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							//skel_view->load_skeleton_given_name("FFA_REGULAR/zrdevpacks/zract_easyRigging/data_attached/_workdemo1_spiderman/jump.asf");
							//skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.25, 1, -2.8));
							load_skel();
						}
						//l_skel1
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel1")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_skeleton_given_name("tmpskel_1.asf");
							skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
							post_redraw();
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel2")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_skeleton_given_name("tmpskel_2.asf");
							skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
							post_redraw();
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_skel3")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_skeleton_given_name("tmpskel_3.asf");
							skel_view->set_skel_origin_ori_translation(Vec3(0, 1, 0), 0, Vec3(1.2, 1, -2.8));
							post_redraw();
						}
						// save skel. 
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel1")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							ds->get_skeleton()->write_pinocchio_file("tmpskel_1.txt");
							ds->get_skeleton()->writeASFFile("tmpskel_1.asf");
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel2")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							ds->get_skeleton()->write_pinocchio_file("tmpskel_2.txt");
							ds->get_skeleton()->writeASFFile("tmpskel_2.asf");
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_skel3")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							ds->get_skeleton()->write_pinocchio_file("tmpskel_3.txt");
							ds->get_skeleton()->writeASFFile("tmpskel_3.asf");
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
						if (pg1->elements.at(cur_btn_idx).label._Equal("ik1")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_base")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							lefthandmode = "select base";
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_ee_left")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							lefthandmode = "select ee left";
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_ee_right")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							lefthandmode = "select ee right";
						}
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
						if (pg1->elements.at(cur_btn_idx).label._Equal("add_bone_\nexist")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							lefthandmode = "add bone";
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
							skel_view->stop_record_anim("anim_1.amc");
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_anim2")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->stop_record_anim("anim_2.amc");
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("s_anim3")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->stop_record_anim("anim_3.amc");
						}
						// load animation 
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim1")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_animation_given_name("anim_1.amc", true);
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim2")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_animation_given_name("anim_2.amc", true);
						}
						if (pg1->elements.at(cur_btn_idx).label._Equal("l_anim3")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
							skel_view->load_animation_given_name("anim_3.amc", true);
						}

						if (pg1->elements.at(cur_btn_idx).label._Equal("l_scene1")) {
							label_content = "[INFO] button clicked!\n" + label_content;
							label_outofdate = true;
						}

						// add button callback
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
				else {// no button, usually. // animation, when intersection! anim only works for right controller? pp- 
					// clean vectors used for intersection calculation (may not need a vector at all, we typically need the first intersec.)
						// boxgui
						gui_intersection_points.clear();
						gui_intersection_colors.clear();
						gui_intersection_box_indices.clear();
						gui_intersection_controller_indices.clear();
						for (auto e : pg1->elements) { e.has_intersec = false; }
						// skel.
						skel_intersection_points.clear();
						skel_intersection_box_indices.clear();
						for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }

					// varible computing 
						hmd_origin.x() = vrpe.get_state().hmd.pose[9];
						hmd_origin.y() = vrpe.get_state().hmd.pose[10];
						hmd_origin.z() = vrpe.get_state().hmd.pose[11];
						//std::cout << hmd_origin << std::endl;
						/*ray_origin[0] = pose[9];
						ray_origin[1] = pose[10];
						ray_origin[2] = pose[11];*/

					// compute intersections 
						vec3 origin, direction;
						// for getting left hand posi. cur.
						vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
							// mark cur. posi as global var.
							cur_left_hand_posi = origin;
							cur_left_hand_dir = direction;
							if(!skel_view->playing)
								skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
							//cur_left_hand_rot = vrpe.get_rotation_matrix();
							vrpe.get_state().controller[1].put_ray(&origin(0), &direction(0));
						// can be optimized later. todo 
						gui_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					
					// has intersec. with skel. joint box 
						if (skel_intersection_points.size() > 0) {
							// front means the first intersection box idx 
							jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 1, 102.0f/255.0f);
							post_redraw();
						}

					// has intersec. with boxgui
						if (gui_intersection_points.size() > 0) {
							/*box3 curbox = pg1->boxvector.at(gui_intersection_box_indices.front());
							pg1->boxvector.at(gui_intersection_box_indices.front()) =
								box3(curbox.get_center() + vec3(0.05, 0, 0) - curbox.get_extent() / 2.0f,
									curbox.get_center() + vec3(0.05, 0, 0) + curbox.get_extent() / 2.0f);*/
							pg1->elements.at(gui_intersection_box_indices.front()).has_intersec = true;
							pg1->push_to_render_vector();// re-gen the vec. for rendering 
							post_redraw();
						}
						else {
							// no intersection gui
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
								post_redraw();
							}
						}
				}
				
				if (state[0] == IS_GRAB) {// for left hand , translation of the selected bone node
					//std::cout << "IS_GRAB~" << "\n";
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					// clear skel. intersection list, tmp usage
						skel_intersection_points.clear();
						skel_intersection_box_indices.clear();
						for (auto jc : jointlist_colors) { jc = rgb(1, 1, 1); }
					skel_joint_box_compute_intersections(origin, direction, 1, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
					// get previous and current controller position
					vec3 last_pos = vrpe.get_last_position();
					vec3 pos = vrpe.get_position();
					mat3 rotation = vrpe.get_rotation_matrix();
					if (skel_intersection_points.size() > 0) {
						jointlist_colors.at(skel_intersection_box_indices.front()) = rgb(1, 0, 0);
						int cur_bone_idx = skel_intersection_box_indices.front();
						Bone* cur_bone = ds->get_skeleton()->find_bone_in_a_list_by_id(cur_bone_idx);
						vec3 oldcenter = jointlist.at(cur_bone_idx).get_center();
						vec3 newcenter = rotation * (oldcenter - last_pos) + pos;
						vec3 tmppoint = newcenter;
						jointlist.at(cur_bone_idx) = box3( // just for intersection test, not for rendering 
							vec3(tmppoint.x() - cur_bone->jointsize_stored_as_bone_parameter / 2, //tmpboxsize
								tmppoint.y() - cur_bone->jointsize_stored_as_bone_parameter / 2,
								tmppoint.z() - cur_bone->jointsize_stored_as_bone_parameter / 2),
							vec3(tmppoint.x() + cur_bone->jointsize_stored_as_bone_parameter / 2,
								tmppoint.y() + cur_bone->jointsize_stored_as_bone_parameter / 2,
								tmppoint.z() + cur_bone->jointsize_stored_as_bone_parameter / 2));// setup new posi. if has intersection 
						// to setup the bone , we have to change translationTransformCurrentJointToNext of the choosen bone
						Mat4 oldt = cur_bone->get_translation_transform_current_joint_to_next();
						Mat4 curt = translate(newcenter - oldcenter);
						Mat4 newt = curt * oldt;
						cur_bone->set_translation_transform_current_joint_to_next(newt);

						skel_view->skeleton_changed(ds->get_skeleton());// update bone
					}
				}
				else {state[0] = IS_OVER;}

				if (b_fast_add_root) { // add joints as boxes 
					// click to add bones 
					vec3 origin, direction;
					vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					//vec3(aTip.x() - cubesize / 2
					//vec3 cubesize_as_vector = vec3(skel_view->cubesize);

					//// for rendering 
					//fast_jointlist.push_back(box3(
					//	origin - cubesize_as_vector / 2.0f, 
					//	origin + cubesize_as_vector / 2.0f));
					//fast_jointlist_colors.push_back(rgb(0,0,1)); // a blue box will add

					// add to skel.
					Bone* current_node = new Bone();
					current_node->set_name("root"); // "new_bone_0, new_bone_1..."
					//ds->get_skeleton()->add_new_bone_to_map("root", current_node); // nullpointer 
								
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

					// adjest the size of the root bone // tobetested 
					current_node->jointsize_stored_as_bone_parameter = tmpboxsize;

					// pf 1h
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

				if (b_fast_add_skel) { // add skel as lines // you need to follow the chains! 
					// clean
						fast_intersection_points.clear();
						fast_intersection_box_indices.clear();
						//for (auto jc : fast_jointlist_colors) { jc = rgb(1, 1, 1); }
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
					//fast_bone_posi_vec_as_chain.push_back(vec3(-100, -100, -100)); // reset mark for bone building 
					//fast_is_reset = true; // for rendering 
					b_fast_reset = false;
				}

				if (vrpe.get_trackable_index() == 0) { // move acco. to left hand 
					vec3 last_pos = vrpe.get_last_position();
					vec3 pos = vrpe.get_position();
					//cur_left_hand_posi = vrpe.get_position();
					cur_left_hand_rot = vrpe.get_rotation_matrix();

					for (auto& tran : movable_box_translations) {
						tran = cur_left_hand_rot * (tran - last_pos) + pos;
					}
					for (auto& rot: movable_box_rotations) {
						rot = quat(cur_left_hand_rot) * rot;
					}
				}

				post_redraw();
			}
			return true;
		}
		return false;
	}
	bool init(cgv::render::context& ctx)
	{
		if (!cgv::utils::has_option("NO_OPENVR"))
			ctx.set_gamma(2.2f);

		cgv::gui::connect_vr_server(true);

		auto view_ptr = find_view_as_node();
		if (view_ptr) {
			view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
			// if the view points to a vr_view_interactor
			vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
			if (vr_view_ptr) {
				// configure vr event processing
				vr_view_ptr->set_event_type_flags(
					cgv::gui::VREventTypeFlags(
						cgv::gui::VRE_KEY +
						cgv::gui::VRE_THROTTLE +
						cgv::gui::VRE_STICK +
						cgv::gui::VRE_STICK_KEY +
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
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_sphere_renderer(ctx, 1);

		skyprog.build_program(ctx, "skycube.glpr");

		img_tex.create_from_images(ctx, skyboxdir + "cm_{xp,xn,yp,yn,zp,zn}.jpg");
		tmp_tex.create_from_images(ctx, skyboxdir + "BluePinkNebular_{xp,xn,yp,yn,zp,zn}.jpg");
		test_tex.create_from_images(ctx, skyboxdir + "igen_2/{xp,xn,yp,yn,zp,zn}.jpg");
		pg1->icon_shader_prog.build_program(ctx, "image.glpr");

		cull_mode = CM_BACKFACE;
		color_mapping = cgv::render::CM_COLOR;
		surface_color = rgb(0.7f, 0.2f, 1.0f);
		illumination_mode = IM_ONE_SIDED;


		cgv::render::gl::ensure_glew_initialized();
		mmesh = std::make_shared<SkinningMesh>();
		mmesh->init_shaders(ctx);
		mmesh->set_rotation_translation(cgv::math::rotate3<double>(0 , vec3(0, 1, 0)),vec3(1.2, 0, -2.8));
		mmesh->set_mesh_scale(mesh_scale);

		fast_bone_posi_vec_as_chain.clear();

		// add- : init
		return true;
	}
	void clear(cgv::render::context& ctx)
	{
		cgv::render::ref_box_renderer(ctx, -1);
		cgv::render::ref_sphere_renderer(ctx, -1);
	}
	void init_frame(cgv::render::context& ctx)
	{
		if (have_new_mesh) {
			// auto-compute mesh normals if not available
			if (!M.has_normals())
				M.compute_vertex_normals();
			// [re-]compute mesh render info
			MI.destruct(ctx);
			MI.construct_vbos(ctx, M);
			// bind mesh attributes to standard surface shader program
			MI.bind(ctx, ctx.ref_surface_shader_program(true));

			// ensure that materials are presented in gui
			//post_recreate_gui();
			have_new_mesh = false;
		}
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
					/*for (size_t i = 0; i < intersection_points.size(); ++i) {
						ctx.output_stream()
							<< "box " << intersection_box_indices[i]
							<< " at (" << intersection_points[i]
							<< ") with controller " << intersection_controller_indices[i] << "\n";
					}*/
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
				glClearColor(0.1f,0.1f,0.1f,1.0f);
				glClear(GL_COLOR_BUFFER_BIT);

				glColor4f(label_color[0], label_color[1], label_color[2], 1);
				ctx.set_cursor(20, (int)ceil(label_size) + 20);
				ctx.enable_font_face(label_font_face, label_size);
				ctx.output_stream() << label_text << "\n";
				ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

				ctx.enable_font_face(label_font_face, 0.7f*label_size);
				/*for (size_t i = 0; i < intersection_points.size(); ++i) {
					ctx.output_stream()
						<< "box " << intersection_box_indices[i]
						<< " at (" << intersection_points[i]
						<< ") with controller " << intersection_controller_indices[i] << "\n";
				}*/
				ctx.output_stream() << label_content;
				ctx.output_stream().flush();

			ctx.pop_pixel_coords();
			label_fbo.pop_viewport(ctx);
			label_fbo.disable(ctx);
			glPopAttrib();
			label_outofdate = false;

			label_tex.generate_mipmaps(ctx);
		}

		// usage description 
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
			ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

			ctx.enable_font_face(ulabel_font_face, 0.7f * ulabel_size);
			/*for (size_t i = 0; i < intersection_points.size(); ++i) {
				ctx.output_stream()
					<< "box " << intersection_box_indices[i]
					<< " at (" << intersection_points[i]
					<< ") with controller " << intersection_controller_indices[i] << "\n";
			}*/
			ctx.output_stream() << ulabel_content;
			ctx.output_stream().flush();

			ctx.pop_pixel_coords();
			ulabel_fbo.pop_viewport(ctx);
			ulabel_fbo.disable(ctx);
			glPopAttrib();
			ulabel_outofdate = false;

			ulabel_tex.generate_mipmaps(ctx);
		}
	}
	void draw_axes(cgv::render::context& ctx, bool transformed)
	{
		float c = transformed ? 0.7f : 1;
		float d = 1 - c;
		float l = 1;
		ctx.set_color(rgb(c, d, d));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(l, 0, 0), 0.02);
		ctx.set_color(rgb(d, c, d));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(0, l, 0), 0.02);
		ctx.set_color(rgb(d, d, c));
		ctx.tesselate_arrow(cgv::math::fvec<double, 3>(0, 0, 0), cgv::math::fvec<double, 3>(0, 0, l), 0.02);
	}
	void draw_surface(context& ctx, bool opaque_part)
	{
		// remember current culling setting
		GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
		GLint cull_face;
		glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

		// ensure that opengl culling is identical to shader program based culling
		if (cull_mode > 0) {
			glEnable(GL_CULL_FACE);
			glCullFace(cull_mode == CM_BACKFACE ? GL_BACK : GL_FRONT);
		}
		else
			glDisable(GL_CULL_FACE);

		// choose a shader program and configure it based on current settings
		shader_program& prog = ctx.ref_surface_shader_program(true);
		prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
		prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
		prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
		// set default surface color for color mapping which only affects 
		// rendering if mesh does not have per vertex colors and color_mapping is on
		prog.set_attribute(ctx, prog.get_color_index(), surface_color);

		// render the mesh from the vertex buffers with selected program
		dmat4 R;
		mesh_orientation.put_homogeneous_matrix(R);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(
			cgv::math::translate4<double>(mesh_location) *
			cgv::math::scale4<double>(mesh_scale, mesh_scale, mesh_scale) *
			R);
		//MI.render_mesh(ctx, ctx.ref_surface_shader_program(true));
		MI.render_mesh(ctx, prog, opaque_part, !opaque_part);
		ctx.pop_modelview_matrix();

		// recover opengl culling mode
		if (is_culling)
			glEnable(GL_CULL_FACE);
		else
			glDisable(GL_CULL_FACE);
		glCullFace(cull_face);
	}
	void draw(cgv::render::context& ctx)
	{
		// draw skybox 
		switch (which_skybox) {
			case 0:
				glDepthMask(GL_FALSE);
					glDisable(GL_CULL_FACE);
						test_tex.enable(ctx, 1);
						skyprog.enable(ctx);
						skyprog.set_uniform(ctx, "img_tex", 1);
						ctx.tesselate_unit_cube();
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
							ctx.tesselate_unit_cube();
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
							ctx.tesselate_unit_cube();
							skyprog.disable(ctx);
						tmp_tex.disable(ctx);
					glEnable(GL_CULL_FACE);
				glDepthMask(GL_TRUE);
				break;

				//test_tex
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

		// render editable skel. dynamically 
		if (ds->get_skeleton() != nullptr)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glEnable(GL_CULL_FACE);
			skel_view->draw_skeleton_subtree(ds->get_skeleton()->get_root(),
				ds->get_skeleton()->get_origin(), ctx, 0, true, true);
			glDisable(GL_CULL_FACE);
			glDisable(GL_BLEND);
		}
		// imitating only when dofs are changed 
			if (b_toggle_imitating && skel_view->should_apply_dofs_to_others_for_imitating) {
				apply_dofs();
				skel_view->should_apply_dofs_to_others_for_imitating = false;
			}
		// draw the other two guys
		if(b_toggle_show_imitating_skel){
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

		// draw line for vr controllers 
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					P.push_back(ray_origin + ray_length * ray_direction);
					rgb c(float(1 - ci), 0.5f*(int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
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

		// draw static boxes
			cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
			renderer.set_render_style(style);
			renderer.set_box_array(ctx, boxes);
			renderer.set_color_array(ctx, box_colors);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
			}
			renderer.disable(ctx);

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

		// draw dynamic boxes 
			renderer.set_render_style(movable_style);
			renderer.set_box_array(ctx, movable_boxes);
			renderer.set_color_array(ctx, movable_box_colors);
			renderer.set_translation_array(ctx, movable_box_translations);
			renderer.set_rotation_array(ctx, movable_box_rotations);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)movable_boxes.size());
			}
			renderer.disable(ctx);

		// draw intersection points
			/*if (!intersection_points.empty()) {
				auto& sr = cgv::render::ref_sphere_renderer(ctx);
				sr.set_position_array(ctx, intersection_points);
				sr.set_color_array(ctx, intersection_colors);
				sr.set_render_style(srs);
				if (sr.validate_and_enable(ctx)) {
					glDrawArrays(GL_POINTS, 0, (GLsizei)intersection_points.size());
					sr.disable(ctx);
				}
			}*/

		// draw info label
		if (label_tex.is_created()) {
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
		if (ulabel_tex.is_created()) {
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
		double s = 0.1;

		// draw quad with ref. 
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

		// draw boxgui, pg1->ele 
		renderer.set_render_style(movable_style);
		renderer.set_box_array(ctx, pg1->boxvector);
		renderer.set_color_array(ctx, pg1->colorvector);
		if (renderer.validate_and_enable(ctx)) {
			glDrawArrays(GL_POINTS, 0, (GLsizei)pg1->boxvector.size());
		}
		renderer.disable(ctx);

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

		if (!skel_view->playing) {
			// render lines
			std::vector<vec3> vertex_array_in_point_list;
			std::vector<rgb> colorarray;
			if (end_point_list.size() > 0)
				for (int i = 0; i < start_point_list.size() - 1; i++) {
					vertex_array_in_point_list.push_back(start_point_list.at(i));
					vertex_array_in_point_list.push_back(end_point_list.at(i));
					colorarray.push_back(rgb(0, 0, 0));
					colorarray.push_back(rgb(0, 0, 0));
				}
			// spec. care needed
			if (start_point_list.size() > 0)
				if (is_even_point && drawingbone) {  
					vertex_array_in_point_list.push_back(start_point_list.at(start_point_list.size() - 1));
					vertex_array_in_point_list.push_back(
						cur_left_hand_posi + vec3( cur_left_hand_dir * 0.2f)); // shall be modified. todo
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
		//render demo box 
			vec3 boxcenter = cur_left_hand_posi + vec3(cur_left_hand_dir * 0.2f);
			box3 demobox = box3(vec3(boxcenter - tmpboxsize / 2.0f),
				vec3(boxcenter + tmpboxsize / 2.0f));
			vector<box3> boxarray; vector<rgb> boxcolorarray;
			boxarray.push_back(demobox);
			boxcolorarray.push_back(rgb(1,1,0));

			renderer.set_render_style(movable_style);
			renderer.set_box_array(ctx, boxarray);
			renderer.set_color_array(ctx, boxcolorarray);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)boxarray.size());
			}
			renderer.disable(ctx);
	}
};

#include <random>

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_test::construct_table(float tw, float td, float th, float tW)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	// construct table
	rgb table_clr = rgb(// 26, 82, 45
		26/255.0f,
		82/255.0f,
		45/255.0f
	);
	float zdiff = -2.8f;
	float xdiff = -0.5f;
	boxes.push_back(box3(
		vec3(-0.5f*tw - 2*tW + xdiff, th, -0.5f*td - 2*tW + zdiff),
		vec3( 0.5f*tw + 2*tW + xdiff, th + tW, 0.5f*td + 2*tW + zdiff)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw + xdiff, 0, -0.5f*td + zdiff), vec3(-0.5f*tw - tW + xdiff, th, -0.5f*td - tW + zdiff)));
	boxes.push_back(box3(vec3(-0.5f*tw + xdiff, 0, 0.5f*td + zdiff), vec3(-0.5f*tw - tW + xdiff, th, 0.5f*td + tW + zdiff)));
	boxes.push_back(box3(vec3(0.5f*tw + xdiff, 0, -0.5f*td + zdiff), vec3(0.5f*tw + tW + xdiff, th, -0.5f*td - tW + zdiff)));
	boxes.push_back(box3(vec3(0.5f*tw + xdiff, 0, 0.5f*td + zdiff), vec3(0.5f*tw + tW + xdiff, th, 0.5f*td + tW + zdiff)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}
/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_test::construct_room(float w, float d, float h, float W, bool walls, bool ceiling)
{
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if (walls) {
		// construct walls
		/*boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));*/

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if (ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_test::construct_environment(float s, float ew, float ed, float eh, float w, float d, float h)
{
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	for (unsigned i = 0; i < n; ++i) {
		float x = i * s - 0.5f*ew;
		for (unsigned j = 0; j < m; ++j) {
			float z = j * s - 0.5f*ed;
			/*if ( (x + 0.5f*s > -0.5f*w && x < 0.5f*w + 1) && (z + 0.5f*s > -0.5f*d && z < 0.5f*d) )
				continue;*/
			if ((x + 0.5f * s > -0.5f * w && x < 0.5f * w) && (z + 0.5f * s > -0.5f * d && z < 0.5f * d))
				continue;
			float h = 0.2f*(std::max(abs(x)-0.5f*w,0.0f)+std::max(abs(z)-0.5f*d,0.0f))*distribution(generator)+0.1f;
			boxes.push_back(box3(vec3(x, 0, z), vec3(x+s, h, z+s)));
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
void vr_test::construct_movable_boxes()
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

	// box2
	//movable_boxes.push_back(box3(-0.5f * 2 * extent, 0.5f * 2 * extent));
	//movable_box_colors.push_back(rgb(distribution(generator),
	//	distribution(generator),
	//	distribution(generator)));
	//movable_box_translations.push_back(cur_left_hand_posi + posi2);
	///*quat rot;
	//rot.normalize();*/
	//movable_box_rotations.push_back(rot);

	/*for (size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.1f;
		extent *= std::min(tw, td)*0.2f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), 
				distribution(generator), 
				distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(
			signed_distribution(generator), 
			signed_distribution(generator), 
			signed_distribution(generator), 
			signed_distribution(generator)
		);
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}*/
}
void vr_test::construct_boxgui() {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	rgb writecol = rgb(1, 1, 1);
	rgb bkgcol = rgb(0.7, 0.6, 0.7);
	rgb col1 = rgb(100 / 255.0f, 46 / 255.0f, 128 / 255.0f);
	float font_size = 100; // pp- yzy font will become small if all are 150. 2h
	float smallbox_font_size = 50;
	float smaller_f = 25;

	//rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f );
	// title
	boxgui_button first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.8f, -3.0f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "<", 120, "xxx", true);
	pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.8f, -2.5f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), ">", 120, "xxx", true);
	pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.8f, 0), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Settings", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	// list
	
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Change Skybox", font_size, "xxx", true);
	pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"xxx", 0, skyboxdir + "/igen_2/xn.jpg", false);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.5f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"xxx", 0, skyboxdir + "cm_xn.jpg", false);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.5, -1.25f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"xxx", 0, skyboxdir + "BluePinkNebular_yn.jpg", false);
		pg1->elements.push_back(first_btn);
	
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save/Load Skel.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"demoskel", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_skel1", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_skel2", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_skel3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 2, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save Skel.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
	
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_skel1", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_skel2", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.25 - 0.5f, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_skel3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save/Load Animation", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
	
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"start_rec", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"pause_rec", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"stop_rec", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_anim1", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_anim2", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 5), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_anim3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.5f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Save Animation", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_anim1", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_anim2", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 2.0f - 0.5, -1.75f + 0.25f * 8), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_anim3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f , -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Bone Edit", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
		//-----------------------------add to existing skel.-------------------//
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"add_bone_\nexist", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn); 
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"add_\nroot", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn); 
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
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
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"f_del_\nall", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.25f + 0.5f - 0.5, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"f_back", smallbox_font_size, "D:/icon_res/default.png", true);
	/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Mode for Left Control.", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);*/
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"adjest_\ncoordi.", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"reset_\nall_\nadjestment", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f+ 0.5f - 0.5, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"scale_\njointbox", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		/*
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f+ 0.5f, -1.75f + 0.25f * 6), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"walk around", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f+ 0.5f, -1.75f + 0.25f * 7), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"walk around", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f + 0.5f - 0.5, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"build_skel", smallbox_font_size, "D:/icon_res/default.png", true); 
			// maybe do not have to click it explicitly 
		pg1->elements.push_back(first_btn);
		/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 1.0f+ 0.5f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_anim3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
	first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Mesh Style", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_demo1", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_demo2", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_demo3", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 3), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_demo4", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(2.5f - 0.05f, .75f + 0.5f + 1, -1.75f + 0.25f * 4), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_demo5", smallbox_font_size, "D:/icon_res/default.png", true);
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
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f+ 1, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
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
		first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f+ 1, -1.75f + 0.25f * 1), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"rotation.", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
			//center_gan = vec3(2.45f - 0.3 - 0.6, 2.0f, -1.75f + 0.25f * 3);
			//extend_gan = vec3(0.6, 0.01, 0.01);
			//gan = box3(vec3(center_gan - extend_gan), vec3(center_gan + extend_gan));
			//boxes.push_back(gan);
			//box_colors.push_back(rgb(1, 0, 0));

			//center_slider = vec3(2.45f - 0.3, 2.0f, -1.75f + 0.25f * 3); // adjestable x 
			//extend_slider = vec3(0.05);
			//slider = box3(vec3(center_slider - extend_slider), vec3(center_slider + extend_slider));
			//boxes.push_back(slider);
			//box_colors.push_back(rgb(1, 0, 0));
		rgb cur_color = rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f);
		first_btn = boxgui_button(vec3(2.45f, 2.0f, -1.75f + 0.25f * 2), 0.1, 0.2, 0.2, cur_color,
			"scale", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);
		/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.5f + 0.06, -1.75f + 0.25f * 5 - 0.06), 0.1, 0.08, 0.08, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"+", 200, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
			//center_gan = vec3(2.45f - 0.3 - 0.6, 2.0f, -0.75f);
			//extend_gan = vec3(0.6,0.01, 0.01);
			//gan = box3(vec3(center_gan - extend_gan),vec3(center_gan + extend_gan));
			//boxes.push_back(gan);
			//box_colors.push_back(cur_color);
		
			//center_slider = vec3(2.45f - 0.3-0.6, 2.0f, -0.75f); // adjestable x 
			//extend_slider = vec3(0.05);
			//slider = box3(vec3(center_slider - extend_slider), vec3(center_slider + extend_slider));
			//boxes.push_back(slider);
			//box_colors.push_back(cur_color);
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

	first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -2.5f), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Skinning", font_size, "D:/icon_res/default.png", true);
	pg1->elements.push_back(first_btn);
		/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"adjest bone posi.", smaller_f, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
		/*first_btn = boxgui_button(vec3(2.5f - 0.05f, 0.5f + 0.25f, -1.75f + 0.25f * 0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"skinning", smallbox_font_size, "D:/icon_res/default.png", true);
		pg1->elements.push_back(first_btn);*/
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
	//---------------------------------------------------------------second part of gui------------------------//
	first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Load Scene", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	first_btn.do_transform = true;
	first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	first_btn.set_trans(vec3(2.25f - 0.5f, 2.5, 0.5f));
	pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"l_scene1", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 0, 2.5, 0.5f));
		pg1->elements.push_back(first_btn);

		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"toggle_addi\n_skel", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 1, 2.5, 0.5f));
		pg1->elements.push_back(first_btn);

		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"toggle_\nimitating", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 2, 2.5, 0.5f));
		pg1->elements.push_back(first_btn);
	first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "IK", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	first_btn.do_transform = true;
	first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	first_btn.set_trans(vec3(2.25f - 0.5f, 2.5 - 0.25 * 1, 0.5f));
	pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"ccd", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 0, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"opti. ccd", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 1, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"ik1", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 2, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		//
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_base", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 3, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		//
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ),
			"s_ee_left", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 4, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		//
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"s_ee_right", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 5, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
		//
		first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.2, rgb(0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f),
			"s_ee_head", smallbox_font_size, "D:/icon_res/default.png", true);
		first_btn.do_transform = true;
		first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
		first_btn.set_trans(vec3(2.25f - 0.5f - 0.25f - 0.5f - 0.25f * 6, 2.5 - 0.25 * 1, 0.5f));
		pg1->elements.push_back(first_btn);
	/*first_btn = boxgui_button(vec3(0), 0.1, 0.2, 0.8, rgb( 0.4f * distribution(generator) + 0.1f, 0.4f * distribution(generator) + 0.3f, 0.4f * distribution(generator) + 0.1f ), "Skel. Retargeting", font_size, "D:/icon_res/icon_chg_skybox.png", true);
	first_btn.do_transform = true;
	first_btn.set_rot(cgv::math::rotate3<double>(-90.0f, vec3(0, 1, 0)));
	first_btn.set_trans(vec3(2.25f - 0.5f, 2.5 - 0.25 * 2, 0.5f));
	pg1->elements.push_back(first_btn);*/
	//
	pg1->push_to_render_vector();
}


#include <cgv/base/register.h>

cgv::base::object_registration<vr_test> vr_test_reg("");

///@}