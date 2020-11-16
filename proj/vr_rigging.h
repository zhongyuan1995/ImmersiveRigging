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
#include <cgv/base/register.h>

#include <fstream>

#include "datastore.h"
#include "skeleton_viewer.h"
#include "mesh.h"
#include "ik_viewer.h"
#include "skinned_mesh_viewer.h"
#include "boxgui.h"

#include "../proj_pinocchio/skeleton.h"
#include "../proj_pinocchio/attachment.h"
#include "../proj_pinocchio/pinocchioApi.h"

class ArgData
{
public:
	ArgData() :
		stopAtMesh(false), stopAfterCircles(false), skelScale(1.), noFit(false),
		skeleton(HumanSkeleton()){}
	bool stopAtMesh;
	bool stopAfterCircles;
	bool noFit;
	double skelScale;
	string filename;
	string motionname;
	Quaternion<> meshTransform;
	Skeleton skeleton;
	string skeletonname;
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

class vr_rigging :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
private:
	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};

	// pointers to data storage structure and viewers 
	DataStore* ds;
	DataStore* tmpdata_1;
	DataStore* tmpdata_2;
	SkeletonViewer* skel_view;
	SkeletonViewer* tmpskel_view_1;
	SkeletonViewer* tmpskel_view_2;
	IKViewer* ik_view;

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

	// camera related varibles 
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
	mat4 camera_to_head_matrix[2];
	mat4 camera_projection_matrix[4];
	vec2 focal_lengths[4];
	vec2 camera_centers[4];
	texture camera_tex;
	shader_program seethrough;
	GLuint camera_tex_id;
	vec2 extent_texcrd;
	vec2 center_left;
	vec2 center_right;

	// skybox related varibles 
	cgv::render::shader_program skyprog;	
	cgv::render::texture img_tex;
	cgv::render::texture tmp_tex;
	cgv::render::texture test_tex;
	std::string projdir;
	int which_skybox = 1; 

	// global varibles for rendering the yellow box on left hand
	float tmpboxsize = 0.03f;
	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	vec3 hmd_origin;
	float enlargestep = 0.0025f;

	// varibles for rendering mesh 
	std::shared_ptr<SkinningMesh> mmesh;
	string g_mesh_filename = "";

	// boolean varibles to control left hand behaviors
	bool b_render_mesh = true;
	bool b_toggle_show_imitating_skel = true;
	bool b_toggle_imitating = true;
	bool keydown = false;
	bool del_keydown = false;
	bool b_fast_add_root = false;
	bool b_fast_add_skel = false;
	bool b_fast_reset = false;
	bool do_ik_ccd = false; // toggle ccd button to start/stop 
	bool toggle_ccd = false; // for ccd we can not select end effector 
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
	bool from_jump_asf = true;
	bool toggle_boxgui = true;
	bool show_mesh_related = false;
	bool show_skel_related = false;

	// rendering text labels for information board 
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
	std::vector<const char*> font_names; // general font information
	std::string font_enum_decl;
	cgv::media::font::font_face_ptr label_font_face; // current font face used
	cgv::media::font::FontFaceAttributes label_face_type;

	// rendering text labels for usage description 
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
	std::vector<const char*> ufont_names; // general font information
	std::string ufont_enum_decl;
	cgv::media::font::font_face_ptr ulabel_font_face; // current font face used
	cgv::media::font::FontFaceAttributes ulabel_face_type;
	bool toggle_usage_description;

	// for boxgui
	boxgui_page* pg1 = new boxgui_page();
	std::vector<vec3> gui_intersection_points;// intersection points gui
	std::vector<rgb>  gui_intersection_colors;
	std::vector<int>  gui_intersection_box_indices;
	std::vector<int>  gui_intersection_controller_indices;
	std::vector<vec3> skel_intersection_points;// intersections with skel joint box 
	std::vector<int>  skel_intersection_box_indices;
	std::vector<vec3> fast_intersection_points;// intersections with skel joint box in fast mode
	std::vector<int>  fast_intersection_box_indices;
	bool confirm_needed = false;
	bool confirmed = false;
	vec3 confirm_gui_posi = 0;

	// arrays for rendering skeletons 
	std::vector<box3> jointlist;
	std::vector<rgb> jointlist_colors;
	std::vector<box3> fast_jointlist;
	std::vector<rgb> fast_jointlist_colors;
	std::vector<vec3> start_point_list;
	std::vector<vec3> end_point_list;
	std::vector<float> end_point_size_list; // no size for root node 
	bool is_even_point = false; // start from -1
	bool fast_is_reset = true;
	bool mesh_scale_mode = false;

	// when adding new bones 
	std::vector<vec3> fast_bone_posi_vec_as_chain;
	int bone_tobeaddednext_idx = -1;
	int bone_tobeadjested_idx = -1;
	int newbone_idx = 0;
	std::string lefthandmode;

	// varibles for defining local dofs 
	float cur_local_frame_rot_rel_XYZ[3] = { 0, 0, 0 };
	int shuffle_local_frame_dir_num = 0;
	int shuffle_dof_def_xyz = 0;
	int num_of_all_choices = 6;
	float def_dof_x_min = -180;
	float def_dof_x_max = 180;
	float def_dof_y_min = -180;
	float def_dof_y_max = 180;
	float def_dof_z_min = -180;
	float def_dof_z_max = 180;
	vec3 roll_yaw_pitch_vec;
	mat3 cur_rot_mat;
	mat3 temp_rot;
	mat3 cur_min_rot_mat;
	mat3 cur_max_rot_mat;
	vec3 left_hand_target_posi;
	vec3 right_hand_target_posi;
	vec3 head_target_posi;

	// varibles for IK algorithms 
	Bone* left_ee;
	Bone* right_ee;
	Bone* hmd_ee;
	Bone* base_bone;

	// control the mirror effect 
	float mirror_plane_z = -2;
	float mirror_plane_x = 1.2;

	// one has to adjest working directory on his computer if not place the whole project in "plugins" dir 
	std::string working_dir = "../../../plugins/vr_rigging/gen_dataset/";
	std::string mesh_dir = "";

public:
	///
	void init_cameras(vr::vr_kit* kit_ptr);
	///
	void start_camera();
	///
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
	mat3 from_global_roll_yaw_pitch_vec_to_matrix();
	///
	mat3 compute_matrix_from_two_dirs(vec3 a, vec3 b);
	///
	void from_matrix_to_euler_angle_as_global_var(mat3 rot_mat);
	///
	void construct_left_hand_box(); 
	///
	void gui_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	///
	void skel_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	///
	void fast_joint_box_compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	///
	void construct_boxgui();
	///
	void render_confirm_panel();
	///
	vec3 compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction);

	/// gui related functions 
	void remove_pg1();
	void take_screen_capture();

	/// mesh related operations
	///
	/// load mesh with cgv::gui::file_open_dialog
	void load_mesh();
	/// with working_dir varible 
	void load_mesh_with_gui();
	///
	void adjest_mesh();
	/// adjesting the hight of the mesh: the same as the headset 
	void translate_model_in_y_dir_upwards();
	/// load additional two guys imitating our main skeleton as mirror effect 
	void load_addi_two_guys(string f);
	///
	void toggle_mesh_render();
	///
	void toggle_mesh_transparent();
	///
	void toggle_mesh_wireframe();
	///
	void toggle_face_culling();

	/// skeleton related operations
	void load_skel_with_dofs();
	///
	void load_demo_skel1();
	///
	void load_demo_skel2();
	///
	void skel_auto_complete();
	///
	void gen_asf_skel_file();
	///
	void toggle_imitating();
	///
	void del_skel();
	///
	void shuffle_frame();
	///
	void load_demo_skel3();
	///
	void toggle_other_twoskel();
	///
	void build_skel();
	///
	void apply_rigged_skel();
	///
	void apply_dofs();
	///
	void save_curskel_to_file();

	/// rigging function 
	void start_autorigging_pinoccio();

	/// animation related functions 
	void load_demo_animation();
	///
	void load_stored_anim1();
	///
	void load_stored_anim2();
	///
	void load_stored_anim3();
	///
	void stop_anim();
	///
	void start_record();
	///
	void stop_record_and_save();

public:
	vr_rigging();
	///
	std::string get_type_name() { return "vr_rigging"; }
	///
	void stream_help(std::ostream& os);
	///
	void on_set(void* member_ptr);
	///
	bool handle(cgv::gui::event& e);
	///
	bool init(cgv::render::context& ctx);
	///
	void clear(cgv::render::context& ctx);
	///
	void init_frame(cgv::render::context& ctx);
	///
	void draw(cgv::render::context& ctx);
	///
	void finish_draw(cgv::render::context& ctx);
	///
	void create_gui();
};

///@}
