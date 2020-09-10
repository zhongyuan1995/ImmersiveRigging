#pragma once

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
#include <libs\cgv_gl\gl\gl_tools.h>

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers


class boxgui_label_texture : public cgv::render::render_types {
public:
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;
	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	boxgui_label_texture(std::string label, float font_size) {
		label_text = label;

		//
		label_font_idx = 0;
		label_upright = false;
		label_size = font_size; // 150
		label_color = rgb(1, 1, 1);
		label_outofdate = true; // whether label texture is out of date
		label_resolution = 256; // resolution of label texture
		// 
		// this two will be initialized during init_frame() func. 
		//label_tex; // texture used for offline rendering of label
		//label_fbo;
	}
};

class boxgui_primitive : public cgv::render::render_types {
	//public:
	//		box3 button;
};

class boxgui_button : public boxgui_primitive, public cgv::render::render_types {
public:
	box3 button;
	// an additional quad will be used as label, attached to this box, which is implicit 
	vec3 center_of_quad;
	vec2 ext_of_quad;
	rgb color;
	std::string label;
	bool has_intersec;
	unsigned int icon_texture;
	bool flag_use_label;
	boxgui_label_texture* gui_label_texture;
	float radio = 19.0f / 4.0f;
	mat3 rot; 
	vec3 trans = vec3(0);
	bool do_transform = false;
	boxgui_button(vec3 posi, float ext_x, float ext_y, float ext_z, rgb col, std::string l, int l_size, std::string iconpath, bool use_label) {
		button = box3(posi - vec3(ext_x / 2.0f, ext_y / 2.0f, ext_z / 2.0f),
			posi + vec3(ext_x / 2.0f, ext_y / 2.0f, ext_z / 2.0f));
		color = col;
		label = l;
		has_intersec = false;
		center_of_quad = posi + vec3(-ext_x / 2.0f - 0.01f, 0, 0);
		ext_of_quad = vec2(ext_y / 2.0f - 0.02f, ext_z / 2.0f - 0.02f);
		double aspect_ptr;
		bool has_alpha_ptr;
		double aspect;
		icon_texture = cgv::render::gl::read_image_to_texture(iconpath, false, &aspect, &has_alpha_ptr);
		flag_use_label = use_label;
		if (use_label) {
			gui_label_texture = new boxgui_label_texture(l, l_size); // create a label texture with given string and default para. 
		}
		radio = (ext_z - 0.04) / (ext_y - 0.04);
		rot.identity();
	}
	box3* get_pointer_to_box() {
		return &button;
	}
	void set_rot(mat3 r) {
		rot = r;
	}
	void set_trans(vec3 t) {
		trans = t;
	}
};


// classes, boxgui 
class boxgui_page : public cgv::render::render_types {
	// can also be modi. in vr scene
public:
	cgv::render::shader_program icon_shader_prog;
	std::vector<box3> boxvector;
	std::vector<rgb> colorvector;

	std::vector<boxgui_button> elements; // only buttons cur. 
	void push_to_render_vector() {
		boxvector.clear();
		colorvector.clear();
		float anim_range = 0.1;

		for (int i = 0; i < elements.size(); i++) {
			if (elements.at(i).do_transform) {
				if (elements.at(i).has_intersec) {//
					box3 curbox = elements.at(i).button;
					vec3 lowerpoint = curbox.get_center() - curbox.get_extent() / 2.0f;
					vec3 upperpoint = curbox.get_center() + curbox.get_extent() / 2.0f;
					box3 newbox = box3(
						elements.at(i).rot * lowerpoint + elements.at(i).trans + vec3(0, 0, anim_range),
						elements.at(i).rot * upperpoint + elements.at(i).trans + vec3(0, 0, anim_range)
					);
					boxvector.push_back(newbox);
					elements.at(i).center_of_quad =
						elements.at(i).button.get_center() + vec3(0, 0, -0.1f / 2.0f - 0.01f) + vec3(0, 0, anim_range);
				}
				else{
					box3 curbox = elements.at(i).button;
					vec3 lowerpoint = curbox.get_center() - curbox.get_extent() / 2.0f;
					vec3 upperpoint = curbox.get_center() + curbox.get_extent() / 2.0f;
					box3 newbox = box3(
						elements.at(i).rot * lowerpoint + elements.at(i).trans,
						elements.at(i).rot * upperpoint + elements.at(i).trans
					);
					boxvector.push_back(newbox);
					elements.at(i).center_of_quad =
						elements.at(i).button.get_center() + vec3(0, 0, -0.1f / 2.0f - 0.01f);
				}
			}
			else {
				if (elements.at(i).has_intersec) {
					box3 curbox = elements.at(i).button;
					box3 newbox = box3(curbox.get_center() + vec3(anim_range, 0, 0) - curbox.get_extent() / 2.0f,
						curbox.get_center() + vec3(anim_range, 0, 0) + curbox.get_extent() / 2.0f);
					boxvector.push_back(newbox);
					elements.at(i).center_of_quad =
						elements.at(i).button.get_center() + vec3(-0.1f / 2.0f - 0.01f, 0, 0) + vec3(anim_range, 0, 0);
				}
				else {
					boxvector.push_back(elements.at(i).button);
					elements.at(i).center_of_quad =
						elements.at(i).button.get_center() + vec3(-0.1f / 2.0f - 0.01f, 0, 0);
				}
			}
			
			colorvector.push_back(elements.at(i).color);
		}
	}
};