// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include <cgv/base/base.h>
#include "Mesh.h"

#include <vector>
#include <fstream>
#include <sstream>
#include <queue>



bool SkinningMesh::init_shaders(cgv::render::context& ctx)
{
	cgv::render::shader_code vs, fs, gs;
	if (!vs.read_and_compile(ctx, "skinning.glvs", cgv::render::ST_VERTEX)) {
		std::cout << "error reading vertex shader\n" << vs.last_error.c_str() << std::endl;
		return false;
	}
	if (!gs.read_and_compile(ctx, "skinning.glgs", cgv::render::ST_GEOMETRY)) {
		std::cout << "error reading geometry shader\n" << gs.last_error.c_str() << std::endl;
		return false;
	}
	if (!fs.read_and_compile(ctx, "skinning.glfs", cgv::render::ST_FRAGMENT)) {
		std::cout << "error reading fragment shader\n" << vs.last_error.c_str() << std::endl;
		return false;
	}
	if (!prog.create(ctx)) {
		std::cout << "error creating program\n" << prog.last_error.c_str() << std::endl;
		return false;
	}
	prog.attach_code(ctx, vs);
	prog.attach_code(ctx, gs);
	prog.attach_code(ctx, fs);
	if (!prog.link(ctx)) {
		std::cout << "link error\n" << prog.last_error.c_str() << std::endl;
		return false;
	}

	//prog.build_program(ctx, "skinning.glpr");
	return true;
}

SkinningMesh::SkinningMesh()
	: has_attachment(false)
{
	glGenBuffers(1, &indexBuffer);
	glGenBuffers(1, &positionBuffer);
	glGenBuffers(1, &boneIndexBuffer);
	glGenBuffers(1, &boneWeightBuffer);
	glGenVertexArrays(1, &vao);

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, positionBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);	

	glBindBuffer(GL_ARRAY_BUFFER, boneWeightBuffer);
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ARRAY_BUFFER, boneIndexBuffer);
	glVertexAttribPointer(2, 4, GL_INT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);

	glEnableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glBindVertexArray(0);

	skinning_matrices = new GLfloat[50 * 16];

	rot.identity();
	trans = cgv::math::fvec<float, 3>(0);
}

SkinningMesh::~SkinningMesh()
{
	glDeleteBuffers(1, &indexBuffer);
	glDeleteBuffers(1, &positionBuffer);
	glDeleteBuffers(1, &boneIndexBuffer);
	glDeleteBuffers(1, &boneWeightBuffer);
	glDeleteVertexArrays(1, &vao);

	delete[] skinning_matrices;
}

bool SkinningMesh::read_obj(const char* filename)
{
	std::ifstream f(filename);

	std::string line;

	cgv::math::fvec<float, 3> p;
	unsigned int i, j, k;

	positions.clear();
	indices.clear();
	reset_bounding_box();

	int n_indices = 0;
	while (std::getline(f, line))
	{
		if (line.length() == 0 || line[0] == '#')
			continue;
		else
		{
			std::stringstream ss(line);
			ss.ignore(1);
			if (line[0] == 'v')
			{
				ss >> p.x() >> p.y() >> p.z();
				p = rot * mesh_scale * p + trans;
				positions.push_back(p);
				add_point(p);
			}
			else if (line[0] == 'f')
			{
				ss >> i;
				ss.ignore(10, ' ');
				ss >> j;
				ss.ignore(10, ' ');
				ss >> k;
				indices.push_back(i - 1);
				indices.push_back(j - 1);
				indices.push_back(k - 1);
				n_indices += 3;
			}
		}
	}

	f.close();
	
	glBindVertexArray(vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, positionBuffer);
	glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(cgv::math::fvec<float, 3>), &positions[0], GL_STATIC_DRAW);	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	return true;
}

struct WeightComparer
{
	bool operator()(const std::pair<int, float>& p1, const std::pair<int, float>& p2)
	{
		return p1.second < p2.second;
	}
};

void SkinningMesh::read_attachment(std::string filename)
{
	std::ifstream f(filename);

	std::string line;

	typedef cgv::math::fvec<int, 4> ivec4;

	std::vector<ivec4> bone_indices;
	std::vector<Vec4> bone_weights;

	while (std::getline(f, line))
	{
		std::stringstream ss(line);
		float w;
		std::priority_queue<std::pair<int, float>, std::vector<std::pair<int, float>>, WeightComparer> vertex_weights;
		int i = 0;
		while (ss >> w)
		{
			vertex_weights.push(std::make_pair(i++, w));
		}
		ivec4 indices;
		Vec4 weights;
		float total_weight = 0;
		for (int i = 0; i < 4; ++i)
		{
			auto p = vertex_weights.top(); vertex_weights.pop();
			indices[i] = p.first;
			weights[i] = p.second;
			total_weight += p.second;
		}
		weights *= 1.0f / total_weight;
		bone_indices.push_back(indices);
		bone_weights.push_back(weights);
	}

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, boneIndexBuffer);
	glBufferData(GL_ARRAY_BUFFER, bone_indices.size() * sizeof(ivec4), &bone_indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ARRAY_BUFFER, boneWeightBuffer);
	glBufferData(GL_ARRAY_BUFFER, bone_weights.size() * sizeof(Vec4), &bone_weights[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindVertexArray(0);
	f.close();

	has_attachment = true;
}

void SkinningMesh::set_skinning_matrices(const std::vector<Mat4>& matrices)
{
	for (size_t i = 0; i < matrices.size(); ++i)
	{
		for (int j = 0; j < 16; ++j)
			skinning_matrices[16 * i + j] = matrices[i][j];
	}
	n_bones = matrices.size();
}

void SkinningMesh::draw(cgv::render::context& ctx) // not called automatically, i invoked this in main draw call 
{
	if (b_face_culling)
		glEnable(GL_CULL_FACE);
	else
		glDisable(GL_CULL_FACE);

	if (b_set_transparent) {
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	}
	else {
		glDisable(GL_BLEND);
	}

	Mat4 modelview = ctx.get_modelview_matrix();
	Mat4 projm = ctx.get_projection_matrix();
	Mat4 mvp = projm * modelview;
	prog.set_uniform(ctx, "modelviewproj", mvp); // must have
	prog.set_uniform(ctx, "skinned", has_attachment);
	prog.enable(ctx);

	glBindVertexArray(vao);

	if (has_attachment)
	{
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);	

		GLint program;
		glGetIntegerv(GL_CURRENT_PROGRAM, &program);
		GLint matrices_location = glGetUniformLocation(program, "bone_matrices");
		glUniformMatrix4fv(matrices_location, n_bones, GL_FALSE, skinning_matrices);
	}
	else
	{
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);
	}	

	if(b_set_polygonmode)
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBuffer); // imp.! 
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	//glDrawArrays(GL_TRIANGLES, 0, indices.size());
	if (b_set_polygonmode)
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	glBindVertexArray(0);

	prog.disable(ctx);

	//glDisable(GL_CULL_FACE);
	//glDisable(GL_BLEND);

}
