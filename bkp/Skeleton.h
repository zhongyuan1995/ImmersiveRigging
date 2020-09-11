// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "Bone.h"
#include "common.h"
#include "IHasBoundingBox.h"

#include <vector>


class SkinningSkeleton : public IHasBoundingBox
{
public:
	SkinningSkeleton();
	~SkinningSkeleton();

	//Loads skeleton data from an ASF file.
	bool fromASFFile(const std::string& filename);
	//writeASFFile
	void writeASFFile(const std::string& filename);
	void writeASFstring_recur(Bone* node, int number_of_level, std::ofstream& fout);
	void printChildRelations(std::ofstream& fout);

	//Writes the current skeleton to a Pinocchio file.
	void write_pinocchio_file(const std::string& filename);

	//Loads a Pinocchio skeleton file and adapts the current skeleton according to this file.
	void read_pinocchio_file(std::string filename);

	//Fille the matrices vector with the skinning matrices for all bones in DFS order
	void get_skinning_matrices(std::vector<Mat4>& matrices);

	//Returns the skeleton's root bone
	Bone* get_root() const;
	void set_root(Bone* r) {
		root = r;
	}

	const Mat4& get_origin() const;
	Vec3 get_origin_vec() const;
	void set_origin(const Mat4&);
	void set_origin_translation(const Vec3&);
	void set_origin_rotation(const Vec3 axis, float angle_degrees);
	// yzy, this may never take effect, we always load skel factor first
	void set_scalefactor(const float factor) { scalefactor = factor; }; 
	float get_scalefactor() { return scalefactor; }
	Bone* find_bone(const std::string& name) const;
	Bone* find_bone_in_a_list_by_id(int idx);
	void clear_bone_list() { bone_list.clear(); }
	void push_back_to_bone_list(Bone* n) { bone_list.push_back(n); }
	void add_new_bone_to_map(std::string name, Bone* b) { bones[name] = b; }
	int get_size_bone_list() { return bones.size(); }
	std::vector<Bone*> get_bone_list() { return bone_list; }
	void apply_dofs_given_skel_pointer(SkinningSkeleton* master) { // iter. all bones O(bonenumber), apply each timmer event?
		// apply dofs the same as master skel. 
		for (int i = 0; i < bone_list.size(); i++) {
			Bone* cur_bone = bone_list.at(i);
			Bone* masterbone = master->get_bone_list().at(i);
			for (int i = 0; i < cur_bone->dof_count(); ++i)
			{
				cur_bone->get_dof(i)->set_value(masterbone->get_dof(i)->get_value());
			}
		}
	}
	void postprocess(Bone* node, const Vec3& global_position);

private:
	Mat4 origin;
	std::string version;
	std::string name;
	Bone* root;
	// this depends on spec. skel size i found in the dataset. we so not have to scale the ones we gen. only for read skel. 
	float scalefactor = 0.06; 

	void recursive_write_pinocchio_file(Bone* node, int& next_node_id, int parent_node_id, std::ofstream& o, const Vec3& global_position = Vec3(0, 0, 0));
	void recursive_read_pinocchio_file(Bone* node, std::ifstream& i, const Vec3 global_position = Vec3(0, 0, 0));
	void recursive_get_skinning_matrices(Bone* node, std::vector<Mat4>& matrices, Mat4 current_matrix);

	std::map<const std::string, Bone*> bones;
	std::vector<Bone*> bone_list;
};
