// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "Skeleton.h"

#include <fstream>
#include <map>
#include <string>
#include <sstream>

#include "math_helper.h"
#include <queue>

SkinningSkeleton::SkinningSkeleton()
{
	origin.identity();
}

SkinningSkeleton::~SkinningSkeleton()
{
	if (root)
		delete root;
}

Bone* SkinningSkeleton::get_root() const { return root; }
const Mat4& SkinningSkeleton::get_origin() const { return origin; }
Vec3 SkinningSkeleton::get_origin_vec() const { return Vec3(origin(0, 3), origin(1, 3), origin(2,3)); }
void SkinningSkeleton::set_origin(const Mat4& m) { origin = m; }
// yzy, used to place skel. 
void SkinningSkeleton::set_origin_translation(const Vec3& v) { origin = translate(v) * origin; }
void SkinningSkeleton::set_origin_rotation(const Vec3 axis, float angle_degrees) { origin = rotate(axis, angle_degrees) * origin; }

enum ParseState
{
	Ignore,
	Root,
	BoneData,
	Hierarchy,
}; 
int bone_idx = 1;
int number_of_level = 1;
void SkinningSkeleton::writeASFstring_recur(Bone* node, int number_of_level, std::ofstream& fout) {
	if (!node->get_name()._Equal("root"))
	{
		fout << " " << "begin\n";
		fout << "\t" << "id " + std::to_string(bone_idx++) + "\n";
		fout << "\t" << "name " + node->get_name() + "\n";
		fout << "\t" << "direction  " // rotation by default, axis:(1.5,y,0), 180 degree , not correct 
			+ std::to_string(node->get_direction_in_world_space().x()) + " "
			+ std::to_string(node->get_direction_in_world_space().y()) + " "
			+ std::to_string(node->get_direction_in_world_space().z()) + "\n";
		fout << "\t" << "length " + std::to_string(node->get_length() / scalefactor) + "\n";
		fout << "\t" << "axis 0 0 0  XYZ\n";// fixed now, can be modified later 
		//node->get_dof()
		/*dof rx ry rz
			limits (-60.0 90.0)
			(-90.0 90.0)
			(-90.0 90.0)*/
		fout << "\t" << "dof rx ry rz\n";
		fout << "\t" << "limits (-180.0 180.0)\n";
		fout << "\t" << "(-180.0 180.0)\n";
		fout << "\t" << "(-180.0 180.0)\n";
		fout << " " << "end\n";
	}
	
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		writeASFstring_recur(node->child_at(i), number_of_level+1, fout);
	}
}

// 
void SkinningSkeleton::printChildRelations(std::ofstream& fout) {
	for (auto b : bone_list) {
		int n = b->childCount();
		if(n > 0)
			fout << "\t" << b->get_name() << " ";
		for (int i = 0; i < n; ++i)
		{
			fout << b->child_at(i)->get_name() << " ";
		}
		if (n > 0)
			fout << "\n";
	}
}

//
/*
some problems.... not used 
//for (int i = 1; i < number_of_level; i++) {
	if (curLevel == 1)
		fout << node->get_name() << " ";
	else if (curLevel > 1)
	{
		int n = node->childCount();
		for (int i = 0; i < n; ++i)
		{
			printGivenLevel(node->child_at(i), curLevel--, fout);
		}
	}
*/



void SkinningSkeleton::writeASFFile(const std::string& filename) {
	std::ofstream fout;

	#ifdef _WIN32
		std::wstring wfilename = cgv::utils::str2wstr(filename);
		fout.open(wfilename);
	#else
		fin.open(filename);
	#endif

		fout << 
			"# AST/ASF file generated using vrRigging by yzy\n"
			"# \n"
			":units\n"
			"  mass 1.0\n"
			"  length 0.45\n"
			"  angle deg\n"

			":documentation\n"
			":root\n"
			"	order TX TY TZ RX RY RZ\n"
			"	axis XYZ\n"
			"	position 0 0 0\n"
			"	orientation 0 0 0\n";

			bone_idx = 1;
			number_of_level = 1;

			fout << ":bonedata\n";
			writeASFstring_recur(root, number_of_level, fout);

			fout << ":hierarchy\n";
			fout << " begin\n";
			/*for (int i = 2; i < 30; i++) {
				printGivenLevel(root, i, fout);
				fout << "\n";
			}*/
			printChildRelations(fout);
			fout << " end\n";

		fout.close();
}

bool SkinningSkeleton::fromASFFile(const std::string& filename)
{
	origin.identity();

	std::ifstream fin;
	reset_bounding_box();
	ParseState state = Ignore;

	Bone* current_node = nullptr;

	bones.clear();

	int n_dofs;

	try
	{
#ifdef _WIN32
		std::wstring wfilename = cgv::utils::str2wstr(filename);
		fin.open(wfilename);
#else
		fin.open(filename);
#endif
		if (!fin.good())
			return false;
		while (!fin.eof())
		{
			char buf[CHARS_PER_LINE];
			fin.getline(buf, CHARS_PER_LINE);
			std::string str(buf);
			str = trim(str); //remove whitespaces
			if (str.find('#') == 0)
				continue; //don't handle comments

			if (str.find(':') == 0)
			{
				if (str.find(":version") == 0)
					version = str.substr(9);
				else if (str.find(":name") == 0)
					name = str.substr(6);
				else if (str.find(":root") == 0)
				{
					state = Root;
					current_node = new Bone();
					current_node->set_name("root");
					bones["root"] = current_node;
				}
				else if (str.find(":bonedata") == 0)
					state = BoneData;
				else if (str.find(":hierarchy") == 0)
					state = Hierarchy;
				else state = Ignore;
				continue;
			}

			switch (state)
			{
			case Ignore:
				//ignore this line
				break;
			case Root:
				if (str.find("order") == 0)
				{
					std::string dofs = str.substr(6);
					std::string dofstr;
					std::stringstream ss(dofs);
					n_dofs = 0;
					while (!ss.eof())
					{
						ss >> dofstr;
						AtomicTransform* dof;
						if (dofstr.find("RX") == 0)
							dof = new AtomicXRotationTransform();
						else if (dofstr.find("RY") == 0)
							dof = new AtomicYRotationTransform();
						else if (dofstr.find("RZ") == 0)
							dof = new AtomicZRotationTransform();
						else if (dofstr.find("TX") == 0)
							dof = new AtomicXTranslationTransform();
						else if (dofstr.find("TY") == 0)
							dof = new AtomicYTranslationTransform();
						else if (dofstr.find("TZ") == 0)
							dof = new AtomicZTranslationTransform();
						dof->set_value(0);
						current_node->add_dof(dof);
						++n_dofs;
					}
				}				
				break;
			case BoneData:
				if (str.find("begin") == 0)
					current_node = new Bone();
				else if (str.find("name") == 0)
				{
					std::string name = str.substr(5);
					current_node->set_name(name);
					bones[name] = current_node;
				}
				else if (str.find("direction") == 0)
				{
					std::string direction = str.substr(10);
					float dx, dy, dz;
					std::stringstream ss(direction);
					ss >> dx >> dy >> dz;
					current_node->set_direction_in_world_space(Vec3(dx, dy, dz));
				}
				else if (str.find("length") == 0)
				{
					std::string length = str.substr(7);
					float l;
					std::stringstream ss(length);
					ss >> l;
					current_node->set_length(scalefactor * l); // scale when reading 
				}
				else if (str.find("axis") == 0)
				{
					std::string axis = str.substr(5);
					float a[3];
					std::string order;
					std::stringstream ss(axis);
					ss >> a[0] >> a[1] >> a[2] >> order;
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
				}
				else if (str.find("dof") == 0)
				{
					std::string dofs = str.substr(4);
					std::string dofstr;
					std::stringstream ss(dofs);
					n_dofs = 0;
					while (!ss.eof())
					{
						ss >> dofstr;
						AtomicTransform* dof;
						if (dofstr.find("rx") == 0)
							dof = new AtomicXRotationTransform();
						else if (dofstr.find("ry") == 0)
							dof = new AtomicYRotationTransform();
						else if (dofstr.find("rz") == 0)
							dof = new AtomicZRotationTransform();
						current_node->add_dof(dof);
						++n_dofs;
					}
				}
				else if (str.find("limits") == 0)
				{
					std::string limits = str.substr(7);
					for (int i = 0; i < n_dofs; ++i)
					{
						if (i > 0)
						{
							fin.getline(buf, CHARS_PER_LINE);
							limits = std::string(buf);
							limits = trim(limits);
						}

						limits = limits.substr(limits.find('(') + 1);
						std::stringstream ss(limits);
						float lower, upper;
						ss >> lower >> upper;
						current_node->get_dof(n_dofs - i - 1)->set_limits(lower, upper);
					}

				}
				break;
			case Hierarchy:
				if (str.find("begin") == std::string::npos && str.find("end") == std::string::npos)
				{
					std::stringstream ss(str);
					std::string parentstr;
					ss >> parentstr;
					Bone* parent = bones[parentstr];
					std::string childstr;
					while (!ss.eof())
					{
						ss >> childstr;
						Bone* child = bones[childstr];
						parent->add_child(child);
					}
				}
			}
		}
		fin.close();
		root = bones["root"];

		//root->revert_dofs();
		bone_list.clear();
		postprocess(root, Vec3(0, 0, 0));
		return true;
	}
	catch (...)
	{
		fin.close();
		return false;
	}
}

void SkinningSkeleton::postprocess(Bone* node, const Vec3& global_position)
{
	//For display adaptation
	auto bone_offset_in_global_system = node->get_direction_in_world_space() * node->get_length();
	auto bone_tip_in_global_system = global_position + bone_offset_in_global_system;
	add_point(bone_tip_in_global_system);
	//bone_list.push_back(node);// dfs traversal order, used to add child, recalculate when skel. changes

	// yzy, for acceleration 
	node->calculate_matrices();
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		postprocess(node->child_at(i), bone_tip_in_global_system);
	}
}

void SkinningSkeleton::write_pinocchio_file(const std::string& filename)
{
	std::ofstream o;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr(filename);
	o.open(wfilename, std::ios::out);
#else
	o.open(filename, std::ios::out);
#endif	

	if (o)
	{
		int next_id = 0;
		recursive_write_pinocchio_file(root, next_id, -1, o);
	}
	o.close();
}

void SkinningSkeleton::recursive_write_pinocchio_file(Bone* node, int& next_node_id, int parent_node_id, std::ofstream& o, const Vec3& global_position)
{
	auto bone_tip = global_position + node->get_direction_in_world_space() * node->get_length();
	auto p = (bone_tip - getMin()) * (1.0f / (getMax().x() - getMin().x()));
	int myIndex = next_node_id++;
	o << myIndex << " " << p.x() << " " << p.y() << " " << p.z() << " " << parent_node_id << std::endl;
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		recursive_write_pinocchio_file(node->child_at(i), next_node_id, myIndex, o, bone_tip);
	}
}

void SkinningSkeleton::read_pinocchio_file(std::string filename)
{
	std::ifstream o;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr(filename);
	o.open(wfilename, std::ios::in);
#else
	o.open(filename, std::ios::in);
#endif
	if (o)
	{
		reset_bounding_box();
		recursive_read_pinocchio_file(root, o);
	}

	o.close();

	postprocess(root, get_origin_vec());
}

void SkinningSkeleton::recursive_read_pinocchio_file(Bone* node, std::ifstream& f, const Vec3 parent_position)
{
	int id, parent_id;
	float x, y, z;
	Vec3 pos = parent_position;
	f >> id >> x >> y >> z >> parent_id;
	pos = Vec3(x, y, z);
	if (node->get_parent() != nullptr)
	{
		auto new_direction = pos - parent_position;
		node->set_length(0.019 * new_direction.length()); //0.019 * 
		new_direction.normalize();
		auto axis = cgv::math::cross(node->get_direction_in_world_space(), new_direction);
		auto angle = acos(cgv::math::dot(node->get_direction_in_world_space(), new_direction)) * 180.0f / 3.1415926f;
		auto t = new AtomicRotationTransform(axis); t->set_value(angle);
		node->add_axis_rotation(t);
		node->set_direction_in_world_space(new_direction);
	}
	else
		;// set_origin_translation(pos); // do nothing to the origin 

	// add_point(pos + get_origin_vec()); // bbox stay unchanged 
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		recursive_read_pinocchio_file(node->child_at(i), f, pos);
	}
}

void SkinningSkeleton::get_skinning_matrices(std::vector<Mat4>& matrices)
{
	matrices.clear();
	Mat4 id;
	id.identity();
	recursive_get_skinning_matrices(root, matrices, id);
}

void SkinningSkeleton::recursive_get_skinning_matrices(Bone* node, std::vector<Mat4>& matrices, Mat4 current_matrix) // imp. 
{
	current_matrix = current_matrix * node->calculate_transform_prev_to_current_with_dofs();
	if (node->get_length() != 0.0f)
	{
		matrices.push_back(origin
			* current_matrix
			* node->get_binding_pose_matrix()
			* cgv::math::inv(origin));
	}
	int n = node->childCount();
	for (int i = 0; i < n; ++i)
	{
		recursive_get_skinning_matrices(node->child_at(i), matrices, current_matrix);
	}
}

Bone* SkinningSkeleton::find_bone(const std::string& name) const
{ 
	auto it = bones.find(name); 
	if (it == bones.end())
		return nullptr;
	else
		return it->second;
}

Bone* SkinningSkeleton::find_bone_in_a_list_by_id(int bone_idx) 
{
	return bone_list.at(bone_idx);
}
