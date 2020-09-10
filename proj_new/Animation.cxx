// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#include "Animation.h"

#include <fstream>

bool Animation::read_amc_file(std::string filename, SkinningSkeleton* s, bool myanim)
{
	frames.clear();

	std::ifstream fin;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr(filename);
	fin.open(wfilename);
#else
	fin.open(filename);
#endif
	if (!fin.good())
		return false;

	try{
		while (!fin.eof())
		{
			char buf[CHARS_PER_LINE];
			fin.getline(buf, CHARS_PER_LINE);
			std::string str(buf);
			str = trim(str); //remove whitespaces

			if (str.empty())
				continue;
			if (str.find('#') == 0)
				continue; //don't handle comments
			if (str.find(':') == 0)
				continue; //don't handle control sequences

			if (std::isdigit(str[0]))// for numbers 
			{
				//new frame
				frames.push_back(std::vector<AnimationFrameBone>());
				continue;
			}

			//must be a bone line
			std::stringstream ss(str);
			std::string bone_name;
			ss >> bone_name;

			Bone* bone = s->find_bone(bone_name);
			if (bone == nullptr)
				return false;

			std::vector<double> dof_values(bone->dof_count());	
			// yzy, ignore first 3 dof on root node 
			if (bone_name._Equal("root")) { 
				if (is_firstroot) {
					// yzy, read and record the first root posi. 
					double tmpvalue; 
					ss >> tmpvalue;
					skel_base_x = tmpvalue;
					ss >> tmpvalue;
					skel_base_y = tmpvalue;
					ss >> tmpvalue;
					skel_base_z = tmpvalue;
					dof_values[0] = 0;
					dof_values[1] = 0;
					dof_values[2] = 0;
					is_firstroot = false;
				}
				else {

					float scale_factor = s->get_scalefactor();
					if (myanim)
						scale_factor = 1;
					// yzy, read and scale the diff. 
					double tmpvalue; 
					ss >> tmpvalue;
					// yzy, it works. we can now load animation without modification from dataset. 
					// relative dist times scalefactor
					dof_values[0] = (tmpvalue - skel_base_x) * scale_factor; 
					ss >> tmpvalue; 
					dof_values[1] = (tmpvalue - skel_base_y) * scale_factor;
					ss >> tmpvalue;
					dof_values[2] = (tmpvalue - skel_base_z) * scale_factor;
				}
				for (int i = 3; i < bone->dof_count(); ++i)
				{
					double value;
					ss >> value;
					dof_values[i] = value;
				}
			}
			else {
				for (int i = 0; i < bone->dof_count(); ++i)
				{
					double value;
					ss >> value;
					dof_values[i] = value;
				}
			}

			frames.back().push_back(AnimationFrameBone());
			frames.back().back().bone = bone;
			frames.back().back().dof_values.resize(bone->dof_count());
			for (int i = 0; i < bone->dof_count(); ++i)
			{
				frames.back().back().dof_values[i] = dof_values[bone->get_dof(i)->get_index_in_amc()];
			}
		}
		fin.close();
	}
	catch (...)
	{
		fin.close();
		return false;
	}
	return true;
}

int Animation::frame_count() const { return frames.size(); }

void Animation::apply_frame(int frame) const
{
	for (auto& bone : frames.at(frame))// bone is a list of 
	{
		for (int i = 0; i < bone.bone->dof_count(); ++i)
		{
			bone.bone->get_dof(i)->set_value(bone.dof_values[i]);
		}
	}
}

void Animation::saveframe_to_vector(int frame) // should finish within 1/120 sec
{
	if (last_frame != frame) {
		std::vector<AnimationFrameBone> cur_frame;
		//cur_frame.resize(skel->get_size_bone_list()); // will crash, p- ?
		for (auto& bone : skel->get_bone_list()) {
			if (bone!=nullptr) {
				AnimationFrameBone cur_bone;
				cur_bone.bone = bone;
				cur_bone.dof_values.resize(bone->dof_count());
				for (int i = 0; i < bone->dof_count(); ++i)
				{
					cur_bone.dof_values[bone->get_dof(i)->get_index_in_amc()] = bone->get_dof(i)->get_value();
				}
				cur_frame.push_back(cur_bone);
			}
		}
		frames_record.push_back(cur_frame);
		std::cout << "recording to list :" << frame <<std::endl;
		last_frame = frame;
	}
}

void Animation::saveframe_to_file(std::string filename) {
	std::ofstream o;
#ifdef _WIN32
	std::wstring wfilename = cgv::utils::str2wstr(filename);
	o.open(wfilename, std::ios::out);
#else
	o.open(filename, std::ios::out);
#endif	

	if (o) {
		int frame_id = 1;
		for (auto f : frames_record) {
			o << frame_id << std::endl;
			for (auto bone : f) {
				if (bone.bone->dof_count() > 0) {
					o << bone.bone->get_name();
					for (int i = 0; i < bone.bone->dof_count(); ++i)
					{
						//bone.bone->get_dof(i)->set_value(bone.dof_values[i]);
						o << " " << bone.dof_values[i];
					}
					o << std::endl;
				}
			}
			frame_id++;
		}
	}

	o.close();
}
