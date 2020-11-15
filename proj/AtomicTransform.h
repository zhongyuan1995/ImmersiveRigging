// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved
//
#pragma once

#include "common.h"
#include <memory>
#include <cgv/gui/control.h>
#include <cgv/signal/signal.h>
#include <cgv/math/inv.h>

#define PI 3.1415926f

class BaseTransform
{
public:
	//Calculates a matrix that represents the current transform.
	virtual Mat4 calculate_matrix() = 0;

	//Optimizes the current value, such that T * local_vector = target in a least-squares sense.
	virtual void optimize_value(const Vec3& local_vector, const Vec3& target) = 0;
};

class StaticTransform : public BaseTransform
{
public:
	StaticTransform(const Mat4& t) : t(t) {}

	Mat4 calculate_matrix() { return t; }
	void optimize_value(const Vec3& local_vector, const Vec3& target) { }

private:
	Mat4 t;
};

//Represents an arbitrary affine transform with exactly one scalar parameter
class AtomicTransform : public cgv::gui::control_provider<double>, public BaseTransform
{
public:
	//Sets the limits of the scalar parameter
	void set_limits(double lower, double upper);

	const double get_lower_limit() const;
	const double get_upper_limit() const;

	//Sets the current scalar parameter. Ignore ud.
	virtual void set_value(const double& value, void* ud = nullptr);

	//Gets the current scalar parameter. Ignore ud.
	const double get_value(void* ud = nullptr) const;

	//Calculates a matrix that represents the current transform.
	virtual Mat4 calculate_matrix() = 0;

	//Optimizes the current value, such that T * local_vector = target in a least-squares sense.
	virtual void optimize_value(const Vec3& local_vector, const Vec3& target, bool inverse = false) = 0;
	virtual void optimize_value(const Vec3& local_vector, const Vec3& target) { optimize_value(local_vector, target, false); }

	//Draws an indicator that visualizes the transform, including its limits.
	virtual void drawIndicator(float size) = 0;
	//Draws an indicator that visualizes the current scalar parameter.
	virtual void drawActualIndicator(float size) = 0;
	// yzy, for axis illu.
	virtual Vec3 get_axis() = 0;

	std::string get_name() const;	

	//Signal that is raised whenever the scalar parameter changes
	cgv::signal::signal<double> changed_signal;

	//Sets the order in which the transform is specified in the animation file.
	void set_index_in_amc(int i) { index_in_amc = i; }

	//Get the order in which the transform is specified in the animation file.
	int get_index_in_amc() const { return index_in_amc; }

protected:
	double lower_limit, upper_limit;
	double value;
	std::string title;
	int index_in_amc;
};

class AtomicRotationTransform : public AtomicTransform
{
public:
	AtomicRotationTransform(Vec3 axis);
	virtual Mat4 calculate_matrix();
	virtual void optimize_value(const Vec3& local_vector, const Vec3& target, bool inverse = false);

	virtual void drawIndicator(float size);
	virtual void drawActualIndicator(float size);
	// yzy
	virtual Vec3 get_axis() { return axis; }
protected:
	Vec3 axis;
};

class AtomicXRotationTransform : public AtomicRotationTransform
{
public:
	AtomicXRotationTransform() : AtomicRotationTransform(Vec3(1, 0, 0)) { title = "X-Rotation"; }
};

class AtomicYRotationTransform : public AtomicRotationTransform
{
public:
	AtomicYRotationTransform() : AtomicRotationTransform(Vec3(0, 1, 0)) { title = "Y-Rotation"; }
};

class AtomicZRotationTransform : public AtomicRotationTransform
{
public:
	AtomicZRotationTransform() : AtomicRotationTransform(Vec3(0, 0, 1)) { title = "Z-Rotation"; }
};

class AtomicTranslationTransform : public AtomicTransform
{
public:
	AtomicTranslationTransform(int dim);
	virtual Mat4 calculate_matrix();
	virtual void optimize_value(const Vec3& local_vector, const Vec3& target, bool inverse = false);

	virtual void drawIndicator(float size) { };
	virtual void drawActualIndicator(float size) { };

	virtual Vec3 get_axis() { return Vec3(0); } // yzy, we have to imp. here
private:
	int dim;
};

class AtomicXTranslationTransform : public AtomicTranslationTransform
{
public:
	AtomicXTranslationTransform() : AtomicTranslationTransform(0) { title = "X-Translation"; }
};

class AtomicYTranslationTransform : public AtomicTranslationTransform
{
public:
	AtomicYTranslationTransform() : AtomicTranslationTransform(1) { title = "Y-Translation"; }
};

class AtomicZTranslationTransform : public AtomicTranslationTransform
{
public:
	AtomicZTranslationTransform() : AtomicTranslationTransform(2) { title = "Z-Translation"; }
};

class InverseTransform : public BaseTransform
{
public:
	InverseTransform(std::shared_ptr<AtomicTransform> t) : t(t) { }

	Mat4 calculate_matrix() { return cgv::math::inv(t->calculate_matrix()); }
	void optimize_value(const Vec3& local_vector, const Vec3& target)
	{
		t->optimize_value(local_vector, target, true);
	}

private:
	std::shared_ptr<AtomicTransform> t;
};
