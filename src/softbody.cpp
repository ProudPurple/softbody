#include "softbody.h"
#include <godot_cpp/core/class_db.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace godot;
using namespace std;

void SoftBody2D::_bind_methods() {
}

SoftBody2D::SoftBody2D() {
	// Initialize any variables here.
	set_process(true);
	time_passed = 0.0;
}

SoftBody2D::~SoftBody2D() {
	// Add your cleanup here.
}

void SoftBody2D::_ready() {
	cur_poly = get_polygon();
	define_new_polygon();
}

void SoftBody2D::_process(double delta) {
	time_passed += delta;
	update_polygon();
}

void SoftBody2D::update_polygon() {
	PackedVector2Array new_poly;
	for (int i = 0; i < target_poly.size(); i++) {
		new_poly.push_back(cur_poly[i].lerp(target_poly[i], 1));
		target_poly[i] += Vector2(-5,5);
	}
	cur_poly = new_poly;
	set_polygon(new_poly);
}

void SoftBody2D::define_new_polygon() {
	target_poly.clear();
	for (int i = 0; i < get_polygon().size() + 3; i++) {
		if (cur_poly.size() <= i)
			cur_poly.push_back(cur_poly[cur_poly.size() - 1] + Vector2(rand() % 5 * 10, rand() % 5 * -10));
		target_poly.push_back(cur_poly[i] + Vector2(100, -100));
	}
}