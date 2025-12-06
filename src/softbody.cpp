#include "softbody.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void SoftBody2D::_bind_methods() {
}

SoftBody2D::SoftBody2D() {
	// Initialize any variables here.
	time_passed = 0.0;
}

SoftBody2D::~SoftBody2D() {
	// Add your cleanup here.
}

void SoftBody2D::_process(double delta) {
	time_passed += delta;

	Vector2 new_position = Vector2(15 * (10.0 + (10.0 * sin(time_passed * 2.0))),(10.0 + (10.0 * cos(time_passed * 1.5))));
	
	set_position(new_position);
}