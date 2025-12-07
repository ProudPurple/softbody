#pragma once
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/tween.hpp>
#include <godot_cpp/classes/method_tweener.hpp>
#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>

namespace godot {

class SoftBody2D : public CollisionPolygon2D{
	GDCLASS(SoftBody2D, CollisionPolygon2D);

private:
	double time_passed;
	PackedVector2Array target_poly;
	PackedVector2Array cur_poly;

protected:
	static void _bind_methods();

public:
	SoftBody2D();
	~SoftBody2D();

	void _ready();
	void _process(double delta) override;
	void update_polygon();
	void define_new_polygon();
};

} // namespace godot