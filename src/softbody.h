#pragma once

#include <godot_cpp/classes/polygon2d.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>

namespace godot {

class SoftBody2D : public CollisionPolygon2D{
	GDCLASS(SoftBody2D, CollisionPolygon2D);

private:
	double time_passed;

protected:
	static void _bind_methods();

public:
	SoftBody2D();
	~SoftBody2D();

	void _process(double delta) override;
};

} // namespace godot