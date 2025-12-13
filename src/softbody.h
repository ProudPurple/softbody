#pragma once
#include <godot_cpp/classes/node.hpp>
#include <vector>
#include <godot_cpp/classes/tween.hpp>
#include <godot_cpp/classes/geometry2d.hpp>
#include <godot_cpp/classes/rigid_body2d.hpp>
#include <godot_cpp/classes/method_tweener.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>
#include <godot_cpp/classes/area2d.hpp>

namespace godot {

	struct Pending {
		Vector2 vel;
		Vector2 pos;
		RigidBody2D* rb;
	};

	struct Spring_Target {
		Vector2 force;
		RigidBody2D* rb;
		Vector2 normal;
		float buildUp;
		bool active;
	};

	class SoftBody2D : public Area2D{
		GDCLASS(SoftBody2D, Area2D);

	private:
		double time_passed;
		CollisionPolygon2D* poly;
		PackedVector2Array target_poly;
		PackedVector2Array init_poly;
		PackedVector2Array pending_poly;
		float IMPACT_FORCE;
		float SPRING_GROWTH_RATE;
		float MAX_FORCE;
		float SPRING_FORCE;
		Spring_Target spring;
		std::vector<Pending> pending;

	protected:
		static void _bind_methods();

	public:
		SoftBody2D();
		~SoftBody2D();

		void _ready();
		//void _physics_process(double delta);
		void _bind_method();
		void set_spring_force(int s_force);
		int get_spring_force() const;
		void set_impact_force(int s_force);
		int get_impact_force() const;
		void set_max_force(int s_force);
		int get_max_force() const;
		void set_growth_force(int s_force);
		int get_growth_force() const;
		void _process(double delta) override;
		void _on_body_entered(Node *body);
		void _on_body_exited(Node *body);
		bool is_point_in_polygon(const Vector2 &point, const PackedVector2Array &poly);
		Vector2 _calculate_surface(RigidBody2D *body);
		void _deform_poly(Vector2 vel, Vector2 pos);
	};

} // namespace godot