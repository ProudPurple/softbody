#pragma once
#include <godot_cpp/classes/node.hpp>
#include <vector>
#include <unordered_map>
#include <godot_cpp/classes/tween.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/classes/node2d.hpp>
#include <godot_cpp/classes/geometry2d.hpp>
#include <godot_cpp/classes/physics_server2d.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/rigid_body2d.hpp>
#include <godot_cpp/classes/method_tweener.hpp>
#include <godot_cpp/classes/collision_polygon2d.hpp>
#include <godot_cpp/classes/area2d.hpp>

using namespace std;

namespace godot {
	struct SpringTarget {
		Vector2 init_force;
		Vector2 init_pos;
		Vector2 collision_normal;
		float buildUp;
		bool active;
	};

	class SpringBody2D : public Area2D{
		GDCLASS(SpringBody2D, Area2D);

	private:
		double time_passed;
		CollisionPolygon2D* poly;
		float IMPACT_FORCE;
		float SPRING_GROWTH_RATE;
		float MAX_FORCE;
		float SPRING_FORCE;
		float normal_weight;
		float force_weight;
		float activation;
		float external_width;
		bool inverted;
		unordered_map<RigidBody2D*, SpringTarget> spring_targets;

	protected:
		static void _bind_methods();

	public:
		SpringBody2D();
		~SpringBody2D();

		void _ready();
		void _bind_method();
		void set_spring_force(float s_force);
		float get_spring_force() const;
		void set_impact_force(float i_force);
		float get_impact_force() const;
		void set_threshold(float thresh);
		float get_threshold() const;
		void set_normal_weight(float weight);
		float get_normal_weight() const;
		void set_max_force(float m_force);
		float get_max_force() const;
		void set_growth_force(float g_force);
		float get_growth_force() const;
		void _physics_process(double delta) override;
		void _on_body_entered(Node *body);
		void _on_body_exited(Node *body);
		void _create_external(CollisionPolygon2D* poly, float width);
		Vector2 _calculate_surface(RigidBody2D* rb, SpringTarget spring);

	};

} // namespace godot