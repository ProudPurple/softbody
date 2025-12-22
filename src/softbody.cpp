#include "softbody.h"
#include <godot_cpp/core/class_db.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace godot;
using namespace std;

void SpringBody2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_on_body_entered", "body"), &SpringBody2D::_on_body_entered);
	ClassDB::bind_method(D_METHOD("_on_body_exited", "body"), &SpringBody2D::_on_body_exited);

	ClassDB::bind_method(D_METHOD("set_spring_force", "s_force"), &SpringBody2D::set_spring_force);
    ClassDB::bind_method(D_METHOD("get_spring_force"), &SpringBody2D::get_spring_force);
	ClassDB::bind_method(D_METHOD("set_impact_force", "i_force"), &SpringBody2D::set_impact_force);
    ClassDB::bind_method(D_METHOD("get_impact_force"), &SpringBody2D::get_impact_force);
	ClassDB::bind_method(D_METHOD("set_max_force", "m_force"), &SpringBody2D::set_max_force);
    ClassDB::bind_method(D_METHOD("get_max_force"), &SpringBody2D::get_max_force);
	ClassDB::bind_method(D_METHOD("set_growth_force", "g_force"), &SpringBody2D::set_growth_force);
    ClassDB::bind_method(D_METHOD("get_growth_force"), &SpringBody2D::get_growth_force);
	ClassDB::bind_method(D_METHOD("set_threshold", "thresh"), &SpringBody2D::set_threshold);
    ClassDB::bind_method(D_METHOD("get_threshold"), &SpringBody2D::get_threshold);
	ClassDB::bind_method(D_METHOD("set_normal_weight", "weight"), &SpringBody2D::set_normal_weight);
    ClassDB::bind_method(D_METHOD("get_normal_weight"), &SpringBody2D::get_normal_weight);

	ADD_GROUP("Physics", "physics_");

    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "physics_release_magnitude", PROPERTY_HINT_RANGE, "-1000,1000,0.1"),"set_spring_force","get_spring_force");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "physics_max_force", PROPERTY_HINT_RANGE, "-1000,1000,0.1"),"set_max_force","get_max_force");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "physics_threshold", PROPERTY_HINT_RANGE, "0,15,0.01"),"set_threshold","get_threshold");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "physics_decay_magnitude", PROPERTY_HINT_RANGE, "-1000,1000,0.1"),"set_growth_force","get_growth_force");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "physics_normal_weight", PROPERTY_HINT_RANGE, "0.6,1,0.01"),"set_normal_weight","get_normal_weight");

	ADD_GROUP("Visual", "visual_");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "visual_impact_force", PROPERTY_HINT_RANGE, "-1000,1000,0.1"),"set_impact_force","get_impact_force");
}

SpringBody2D::SpringBody2D() {
	// Initialize any variables here.
	set_process(true);
	time_passed = 0.0;
	IMPACT_FORCE = 10;
	MAX_FORCE = 100;
	SPRING_FORCE = 10;
	SPRING_GROWTH_RATE = 10;
	activation = 4;
	normal_weight = 0.7;
	force_weight = 0.3;
	external_width = 500;
	inverted = true;
}

void SpringBody2D::set_spring_force(float s_force) { SPRING_FORCE = s_force; }
float SpringBody2D::get_spring_force() const { return SPRING_FORCE; }
void SpringBody2D::set_threshold(float thresh) { activation = thresh; }
float SpringBody2D::get_threshold() const { return activation; }
void SpringBody2D::set_normal_weight(float weight) {normal_weight = weight, force_weight = 1 - weight;}
float SpringBody2D::get_normal_weight() const { return normal_weight; }
void SpringBody2D::set_impact_force(float i_force) { IMPACT_FORCE = i_force; }
float SpringBody2D::get_impact_force() const { return IMPACT_FORCE; }
void SpringBody2D::set_max_force(float m_force) { MAX_FORCE = m_force; }
float SpringBody2D::get_max_force() const { return MAX_FORCE; }
void SpringBody2D::set_growth_force(float g_force) { SPRING_GROWTH_RATE = g_force; }
float SpringBody2D::get_growth_force() const { return SPRING_GROWTH_RATE; }

SpringBody2D::~SpringBody2D() {
	// Add your cleanup here.
}

void SpringBody2D::_ready() {
	poly = get_node<CollisionPolygon2D>("CollisionPolygon2D");
	connect("body_entered", callable_mp(this, &SpringBody2D::_on_body_entered));
	connect("body_exited", callable_mp(this, &SpringBody2D::_on_body_exited));
}

void SpringBody2D::_physics_process(double delta) {
	time_passed += delta;
	for (auto& it : spring_targets) {
		SpringTarget& spring = it.second;
		if (spring.active) {
			Vector2 vel = it.first->get_linear_velocity();
			Vector2 normal = spring.collision_normal;	

			if (normal == Vector2(0,0)) {
				spring.collision_normal = _calculate_surface(it.first, spring);
			} else {
				it.first->set_gravity_scale(0);
				Vector2 temp = _calculate_surface(it.first, spring);
				if (temp != Vector2(0,0)) {
					spring.collision_normal = temp;
					normal = temp;
				}
				//print_line(vel.dot(normal) <= 0);
				// Separate normal and tangential components
				Vector2 vel_normal = normal * vel.dot(normal);
				Vector2 vel_tangent = vel - vel_normal;
				// Reverse normal to create slingshot
				spring.buildUp += (vel.dot(normal) < -activation) ? -vel.dot(normal) * delta : 0;
				float decay = exp(-delta * spring.buildUp);
				vel_normal *= decay;
				vel_tangent *= decay;
				// Target velocity combines tangential motion + slingshot
				Vector2 target_vel_normal = (vel_normal + vel_tangent) / SPRING_GROWTH_RATE;
				Vector2 impulse = (target_vel_normal - vel) * it.first->get_mass();
				if (((vel * delta).dot(normal) >= -activation && spring.buildUp >= 0)) {
					Vector2 force_dir = Vector2(spring.init_force[0], spring.init_force[1]).normalized();
					Vector2 dir = (force_dir * force_weight + spring.collision_normal * normal_weight).normalized();
					impulse = dir * spring.buildUp * (SPRING_FORCE * 10) * delta;
				}
				// Clamp maximum force
				if (impulse.length() > MAX_FORCE)
					impulse = impulse.normalized() * MAX_FORCE;
				
				// Apply impulse
				it.first->apply_central_impulse(impulse);
			}
		}
    }
}

void SpringBody2D::_on_body_entered(Node *body) {
	if (body->is_class("RigidBody2D")) {
		RigidBody2D *rb = Object::cast_to<RigidBody2D>(body);

		float initial_delta = 1.0 / 30.0f; // approximate 1 fram
		Vector2 prev_pos = rb->get_global_position() - rb->get_linear_velocity() * initial_delta;

		spring_targets[rb] = SpringTarget{rb->get_linear_velocity(), prev_pos, Vector2(0,0), 0, true};
	}
}

void SpringBody2D::_on_body_exited(Node *body) {
	if (body->is_class("RigidBody2D")) {
		RigidBody2D *rb = Object::cast_to<RigidBody2D>(body);
		if (spring_targets.find(rb) != spring_targets.end()) {
			rb->set_gravity_scale(1);
			spring_targets.erase(rb);
		}
	}
}

Vector2 SpringBody2D::_calculate_surface(RigidBody2D* rb, SpringTarget spring) {
    Vector2 p0 = spring.init_pos, p1 = rb->get_global_position();
    Vector2 vel = rb->get_linear_velocity().normalized();

    if (vel.length() == 0) {
        return Vector2(0,0);
	}

    PackedVector2Array poly_points = poly->get_polygon();
    Transform2D xf = poly->get_global_transform();
    Geometry2D* geom = Geometry2D::get_singleton();

    Vector2 best_normal = Vector2(0,0);
    float best_dot = -FLT_MAX, best_dist = FLT_MAX;

	for (int i = 0; i < poly_points.size() - (inverted ? 5 : 0); i++) {
		Vector2 a = xf.xform(poly_points[i]);
		Vector2 b = xf.xform(poly_points[(i+1)%poly_points.size()]);

		if (!geom->segment_intersects_segment(p0, p1, a, b))
			continue;
		Vector2 edge = b - a;
		Vector2 normal = Vector2(-edge.y, edge.x).normalized();

		Vector2 edge_mid = (a + b) * 0.5f;
		Vector2 to_body = rb->get_global_position() - edge_mid;

		// Enforce correct normal orientation
		if (normal.dot(to_body) < 0)
			normal = -normal;

		float dist = p0.distance_to(edge_mid);
		if (dist < best_dist) {
			best_dist = dist;
			best_normal = normal;
		}
	}


    return best_normal * -1;
}


/* SUDOCODE TIME
ON_HIT() {
	CREATE_NODE AT HIT POSITION
	MOVE NODE Normalized
	APPLY SLIGHT ADDITIVE OPPOSITE VELOCITY
}

*/