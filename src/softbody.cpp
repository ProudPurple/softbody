#include "softbody.h"
#include <godot_cpp/core/class_db.hpp>
#include <iostream>
#include <algorithm>
#include <cmath>

using namespace godot;
using namespace std;

void SoftBody2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_on_body_entered", "body"), &SoftBody2D::_on_body_entered);
	ClassDB::bind_method(D_METHOD("_on_body_exited", "body"), &SoftBody2D::_on_body_exited);
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
	poly = get_node<CollisionPolygon2D>("CollisionPolygon2D");
	target_poly = poly->get_polygon();
	init_poly = target_poly;
	connect("body_entered", callable_mp(this, &SoftBody2D::_on_body_entered));
	connect("body_exited", callable_mp(this, &SoftBody2D::_on_body_exited));
	RETURN_FORCE = 5;
	IMPACT_FORCE = 0.1;
	MAX_FORCE = 1000;
	BODY_FORCE = 5;
	SPRING_FORCE = 4;
	spring.active = false;
	spring.rb = nullptr;
}

void SoftBody2D::_process(double delta) {
	time_passed += delta;
	PackedVector2Array new_poly, cur_poly = poly->get_polygon();
	for (int i = 0; i < cur_poly.size(); i++) {
		new_poly.push_back(cur_poly[i].lerp(target_poly[i], 5));
		target_poly[i] = target_poly[i].lerp(init_poly[i], 3); 
		//print_line(new_poly);
	}
	if (spring.active) {
		Vector2 local_pos = to_local(spring.rb->get_position());
		PackedVector2Array polygon = poly->get_polygon();
		if (is_point_in_polygon(local_pos, polygon)) {
			spring.active = false;
			spring.rb = nullptr;
			spring.force = Vector2(0,0);
			return;
		}
        // Get per-frame displacement of the rigid body
        Vector2 vel = spring.rb->get_linear_velocity() * delta; // crucial

        Vector2 normal = spring.normal;

        // Separate normal and tangential components
        Vector2 vel_normal = normal * vel.dot(normal);
        Vector2 vel_tangent = vel - vel_normal;

        // Reverse normal to create slingshot
        Vector2 slingshot_velocity = -vel_normal;

        // Optional: gradually add some growth along the normal for springiness
        float SLING_GROWTH_RATE = 50; // tweak to taste
        slingshot_velocity += normal * SLING_GROWTH_RATE;

        // Target velocity combines tangential motion + slingshot
        Vector2 target_velocity = vel_tangent + slingshot_velocity;

        // Compute impulse needed to reach target velocity
        Vector2 impulse = (target_velocity - vel) * spring.rb->get_mass() * 100;

        // Clamp maximum force
        if (impulse.length() > MAX_FORCE)
            impulse = impulse.normalized() * MAX_FORCE;

        // Apply impulse
        spring.rb->apply_central_impulse(impulse);
    }
}

void SoftBody2D::_on_body_entered(Node *body) {
	if (body->is_class("RigidBody2D") && !spring.active) {
		RigidBody2D *rb = Object::cast_to<RigidBody2D>(body);
		Vector2 normal = _calculate_surface(rb);
		if (rb->get_linear_velocity().dot(normal) > 0)
    		normal = -normal;
		spring = {rb->get_linear_velocity(), rb, normal, true};
		print_line("START");
	}
}

void SoftBody2D::_on_body_exited(Node *body) {
	if (!spring.active) {
		_on_body_entered(body);
	} else if (body->is_class("RigidBody2D")) {
		spring.active = false;
		print_line("STOP");
	}
}

void SoftBody2D::_deform_poly(Vector2 vel, Vector2 pos) {
	PackedVector2Array polygon = poly->get_polygon();
	for (auto& point : polygon) {
		Vector2 dis = Vector2(pos[0] - point[0], pos[1] - point[1]);
		point += dis * IMPACT_FORCE * -1;
	}
	target_poly = polygon;
}

bool SoftBody2D::is_point_in_polygon(const Vector2 &point, const PackedVector2Array &poly) {
    bool inside = false;
    int n = poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vector2 &pi = poly[i];
        const Vector2 &pj = poly[j];
        if (((pi.y > point.y) != (pj.y > point.y)) &&
            (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
}


Vector2 SoftBody2D::_calculate_surface(RigidBody2D* body) {
	PackedVector2Array cur_poly = poly->get_polygon();
	pair<float, int> best_dis = {10000,0};
	vector<Vector2> normals;
	for (int i = 0; i < cur_poly.size(); i++) {
		Vector2 edge = cur_poly[(i + 1) % cur_poly.size()] - cur_poly[i];
		Vector2 normal = Vector2(-edge.y, edge.x).normalized();
		normals.push_back(normal);
		float dis = abs(normal.dot(to_local(body->get_position()) - cur_poly[i]));
		if (dis < best_dis.first)
			best_dis = {dis, i};
	}
	return normals[best_dis.second];
}

/* SUDOCODE TIME
ON_HIT() {
	CREATE_NODE AT HIT POSITION
	MOVE NODE Normalized
	APPLY SLIGHT ADDITIVE OPPOSITE VELOCITY
}

*/