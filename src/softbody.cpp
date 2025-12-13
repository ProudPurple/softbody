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

	ClassDB::bind_method(D_METHOD("set_spring_force", "s_force"), &SoftBody2D::set_spring_force);
    ClassDB::bind_method(D_METHOD("get_spring_force"), &SoftBody2D::get_spring_force);
	ClassDB::bind_method(D_METHOD("set_impact_force", "i_force"), &SoftBody2D::set_impact_force);
    ClassDB::bind_method(D_METHOD("get_impact_force"), &SoftBody2D::get_impact_force);
	ClassDB::bind_method(D_METHOD("set_max_force", "m_force"), &SoftBody2D::set_max_force);
    ClassDB::bind_method(D_METHOD("get_max_force"), &SoftBody2D::get_max_force);
	ClassDB::bind_method(D_METHOD("set_growth_force", "g_force"), &SoftBody2D::set_growth_force);
    ClassDB::bind_method(D_METHOD("get_growth_force"), &SoftBody2D::get_growth_force);

    ADD_PROPERTY(PropertyInfo(Variant::INT, "spring_force", PROPERTY_HINT_RANGE, "-1000,1000,1"),"set_spring_force","get_spring_force");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "impact_force", PROPERTY_HINT_RANGE, "-1000,1000,1"),"set_impact_force","get_impact_force");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_force", PROPERTY_HINT_RANGE, "-1000,1000,1"),"set_max_force","get_max_force");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "growth_force", PROPERTY_HINT_RANGE, "-1000,1000,1"),"set_growth_force","get_growth_force");
}

SoftBody2D::SoftBody2D() {
	// Initialize any variables here.
	set_process(true);
	time_passed = 0.0;
	IMPACT_FORCE = 1;
	MAX_FORCE = 1000;
	SPRING_FORCE = 4;
	SPRING_GROWTH_RATE = 50;
	spring.active = false;
	spring.rb = nullptr;
}

void SoftBody2D::set_spring_force(int s_force) { SPRING_FORCE = s_force; }
int SoftBody2D::get_spring_force() const { return SPRING_FORCE; }
void SoftBody2D::set_impact_force(int i_force) { IMPACT_FORCE = i_force; }
int SoftBody2D::get_impact_force() const { return IMPACT_FORCE; }
void SoftBody2D::set_max_force(int m_force) { MAX_FORCE = m_force; }
int SoftBody2D::get_max_force() const { return MAX_FORCE; }
void SoftBody2D::set_growth_force(int g_force) { SPRING_GROWTH_RATE = g_force; }
int SoftBody2D::get_growth_force() const { return SPRING_GROWTH_RATE; }

SoftBody2D::~SoftBody2D() {
	// Add your cleanup here.
}

void SoftBody2D::_ready() {
	poly = get_node<CollisionPolygon2D>("CollisionPolygon2D");
	target_poly = poly->get_polygon();
	init_poly = target_poly;
	connect("body_entered", callable_mp(this, &SoftBody2D::_on_body_entered));
	connect("body_exited", callable_mp(this, &SoftBody2D::_on_body_exited));
}



void SoftBody2D::_physics_process(double delta) {
	time_passed += delta;
	this->queue_redraw();
	for (auto it = prev_pos.begin(); it != prev_pos.end(); ) {
		if (prev_pos.size() != 0) {
			if (spring.normal != Vector2(0,0))
				break;
			RigidBody2D* rb = it->first;
			Vector2 prev = it->second;

			// Calculate normal using prev_pos
			Vector2 normal = _calculate_surface(rb);

			if (rb->get_linear_velocity().dot(normal) > 0)
				normal = -normal;
			// Update spring if this is the active body
			if (rb == spring.rb) {
				spring.normal = normal;
				spring.prev = prev;
			}

			// Delete prev_pos immediately after using it
			it = prev_pos.erase(it); // safe; returns next iterator
		}
	}
	PackedVector2Array new_poly, cur_poly = poly->get_polygon();
	for (int i = 0; i < cur_poly.size(); i++) {
		new_poly.push_back(cur_poly[i].lerp(target_poly[i], 5));
		target_poly[i] = target_poly[i].lerp(init_poly[i], 3); 
	}
	if (spring.active) {
        Vector2 vel = spring.rb->get_linear_velocity() * delta;

		Vector2 normal = spring.normal;	
		// Separate normal and tangential components
		Vector2 vel_normal = normal * vel.dot(normal);
		Vector2 vel_tangent = vel - vel_normal;
		//print_line(normal);
		// Reverse normal to create slingshot
		float activation = 2.5;
		spring.buildUp += (vel.dot(normal) < -activation) ? -vel.dot(normal) : 0;
		Vector2 slingshot_velocity = -vel_normal;
		// Optional: gradually add some growth along the normal for springiness
		slingshot_velocity += normal * SPRING_GROWTH_RATE;
		float decay = exp(-delta / 0.3f);
		vel_normal *= decay;
		vel_tangent *= decay;
		// Target velocity combines tangential motion + slingshot
		Vector2 target_vel_normal = vel_normal + vel_tangent;
		Vector2 impulse = (target_vel_normal - vel) * spring.rb->get_mass() * SPRING_GROWTH_RATE / delta;
		if (vel.dot(normal) >= -activation - 1 && spring.buildUp != 0) {
			if (spring.relase == Vector2(0,0))
				spring.relase = normal;
			print_line(spring.relase);
			impulse = -spring.force * spring.relase * spring.buildUp * SPRING_FORCE;
		}
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
		float initial_delta = 1.0 / 60.0f; // approximate 1 frame
		Vector2 prev_global = rb->get_global_position();

		// Convert to local space
		prev_pos[rb] = prev_global;
		Vector2 normal = _calculate_surface(rb);
		spring = {rb->get_linear_velocity(), rb, Vector2(0,0), Vector2(0,0), 0, Vector2(0,0), true};
		print_line("START");
	}
}

void SoftBody2D::_on_body_exited(Node *body) {
	if (body->is_class("RigidBody2D")) {
		spring.active = false;
		print_line("STOP");
	}
}

void SoftBody2D::_deform_poly(Vector2 vel, Vector2 pos) {
	PackedVector2Array polygon = poly->get_polygon();
	for (auto& point : polygon) {
		Vector2 dis = Vector2(pos[0] - point[0], pos[1] - point[1]);
		point += dis * IMPACT_FORCE * 0.1 * -1;
	}
	target_poly = polygon;
}

bool SoftBody2D::is_point_in_polygon(const Vector2 &point, const PackedVector2Array &poly) {
    bool inside = false;
    int n = poly.size();
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Vector2 &pi = poly[i];
        const Vector2 &pj = poly[j];
        if (((pi.y > point.y) != (pj.y > point.y)) && (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x)) {
            inside = !inside;
        }
    }
    return inside;
}

void SoftBody2D::_draw() {
	if (spring.rb) {
		Vector2 prev_local = to_local(prev_pos[spring.rb]);
		Vector2 curr_local = to_local(spring.rb->get_global_position());


        Color col = Color(1, 0, 0); // red
        draw_line(prev_local, curr_local, col, 50.0); // 2 pixels thick

        // Optional: draw polygon edges
        PackedVector2Array poly_points = poly->get_polygon();
        for (int i = 0; i < poly_points.size(); i++) {
            // SoftBody2D local space
			PackedVector2Array cur_poly = poly->get_polygon();
			Vector2 a = poly->to_global(cur_poly[i]);
				Vector2 b = poly->to_global(cur_poly[(i + 1) % cur_poly.size()]);
			//print_line(curr_local);
            draw_line(a, b, Color(0, 1, 0), 5.0); // green edges
        }
    }
}

Vector2 SoftBody2D::_calculate_surface(RigidBody2D* body) {
       if (prev_pos.find(body) == prev_pos.end())
        return Vector2(0, 0);

    Vector2 p0 = prev_pos[body];
    Vector2 p1 = body->get_global_position();

    Vector2 motion = body->get_linear_velocity();
    if (motion.length() == 0)
        return Vector2(0,0);
    motion = motion.normalized();

    PackedVector2Array poly_points = poly->get_polygon();
    Transform2D xf = poly->get_global_transform();
    Geometry2D* geom = Geometry2D::get_singleton();

    Vector2 best_normal = Vector2(0,0);
    float best_dot = -FLT_MAX;

    for (int i = 0; i < poly_points.size(); i++) {
        Vector2 a = xf.xform(poly_points[i]);
        Vector2 b = xf.xform(poly_points[(i+1)%poly_points.size()]);

        // Check if motion crosses this edge
        if (!geom->segment_intersects_segment(p0, p1, a, b))
            continue;

        Vector2 edge = b - a;
        Vector2 normal(-edge.y, edge.x);
        normal.normalize();

        // Make sure normal opposes motion
        if (normal.dot(motion) > 0)
            normal = -normal;

        float dot = -normal.dot(motion); // how well it opposes motion
        if (dot > best_dot) {
            best_dot = dot;
            best_normal = normal;
        }
    }

    return best_normal;
}


/* SUDOCODE TIME
ON_HIT() {
	CREATE_NODE AT HIT POSITION
	MOVE NODE Normalized
	APPLY SLIGHT ADDITIVE OPPOSITE VELOCITY
}

*/