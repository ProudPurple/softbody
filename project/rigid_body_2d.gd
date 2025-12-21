extends RigidBody2D

func _process(_delta: float) -> void:
	if (gravity_scale != 0):
		if (Input.is_key_pressed(KEY_DOWN)):
			apply_central_impulse(Vector2(0, 30));
		if (Input.is_key_pressed(KEY_UP)):
			apply_central_impulse(Vector2(0,-20));
		if (Input.is_key_pressed(KEY_RIGHT)):
			apply_central_impulse(Vector2(10,0));
		if (Input.is_key_pressed(KEY_LEFT)):
			apply_central_impulse(Vector2(-10,0));
