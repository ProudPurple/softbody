extends RigidBody2D

func _process(delta: float) -> void:
	if (Input.is_key_pressed(KEY_DOWN)):
		apply_central_impulse(Vector2(0, 50));
	elif (Input.is_key_pressed(KEY_UP)):
		apply_central_impulse(Vector2(0,-50));
	elif (Input.is_key_pressed(KEY_RIGHT)):
		apply_central_impulse(Vector2(50,0));
	elif (Input.is_key_pressed(KEY_LEFT)):
		apply_central_impulse(Vector2(-50,0));
