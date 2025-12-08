extends RigidBody2D

func _process(delta: float) -> void:
	print(get_linear_velocity() * delta)
