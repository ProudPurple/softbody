extends RigidBody2D

# Get references to your nodes (ensure they are set up in the scene)
@onready var collision_shape = $SoftBody2D
@onready var polygon_shape = $Polygon2D
# @onready var parent_physics_body = $StaticBody2D # If needed

func _ready():
	# Link the points directly
	polygon_shape.polygon = collision_shape.polygon

func _process(delta: float) -> void:
	polygon_shape.polygon = collision_shape.polygon
	polygon_shape.position = collision_shape.position
	
	# Optional: If your Polygon2D is a child of the physics body,
	# you might need to add the collision shape dynamically or adjust positioning.
	# parent_physics_body.add_child(collision_shape)
	
	# Example for a simple setup where both are siblings:
	# collision_shape.polygon = self.polygon

	# For more complex setups (like linking to a parent's Polygon2D):
	# var visual_polygon = get_parent() as Polygon2D
	# if visual_polygon:
	#     collision_shape.polygon = visual_polygon.polygon
