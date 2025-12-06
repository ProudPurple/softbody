@tool
extends StaticBody2D

# Get references to your nodes (ensure they are set up in the scene)
@onready var collision_shape = $CollisionPolygon2D
@onready var polygon_shape = $Polygon2D
# @onready var parent_physics_body = $StaticBody2D # If needed

func _ready():
	# Link the points directly
	polygon_shape.polygon = collision_shape.polygon

func _process(_delta: float) -> void:
	polygon_shape.polygon = collision_shape.polygon
	polygon_shape.position = collision_shape.position
	polygon_shape.rotation = collision_shape.rotation
