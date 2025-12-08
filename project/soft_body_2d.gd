@tool

extends SoftBody2D

@onready var col = $CollisionPolygon2D
@onready var pol = $Polygon2D

func _physics_process(_delta: float):
	pol.polygon = col.polygon
	pol.scale = col.scale
	pol.position = col.position
