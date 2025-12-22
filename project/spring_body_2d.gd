@tool

extends SpringBody2D

@onready var col = $CollisionPolygon2D
@onready var pol = $Polygon2D

func _process(_delta: float):
	pol.polygon = col.polygon
	pol.scale = col.scale
	pol.position = col.position
