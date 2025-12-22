@tool
extends EditorPlugin

func _enter_tree():
	var icon = preload("res://addons/springbody2d/springicon.svg")
	var theme = EditorInterface.get_editor_theme()
	theme.set_icon("SpringBody2D", "EditorIcons", icon)

func _exit_tree():
	var theme = EditorInterface.get_editor_theme()
	theme.clear_icon("SpringBody2D", "EditorIcons")
