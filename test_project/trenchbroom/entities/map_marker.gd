@tool
extends Marker3D

@export var target_name := ""

func _func_godot_apply_properties(properties: Dictionary) -> void:
	target_name = str(properties.get("targetname", ""))
