@tool
extends Area3D

@export var target_name := ""

func _func_godot_apply_properties(properties: Dictionary) -> void:
	target_name = str(properties.get("target", ""))

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	body_entered.connect(_teleport)

func _teleport(body: Node3D) -> void:
	if not body.is_in_group("map_player"):
		return
	for marker in get_tree().get_nodes_in_group("map_teleport_destination"):
		if marker.target_name == target_name:
			var pose := body.global_transform as Transform3D
			pose.origin = marker.global_position + Vector3.UP * 0.1
			body.set_deferred("global_transform", pose)
			body.set_deferred("linear_velocity", Vector3.ZERO)
			return
