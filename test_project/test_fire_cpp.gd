extends Node3D

const FRACTURE_SAMPLES := ["Primitive", "FakeConvex", "RealConvex"]

func _unhandled_input(event: InputEvent) -> void:
	if event.is_action_pressed("ui_cancel"):
		get_tree().quit()
	elif event.is_action_pressed("ui_accept"):
		var sample := get_node_or_null("Primitive") as RigidBody3D
		if sample:
			var fire := sample.get_node("FireComponent3D") as FireComponent3D
			fire.apply_fire(sample.global_position, fire.get_max_hp())
	elif event.is_action_pressed("ui_up"):
		for sample in FRACTURE_SAMPLES:
			var fracture := get_node_or_null(sample + "/DestronoiNode") as DestronoiNode
			if fracture:
				fracture.destroy()
	elif event is InputEventKey and event.pressed and not event.echo and event.physical_keycode == KEY_R:
		get_tree().reload_current_scene()
