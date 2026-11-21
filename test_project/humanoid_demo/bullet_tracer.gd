extends Trail3D

const TRAVEL_SPEED := 95.0

func launch(origin: Vector3, endpoint: Vector3) -> void:
	global_position = origin
	reset_physics_interpolation()
	emitting = true
	var travel_time := clampf(origin.distance_to(endpoint) / TRAVEL_SPEED, 0.075, 0.24)
	var tween := create_tween()
	tween.tween_property(self, "global_position", endpoint, travel_time)
	tween.tween_callback(_finish_travel)
	tween.tween_interval(lifetime + 0.05)
	tween.tween_callback(queue_free)

func _finish_travel() -> void:
	emitting = false
	$Head.hide()
