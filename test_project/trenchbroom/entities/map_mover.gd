@tool
extends AnimatableBody3D

@export var motion := Vector3.UP
@export var travel := 1.0
@export var speed := 3.0
@export var wait_time := 1.0
@export var lift := false
var rest := Vector3.ZERO
var open := Vector3.ZERO
@export var bounds := AABB()
var nearby_time := 0.0
var moving := false

func _func_godot_apply_properties(properties: Dictionary) -> void:
	lift = properties.get("classname") == "func_plat"
	var angle := float(properties.get("angle", 0.0))
	if angle == -1.0 or lift:
		motion = Vector3.UP
	elif angle == -2.0:
		motion = Vector3.DOWN
	else:
		motion = Vector3(cos(deg_to_rad(angle)), 0, -sin(deg_to_rad(angle)))
	speed = maxf(0.1, float(properties.get("speed", 100.0)) / 32.0)
	wait_time = maxf(0.3, float(properties.get("wait", 1.0)))
	for node in get_children():
		if node is MeshInstance3D:
			bounds = node.get_aabb()
			break
	var dimensions := motion.abs().dot(bounds.size)
	var height := float(properties.get("height", 0.0)) / 32.0
	travel = height if lift and height > 0.0 else maxf(0.25, dimensions - float(properties.get("lip", 8.0)) / 32.0)

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	rest = position
	open = position + motion * travel
	if lift:
		open = position
		rest = position - motion * travel
		position = rest

func _physics_process(delta: float) -> void:
	if Engine.is_editor_hint():
		return
	var player := get_tree().get_first_node_in_group("map_player") as Node3D
	if not player:
		return
	if bounds.grow(1.25).has_point(to_local(player.global_position + Vector3.UP * 0.7)):
		nearby_time = wait_time
	else:
		nearby_time = maxf(0.0, nearby_time - delta)
	var target := open if nearby_time > 0.0 else rest
	position = position.move_toward(target, speed * delta)
