extends Node3D

const HUMANOID = preload("res://gde_test/assets/humanoid/humanoid.tscn")
const LISTENER = preload("res://humanoid_demo/psx_characters/extras/male/Character_Sheriff_N_01.fbx")

var props: Dictionary[StringName, Node3D] = {}
var selected: Node3D
var button_on := false

func _ready() -> void:
	_make_prop(&"button", Vector3(-3.0, 0.55, 2.4), Vector3(0.4, 1.1, 0.4), Color(0.78, 0.34, 0.14))
	_make_prop(&"door", Vector3(-1.25, 0.95, 2.4), Vector3(0.95, 1.9, 0.18), Color(0.39, 0.25, 0.17))
	_make_prop(&"push", Vector3(0.75, 0.40, 2.4), Vector3(0.8, 0.8, 0.8), Color(0.54, 0.38, 0.22))
	var listener = HUMANOID.instantiate()
	add_child(listener)
	listener.position = Vector3(2.8, 0, 2.4)
	listener.load_model(LISTENER, true)
	listener.play_torso(&"idle_unarmed")
	listener.play_legs(&"idle_unarmed")
	props[&"talk0"] = listener

func _make_prop(action: StringName, pos: Vector3, size: Vector3, color: Color) -> void:
	var prop := AnimatableBody3D.new()
	var mesh_instance := MeshInstance3D.new()
	var mesh := BoxMesh.new()
	mesh.size = size
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	mesh.material = material
	mesh_instance.mesh = mesh
	prop.add_child(mesh_instance)
	var collision := CollisionShape3D.new()
	var shape := BoxShape3D.new()
	shape.size = size
	collision.shape = shape
	prop.add_child(collision)
	if action == &"door":
		var offset := Vector3(size.x * 0.5, 0, 0)
		mesh_instance.position = offset
		collision.position = offset
		pos -= offset
	prop.position = pos
	add_child(prop)
	props[action] = prop

func interact(actor: PhysicsBody3D, controller: HumanoidController3D) -> void:
	var nearest := &""
	var distance := 1.45
	for action in props:
		var offset := props[action].global_position - actor.global_position
		var horizontal := Vector2(offset.x, offset.z).length()
		if horizontal < distance and absf(offset.y) < 1.5:
			distance = horizontal
			nearest = action
	if nearest != &"":
		var prop := props[nearest]
		if controller.interact(nearest, prop.global_position - actor.global_position):
			selected = prop

func contact(clip: StringName, actor: PhysicsBody3D) -> void:
	if not is_instance_valid(selected):
		return
	match clip:
		&"button":
			button_on = not button_on
			var material := (selected.get_child(0) as MeshInstance3D).mesh.material as StandardMaterial3D
			material.albedo_color = Color(0.39, 0.77, 0.33) if button_on else Color(0.78, 0.34, 0.14)
		&"door":
			var open := is_zero_approx(selected.rotation.y)
			create_tween().set_process_mode(Tween.TWEEN_PROCESS_PHYSICS).tween_property(selected, "rotation:y", -PI / 2.0 if open else 0.0, 0.35)
		&"push":
			var direction := selected.global_position - actor.global_position
			direction.y = 0.0
			create_tween().set_process_mode(Tween.TWEEN_PROCESS_PHYSICS).tween_property(selected, "global_position",
				selected.global_position + direction.normalized() * 0.75, 0.30)
	selected = null
