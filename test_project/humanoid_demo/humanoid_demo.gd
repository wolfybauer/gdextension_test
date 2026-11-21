extends Node3D

const SKULLGUY = preload("skullguy1.glb")
const GENERIC = preload("generic_guy.glb")
const MOVE_SPEED := 5.0
const JUMP_SPEED := 4.245

@onready var player: CharacterBody3D = $Player
@onready var rig = $Player/Humanoid
var use_skullguy := true
var equipped := true
var action_active := false
var weapon := MeshInstance3D.new()

func _ready() -> void:
	var mesh := BoxMesh.new()
	mesh.size = Vector3(0.08, 0.12, 0.3)
	weapon.mesh = mesh
	weapon.position.z = -0.13
	rig.mount_weapon(weapon, equipped)
	rig.torso_animation_finished.connect(func(_clip): action_active = false)

func _physics_process(delta: float) -> void:
	var move := Input.get_vector("move_left", "move_right", "move_up", "move_down")
	var aim_input := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var aiming := not aim_input.is_zero_approx()
	var facing := aim_input if aiming else move

	# The fixed camera looks down -Z, as in meleeguy0.
	var target_velocity := Vector2(move.x / 1.5, move.y) * MOVE_SPEED
	var weight := 1.0 - pow(0.85, delta * 60.0)
	player.velocity.x = lerpf(player.velocity.x, target_velocity.x, weight)
	player.velocity.z = lerpf(player.velocity.z, target_velocity.y, weight)
	if not player.is_on_floor():
		player.velocity += player.get_gravity() * delta
	player.move_and_slide()

	if not facing.is_zero_approx():
		rig.rotation.y = lerp_angle(rig.rotation.y, atan2(-facing.x, -facing.y), weight)
	if aiming:
		rig.set_aim_target(player.global_position + Vector3(aim_input.x * 3.0, 1.2, aim_input.y * 3.0))
	else:
		rig.clear_aim()

	var stance := &"pistol" if equipped else &"unarmed"
	var torso_clip := &"idle_unarmed" if move.is_zero_approx() else &"fwd_unarmed"
	if not player.is_on_floor():
		_play_loop(rig.get_legs_player(), &"fall")
		torso_clip = &"fall"
	elif aiming:
		var forward := aim_input.normalized()
		var right := Vector2(-forward.y, forward.x)
		rig.play_locomotion(Vector2(move.dot(right), move.dot(forward)), stance)
	else:
		_play_loop(rig.get_legs_player(), torso_clip)

	if not action_active:
		if aiming:
			torso_clip = StringName("aim_" + String(stance))
		_play_loop(rig.get_torso_player(), torso_clip)

func _play_loop(layer: AnimationPlayer, clip: StringName) -> void:
	var qualified := StringName("main/" + String(clip))
	if layer and layer.has_animation(qualified) and layer.current_animation != qualified:
		layer.play(qualified, 0.15)

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	match event.physical_keycode:
		KEY_TAB:
			var next_model := GENERIC if use_skullguy else SKULLGUY
			if rig.load_model(next_model, use_skullguy):
				use_skullguy = not use_skullguy
				action_active = false
		KEY_SHIFT:
			if player.is_on_floor():
				player.velocity.y = JUMP_SPEED
		KEY_SPACE:
			action_active = rig.play_torso(&"shoot_pistol" if equipped else &"punch0", 0.08)
		KEY_E:
			if equipped:
				action_active = rig.play_torso(&"reload", 0.12)
		KEY_Q:
			equipped = not equipped
			rig.mount_weapon(weapon, equipped)
			action_active = rig.play_torso(&"equip_twohand" if equipped else &"stow_twohand", 0.12)
		KEY_ESCAPE:
			get_tree().quit()
