extends Node3D

const SKULLGUY = preload("skullguy1.glb")
const GENERIC = preload("generic_guy.glb")
const PISTOL = preload("pistol.glb")
const SKULLGUY_GRIP := Vector3(-0.0134877, -0.0914844, 0.0357493)
const SKULLGUY_GRIP_ROTATION := Vector3(-1.29335, -2.36534, -2.24928)
const SKULLGUY_STOW := Vector3(0.19507, -0.101801, 0.0315023)
const SKULLGUY_STOW_ROTATION := Vector3(-0.933135, 2.96802, 2.95033)
const GENERIC_GRIP := Vector3(-0.00806117, 0.0938622, 0.0486137)
const GENERIC_GRIP_ROTATION := Vector3(1.35223, -2.99427, 1.73925)
const GENERIC_STOW := Vector3(-0.210638, -0.194035, -0.0325694)
const GENERIC_STOW_ROTATION := Vector3(-1.1832, 1.5865, 1.45052)
# The original aim target summed the input and an equally long rotated vector.
# The resulting direction is halfway between them.
const PISTOL_AIM_OFFSET := 77.0 / 2.0
const UNARMED_AIM_OFFSET := 45.0 / 2.0
const SKULLGUY_PISTOL_AIM_HEIGHT := 1.65
const GENERIC_PISTOL_AIM_HEIGHT := 0.9
const AIM_TARGET_DISTANCE := 3.0
const BACKPEDAL_CORRECTION_RATE := 24.0
const RECOIL_DURATION := 0.15
const RECOIL_TARGET_LIFT := 0.32
const MOVE_SPEED := 5.0
const JUMP_SPEED := 4.245

@onready var player: CharacterBody3D = $Player
@onready var rig = $Player/Humanoid
var use_skullguy := true
var equipped := true
var action_active := false
var recoil_remaining := 0.0
var backpedal_correction := Vector2.ZERO
var weapon := PISTOL.instantiate() as Node3D

func _ready() -> void:
	_mount_pistol()
	rig.torso_animation_finished.connect(func(_clip): action_active = false)

func _mount_pistol() -> void:
	rig.mount_weapon(weapon, equipped)
	if use_skullguy:
		weapon.position = SKULLGUY_GRIP if equipped else SKULLGUY_STOW
		weapon.rotation = SKULLGUY_GRIP_ROTATION if equipped else SKULLGUY_STOW_ROTATION
	else:
		weapon.position = GENERIC_GRIP if equipped else GENERIC_STOW
		weapon.rotation = GENERIC_GRIP_ROTATION if equipped else GENERIC_STOW_ROTATION

func _physics_process(delta: float) -> void:
	var move := Input.get_vector("move_left", "move_right", "move_up", "move_down")
	var aim_input := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var aiming := not aim_input.is_zero_approx()
	var recoiling := recoil_remaining > 0.0
	var facing := aim_input if aiming else move
	var backpedaling := aiming and equipped and player.is_on_floor() and move.dot(aim_input) <= -0.701
	if not backpedaling:
		backpedal_correction = Vector2.ZERO
	elif not recoiling and not action_active:
		_update_backpedal_correction(aim_input, delta)

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
	if aiming or recoiling:
		var aim_direction := facing
		if aim_direction.is_zero_approx():
			var forward: Vector3 = -rig.global_basis.z
			aim_direction = Vector2(forward.x, forward.z).normalized()
		_update_aim_target(aim_direction)
	else:
		rig.clear_aim()
	recoil_remaining = maxf(recoil_remaining - delta, 0.0)

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
		if aiming or recoiling:
			torso_clip = StringName("aim_" + String(stance))
		_play_loop(rig.get_torso_player(), torso_clip)

func _update_aim_target(direction: Vector2) -> void:
	var aim_offset := PISTOL_AIM_OFFSET if equipped else UNARMED_AIM_OFFSET
	var chest_direction := direction.rotated(deg_to_rad(aim_offset) + backpedal_correction.x)
	var aim_height := 1.2
	if equipped:
		aim_height = SKULLGUY_PISTOL_AIM_HEIGHT if use_skullguy else GENERIC_PISTOL_AIM_HEIGHT
		# Kick immediately, then settle onto the same aiming pose.
		var recoil_weight := recoil_remaining / RECOIL_DURATION
		aim_height += backpedal_correction.y + RECOIL_TARGET_LIFT * recoil_weight * recoil_weight
	rig.set_aim_target(player.global_position + Vector3(chest_direction.x * AIM_TARGET_DISTANCE, aim_height, chest_direction.y * AIM_TARGET_DISTANCE))

func _update_backpedal_correction(direction: Vector2, delta: float) -> void:
	var torso: AnimationPlayer = rig.get_torso_player()
	if not torso or torso.current_animation != &"main/aim_pistol":
		return
	var barrel := -weapon.global_basis.z.normalized()
	var horizontal := Vector2(barrel.x, barrel.z)
	if horizontal.normalized().dot(direction) < 0.95:
		return

	# The backward clip moves Hips, which also moves the chest and hand.
	# Correct the resulting barrel drift without changing either animation.
	var weight := 1.0 - exp(-BACKPEDAL_CORRECTION_RATE * delta)
	backpedal_correction.x = clampf(backpedal_correction.x + horizontal.angle_to(direction) * weight, -0.2, 0.2)
	backpedal_correction.y = clampf(backpedal_correction.y - barrel.y * AIM_TARGET_DISTANCE * weight, -0.55, 0.55)

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
				backpedal_correction = Vector2.ZERO
				_mount_pistol()
				action_active = false
				recoil_remaining = 0.0
		KEY_SHIFT:
			if player.is_on_floor():
				player.velocity.y = JUMP_SPEED
		KEY_SPACE:
			if equipped:
				if not action_active:
					recoil_remaining = RECOIL_DURATION
			else:
				action_active = rig.play_torso(&"punch0", 0.08)
		KEY_E:
			if equipped:
				recoil_remaining = 0.0
				action_active = rig.play_torso(&"reload", 0.12)
		KEY_Q:
			recoil_remaining = 0.0
			equipped = not equipped
			_mount_pistol()
			action_active = rig.play_torso(&"equip_twohand" if equipped else &"stow_twohand", 0.12)
		KEY_ESCAPE:
			get_tree().quit()
