extends Node3D

const SKULLGUY = preload("skullguy1.glb")
const GENERIC = preload("generic_guy.glb")
const WEAPONS := [
	{"scene": preload("pistol.glb"), "name": "Pistol", "stance": &"pistol", "cooldown": 0.2, "automatic": false, "recoil": 0.32},
	{"scene": preload("revolver.glb"), "name": "Revolver", "stance": &"pistol", "cooldown": 0.3, "automatic": false, "recoil": 0.42},
	{"scene": preload("shotgun.glb"), "name": "Shotgun", "stance": &"twohand", "cooldown": 0.35, "automatic": false, "recoil": 0.52},
	{"scene": preload("smg.glb"), "name": "SMG", "stance": &"pistol", "cooldown": 0.1, "automatic": true, "recoil": 0.24},
	{"scene": preload("knife.glb"), "name": "Knife", "stance": &"melee", "cooldown": 0.2, "automatic": false, "recoil": 0.0},
]
# Mounts are Skullguy grip/stow, then generic grip/stow.
const WEAPON_MOUNTS := {
	&"melee": [
		[Vector3(0.0180969, -0.0501575, 0.024302), Vector3(-0.732574, 2.0172, -0.605067)],
		[Vector3(-0.186177, -0.092466, -0.0394792), Vector3(-0.931017, 2.99635, 0.242755)],
		[Vector3(-0.0192128, 0.0666303, 0.0332389), Vector3(-0.0461587, -1.45172, 2.09839)],
		[Vector3(0.190635, -0.0629065, 0.0637274), Vector3(-0.521793, -0.0928114, 0.00531627)],
	],
	&"pistol": [
		[Vector3(-0.0134877, -0.0914844, 0.0357493), Vector3(-1.29335, -2.36534, -2.24928)],
		[Vector3(0.19507, -0.101801, 0.0315023), Vector3(-0.933135, 2.96802, 2.95033)],
		[Vector3(-0.00806117, 0.0938622, 0.0486137), Vector3(1.35223, -2.99427, 1.73925)],
		[Vector3(-0.210638, -0.194035, -0.0325694), Vector3(-1.1832, 1.5865, 1.45052)],
	],
	&"twohand": [
		[Vector3(-0.0710149, -0.0351743, 0.00805993), Vector3(-1.41816, -1.71328, -2.89253)],
		[Vector3(0.0919104, 0.304443, 0.140105), Vector3(-1.1174, -1.52794, 0.149945)],
		[Vector3(0.0766877, 0.0590649, 0.0290658), Vector3(1.54858, 0.329305, -1.156)],
		[Vector3(-0.144882, 0.25231, -0.148223), Vector3(-0.997982, -1.39961, -0.054284)],
	],
}
# The original aim target summed the input and an equally long rotated vector.
# The resulting direction is halfway between them.
const AIM_PROFILES := {
	&"unarmed": [Vector2(45.0 / 2.0, 1.2), Vector2(45.0 / 2.0, 1.2)],
	&"melee": [Vector2(50.0 / 2.0, 1.2), Vector2(50.0 / 2.0, 1.2)],
	&"pistol": [Vector2(77.0 / 2.0, 1.65), Vector2(77.0 / 2.0, 0.9)],
	&"twohand": [Vector2(42.6, 1.22), Vector2(40.1, 0.8)],
}
const AIM_TARGET_DISTANCE := 3.0
const BACKPEDAL_CORRECTION_RATE := 24.0
const RECOIL_DURATION := 0.15
const MOVE_SPEED := 5.0
const JUMP_SPEED := 4.245

@onready var player: CharacterBody3D = $Player
@onready var rig = $Player/Humanoid
var use_skullguy := true
var equipped := true
var action_active := false
var recoil_remaining := 0.0
var backpedal_correction := Vector2.ZERO
var weapon_index := 0
var shot_cooldown := 0.0
var weapon: Node3D

func _ready() -> void:
	_select_weapon(0)
	rig.torso_animation_finished.connect(func(_clip): action_active = false)

func _select_weapon(index: int) -> void:
	if weapon and index == weapon_index:
		return
	var next_weapon := (WEAPONS[index].scene as PackedScene).instantiate() as Node3D
	if weapon:
		weapon.get_parent().remove_child(weapon)
		weapon.queue_free()
		rig.get_torso_player().stop()
		action_active = false
	weapon_index = index
	weapon = next_weapon
	recoil_remaining = 0.0
	backpedal_correction = Vector2.ZERO
	_mount_weapon()
	$Controls.text = "%s  |  1 Pistol  2 Revolver  3 Shotgun  4 SMG  5 Knife\nWASD Move  •  Arrows Aim  •  Space Attack  •  E Reload  •  Q Equip/stow  •  Shift Jump  •  Tab Swap model" % WEAPONS[index].name

func _mount_weapon() -> void:
	rig.mount_weapon(weapon, equipped)
	var mount_index := 0 if use_skullguy else 2
	if not equipped:
		mount_index += 1
	var mount: Array = WEAPON_MOUNTS[WEAPONS[weapon_index].stance][mount_index]
	weapon.position = mount[0]
	weapon.rotation = mount[1]

func _has_firearm() -> bool:
	return equipped and WEAPONS[weapon_index].stance != &"melee"

func _try_fire() -> void:
	if action_active or shot_cooldown > 0.0:
		return
	recoil_remaining = RECOIL_DURATION
	shot_cooldown = WEAPONS[weapon_index].cooldown

func _physics_process(delta: float) -> void:
	shot_cooldown = maxf(shot_cooldown - delta, 0.0)
	if equipped and WEAPONS[weapon_index].automatic and Input.is_physical_key_pressed(KEY_SPACE):
		_try_fire()
	var move := Input.get_vector("move_left", "move_right", "move_up", "move_down")
	var aim_input := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	var aiming := not aim_input.is_zero_approx()
	var recoiling := recoil_remaining > 0.0
	var facing := aim_input if aiming else move
	var backpedaling := aiming and _has_firearm() and player.is_on_floor() and move.dot(aim_input) <= -0.701
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

	var stance: StringName = WEAPONS[weapon_index].stance if equipped else &"unarmed"
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
	var stance: StringName = WEAPONS[weapon_index].stance if equipped else &"unarmed"
	var profile: Vector2 = AIM_PROFILES[stance][0 if use_skullguy else 1]
	var chest_direction := direction.rotated(deg_to_rad(profile.x) + backpedal_correction.x)
	# Kick immediately, then settle onto the same aiming pose.
	var recoil_weight := recoil_remaining / RECOIL_DURATION
	var recoil_lift := float(WEAPONS[weapon_index].recoil) * recoil_weight * recoil_weight
	var aim_height := profile.y + backpedal_correction.y + recoil_lift
	rig.set_aim_target(player.global_position + Vector3(chest_direction.x * AIM_TARGET_DISTANCE, aim_height, chest_direction.y * AIM_TARGET_DISTANCE))

func _update_backpedal_correction(direction: Vector2, delta: float) -> void:
	var torso: AnimationPlayer = rig.get_torso_player()
	if not torso or torso.current_animation != StringName("main/aim_" + String(WEAPONS[weapon_index].stance)):
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
	if event.physical_keycode >= KEY_1 and event.physical_keycode <= KEY_5:
		_select_weapon(event.physical_keycode - KEY_1)
		return
	match event.physical_keycode:
		KEY_TAB:
			var next_model := GENERIC if use_skullguy else SKULLGUY
			if rig.load_model(next_model, use_skullguy):
				use_skullguy = not use_skullguy
				backpedal_correction = Vector2.ZERO
				_mount_weapon()
				action_active = false
				recoil_remaining = 0.0
		KEY_SHIFT:
			if player.is_on_floor():
				player.velocity.y = JUMP_SPEED
		KEY_SPACE:
			if _has_firearm():
				_try_fire()
			elif not action_active:
				action_active = rig.play_torso(&"melee1" if equipped else &"punch0", 0.08)
		KEY_E:
			if _has_firearm():
				recoil_remaining = 0.0
				action_active = rig.play_torso(&"reload", 0.12)
		KEY_Q:
			recoil_remaining = 0.0
			equipped = not equipped
			_mount_weapon()
			var action_name := "equip_" if equipped else "stow_"
			var animation_stance := "melee" if WEAPONS[weapon_index].stance == &"melee" else "twohand"
			action_active = rig.play_torso(StringName(action_name + animation_stance), 0.12)
		KEY_ESCAPE:
			get_tree().quit()
