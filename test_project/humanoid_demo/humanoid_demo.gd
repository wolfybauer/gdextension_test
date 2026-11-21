extends Node3D

const SKULLGUY = preload("skullguy1.glb")
const GENERIC = preload("generic_guy.glb")
const JabHipCompensation = preload("jab_hip_compensation.gd")
const ENEMY_SCENE = preload("enemies/demo_enemy.tscn")
const MAX_ENEMIES := 8
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
var equipped := false
var action_active := false
var recoil_remaining := 0.0
var backpedal_correction := Vector2.ZERO
var weapon_index := 0
var shot_cooldown := 0.0
var weapon: Node3D
var appearance_rng := RandomNumberGenerator.new()

func _ready() -> void:
	_select_weapon(0)
	JabHipCompensation.new().setup(rig)
	appearance_rng.randomize()
	for position in [Vector3(-4.5, 0, -3), Vector3(4.5, 0, -3), Vector3(0, 0, -5.5)]:
		_spawn_enemy(position)
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
	$Controls.text = "%s  |  1 Pistol  2 Revolver  3 Shotgun  4 SMG  5 Knife\nWASD Move  •  Arrows Aim  •  Space Attack  •  E Reload  •  Shift Jump  •  Tab Swap model  •  R Spawn enemy  •  B Blood preview" % WEAPONS[index].name

func _mount_weapon() -> void:
	var index := 0 if use_skullguy else 2
	var mounts: Array = WEAPON_MOUNTS[WEAPONS[weapon_index].stance]
	var grip: Array = mounts[index]
	var stow: Array = mounts[index + 1]
	rig.configure_aim_weapon(weapon,
		Transform3D(Basis.from_euler(grip[1]), grip[0]),
		Transform3D(Basis.from_euler(stow[1]), stow[0]))

func _sync_weapon_aim(aiming: bool) -> void:
	if equipped == aiming:
		return
	equipped = aiming
	rig.set_weapon_aiming(aiming)
	if not aiming:
		recoil_remaining = 0.0
		var torso: AnimationPlayer = rig.get_torso_player()
		if torso.current_animation == &"main/reload":
			torso.stop()
			action_active = false

func _has_firearm() -> bool:
	return equipped and WEAPONS[weapon_index].stance != &"melee"

func _try_fire() -> void:
	if action_active or shot_cooldown > 0.0:
		return
	recoil_remaining = RECOIL_DURATION
	shot_cooldown = WEAPONS[weapon_index].cooldown
	var two_handed: bool = WEAPONS[weapon_index].stance == &"twohand"
	_hit_enemy(_attack_direction(), 18.0 if two_handed else 24.0, 2 if two_handed else 1)

func _spawn_enemy(position: Vector3) -> void:
	if $Enemies.get_child_count() >= MAX_ENEMIES:
		return
	var enemy := ENEMY_SCENE.instantiate()
	$Enemies.add_child(enemy)
	enemy.global_position = position
	if not enemy.initialize(player, appearance_rng):
		enemy.queue_free()

func _get_aim_input() -> Vector2:
	return Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")

func _attack_direction() -> Vector3:
	var aim_input := _get_aim_input()
	if not aim_input.is_zero_approx():
		return Vector3(aim_input.x, 0, aim_input.y).normalized()
	return -rig.global_basis.z.normalized()

func _hit_enemy(direction: Vector3, distance: float, damage: int) -> void:
	var origin := player.global_position + Vector3.UP * 1.05
	var query := PhysicsRayQueryParameters3D.create(origin, origin + direction * distance, 3, [player.get_rid()])
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	var enemy := hit.get("collider") as Node
	if enemy and enemy.is_in_group("demo_enemies"):
		enemy.take_hit(direction, damage)

func _physics_process(delta: float) -> void:
	shot_cooldown = maxf(shot_cooldown - delta, 0.0)
	var move := Input.get_vector("move_left", "move_right", "move_up", "move_down")
	var aim_input := _get_aim_input()
	var aiming := not aim_input.is_zero_approx()
	_sync_weapon_aim(aiming)
	if equipped and WEAPONS[weapon_index].automatic and Input.is_physical_key_pressed(KEY_SPACE):
		_try_fire()
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
	var torso: AnimationPlayer = rig.get_torso_player()
	var right_jab := torso.current_animation == &"main/punch_right_jab"
	if (aiming or recoiling) and not right_jab:
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
				JabHipCompensation.new().setup(rig)
				use_skullguy = not use_skullguy
				backpedal_correction = Vector2.ZERO
				_mount_weapon()
				action_active = false
				recoil_remaining = 0.0
		KEY_SHIFT:
			if player.is_on_floor():
				player.velocity.y = JUMP_SPEED
		KEY_SPACE:
			_sync_weapon_aim(not _get_aim_input().is_zero_approx())
			if _has_firearm():
				_try_fire()
			elif not action_active:
				var clips := [&"melee1", &"melee2", &"punch_right_jab"] if equipped else [&"punch0", &"punch1", &"punch2", &"punch_right_jab"]
				action_active = rig.play_torso(clips.pick_random(), 0.08)
				if action_active:
					_hit_enemy(_attack_direction(), 1.8, 2 if equipped else 1)
		KEY_E:
			_sync_weapon_aim(not _get_aim_input().is_zero_approx())
			if _has_firearm():
				recoil_remaining = 0.0
				action_active = rig.play_torso(&"reload", 0.12)
		KEY_R:
			var spread := appearance_rng.randf_range(-0.8, 0.8)
			var spawn_position := player.global_position + _attack_direction().rotated(Vector3.UP, spread) * 5.0
			spawn_position.y = 0.0
			_spawn_enemy(spawn_position)
		KEY_B:
			$Player/BloodController.splatter(-rig.global_basis.z)
		KEY_ESCAPE:
			get_tree().quit()
