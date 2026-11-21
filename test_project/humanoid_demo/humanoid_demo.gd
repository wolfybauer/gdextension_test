extends Node3D

const SKULLGUY = preload("skullguy1.glb")
const GENERIC = preload("generic_guy.glb")
const ENEMY_SCENE = preload("enemies/demo_enemy.tscn")
const BULLET_TRACER = preload("bullet_tracer.tscn")
const MAX_ENEMIES := 8
const BAT_REACH := 2.1
const SHOTGUN_SPREAD := [-27.0, -13.0, -3.0, 3.0, 13.0, 27.0]
const WEAPONS := [
	{"scene": preload("pistol.glb"), "muzzle": Vector3(0, 0.082, -0.217), "name": "Pistol", "stance": &"pistol", "cooldown": 0.2, "automatic": false, "recoil": 0.32},
	{"scene": preload("revolver.glb"), "muzzle": Vector3(0, 0.085, -0.313), "name": "Revolver", "stance": &"pistol", "cooldown": 0.3, "automatic": false, "recoil": 0.42},
	{"scene": preload("shotgun.glb"), "muzzle": Vector3(0, 0.044, -0.785), "name": "Shotgun", "stance": &"twohand", "cooldown": 0.35, "automatic": false, "recoil": 0.52},
	{"scene": preload("smg.glb"), "muzzle": Vector3(0, 0.087, -0.371), "name": "SMG", "stance": &"pistol", "cooldown": 0.1, "automatic": true, "recoil": 0.24},
	{"scene": preload("knife.glb"), "name": "Knife", "stance": &"melee", "cooldown": 0.2, "automatic": false, "recoil": 0.0},
	{"scene": preload("bat/wooden_bat.tscn"), "name": "Wooden Bat", "stance": &"bat", "cooldown": 0.0, "automatic": false, "recoil": 0.0},
]
# Mounts are Skullguy grip/stow, then generic grip/stow.
const WEAPON_MOUNTS := {
	&"bat": [
		[Vector3(0.003, -0.058, 0.002), Vector3(2.18, 0, 1.02)],
		[Vector3(0.13, -0.30, 0.15), Vector3(0, 0, -0.5)],
		[Vector3(-0.005, 0.058, -0.002), Vector3(0.96, 0, -1.02)],
		[Vector3(-0.13, -0.30, -0.12), Vector3(0.23, 0, 0.38)],
	],
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
const WEAPON_STYLES := {
	&"pistol": HumanoidController3D.PISTOL,
	&"twohand": HumanoidController3D.LONG_GUN,
	&"melee": HumanoidController3D.KNIFE,
	&"bat": HumanoidController3D.BAT,
}

@onready var player: CharacterBody3D = $Player
@onready var rig = $Player/Humanoid
@onready var controller: HumanoidController3D = $Player/Controller
var use_skullguy := true
var weapon_index := 0
var weapon: Node3D
var appearance_rng := RandomNumberGenerator.new()

func _ready() -> void:
	controller.configure(player, rig)
	controller.aiming_changed.connect(_update_bat_grip)
	controller.fire.connect(_fire_weapon)
	controller.melee_attack.connect(_melee_attack)
	_select_weapon(0)
	appearance_rng.randomize()
	for pos in [Vector3(-4.5, 0, -3), Vector3(4.5, 0, -3), Vector3(0, 0, -5.5)]:
		_spawn_enemy(pos)

func _update_bat_grip(aiming: bool) -> void:
	rig.set_bat_grip(weapon if aiming and WEAPONS[weapon_index].stance == &"bat" else null)

func _select_weapon(index: int) -> void:
	if weapon and index == weapon_index:
		return
	var next_weapon := (WEAPONS[index].scene as PackedScene).instantiate() as Node3D
	if weapon:
		rig.set_bat_grip(null)
		weapon.get_parent().remove_child(weapon)
		weapon.queue_free()
	weapon_index = index
	weapon = next_weapon
	_mount_weapon()
	$Controls.text = "%s  |  1 Pistol  2 Revolver  3 Shotgun  4 SMG  5 Knife  6 Bat\nWASD Move  •  Arrows Aim  •  Space Attack  •  E Reload  •  Shift Jump  •  Tab Swap model  •  R Spawn enemy  •  B Blood preview" % WEAPONS[index].name

func _mount_weapon() -> void:
	var index := 0 if use_skullguy else 2
	var mounts: Array = WEAPON_MOUNTS[WEAPONS[weapon_index].stance]
	var grip: Array = mounts[index]
	var stow: Array = mounts[index + 1]
	rig.configure_aim_weapon(weapon,
		Transform3D(Basis.from_euler(grip[1]), grip[0]),
		Transform3D(Basis.from_euler(stow[1]), stow[0]))
	var setup: Dictionary = WEAPONS[weapon_index]
	controller.configure_weapon(weapon, WEAPON_STYLES[setup.stance],
		setup.cooldown, setup.automatic, setup.recoil, use_skullguy)
	_update_bat_grip(controller.is_aiming())

func _fire_weapon(direction: Vector3) -> void:
	var shotgun: bool = WEAPONS[weapon_index].stance == &"twohand"
	var muzzle := weapon.to_global(WEAPONS[weapon_index].muzzle)
	var origin := player.global_position + Vector3.UP * 1.05
	var spread := SHOTGUN_SPREAD if shotgun else [0.0]
	var damage: Dictionary[Node, int] = {}
	for angle in spread:
		var pellet_direction := direction.rotated(Vector3.UP, deg_to_rad(angle))
		var endpoint := origin + pellet_direction * (18.0 if shotgun else 24.0)
		var hit := _trace_attack(origin, endpoint)
		var enemy := _hit_target(hit)
		if enemy:
			damage[enemy] = damage.get(enemy, 0) + 1
		_spawn_tracer(muzzle, pellet_direction, hit.get("position", endpoint))
	for enemy in damage:
		enemy.take_hit(direction, damage[enemy])

func _spawn_tracer(muzzle: Vector3, direction: Vector3, hit_point: Vector3) -> void:
	# Keep the lower damage ray's hit depth without tilting the tracer down.
	var distance := maxf((hit_point - muzzle).dot(direction), 0.0)
	var tracer := BULLET_TRACER.instantiate()
	add_child(tracer)
	tracer.launch(muzzle, muzzle + direction * distance)

func _spawn_enemy(pos: Vector3) -> void:
	if $Enemies.get_child_count() >= MAX_ENEMIES:
		return
	var enemy := ENEMY_SCENE.instantiate()
	$Enemies.add_child(enemy)
	enemy.global_position = pos
	if not enemy.initialize(player, appearance_rng):
		enemy.queue_free()

func _trace_attack(origin: Vector3, endpoint: Vector3) -> Dictionary:
	var query := PhysicsRayQueryParameters3D.create(origin, endpoint, 3, [player.get_rid()])
	return get_world_3d().direct_space_state.intersect_ray(query)

func _hit_target(hit: Dictionary) -> Node:
	var enemy := hit.get("collider") as Node
	return enemy if enemy and enemy.is_in_group("demo_enemies") else null

func _melee_attack(direction: Vector3, style: int) -> void:
	var distance := BAT_REACH if style == HumanoidController3D.BAT else 1.8
	var origin := player.global_position + Vector3.UP * 1.05
	var enemy := _hit_target(_trace_attack(origin, origin + direction * distance))
	if enemy:
		enemy.take_hit(direction, 1 if style == HumanoidController3D.UNARMED else 2)

func _read_input() -> void:
	controller.set_input(
		Input.get_vector("move_left", "move_right", "move_up", "move_down"),
		Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down"),
		Input.is_physical_key_pressed(KEY_SPACE))

func _physics_process(delta: float) -> void:
	_read_input()
	controller.step(delta)

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo:
		return
	if event.physical_keycode >= KEY_1 and event.physical_keycode <= KEY_6:
		_select_weapon(event.physical_keycode - KEY_1)
		return
	match event.physical_keycode:
		KEY_TAB:
			var next_model := GENERIC if use_skullguy else SKULLGUY
			if rig.load_model(next_model, use_skullguy):
				use_skullguy = not use_skullguy
				_mount_weapon()
		KEY_SHIFT:
			controller.jump()
		KEY_SPACE:
			_read_input()
			controller.attack()
		KEY_E:
			_read_input()
			controller.reload()
		KEY_R:
			var spread := appearance_rng.randf_range(-0.8, 0.8)
			var spawn_position := player.global_position + controller.get_attack_direction().rotated(Vector3.UP, spread) * 5.0
			spawn_position.y = 0.0
			_spawn_enemy(spawn_position)
		KEY_B:
			$Player/BloodController.splatter(-rig.global_basis.z)
		KEY_ESCAPE:
			get_tree().quit()
