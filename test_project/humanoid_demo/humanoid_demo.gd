extends Node3D

const MODELS := [
	{"scene": preload("psx_characters/extras/male/Character_Sheriff_N.fbx"), "name": "Sheriff", "positive_z": true},
	{"scene": preload("generic_guy.glb"), "name": "Generic", "positive_z": true},
	{"scene": preload("skullguy1.glb"), "name": "Skullguy", "positive_z": false},
]
const ENEMY_SCENE = preload("enemies/demo_enemy.tscn")
const BULLET_TRACER = preload("bullet_tracer.tscn")
const MAX_ENEMIES := 8
const Combat = preload("demo_combat.gd")
const Loadout = preload("demo_loadout.gd")
const Throw = preload("demo_throw.gd")
const Interactions = preload("demo_interactions.gd")
const PSX_POOL := "res://humanoid_demo/psx_characters"

@export_range(0.0, 3.0, 0.05) var enemy_aim_error_scale := 1.0:
	set(value):
		enemy_aim_error_scale = value
		_update_enemy_difficulty()
@export_range(0.25, 3.0, 0.05) var enemy_attack_interval_scale := 1.2:
	set(value):
		enemy_attack_interval_scale = value
		_update_enemy_difficulty()
@export_range(0.0, 2.0, 0.05, "suffix:s") var player_hit_recovery := 0.4

@onready var player: CharacterBody3D = $Player
@onready var rig = $Player/Humanoid
@onready var controller: HumanoidController3D = $Player/Controller
var model_index := 0
var weapon_index := 0
var weapon: Node3D
var throw_preview := false
var pickup_target: RigidBody3D
var throws: Throw
var interactions: Interactions
var appearance_rng := RandomNumberGenerator.new()
var enemy_models: Array[String] = []

func _ready() -> void:
	controller.configure(player, rig)
	controller.aiming_changed.connect(_update_bat_grip)
	controller.fire.connect(_fire_weapon)
	controller.melee_attack.connect(_melee_attack)
	controller.item_released.connect(_release_item)
	controller.interaction_contact.connect(_interaction_contact)
	throws = Throw.new()
	add_child(throws)
	throws.exploded.connect(_blast)
	interactions = Interactions.new()
	add_child(interactions)
	_select_weapon(0)
	appearance_rng.randomize()
	_collect_models(PSX_POOL)
	for pos in [Vector3(-4.5, 0, -3), Vector3(4.5, 0, -3), Vector3(0, 0, -5.5)]:
		_spawn_enemy(pos)

func _update_bat_grip(aiming: bool) -> void:
	rig.set_bat_grip(weapon if aiming and not controller.is_throwing() and Loadout.WEAPONS[weapon_index].stance == &"bat" else null)

func _select_weapon(index: int) -> void:
	if controller.is_throwing() or (weapon and index == weapon_index):
		return
	var next_weapon := (Loadout.WEAPONS[index].scene as PackedScene).instantiate() as Node3D
	_equip(next_weapon, index)

func _equip(next_weapon: Node3D, index: int) -> void:
	if weapon:
		rig.set_bat_grip(null)
		weapon.get_parent().remove_child(weapon)
		weapon.queue_free()
	weapon_index = index
	weapon = next_weapon
	_mount_weapon()
	_update_controls()

func _update_controls() -> void:
	$Controls.text = "%s  •  %s  |  1 Pistol  2 Revolver  3 Shotgun  4 SMG  5 Knife  6 Bat  7 Grenade\nWASD Move  •  Arrows Aim  •  Space Attack  •  E Reload  •  Shift Jump  •  Tab Swap model  •  R Spawn enemy  •  Hold/release G Throw  •  F Interact" % [MODELS[model_index].name, Loadout.WEAPONS[weapon_index].name if weapon else "Unarmed"]

func _mount_weapon() -> void:
	var positive_z: bool = MODELS[model_index].positive_z
	if weapon:
		Loadout.mount_weapon(rig, controller, weapon, Loadout.WEAPONS[weapon_index], positive_z)
	_update_bat_grip(controller.is_aiming())

func _release_item(item: Node3D, direction: Vector3) -> void:
	var slot := weapon_index
	weapon = null
	throws.launch(player, item, direction, slot, Loadout.WEAPONS[slot].stance == &"carry")
	_update_controls()

func _blast(center: Vector3) -> void:
	for actor in $Enemies.get_children() + [player]:
		var target: Vector3 = actor.global_position + Vector3.UP * 0.85
		if target.distance_to(center) > 2.2:
			continue
		var cover := PhysicsRayQueryParameters3D.create(center, target, 1, [actor.get_rid()])
		if not get_world_3d().direct_space_state.intersect_ray(cover).is_empty():
			continue
		var outward := (target - center).normalized()
		if actor == player:
			_hit_player(outward, 3)
		else:
			actor.take_hit(outward, 3)

func _interaction_contact(clip: StringName) -> void:
	if clip == &"pickup0" and is_instance_valid(pickup_target):
		var item := pickup_target.get_child(1) as Node3D
		var slot: int = pickup_target.get_meta(&"slot")
		pickup_target.queue_free()
		pickup_target = null
		_equip(item, slot)
	else:
		interactions.contact(clip, player)

func _interact() -> void:
	var item := throws.nearest_item(player)
	if item and controller.interact(&"pickup0", item.global_position - player.global_position):
		pickup_target = item
	elif not item:
		interactions.interact(player, controller)

func _fire_weapon(direction: Vector3) -> void:
	var shotgun: bool = Loadout.WEAPONS[weapon_index].stance == &"twohand"
	var muzzle := weapon.to_global(Loadout.WEAPONS[weapon_index].muzzle)
	var origin := player.global_position + Vector3.UP * 1.05
	var spread := Combat.SHOTGUN_SPREAD if shotgun else [0.0]
	var damage: Dictionary[Node, int] = {}
	for angle in spread:
		var pellet_direction := direction.rotated(Vector3.UP, deg_to_rad(angle))
		var endpoint := origin + pellet_direction * (18.0 if shotgun else 24.0)
		var hit := Combat.trace(player, origin, endpoint)
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

func _collect_models(path: String) -> void:
	for entry in ResourceLoader.list_directory(path):
		if entry.ends_with("/"):
			_collect_models(path.path_join(entry.trim_suffix("/")))
		elif entry.ends_with(".fbx"):
			enemy_models.append(path.path_join(entry))

func _spawn_enemy(pos: Vector3) -> void:
	if enemy_models.is_empty() or $Enemies.get_child_count() >= MAX_ENEMIES:
		return
	var enemy := ENEMY_SCENE.instantiate()
	$Enemies.add_child(enemy)
	enemy.global_position = pos
	var model := load(enemy_models[appearance_rng.randi_range(0, enemy_models.size() - 1)]) as PackedScene
	if enemy.initialize(player, model, appearance_rng, _hit_player, _spawn_tracer):
		enemy.set_difficulty(enemy_aim_error_scale, enemy_attack_interval_scale)
	else:
		enemy.queue_free()

func _update_enemy_difficulty() -> void:
	if is_node_ready():
		for enemy in $Enemies.get_children():
			enemy.set_difficulty(enemy_aim_error_scale, enemy_attack_interval_scale)

func _hit_player(direction: Vector3, damage: int) -> void:
	controller.react_to_hit(direction, minf(2.5 + damage * 0.5, 4.5), player_hit_recovery)
	$Player/BloodController.splatter(direction)

func _hit_target(hit: Dictionary) -> Node:
	var enemy := hit.get("collider") as Node
	return enemy if enemy and enemy.is_in_group("demo_enemies") else null

func _melee_attack(direction: Vector3, style: int) -> void:
	var hits := Combat.melee_targets(player, direction, style, 2)
	for target in hits:
		if target.is_in_group("demo_enemies"):
			target.take_hit(hits[target], 1 if style == HumanoidController3D.UNARMED else 2)

func _read_input() -> void:
	controller.set_input(
		Input.get_vector("move_left", "move_right", "move_up", "move_down"),
		Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down"),
		Input.is_physical_key_pressed(KEY_SPACE) and not throw_preview)

func _physics_process(delta: float) -> void:
	_read_input()
	var was_throwing := controller.is_throwing()
	controller.step(delta)
	if was_throwing and not controller.is_throwing():
		_update_bat_grip(controller.is_aiming())
	if throw_preview and weapon:
		throws.preview(player, controller.get_attack_direction())
	else:
		throws.clear_preview()

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or event.echo:
		return
	if event.physical_keycode == KEY_G:
		if not event.pressed and throw_preview and weapon:
			_read_input()
			if controller.throw_item(weapon):
				rig.set_bat_grip(null)
		throw_preview = event.pressed and weapon != null and not controller.is_throwing()
		return
	if not event.pressed:
		return
	if event.physical_keycode >= KEY_1 and event.physical_keycode <= KEY_7:
		_select_weapon(event.physical_keycode - KEY_1)
		return
	match event.physical_keycode:
		KEY_F:
			_interact()
		KEY_TAB:
			if controller.is_throwing():
				return
			var next_index := (model_index + 1) % MODELS.size()
			var next_model: Dictionary = MODELS[next_index]
			if rig.load_model(next_model.scene, next_model.positive_z):
				model_index = next_index
				_mount_weapon()
				_update_controls()
		KEY_SHIFT:
			controller.jump()
		KEY_SPACE:
			_read_input()
			if not throw_preview:
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
