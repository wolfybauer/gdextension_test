extends Node3D

const ENEMY_SCENE = preload("enemies/demo_enemy.tscn")
const BULLET_TRACER = preload("bullet_tracer.tscn")
const MAX_ENEMIES := 8
const Combat = preload("demo_combat.gd")
const Loadout = preload("demo_loadout.gd")
const MODELS = Loadout.MODELS
const Throw = preload("demo_throw.gd")
const Interactions = preload("demo_interactions.gd")
const Yard = preload("demo_yard.gd")
const LEVEL_SCENES := [
	"res://humanoid_demo/humanoid_demo.tscn",
	"res://humanoid_demo/level_solstice.tscn",
	"res://humanoid_demo/level_wicked_night.tscn",
	"res://humanoid_demo/level_mechanics_workshop.tscn",
]
const MAP_CUTAWAY_SHADER = preload("res://trenchbroom/map_cutaway.gdshader")
const MAP_CAMERA_HEIGHT := 0.95
const MAP_CAMERA_DISTANCE := 7.0
const MAP_CAMERA_CLEARANCE := 0.25
const PLAYER_MAX_HEALTH := 12
const PSX_POOL := "res://humanoid_demo/psx_characters"

@export var imported_level: PackedScene
@export var level_index := 0
@export_range(0.0, 2.0, 0.05) var imported_ambient_energy := 0.75

@export_range(0.0, 3.0, 0.05) var enemy_aim_error_scale := 1.0:
	set(value):
		enemy_aim_error_scale = value
		_update_enemy_difficulty()
@export_range(0.25, 3.0, 0.05) var enemy_attack_interval_scale := 1.2:
	set(value):
		enemy_attack_interval_scale = value
		_update_enemy_difficulty()
@export_range(0.0, 2.0, 0.05, "suffix:s") var player_hit_recovery := 0.4

@onready var player: RigidCharacterBody3D = $Player
@onready var rig = $Player/Humanoid
@onready var camera: Camera3D = $Player/Camera3D
@onready var fade_target: Marker3D = $Player/FadeTarget
@onready var spawn_transform: Transform3D = player.global_transform
@onready var controller: HumanoidController3D = $Player/Controller
var model_index := 0
var weapon_index := 0
var weapon: Node3D
var grip_profile: Dictionary
var throw_preview := false
var pickup_target: RigidBody3D
var throws: Throw
var interactions: Interactions
var appearance_rng := RandomNumberGenerator.new()
var enemy_models: Array[String] = []
var yard: Yard
var player_health := PLAYER_MAX_HEALTH
var last_damage := ""
var map_camera_distance := MAP_CAMERA_DISTANCE

func _ready() -> void:
	controller.configure(player, rig)
	player.impact_damage.connect(_on_player_impact)
	if imported_level:
		var original := $Level
		remove_child(original)
		original.queue_free()
		var map := imported_level.instantiate()
		add_child(map)
		_setup_imported_camera()
		_soften_map_cutaways(map)
		var marker := get_tree().get_first_node_in_group("map_player_start") as Node3D
		if marker:
			spawn_transform.origin = marker.global_position - Vector3.UP * 0.5
			_place_player_at_spawn()
	else:
		yard = Yard.new()
		add_child(yard)
	controller.weapon_equipped_changed.connect(_update_weapon_grip)
	controller.fire.connect(_fire_weapon)
	controller.melee_attack.connect(_melee_attack)
	controller.item_released.connect(_release_item)
	controller.interaction_contact.connect(_interaction_contact)
	throws = Throw.new()
	add_child(throws)
	throws.exploded.connect(_blast)
	if yard:
		interactions = Interactions.new()
		add_child(interactions)
		interactions.ignition_requested.connect(yard.ignite_crates)
	_select_weapon(0)
	_update_health()
	appearance_rng.randomize()
	_collect_models(PSX_POOL)
	if imported_level:
		var markers := get_tree().get_nodes_in_group("map_enemy_start")
		markers.sort_custom(func(a: Node3D, b: Node3D) -> bool:
			return a.global_position.distance_squared_to(spawn_transform.origin) < b.global_position.distance_squared_to(spawn_transform.origin))
		for marker in markers.slice(0, 3):
			_spawn_enemy(marker.global_position - Vector3.UP * 0.5)
	else:
		for pos in [Vector3(2.0, 0.05, -5.0), Vector3(6.2, 0.05, -5.8), Vector3(10.0, 0.05, -4.5)]:
			_spawn_enemy(pos)

func _update_weapon_grip(equipped: bool) -> void:
	var stance: StringName = Loadout.WEAPONS[weapon_index].stance
	if equipped and not controller.is_throwing() and (stance == &"bat" or controller.is_smashing()):
		Loadout.apply_support_grip(rig, weapon, grip_profile, stance)
	else:
		rig.set_bat_grip(null)

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
	$Controls.text = "%s  •  %s  |  1 Pistol  2 Revolver  3 Shotgun  4 SMG  5 Knife  6 Bat  7 Grenade\nWASD Move  •  Arrows Aim  •  Space Attack  •  E Reload  •  Shift Jump  •  Tab Swap model  •  R Spawn enemy  •  Hold/release G Throw  •  F Interact  •  K Grip tuner  •  F9 Change level%s" % [MODELS[model_index].name, Loadout.WEAPONS[weapon_index].name if weapon else "Unarmed", "  •  H Hazards" if yard else ""]

func _update_health() -> void:
	$Health.text = "Health %d / %d%s" % [player_health, PLAYER_MAX_HEALTH, last_damage]

func _mount_weapon() -> void:
	var positive_z: bool = MODELS[model_index].positive_z
	if weapon:
		var setup: Dictionary = Loadout.WEAPONS[weapon_index]
		grip_profile = Loadout.read_grip_profile(rig.model_scene, setup, positive_z)
		Loadout.mount_weapon(rig, controller, weapon, setup, positive_z, grip_profile)
	_update_weapon_grip(controller.is_weapon_equipped())

func _release_item(item: Node3D, direction: Vector3) -> void:
	var slot := weapon_index
	weapon = null
	throws.launch(player, item, direction, slot, Loadout.WEAPONS[slot].stance == &"carry")
	_update_controls()

func _blast(center: Vector3) -> void:
	for actor in $Enemies.get_children() + [player] + get_tree().get_nodes_in_group("demo_burnable"):
		var crate: bool = actor.is_in_group("demo_burnable")
		var target: Vector3 = actor.global_position + (Vector3.ZERO if crate else Vector3.UP * 0.85)
		if target.distance_to(center) > 2.2:
			continue
		var cover := PhysicsRayQueryParameters3D.create(center, target, 1, [actor.get_rid()])
		if not get_world_3d().direct_space_state.intersect_ray(cover).is_empty():
			continue
		var outward := (target - center).normalized()
		if crate:
			actor.ignite()
			actor.apply_damage(35, outward)
		elif actor == player:
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
	elif interactions:
		interactions.contact(clip, player)

func _interact() -> void:
	var item := throws.nearest_item(player)
	if item and controller.interact(&"pickup0", item.global_position - player.global_position):
		pickup_target = item
	elif not item and interactions:
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
		var target := hit.get("collider") as Node
		if target and (target.is_in_group("demo_enemies") or target.is_in_group("demo_burnable")):
			damage[target] = damage.get(target, 0) + 1
		_spawn_tracer(muzzle, pellet_direction, hit.get("position", endpoint))
	for target in damage:
		if target.is_in_group("demo_burnable"):
			target.apply_damage(damage[target] * 12, direction)
		else:
			target.take_hit(direction, damage[target])

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
	enemy.position = pos
	$Enemies.add_child(enemy)
	var model := load(enemy_models[appearance_rng.randi_range(0, enemy_models.size() - 1)]) as PackedScene
	if enemy.initialize(player, model, appearance_rng, _hit_player, _spawn_tracer):
		enemy.set_difficulty(enemy_aim_error_scale, enemy_attack_interval_scale)
	else:
		enemy.queue_free()

func _update_enemy_difficulty() -> void:
	if is_node_ready():
		for enemy in $Enemies.get_children():
			enemy.set_difficulty(enemy_aim_error_scale, enemy_attack_interval_scale)

func _on_player_impact(damage: int, direction: Vector3, landing: bool) -> void:
	_damage_player(direction, damage, 0.0, "Hard landing" if landing else "Prop impact")

func _hit_player(direction: Vector3, damage: int) -> void:
	_damage_player(direction, damage, minf(2.5 + damage * 0.5, 4.5), "Combat hit")

func _damage_player(direction: Vector3, damage: int, knockback: float, cause: String) -> void:
	if player_health <= 0 or damage <= 0:
		return
	player_health = maxi(player_health - damage, 0)
	last_damage = "  •  %s −%d" % [cause, damage]
	_update_health()
	$Player/BloodController.splatter(direction)
	if player_health > 0:
		controller.react_to_hit(direction, knockback, player_hit_recovery)
	else:
		throw_preview = false
		var duration := controller.play_defeat()
		get_tree().create_timer(maxf(duration, 1.0), false).timeout.connect(_respawn)

func _place_player_at_spawn() -> void:
	PhysicsServer3D.body_set_state(player.get_rid(), PhysicsServer3D.BODY_STATE_TRANSFORM, spawn_transform)
	player.global_transform = spawn_transform
	player.linear_velocity = Vector3.ZERO

func _respawn() -> void:
	_place_player_at_spawn()
	player_health = PLAYER_MAX_HEALTH
	last_damage = ""
	_update_health()

func _melee_attack(direction: Vector3, style: int) -> void:
	var hits := Combat.melee_targets(player, direction, style, 3)
	for target in hits:
		if target.is_in_group("demo_enemies"):
			target.take_hit(hits[target], Combat.melee_damage(style))
		elif target.is_in_group("demo_burnable"):
			target.apply_damage(Combat.melee_damage(style) * 20, hits[target])

func _read_input() -> void:
	var right := camera.global_basis.x
	var ground_right := Vector2(right.x, right.z).normalized()
	var ground_down := Vector2(-ground_right.y, ground_right.x)
	var move := Input.get_vector("move_left", "move_right", "move_up", "move_down")
	var aim := Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down")
	controller.set_input(
		ground_right * move.x + ground_down * move.y,
		ground_right * aim.x + ground_down * aim.y,
		Input.is_physical_key_pressed(KEY_SPACE) and not throw_preview)

func _setup_imported_camera() -> void:
	var atmosphere := Environment.new()
	atmosphere.background_mode = Environment.BG_COLOR
	atmosphere.background_color = Color(0.075, 0.09, 0.12)
	atmosphere.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	atmosphere.ambient_light_color = Color(0.92, 0.95, 1.0)
	atmosphere.ambient_light_energy = imported_ambient_energy
	camera.environment = atmosphere
	camera.position = Vector3.UP * MAP_CAMERA_HEIGHT + camera.basis.z * map_camera_distance

func _soften_map_cutaways(map: Node) -> void:
	var copies: Dictionary[Material, ShaderMaterial] = {}
	for child in map.find_children("*", "MeshInstance3D", true, false):
		var mesh := child as MeshInstance3D
		if mesh.mesh == null:
			continue
		for surface in mesh.mesh.get_surface_count():
			var material := mesh.get_active_material(surface)
			if material is ShaderMaterial and mesh.get_parent() is FadeWall3D:
				material.set_shader_parameter("camera_cutaway", true)
			elif material is BaseMaterial3D:
				if not copies.has(material):
					var copy := ShaderMaterial.new()
					copy.shader = MAP_CUTAWAY_SHADER
					copy.set_shader_parameter("albedo_color", material.albedo_color)
					copy.set_shader_parameter("roughness", material.roughness)
					copy.set_shader_parameter("use_texture", material.albedo_texture != null)
					if material.albedo_texture:
						copy.set_shader_parameter("albedo_texture", material.albedo_texture)
					copies[material] = copy
				mesh.set_surface_override_material(surface, copies[material])

func _update_imported_camera(delta: float) -> void:
	var focus := player.to_global(Vector3.UP * MAP_CAMERA_HEIGHT)
	var axis := camera.global_basis.z.normalized()
	var query := PhysicsRayQueryParameters3D.create(
		focus, focus + axis * MAP_CAMERA_DISTANCE, 1, [player.get_rid()])
	var hit := get_world_3d().direct_space_state.intersect_ray(query)
	var available := MAP_CAMERA_DISTANCE
	if not hit.is_empty():
		available = maxf(camera.near + 0.02, focus.distance_to(hit.position) - MAP_CAMERA_CLEARANCE)
	# Moving along the viewing axis preserves the orthographic framing and input directions.
	map_camera_distance = minf(available, move_toward(map_camera_distance, available, 10.0 * delta))
	camera.position = Vector3.UP * MAP_CAMERA_HEIGHT + camera.basis.z * map_camera_distance

func _physics_process(delta: float) -> void:
	if imported_level:
		_update_imported_camera(delta)
	var tree := get_tree()
	FadeWall3D.check_fade_walls(tree, fade_target, camera, 5.0, 5.0, 0.25)
	FadeFloor3D.check_fade_floors(tree, player, 2.0)
	FadeFloor3D.check_fade_objects(tree, player, 2.0)
	if player_health > 0 and player.global_position.y < -12.0:
		_respawn()
	if player_health <= 0:
		throws.clear_preview()
		controller.step(delta, false)
		return
	_read_input()
	controller.step(delta)
	_update_weapon_grip(controller.is_weapon_equipped())
	if throw_preview and weapon:
		throws.preview(player, controller.get_attack_direction())
	else:
		throws.clear_preview()

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or event.echo:
		return
	if event.pressed and event.physical_keycode == KEY_F9:
		get_tree().change_scene_to_file(LEVEL_SCENES[(level_index + 1) % LEVEL_SCENES.size()])
		return
	if event.pressed and event.physical_keycode == KEY_K:
		get_tree().change_scene_to_file("res://humanoid_demo/grip_tuner.tscn")
		return
	if event.pressed and event.physical_keycode == KEY_H and yard:
		yard.release_hazards()
		return
	if player_health <= 0:
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
			var floor_hit := get_world_3d().direct_space_state.intersect_ray(
				PhysicsRayQueryParameters3D.create(spawn_position + Vector3.UP * 1.4, spawn_position - Vector3.UP * 12.0, 1, [player.get_rid()]))
			if not floor_hit.is_empty() and floor_hit.normal.y > 0.65:
				_spawn_enemy(floor_hit.position + Vector3.UP * 0.06)
		KEY_B:
			$Player/BloodController.splatter(-rig.global_basis.z)
		KEY_ESCAPE:
			get_tree().quit()
