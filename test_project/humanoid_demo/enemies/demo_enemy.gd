extends RigidCharacterBody3D

const Loadout = preload("../demo_loadout.gd")
const Combat = preload("../demo_combat.gd")

# The first six profiles correspond to the demo loadout. Index six is unarmed.
const PROFILES := [
	{"range": 6.0, "spacing": 3.0, "interval": 1.35, "burst": 1},
	{"range": 8.0, "spacing": 4.5, "interval": 1.7, "burst": 1},
	{"range": 3.8, "spacing": 1.8, "interval": 1.85, "burst": 1},
	{"range": 5.5, "spacing": 2.5, "interval": 1.8, "burst": 3},
	{"range": 0.0, "spacing": 0.0, "interval": 1.1, "burst": 1},
	{"range": 0.0, "spacing": 0.0, "interval": 1.6, "burst": 1},
	{"range": 0.0, "spacing": 0.0, "interval": 1.25, "burst": 1},
]

@onready var rig = $Humanoid
@onready var controller: HumanoidController3D = $Controller
@onready var blood: BloodController3D = $BloodController

var target: PhysicsBody3D
var health := 3
var weapon: Node3D
var weapon_setup: Dictionary
var style := HumanoidController3D.UNARMED
var profile: Dictionary
var attack_cooldown := 0.8
var shots_left := 0
var aim_error := 0.0
var cadence := 1.0
var attack_interval_scale := 1.0
var hit_player: Callable
var spawn_tracer: Callable

func initialize(player: PhysicsBody3D, model: PackedScene, rng: RandomNumberGenerator,
		on_player_hit: Callable, on_tracer: Callable) -> bool:
	target = player
	hit_player = on_player_hit
	spawn_tracer = on_tracer
	if not model or not rig.load_model(model, true):
		return false
	controller.configure(self, rig)
	impact_damage.connect(_on_impact_damage)
	controller.melee_attack.connect(_on_melee_attack)
	controller.fire.connect(_on_fire)
	var index := rng.randi_range(0, PROFILES.size() - 1)
	profile = PROFILES[index]
	aim_error = rng.randf_range(8.0, 16.0)
	cadence = rng.randf_range(0.9, 1.25)
	controller.set_aim_error(aim_error)
	attack_cooldown = rng.randf_range(0.8, 1.5)
	if index < PROFILES.size() - 1:
		weapon_setup = Loadout.WEAPONS[index]
		style = Loadout.WEAPON_STYLES[weapon_setup.stance]
		weapon = weapon_setup.scene.instantiate()
		var grip_profile := Loadout.read_grip_profile(model, weapon_setup, true)
		Loadout.mount_weapon(rig, controller, weapon, weapon_setup, true, grip_profile)
		if style == HumanoidController3D.BAT:
			controller.weapon_equipped_changed.connect(func(equipped: bool) -> void:
				if equipped:
					Loadout.apply_support_grip(rig, weapon, grip_profile, &"bat")
				else:
					rig.set_bat_grip(null))
	return true

func set_difficulty(aim_error_scale: float, interval_scale: float) -> void:
	controller.set_aim_error(aim_error * aim_error_scale)
	attack_interval_scale = interval_scale

func _physics_process(delta: float) -> void:
	if global_position.y < -12.0:
		queue_free()
		return
	if health <= 0:
		controller.step(delta, false)
		return
	if not is_instance_valid(target):
		controller.set_input(Vector2.ZERO, Vector2.ZERO)
		controller.step(delta)
		return
	var offset := target.global_position - global_position
	var horizontal := Vector2(offset.x, offset.z)
	var distance := horizontal.length()
	var direction := horizontal.normalized()
	var ranged: bool = profile.range > 0.0
	var reach: float = profile.range if ranged else Combat.melee_reach(style)
	var can_attack := distance <= reach and absf(offset.y) < 1.1
	if can_attack and ranged:
		var height := Vector3.UP * 1.05
		can_attack = Combat.trace(self, global_position + height,
			target.global_position + height).get("collider") == target
	var move := direction
	if can_attack:
		move = -direction * 0.6 if ranged and distance < profile.spacing else Vector2.ZERO
	controller.set_input(move, direction if can_attack else Vector2.ZERO)
	controller.step(delta)

	attack_cooldown = maxf(attack_cooldown - delta, 0.0)
	if not can_attack:
		shots_left = 0
		return
	if attack_cooldown == 0.0 and controller.attack():
		if shots_left == 0:
			shots_left = profile.burst
		shots_left -= 1
		attack_cooldown = 0.13 if shots_left > 0 else profile.interval * cadence * attack_interval_scale

func _on_melee_attack(direction: Vector3, impact_style: int) -> void:
	var hits := Combat.melee_targets(self, direction, impact_style, 1)
	if is_instance_valid(target) and hits.has(target):
		hit_player.call(hits[target], Combat.melee_damage(impact_style))

func _on_fire(direction: Vector3) -> void:
	var muzzle := weapon.to_global(weapon_setup.muzzle)
	var origin := global_position + Vector3.UP * 1.05
	var spread := Combat.SHOTGUN_SPREAD if style == HumanoidController3D.LONG_GUN else [0.0]
	var hits := 0
	for angle in spread:
		var pellet_direction := direction.rotated(Vector3.UP, deg_to_rad(angle))
		var end := origin + pellet_direction * float(profile.range)
		var impact := Combat.trace(self, origin, end)
		spawn_tracer.call(muzzle, pellet_direction, impact.get("position", end))
		if is_instance_valid(target) and impact.get("collider") == target:
			hits += 1
	if hits > 0:
		hit_player.call(direction, hits)

func _on_impact_damage(damage: int, direction: Vector3, _landing: bool) -> void:
	take_hit(direction, damage, false)

func take_hit(direction: Vector3, damage: int, apply_impulse := true) -> void:
	if health <= 0 or damage <= 0:
		return
	health = maxi(0, health - damage)
	blood.splatter(direction)
	if apply_impulse:
		controller.apply_knockback(direction, minf(3.0 + damage, 7.0))
	if health > 0:
		controller.play_reaction(&"damage0")
		return
	# Keep the capsule on the floor, but let living actors and attacks pass through.
	collision_layer = 0
	collision_mask = 1
	if is_instance_valid(target):
		add_collision_exception_with(target)
	var timeout := maxf(controller.play_defeat() + 0.4, 1.0)
	get_tree().create_timer(timeout, false).timeout.connect(queue_free)
