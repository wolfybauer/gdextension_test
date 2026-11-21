extends CharacterBody3D

const Appearance = preload("enemy_appearance.gd")
const REACH := 1.5

@onready var rig = $Humanoid
@onready var controller: HumanoidController3D = $Controller
@onready var blood: BloodController3D = $BloodController

var target: CharacterBody3D
var appearance: Appearance
var health := 3
var attack_cooldown := 0.0

func initialize(player: CharacterBody3D, rng: RandomNumberGenerator) -> bool:
	target = player
	appearance = Appearance.new()
	appearance.roll(rng)
	if not appearance.apply(rig):
		return false
	controller.configure(self, rig)
	controller.melee_attack.connect(_on_melee_attack)
	return true

func _physics_process(delta: float) -> void:
	if health <= 0 or not is_instance_valid(target):
		return
	var offset := target.global_position - global_position
	var horizontal := Vector2(offset.x, offset.z)
	var direction := horizontal.normalized()
	var in_range := horizontal.length() <= REACH and absf(offset.y) < 1.1
	controller.set_input(Vector2.ZERO if in_range else direction, direction)
	controller.step(delta)

	attack_cooldown = maxf(attack_cooldown - delta, 0.0)
	if in_range and attack_cooldown == 0.0 and controller.attack():
		attack_cooldown = 1.35

func _on_melee_attack(direction: Vector3, _style: int) -> void:
	if not is_instance_valid(target):
		return
	var offset := target.global_position - global_position
	if Vector2(offset.x, offset.z).length() <= REACH + 0.25 and absf(offset.y) < 1.1:
		target.get_node("BloodController").splatter(direction)

func take_hit(direction: Vector3, damage: int) -> void:
	if health <= 0:
		return
	health = maxi(0, health - damage)
	blood.splatter(direction)
	if health > 0:
		controller.play_reaction(&"damage0")
		return
	$CollisionShape3D.set_deferred("disabled", true)
	controller.play_reaction(&"death0")
	rig.play_legs(&"death0", 0.1)
	var torso: AnimationPlayer = rig.get_torso_player()
	var timeout := maxf(torso.current_animation_length + 0.4, 1.0)
	get_tree().create_timer(timeout, false).timeout.connect(queue_free)
