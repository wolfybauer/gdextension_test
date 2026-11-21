extends CharacterBody3D

const Appearance = preload("enemy_appearance.gd")
const SPEED := 2.3
const REACH := 1.5

@onready var rig = $Humanoid
@onready var blood: BloodController3D = $BloodController

var target: CharacterBody3D
var appearance: Appearance
var health := 3
var windup := 0.0
var attack_cooldown := 0.0
var attacking := false

func initialize(player: CharacterBody3D, rng: RandomNumberGenerator) -> bool:
	target = player
	appearance = Appearance.new()
	appearance.roll(rng)
	if not appearance.apply(rig):
		return false
	rig.torso_animation_finished.connect(func(_clip): attacking = false)
	rig.play_torso(&"idle_unarmed")
	rig.play_legs(&"idle_unarmed")
	return true

func _physics_process(delta: float) -> void:
	if health <= 0 or not is_instance_valid(target):
		return
	var offset := target.global_position - global_position
	var horizontal := Vector2(offset.x, offset.z)
	var distance := horizontal.length()
	var direction := horizontal.normalized()
	if distance > 0.01:
		rig.rotation.y = lerp_angle(rig.rotation.y, atan2(-direction.x, -direction.y), 1.0 - exp(-10.0 * delta))
	var closing_in := distance > REACH and not attacking
	velocity.x = direction.x * SPEED if closing_in else 0.0
	velocity.z = direction.y * SPEED if closing_in else 0.0
	if not is_on_floor():
		velocity += get_gravity() * delta
	move_and_slide()

	var locomotion := &"fwd_unarmed" if closing_in else &"idle_unarmed"
	if rig.get_legs_player().current_animation != StringName("main/" + String(locomotion)):
		rig.play_legs(locomotion, 0.15)
	if not attacking and rig.get_torso_player().current_animation != StringName("main/" + String(locomotion)):
		rig.play_torso(locomotion, 0.15)

	attack_cooldown = maxf(attack_cooldown - delta, 0.0)
	if attacking and windup > 0.0:
		windup = maxf(windup - delta, 0.0)
		if windup == 0.0 and distance <= REACH + 0.25 and absf(offset.y) < 1.1:
			target.get_node("BloodController").splatter(Vector3(direction.x, 0, direction.y))
	elif not attacking and attack_cooldown == 0.0 and distance <= REACH and absf(offset.y) < 1.1:
		attacking = rig.play_torso(&"punch0", 0.08)
		windup = 0.18
		attack_cooldown = 1.35

func take_hit(direction: Vector3, damage: int) -> void:
	if health <= 0:
		return
	health = maxi(0, health - damage)
	blood.splatter(direction)
	windup = 0.0
	if health > 0:
		attacking = rig.play_torso(&"damage0", 0.08)
		return
	$CollisionShape3D.set_deferred("disabled", true)
	rig.play_torso(&"death0", 0.1)
	rig.play_legs(&"death0", 0.1)
	var torso: AnimationPlayer = rig.get_torso_player()
	var timeout := maxf(torso.current_animation_length + 0.4, 1.0)
	get_tree().create_timer(timeout, false).timeout.connect(queue_free)
