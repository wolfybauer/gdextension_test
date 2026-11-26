@tool
extends RigidBody3D

@export var health := 75
@export var burnable := false
@export var start_fire := false
var fire: FireComponent3D
var fracture: DestronoiNode
var broken := false

func _func_godot_apply_properties(properties: Dictionary) -> void:
	health = maxi(1, int(properties.get("health", 75)))
	mass = maxf(0.1, float(properties.get("mass", 5.0)))
	burnable = properties.get("classname") == "func_burnable"
	start_fire = bool(properties.get("start_fire", false))

func _ready() -> void:
	if Engine.is_editor_hint():
		return
	add_to_group("demo_burnable")
	add_to_group("fade_obj")
	fracture = DestronoiNode.new()
	add_child(fracture)
	if burnable:
		fire = FireComponent3D.new()
		fire.set_max_hp(12)
		add_child(fire)
		fire.damage.connect(func(amount: int) -> void: apply_damage(amount, Vector3.ZERO))
		if start_fire:
			ignite.call_deferred()

func ignite() -> void:
	if fire and not broken:
		fire.apply_fire(global_position, fire.get_max_hp())

func apply_damage(amount: int, direction: Vector3) -> void:
	if broken:
		return
	health -= amount
	if health <= 0:
		broken = true
		fracture.destroy(2.5, direction * 3.0)
	elif not direction.is_zero_approx():
		apply_central_impulse(direction * minf(amount * 0.12, 4.0))
