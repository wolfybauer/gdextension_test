extends RigidBody3D

@export var health := 75

@onready var fire: FireComponent3D = $FireComponent
@onready var fracture: DestronoiNode = $Destronoi
var broken := false

func _ready() -> void:
	fire.damage.connect(func(amount: int) -> void: apply_damage(amount, Vector3.ZERO))

func ignite() -> void:
	if not broken:
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
