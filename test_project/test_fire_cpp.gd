extends Node3D

@onready var real_convex: RigidBody3D = $RealConvex

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed("ui_accept"):
		real_convex.emit_signal("spread_fire", Vector3(10,randf(),-10), 20)
