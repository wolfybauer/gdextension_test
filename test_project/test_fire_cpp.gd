extends Node3D

@onready var real_convex: RigidBody3D = $RealConvex

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed("ui_cancel"):
		get_tree().quit()
	if Input.is_action_just_pressed("ui_accept"):
		var pos:Vector3 = Vector3(randf_range(-10.0,10.0),randf_range(-10.0,10.0),randf_range(-10.0,10.0))
		var dam:int = randi_range(10,100)
		print('applying %d damage from world_pos=' % dam, pos)
		real_convex.emit_signal("spread_fire", pos, dam)
