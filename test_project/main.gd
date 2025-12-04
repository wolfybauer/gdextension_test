extends Node3D

@onready var cpp: DestronoiNode = $BoxCPP/DestronoiNode
@onready var gd: DestronoiGD = $BoxGD/DestronoiGD

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed('ui_cancel'):
		get_tree().quit()
	if Input.is_action_just_pressed('ui_accept'):
		if cpp:
			cpp.destroy(5, 5, 6.0)
		if gd:
			gd.destroy(5, 5, 6.0)

func _ready() -> void:
	cpp.generate()
	
	
