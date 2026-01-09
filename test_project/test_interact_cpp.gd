extends Node3D

@onready var player: RigidCharacterBody3D = $RigidCharacterBody3D
@onready var camera_3d: Camera3D = $RigidCharacterBody3D/CameraPivot/Camera3D

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta: float) -> void:
	FadeWall3D.check_fade_walls(get_tree(), player, camera_3d, 2.0, 5.0, 0.25)
