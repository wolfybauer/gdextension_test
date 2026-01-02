extends Node3D

@onready var camera_3d: Camera3D = $Camera3D
@onready var player: AnimatableBody3D = $Player
@onready var cam_toggle: CheckButton = $UI/CamToggle

func set_cam_persp():
	camera_3d.projection = Camera3D.PROJECTION_PERSPECTIVE
	camera_3d.global_position = Vector3(5.0,4.0,5.0)
	camera_3d.global_rotation_degrees = Vector3(-32.3,30.0,0.0)

func set_cam_iso():
	camera_3d.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera_3d.global_position = Vector3(7.0,10.0,10.0)
	camera_3d.global_rotation_degrees = Vector3(-32.3,30.0,0.0)
	camera_3d.size = 5
	

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	var tw:Tween = get_tree().create_tween()
	tw.tween_property(player, 'global_position', Vector3(5.0,1.0,-2.0), 1.0)
	tw.tween_property(player, 'global_position', Vector3(-5.0,1.0,-2.0), 1.0)
	tw.set_loops()
	
	cam_toggle.toggled.connect(func(e):
		if e: set_cam_persp()
		else: set_cam_iso()
	)
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	get_tree().call_group('fade_wall', 'check_fade', player, camera_3d, 2.0, 5.0, 0.25)
	get_tree().call_group('fade_wall_gd', 'check_fade', player, camera_3d, 2.0)

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed('ui_cancel'):
		get_tree().quit()
