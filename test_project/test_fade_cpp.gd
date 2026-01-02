extends Node3D

@onready var player: AnimatableBody3D = $Player
@onready var camera_3d: Camera3D = $Camera3D

@onready var cam_toggle: CheckButton = $UI/PanelContainer/HBoxContainer/CamToggle
@onready var pause_button: Button = $UI/PanelContainer/HBoxContainer/PauseButton

var tw:Tween
var paused:bool = false

func set_cam_persp():
	camera_3d.projection = Camera3D.PROJECTION_PERSPECTIVE
	camera_3d.global_position = Vector3(5.0,5.0,5.0)
	camera_3d.global_rotation_degrees = Vector3(-32.3,30.0,0.0)
	cam_toggle.text = "PERSP"

func set_cam_iso():
	camera_3d.projection = Camera3D.PROJECTION_ORTHOGONAL
	camera_3d.global_position = Vector3(7.0,10.0,10.0)
	camera_3d.global_rotation_degrees = Vector3(-32.3,30.0,0.0)
	camera_3d.size = 10
	cam_toggle.text = "ORTHO"

func toggle_paused():
	paused = not paused
	if paused:
		tw.pause()
		pause_button.text = "PLAY"
	else:
		tw.play()
		pause_button.text = "PAUSE"

# Called when the node enters the scene tree for the first time.
func _ready() -> void:
	tw = get_tree().create_tween()
	tw.tween_property(player, 'global_position', Vector3(5.0,0.0,-1.2), 1.0)
	tw.tween_property(player, 'global_position', Vector3(5.0,5.0,-1.2), 1.0)
	tw.tween_property(player, 'global_position', Vector3(-5.0,5.0,-1.2), 1.0)
	tw.tween_property(player, 'global_position', Vector3(-5.0,0.0,-1.2), 1.0)
	tw.set_loops()
	
	cam_toggle.toggled.connect(func(e):
		if e: set_cam_persp()
		else: set_cam_iso()
	)
	pause_button.pressed.connect(toggle_paused)
	set_cam_iso()


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta: float) -> void:
	get_tree().call_group('fade_wall', 'check_fade', player, camera_3d, 1.5, 5.0, 0.25)
	get_tree().call_group('fade_floor_gd', 'check_fade', player, camera_3d)
	get_tree().call_group('fade_wall_gd', 'check_fade', player, camera_3d, 2.0)

func _unhandled_input(_event: InputEvent) -> void:
	if Input.is_action_just_pressed('ui_cancel'):
		get_tree().quit()
