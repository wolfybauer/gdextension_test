extends RigidCharacterBody3D

const ROTATION_SPEED := 6.0 

@export var speed := 8.0
@export var damp:float = 10.0
@export var jump_height: float = 2.0

const SMOOTH_ACCELERATION_CURVE = preload("res://smooth_acceleration_curve.tres")

@onready var camera_pivot: Node3D = $CameraPivot
@onready var camera_3d: Camera3D = $CameraPivot/Camera3D

var direction:Vector3
var _wants_to_jump:bool = false
var _jump_released:bool = false

func _ready() -> void:
	mass = 60.0
	acceleration_magnitude = 50.0

func _input(event):
	if event.is_action_pressed("ui_cancel"): get_tree().quit()

	if event.is_action_pressed("ui_accept") and is_on_floor():
		_wants_to_jump = true
		_jump_released = false
		var velocity: float = sqrt(2.0 * get_gravity().length() * jump_height)
		apply_central_impulse(mass * velocity * up_direction)
	else:
		_wants_to_jump = false
	
	if event.is_action_released("ui_accept"):
		_jump_released = true

func _physics_process(_delta: float) -> void:
	linear_damp = 0.0

	# Get the input direction and handle the movement/deceleration.
	# As good practice, you should replace UI actions with custom gameplay actions.
	var input_dir := Input.get_vector("move_left", "move_right", "move_up", "move_down").rotated(-camera_3d.rotation.y)
	direction = (camera_pivot.basis * Vector3(input_dir.x, 0, input_dir.y)).normalized()
	if direction or _wants_to_jump:
		target_velocity.x = lerp(target_velocity.x, direction.x * speed, 0.1)
		target_velocity.z = lerp(target_velocity.z, direction.z * speed, 0.1)
		
		##now rotate the model
		#if direction:
			#rotate_model(direction, delta)
			#player_animation_state = animation_state.RUNNING
	else:
		if is_on_floor():
			linear_damp = damp
		target_velocity.x = 0.0
		target_velocity.z = 0.0
		#player_animation_state = animation_state.IDLE
	
	#if not is_on_floor():
		#player_animation_state = animation_state.JUMPING
	if _jump_released and linear_velocity.dot(up_direction) > 0.0:
		var lv:Vector3 = linear_velocity
		lv -= up_direction * linear_velocity.dot(up_direction) * 0.1
		linear_velocity = lv

	move_and_slide()

func _modify_move_force(move_force:Vector3) -> Vector3:
	var horizontal_velocity: Vector3 = Vector3(linear_velocity.x, 0.0, linear_velocity.z)
	var x_offset: float = ((horizontal_velocity.x * basis.x)).normalized().dot(
			(target_velocity.x * basis.x).normalized())
	var z_offset: float =((horizontal_velocity.z * basis.z)).normalized().dot(
			(target_velocity.z * basis.z).normalized())
	
	if x_offset < 0.0:
		move_force.x *= SMOOTH_ACCELERATION_CURVE.sample(x_offset)
	if z_offset < 0.0:
		move_force.z *= SMOOTH_ACCELERATION_CURVE.sample(z_offset)
	
	return move_force
