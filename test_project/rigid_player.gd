extends RigidCharacterBody3D

const ROTATION_SPEED:float = 6.0 
const SMOOTH_ACCELERATION_CURVE = preload("res://smooth_acceleration_curve.tres")

@export var speed:float = 8.0
@export var damp:float = 15.0
@export var jump_height:float = 2.0
@export_range(0.05, 0.6, 0.05) var jump_cut:float = 0.8
@export_range(1.0, 20.0, 1.0) var rotation_speed:float = 7.0

@onready var camera_pivot: Node3D = $CameraPivot
@onready var camera_3d: Camera3D = $CameraPivot/Camera3D
@onready var playermodel : Node3D = $playermodel
@onready var animation_player : AnimationPlayer = $"playermodel/character-male-e2/AnimationPlayer"
@onready var ddraw:DDraw = DDraw.new()

var direction:Vector3
var aim_direction:Vector3
var _wants_to_jump:bool = false
var _jump_released:bool = false

enum animation_state {IDLE,RUNNING,JUMPING}
var player_animation_state : animation_state = animation_state.IDLE

#### GRABBING ####
@export var grab_distance: float = 2.0
@export var grab_strength: float = 120.0
@export var grab_damping: float = 18.0

var grabbed_body: RigidBody3D = null
var grab_local_offset: Vector3 = Vector3.ZERO
var _trying_grab:bool = false
var _grab_dir: Vector3
var _ray_from:Vector3
var _ray_to:Vector3
#### ####

func _ready() -> void:
	add_child(ddraw)
	mass = 60.0
	acceleration_magnitude = 50.0

func _process(_delta: float) -> void:
	if _trying_grab:
		ddraw.draw_line(_ray_from, _ray_to)

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
	

	if event.is_action_pressed("grab"):
		_trying_grab = true
		try_grab()

	if event.is_action_released("grab"):
		_trying_grab = false
		grabbed_body = null


func _physics_process(delta: float) -> void:
	linear_damp = 0.0
	
	_grab_dir = -playermodel.basis.z.normalized()
	_ray_from = playermodel.global_transform.origin + up_direction * neck_height
	_ray_to = _ray_from + _grab_dir * grab_distance

	# Get the input direction and handle the movement/deceleration.
	# As good practice, you should replace UI actions with custom gameplay actions.
	var input_dir:Vector2 = Input.get_vector("move_left", "move_right", "move_up", "move_down").rotated(-camera_3d.rotation.y)
	var aim_dir:Vector2 = Input.get_vector("ui_left", "ui_right", "ui_up", "ui_down").rotated(-camera_3d.rotation.y)
	direction = (camera_pivot.basis * Vector3(input_dir.x, 0, input_dir.y)).normalized()
	aim_direction = (camera_pivot.basis * Vector3(aim_dir.x, 0, aim_dir.y)).normalized()
	
	if aim_direction:
		rotate_model(aim_direction, delta)
		
	if direction or _wants_to_jump:
		target_velocity.x = lerp(target_velocity.x, direction.x * speed, 0.1)
		target_velocity.z = lerp(target_velocity.z, direction.z * speed, 0.1)
		
		##now rotate the model
		if direction:
			if not aim_direction:
				rotate_model(direction, delta)
			player_animation_state = animation_state.RUNNING
	else:
		if is_on_floor():
			linear_damp = damp
		target_velocity.x = 0.0
		target_velocity.z = 0.0
		player_animation_state = animation_state.IDLE
	
	if not is_on_floor():
		player_animation_state = animation_state.JUMPING

	if _jump_released and linear_velocity.dot(up_direction) > 0.0:
		var lv:Vector3 = linear_velocity
		lv -= up_direction * linear_velocity.dot(up_direction) * jump_cut
		linear_velocity = lv

	move_and_slide()
	if grabbed_body != null:
		apply_grab_force(delta)


	# update animation
	match player_animation_state:
		animation_state.IDLE:
			animation_player.play("idle")
		animation_state.RUNNING:
			animation_player.play("sprint")
		animation_state.JUMPING:
			animation_player.play("jump")

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

func rotate_model(dir: Vector3, delta : float) -> void:
	#rotate the model to match the springarm
	#playermodel.basis = lerp(playermodel.basis, Basis.looking_at(dir), 5.0 * delta)
	playermodel.basis = lerp(playermodel.basis, Basis.looking_at(dir), rotation_speed * delta)

func try_grab() -> void:
	var space_state: PhysicsDirectSpaceState3D = get_world_3d().direct_space_state

	var query: PhysicsRayQueryParameters3D = PhysicsRayQueryParameters3D.create(_ray_from, _ray_to)
	query.collide_with_bodies = true
	query.exclude = [self]

	var result: Dictionary = space_state.intersect_ray(query)
	if result.is_empty():
		print('empty!')
		return

	var body: RigidBody3D = result["collider"] as RigidBody3D
	print(result["collider"])
	if body == null:
		return

	grabbed_body = body
	grab_local_offset = body.global_transform.origin - result["position"]

func apply_grab_force(_delta: float) -> void:
	if grabbed_body == null:
		return

	var current_pos: Vector3 = grabbed_body.global_transform.origin - grab_local_offset
	var displacement: Vector3 = _ray_to - current_pos
	if displacement.dot(_grab_dir) < 0.0:
		grabbed_body = null
		return


	var body_velocity: Vector3 = grabbed_body.linear_velocity
	var force: Vector3 = displacement * grab_strength - body_velocity * grab_damping

	grabbed_body.apply_central_force(force)
	apply_central_force(-force) # reaction force on player
