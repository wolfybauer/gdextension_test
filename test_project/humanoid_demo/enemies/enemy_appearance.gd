extends RefCounted

const BODIES := [preload("npc0.glb"), preload("npc1.glb")]
const PALETTE = preload("palette.png")
const SHADER = preload("npc_palette.gdshader")
const SKIN := [Color("#e4b493"), Color("#bb8663"), Color("#8a583d"), Color("#613e31")]
const COATS := [Color("#74462d"), Color("#37434d"), Color("#5e322b"), Color("#786951"), Color("#454034")]
const CLOTHES := [Color("#9caaa9"), Color("#b49671"), Color("#765859"), Color("#c4b18d")]
const HATS := [Color("#382c26"), Color("#8c6d49"), Color("#64554a"), Color("#535258")]

var body_index: int
var skin: Color
var coat: Color
var clothes: Color
var hat: Color
var hat_style: int
var has_duster: bool

func roll(rng: RandomNumberGenerator) -> void:
	body_index = rng.randi_range(0, BODIES.size() - 1)
	skin = SKIN[rng.randi_range(0, SKIN.size() - 1)]
	coat = COATS[rng.randi_range(0, COATS.size() - 1)]
	clothes = CLOTHES[rng.randi_range(0, CLOTHES.size() - 1)]
	hat = HATS[rng.randi_range(0, HATS.size() - 1)]
	hat_style = rng.randi_range(0, 2)
	has_duster = rng.randf() < 0.4

func apply(rig) -> bool:
	if not rig.load_model(BODIES[body_index], true):
		return false
	var skeleton: Skeleton3D = rig.get_skeleton()
	var material := ShaderMaterial.new()
	material.shader = SHADER
	material.set_shader_parameter("palette", PALETTE)
	material.set_shader_parameter("skin", skin)
	material.set_shader_parameter("coat", coat)
	material.set_shader_parameter("clothes", clothes)
	for mesh: MeshInstance3D in skeleton.find_children("*", "MeshInstance3D", true, false):
		mesh.set_surface_override_material(0, material)
	if hat_style > 0:
		_add_hat(skeleton)
	if has_duster:
		_add_duster(skeleton)
	return true

func _add_hat(skeleton: Skeleton3D) -> void:
	var attachment := _attach(skeleton, "Head", "Hat")
	var western := hat_style == 1
	var brim_radius := 0.235 if western else 0.17
	var crown_height := 0.115 if western else 0.16
	var material := _cloth_material(hat)
	var band := _cloth_material(hat.darkened(0.4))
	_cylinder(attachment, material, Vector3(0, 0.207, 0), brim_radius, brim_radius, 0.024, 12)
	_cylinder(attachment, material, Vector3(0, 0.22 + crown_height * 0.5, 0), 0.091, 0.115, crown_height, 8)
	_cylinder(attachment, band, Vector3(0, 0.237, 0), 0.115, 0.119, 0.029, 8)

func _add_duster(skeleton: Skeleton3D) -> void:
	var attachment := _attach(skeleton, "Hips", "Duster")
	var material := _cloth_material(coat)
	# Split tails suggest a long coat and leave the knees free to read in silhouette.
	for side in [-1.0, 1.0]:
		var tail := BoxMesh.new()
		tail.size = Vector3(0.16, 0.51, 0.055)
		var panel := _add_mesh(attachment, tail, material, Vector3(side * 0.104, -0.17, -0.16))
		panel.rotation_degrees = Vector3(11, side * -15, side * 6)

func _attach(skeleton: Skeleton3D, bone: String, label: String) -> BoneAttachment3D:
	var attachment := BoneAttachment3D.new()
	attachment.name = label
	attachment.bone_name = bone
	skeleton.add_child(attachment)
	return attachment

func _cloth_material(color: Color) -> StandardMaterial3D:
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.9
	return material

func _cylinder(parent: Node3D, material: Material, offset: Vector3, top: float, bottom: float, height: float, sides: int) -> void:
	var cylinder := CylinderMesh.new()
	cylinder.top_radius = top
	cylinder.bottom_radius = bottom
	cylinder.height = height
	cylinder.radial_segments = sides
	cylinder.rings = 0
	_add_mesh(parent, cylinder, material, offset)

func _add_mesh(parent: Node3D, mesh: Mesh, material: Material, offset: Vector3) -> MeshInstance3D:
	var part := MeshInstance3D.new()
	part.mesh = mesh
	part.material_override = material
	part.position = offset
	parent.add_child(part)
	return part
