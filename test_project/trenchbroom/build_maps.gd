extends SceneTree

const SETTINGS = preload("res://trenchbroom/map_settings.tres")
const ROOT := "res://trenchbroom/"

func _initialize() -> void:
	for name in OS.get_cmdline_user_args():
		if not build_map(name):
			quit(1)
			return
	quit()

func build_map(name: String) -> bool:
	var scene := FuncGodotMap.new()
	scene.name = name
	scene.local_map_file = ROOT + "maps/" + name + ".map"
	scene.map_settings = SETTINGS
	scene.build()
	if scene.get_child_count() == 0:
		scene.free()
		return false
	var output := PackedScene.new()
	var ok := output.pack(scene) == OK
	if ok:
		var dir := ROOT + "baked"
		DirAccess.make_dir_recursive_absolute(dir)
		ok = ResourceSaver.save(output, dir + "/" + name + ".scn", ResourceSaver.FLAG_COMPRESS) == OK
		print(name, ": ", scene.get_child_count(), " top-level entities")
	scene.free()
	return ok
