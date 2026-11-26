extends SceneTree

const CONFIG = preload("res://trenchbroom/game_config.tres")
const OUTPUT := "res://trenchbroom/profile"

func _initialize() -> void:
	var folder := ProjectSettings.globalize_path(OUTPUT)
	DirAccess.make_dir_recursive_absolute(folder)
	var config := FileAccess.open(OUTPUT + "/GameConfig.cfg", FileAccess.WRITE)
	if config == null:
		quit(1)
		return
	config.store_string(CONFIG._build_class_text())
	config.close()
	CONFIG.fgd_file.do_export_file(FuncGodotFGDFile.FuncGodotTargetMapEditors.TRENCHBROOM, folder)
	if CONFIG.icon.get_image().save_png(OUTPUT + "/icon.png") != OK:
		quit(1)
		return
	quit()
