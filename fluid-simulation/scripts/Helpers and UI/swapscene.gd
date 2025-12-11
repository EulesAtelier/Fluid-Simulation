extends Button
@export var level: String 
func _on_pressed() -> void:
	get_tree().change_scene_to_file("res://scenes/"+level+".tscn")
