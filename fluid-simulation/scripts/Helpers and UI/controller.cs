using Godot;
using System;

public partial class controller : Control
{
	[Export]
	CheckBox toggleRandomPositions;
	[Export]
	CheckBox toggleDebug;
	[Export]
	ColorPickerButton colorPicker;
	[Export]
	fluid2D simulation;
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		simulation.debugMode = toggleDebug.ButtonPressed;
		simulation.randomizePositions = toggleRandomPositions.ButtonPressed;
		simulation.fluidColor = colorPicker.Color;
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		simulation.debugMode = toggleDebug.ButtonPressed;
		simulation.randomizePositions = toggleRandomPositions.ButtonPressed;
		if (Input.IsKeyPressed(Key.R))
		{
			simulation._Ready();
		}
		Vector2 dir = Vector2.Zero;
		float x = Input.GetAxis("ui_left", "ui_right");
		float y = Input.GetAxis("ui_up", "ui_down");
		dir.X = x;
		dir.Y = y;
		dir.Normalized();
		if (dir == Vector2.Zero)
		{
			dir = Vector2.Down;
		}
		simulation.directionalForce = dir;
		simulation.fluidColor = colorPicker.Color;
	}
}
