using Godot;
using System;

public partial class controller3D : Control
{
	[Export]
	CheckBox toggleRandomPositions;
	[Export]
	CheckBox toggleDebug;
	[Export]
	ColorPickerButton colorPicker;
	[Export]
	fluid3DAdvanced simulation;
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
		Vector3 dir = Vector3.Zero;
		float x = Input.GetAxis("ui_left", "ui_right");
		float z = Input.GetAxis("ui_up", "ui_down");
		dir.X = x;
		dir.Z = z;
		if (Input.IsKeyPressed(Key.Shift))
		{
			dir.Y = 1.0f;
		}
		dir.Normalized();
		if (dir == Vector3.Zero)
		{
			dir = Vector3.Down;
		}
		simulation.directionalForce = dir;
		simulation.fluidColor = colorPicker.Color;
	}
}
