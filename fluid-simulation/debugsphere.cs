
using Godot;
using System;
public partial class debugsphere : ImmediateMesh
{
    [Export] public float Radius = 0.5f;
    [Export] public int Segments = 32;
    public void DrawCircle(float r)
    {
        SurfaceBegin(Mesh.PrimitiveType.LineStrip);

        for (int i = 0; i <= Segments; i++)
        {
            float angle = (i / (float)Segments) * Mathf.Tau;
            float x = Mathf.Cos(angle) * r;
            float z = Mathf.Sin(angle) * r;
            SurfaceAddVertex(new Vector3(x, 0, z));
        }

        SurfaceEnd();
    }
}