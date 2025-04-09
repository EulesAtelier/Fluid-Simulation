using Godot;
using System;
using System.Diagnostics;
using System.Threading.Tasks;
public partial class fluid3D : Node3D
{
    // Member variables here, example:
    [Export]
    public float gravity = 10000f;
    [Export]
    public int numParticles = 15;
    [Export]
    public float particleSize = 10;
    [Export]
    public float partSpacing = 5.0f;
    [Export]

    private Vector3[] velocity;
    [Export]

    private Vector3[] position;
    [Export]
    public float targetDensity;
    [Export]
    public float pressureMultiplier;
    [Export]
    public float[] densities;
    [Export]
    public PackedScene particleInstance;
    public MeshInstance3D[] particles;
    public Node particleHandler;

    public override void _Ready()
    {
        position = new Vector3[numParticles];
        velocity = new Vector3[numParticles];
        densities = new float[numParticles];
        int particleRow = (int)Math.Cbrt(numParticles); // Approximate cubic root for rows
        int particleCol = (int)Math.Sqrt(numParticles / particleRow); // Adjust based on available particles
        int particleDepth = (numParticles - 1) / (particleRow * particleCol) + 1; // Ensures full depth distribution
        float spacing = particleSize * 2 + partSpacing;
        // GD.Print(position.Length);
        Random rng = new(123);
        for (int i = 0; i < numParticles; i++)
        {
            // float x = (float)((rng.NextDouble() - 0.5) * boundsSize.X);
            // float y = (float)((rng.NextDouble() - 0.5) * boundsSize.Y);
            // float z = (float)((rng.NextDouble() - 0.5) * boundsSize.Z);

            // float x = (i % particleRow - particleRow / 2f + 0.5f) * spacing;
            // float y = ((i / particleRow) % particleCol - particleCol / 2f + 0.5f) * spacing;
            // float z = (i / (particleRow * particleCol) - particleDepth / 2f + 0.5f) * spacing;


            // position[i] = new Vector3(x, y, z);
            position[i] = new Vector3(
    (float)GD.RandRange(-partSpacing, partSpacing), // Random X within -3 to 3
    (float)GD.RandRange(-partSpacing, partSpacing), // Random Y within -3 to 3
    (float)GD.RandRange(-partSpacing, partSpacing)  // Random Z within -3 to 3
);


            // particles[i] = (MeshInstance3D)particleInstance.Instantiate();
            // particles[i].SetPosition(position[i]);
            // AddChild(particles[i]);
        }
        UpdateDensities();
        // Draw();
    }
    public float ConvertDensityToPressure(float density)
    {
        float DensityError = density - targetDensity;
        float pressure = DensityError * pressureMultiplier;
        // GD.Print("DensityError = " + DensityError + " Pressure: " + pressure);
        return pressure;
    }
    public override void _Process(double delta)
    {
        for (int i = 0; i < numParticles; i++)
        {
            DebugDraw3D.DrawSphere(position[i], particleSize, Colors.Blue, 0);
        }
        DebugDraw2D.SetText("TPS", Engine.PhysicsTicksPerSecond);
        DebugDraw2D.SetText("Frames drawn", Engine.GetFramesDrawn());
        DebugDraw2D.SetText("FPS", Engine.GetFramesPerSecond());
        DebugDraw2D.SetText("TPS", Engine.PhysicsTicksPerSecond);
        DebugDraw2D.SetText("delta", delta);
        DebugDraw3D.DrawBox(Vector3.Zero, Quaternion.Identity, boundsSize, Colors.Red, true, 1);

    }
    public override void _PhysicsProcess(double delta)
    {
        float deltaTime = (float)delta;

        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += Vector3.Down * (gravity * deltaTime);
            densities[i] = CalculateDensity(position[i]);
        });
        Parallel.For(0, numParticles, i =>
        {
            Vector3 pressureForce = CalculatePressureForce(i);
            Vector3 pressureAcceleration = pressureForce / densities[i];
            velocity[i] -= pressureAcceleration * deltaTime;
        });
        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += Vector3.Down * (gravity * deltaTime);
            densities[i] = CalculateDensity(position[i]);
            position[i] += velocity[i] * deltaTime;
            checkBounds(ref position[i], ref velocity[i]);
            // GD.Print("V " + velocity[i] + " | P " + position[i] + " | D" + densities[i]);
        });
        // GD.Print("Pressure: " + pressureForce + " PressureAcc: " + pressureAcceleration);
    }
    [Export]
    public Vector3 boundsSize;
    [Export]
    public float dampeningForce;
    public void checkBounds(ref Vector3 cPos, ref Vector3 cVel)
    {
        Vector3 halfBoundsSize = boundsSize / 2;
        if (Math.Abs(cPos.X) > halfBoundsSize.X)
        {
            cPos.X = halfBoundsSize.X * Math.Sign(cPos.X);
            cVel.X *= -1 * dampeningForce;
        }
        if (Math.Abs(cPos.Y) > halfBoundsSize.Y)
        {
            cPos.Y = halfBoundsSize.Y * Math.Sign(cPos.Y);
            cVel.Y *= -1 * dampeningForce;
        }
        if (Math.Abs(cPos.Z) > halfBoundsSize.Z)
        {
            cPos.Z = halfBoundsSize.Z * Math.Sign(cPos.Z);
            cVel.Z *= -1 * dampeningForce;
        }
    }
    [Export]
    public float smoothingRadius = 1.0f;
    public float SmoothingKernel(float radius, float dist)
    {
        float volume = (float)(Math.PI * Math.Pow(radius, 8) / 4);
        float val = Math.Max(0, radius * radius - dist * dist);
        return val * val * val / volume;
    }
    [Export]
    public float mass = 1;
    public float CalculateDensity(Vector3 samplePoint)
    {
        float density = 0;

        foreach (Vector3 pos in position)
        {
            float dst = (pos - samplePoint).Length();
            float influence = SmoothingKernel(smoothingRadius, dst);
            density += mass * influence;
        }
        return density;
    }
    public Vector3 CalculatePressureForce(int particleIndex)
    {
        Vector3 pressureForce = Vector3.Zero;
        for (int otherParticleIndex = 0; otherParticleIndex < numParticles; otherParticleIndex++)
        {
            if (particleIndex == otherParticleIndex) continue;

            Vector3 offset = position[otherParticleIndex] - position[particleIndex];
            float dst = offset.Length();
            Vector3 dir;
            if (dst == 0)
            {
                Random rng = new(123);
                float x = (float)((rng.NextDouble()));
                float y = (float)((rng.NextDouble()));
                float z = (float)((rng.NextDouble()));

                dir = new Vector3(x, y, z);
            }
            else
            {
                dir = offset / dst;
            }
            float slope = SmoothingKernelDerivative(dst, smoothingRadius);
            float density = densities[otherParticleIndex];
            float sharedPressure = CalculateSharedPressure(density, densities[particleIndex]);
            // GD.Print("Mass: " + mass + "|" + "Dir: " + dir + "|" + "Slope: " + slope + "|" + "Pressure: " + ConvertDensityToPressure(density) + "|" + "density: " + density + "|");
            pressureForce += mass * dir * slope * sharedPressure / density;
        }

        return pressureForce;
    }
    public float CalculateSharedPressure(float densityA, float densityB)
    {
        return (ConvertDensityToPressure(densityA) + ConvertDensityToPressure(densityB)) / 2;
    }
    public void UpdateDensities()
    {
        Parallel.For(0, numParticles, i =>
        {
            densities[i] = CalculateDensity(position[i]);
        });
    }
    static float SmoothingKernelDerivative(float dist, float radius)
    {
        // GD.Print(dist + " + " + radius);
        if (dist >= radius) return 0;
        float f = radius * radius - dist * dist;
        float scale = (float)(-24 / (Math.PI * Math.Pow(radius, 8)));
        // GD.Print("scale: " + scale + "|" + "radius: " + radius + "|" + "dist: " + dist + "|" + "math: " + (Math.PI * Math.Pow(radius, 8)));
        return scale * dist * f * f;
    }
}
