using Godot;
using System;
using System.Threading.Tasks;
public partial class NewScript : Node2D
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

    private Vector2[] velocity;
    [Export]

    private Vector2[] position;
    [Export]
    public float targetDensity;
    [Export]
    public float pressureMultiplier;
    [Export]
    public float[] densities;

    public override void _Ready()
    {

        position = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        densities = new float[numParticles];
        int particleRow = (int)(Math.Sqrt(numParticles));
        int particleCol = (numParticles - 1) / particleRow + 1;
        float spacing = particleSize * 2 + partSpacing;
        // GD.Print(position.Length);
        Random rng = new(123);
        for (int i = 0; i < numParticles; i++)
        {
            float x = (float)((rng.NextDouble() - 0.5) * boundsSize.X);
            float y = (float)((rng.NextDouble() - 0.5) * boundsSize.Y);
            // float x = (i % particleRow - particleRow / 2f + 0.5f) * spacing;
            // float y = (i / particleRow - particleCol / 2f + 0.5f) * spacing;
            position[i] = new Vector2(x, y);
        }
        UpdateDensities();
    }
    public float ConvertDensityToPressure(float density)
    {
        float DensityError = density - targetDensity;
        float pressure = DensityError * pressureMultiplier;
        // GD.Print("DensityError = " + DensityError + " Pressure: " + pressure);
        return pressure;
    }
    public override void _PhysicsProcess(double delta)
    {
        float deltaTime = (float)delta;

        Parallel.For(0, numParticles, i =>
        {
            // velocity[i] += Vector2.Down * (gravity * deltaTime);
            densities[i] = CalculateDensity(position[i]);
        });
        Parallel.For(0, numParticles, i =>
        {
            Vector2 pressureForce = CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / densities[i];
            velocity[i] -= pressureAcceleration * deltaTime;
        });
        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += Vector2.Down * (gravity * deltaTime);
            densities[i] = CalculateDensity(position[i]);
            position[i] += velocity[i] * deltaTime;
            checkBounds(ref position[i], ref velocity[i]);
            // GD.Print("V " + velocity[i] + " | P " + position[i] + " | D" + densities[i]);

        });

        // GD.Print("Pressure: " + pressureForce + " PressureAcc: " + pressureAcceleration);

        QueueRedraw();
    }
    [Export]
    public Vector2 boundsSize;

    public override void _Draw()
    {
        Rect2 rect2 = new Rect2(Vector2.Zero - (boundsSize / 2), boundsSize);
        for (int i = 0; i < position.Length; i++)
        {
            DrawCircle(position[i], particleSize, Colors.White);
        }
        DrawRect(rect2, Colors.Red, false, 1, false);
    }

    [Export]
    public float dampeningForce;
    public void checkBounds(ref Vector2 cPos, ref Vector2 cVel)
    {
        Vector2 halfBoundsSize = boundsSize / 2 - Vector2.One * 20f;
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
    public float CalculateDensity(Vector2 samplePoint)
    {
        float density = 0;

        foreach (Vector2 pos in position)
        {
            float dst = (pos - samplePoint).Length();
            float influence = SmoothingKernel(smoothingRadius, dst);
            density += mass * influence;
        }
        return density;
    }
    public Vector2 CalculatePressureForce(int particleIndex)
    {
        Vector2 pressureForce = Vector2.Zero;
        for (int otherParticleIndex = 0; otherParticleIndex < numParticles; otherParticleIndex++)
        {
            if (particleIndex == otherParticleIndex) continue;

            Vector2 offset = position[otherParticleIndex] - position[particleIndex];
            float dst = offset.Length();
            Vector2 dir;
            if (dst == 0)
            {
                Random rng = new(123);
                float x = (float)((rng.NextDouble()));
                float y = (float)((rng.NextDouble()));

                dir = new Vector2(x, y);
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

            // GD.Print("Mass: " + mass + "|" + "Dir: " + dir + "|" + "Slope: " + slope + "|" + "Pressure: " + ConvertDensityToPressure(density) + "|" + "density: " + density + "|");
            // pressureForce += mass * dir * slope * ConvertDensityToPressure(density) / density;
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
