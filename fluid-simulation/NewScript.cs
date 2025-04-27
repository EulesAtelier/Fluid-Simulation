using Godot;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Vector2 = Godot.Vector2;
public partial class NewScript : Node2D
{
    // Simulation Variables
    [Export]
    public int numParticles = 15;
    [Export]
    public float particleSize = 10;
    [Export]
    public float partSpacing = 5.0f;
    [Export]
    public float gravity = 10000f;
    [Export]
    public float targetDensity;
    [Export]
    public float pressureMultiplier;
    //Particle Values
    private Vector2[] velocity;
    private Vector2[] position;
    public float[] densities;
    private Vector2[] predictedPosition;



    // Optimizations
    public int[] spatialLookup;
    public int[] startIndices;
    float cellSize;
    public override void _Ready()
    {

        position = new Vector2[numParticles];
        predictedPosition = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        densities = new float[numParticles];
        spatialLookup = new int[numParticles];
        startIndices = new int[numParticles];
        cellSize = smoothingRadius;

        int particleRow = (int)(Math.Sqrt(numParticles));
        int particleCol = (numParticles - 1) / particleRow + 1;
        float spacing = particleSize * 2 + partSpacing;
        // Random rng = new(123);
        for (int i = 0; i < numParticles; i++)
        {
            // float x = (float)((rng.NextDouble() - 0.5) * boundsSize.X);
            // float y = (float)((rng.NextDouble() - 0.5) * boundsSize.Y);
            float x = (i % particleRow - particleRow / 2f + 0.5f) * spacing;
            float y = (i / particleRow - particleCol / 2f + 0.5f) * spacing;
            position[i] = new Vector2(x, y);

        }
        // updateSpatialLookup();

    }
    public override void _PhysicsProcess(double delta)
    {
        // updateSpatialLookup();
        float deltaTime = (float)delta;
        Parallel.For(0, numParticles, i =>
        {
            velocity[i] += Vector2.Down * (gravity * deltaTime);
            predictedPosition[i] = position[i] + (velocity[i] * 1 / 120);
        });
        Parallel.For(0, numParticles, i =>
        {
            densities[i] = CalculateDensity(predictedPosition[i]);
        });
        Parallel.For(0, numParticles, i =>
        {
            Vector2 pressureForce = CalculatePressureForce(i);
            Vector2 pressureAcceleration = pressureForce / densities[i];
            velocity[i] -= pressureAcceleration * deltaTime;
            velocity[i] -= CalculateViscocityForces(i) * deltaTime;
        });
        Parallel.For(0, numParticles, i =>
        {
            position[i] += velocity[i] * deltaTime;
            checkBounds(ref position[i], ref velocity[i]);
            // GD.Print("V " + velocity[i] + " | P " + position[i] + " | D" + densities[i]);

        });

        // GD.Print("Pressure: " + pressureForce + " PressureAcc: " + pressureAcceleration);

        QueueRedraw();
        updateSpatialLookup();
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
        // foreach (var kvp in grid)
        // {
        //     Vector2I cell = kvp.Key;
        //     if (kvp.Value.Count == 0) continue;

        //     Vector2 topLeft = new Vector2(cell.X * cellSize, cell.Y * cellSize);
        //     Vector2 size = new Vector2(cellSize, cellSize);
        //     DrawRect(new Rect2(topLeft, size), Colors.Green, false); // false = outline only
        // }
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

    // -- Math ---

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
    public float CalculateDensity(Vector2 posit)
    {
        float density = 0;
        // List<int> neighbors = GetNeighborIndices(samplePoint);
        foreach (Vector2 pos in position)
        {
            float dst = (posit - pos).Length();
            float influence = SmoothingKernel(smoothingRadius, dst);
            density += mass * influence;
        }
        return density;
    }
    public Vector2 CalculatePressureForce(int particleIndex)
    {
        Vector2 pressureForce = Vector2.Zero;
        Random rng = new Random();

        for (int otherParticleIndex = 0; otherParticleIndex < numParticles; otherParticleIndex++)
        {
            if (predictedPosition[particleIndex] == predictedPosition[otherParticleIndex]) continue;

            Vector2 offset = predictedPosition[otherParticleIndex] - predictedPosition[particleIndex];
            float dst = offset.Length();
            Vector2 dir;
            if (dst == 0)
            {
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

    public float ConvertDensityToPressure(float density)
    {
        float DensityError = density - targetDensity;
        float pressure = DensityError * pressureMultiplier;
        // GD.Print("DensityError = " + DensityError + " Pressure: " + pressure);
        return pressure;
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
    [Export]
    public float viscoucityStrength = 1.0f;
    Vector2 CalculateViscocityForces(int partIndex)
    {
        Vector2 viscoucityForce = Vector2.Zero;
        Vector2 pos = position[partIndex];
        for (int i = 0; i < numParticles; i++)
        {
            float dst = (pos - position[i]).Length();
            float influence = ViscositySmoothingKernel(dst, smoothingRadius);
            viscoucityForce += (velocity[i] - velocity[partIndex]) * influence;
        }
        return viscoucityForce * viscoucityStrength;
    }
    static float ViscositySmoothingKernel(float dst, float radius)
    {
        float volume = (float)(Math.PI * Math.Pow(radius, 8) / 4);
        float val = Math.Max(0, radius * radius - dst * dst);
        return val * val * val / volume;
    }


    // --- Optimization Functions ---

    public Vector2I PositiontoCellCord(Vector2 point)
    {
        return new Vector2I((int)(point.X / smoothingRadius), (int)(point.Y / smoothingRadius));
    }
    public void updateSpatialLookup()
    {
        String print = "";
        Parallel.For(0, numParticles, i =>
        {
            Vector2I cell = PositiontoCellCord(position[i]);
            spatialLookup[i] = ((cell.X * 15823) + (cell.Y * 9737333)) % numParticles;
            startIndices[i] = int.MaxValue;
        });
        Array.Sort(spatialLookup);
        for (int i = 0; i < numParticles; i++)
        {
            print += spatialLookup[i] + " ";
        }
        GD.Print(print);
        Parallel.For(0, numParticles, i =>
       {
           int key = spatialLookup[i];
           int keyPrevious = i == 0 ? int.MaxValue : spatialLookup[i - 1];
           if (keyPrevious != key)
           {
               startIndices[key] = i;
           }
       });
        startIndicesPrint();
    }
    public void startIndicesPrint()
    {
        String print = "";
        for (int i = 0; i < spatialLookup.Length; i++)
        {
            print += startIndices[i] + " ";
        }
        GD.Print(print);
    }
    public int getKeyFromHash(int hash)
    {
        return hash % (int)spatialLookup.Length;
    }
    public void ForEachPointWithinRadius(Vector2 samplePoint)
    {
        Vector2 p = PositiontoCellCord(samplePoint);
        float squaredRadius = smoothingRadius * smoothingRadius;
        for (int startOffsetRow = -1; startOffsetRow < 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol < 1; startOffsetCol++)
            {
                int key = getKeyFromHash(HashCell(p + new Vector2(startOffsetRow, startOffsetCol)));
                int cellStartIndex = startIndices[key];
                for (int i = cellStartIndex; i < spatialLookup.Length; i++)
                {
                    if (spatialLookup[i] != key) break;
                    int particleIndex = spatialLookup[i];//need to look at this
                    Vector2 temp = (position[i] - samplePoint);
                    float sqrDist = temp.LengthSquared();
                    if (sqrDist <= squaredRadius)
                    {
                        //now I can do the calculation
                    }
                }

            }
        }
    }
    public int HashCell(Vector2 cell)
    {
        return (int)((cell.X * 15823) + (cell.Y * 9737333));
    }
}