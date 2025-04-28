using Godot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
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
    public Vector2I[] spatialLookup;
    public uint[] startIndices;
    [Export]
    public float cellSize = 25.0f;
    float squaredRadius;

    public override void _Ready()
    {

        position = new Vector2[numParticles];
        predictedPosition = new Vector2[numParticles];
        velocity = new Vector2[numParticles];
        densities = new float[numParticles];
        spatialLookup = new Vector2I[numParticles];
        startIndices = new uint[numParticles];

        squaredRadius = smoothingRadius * smoothingRadius;


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
        updateSpatialLookup(position);

    }
    float accumulator = 0f;
    float fixedTimeStep = 1f / 30f; // 30 Hz simulation

    public override void _PhysicsProcess(double delta)
    {
        accumulator += (float)delta;

        while (accumulator >= fixedTimeStep)//reduce physics ticks
        {
            accumulator -= fixedTimeStep;
            float deltaTime = (float)delta;
            updateSpatialLookup(position);

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
                // velocity[i] -= CalculateViscocityForces(i) * deltaTime;
            });
            Parallel.For(0, numParticles, i =>
            {
                position[i] += velocity[i] * deltaTime;
                checkBounds(ref position[i], ref velocity[i]);
            });
            QueueRedraw();
        }
    }
    [Export]
    public Vector2 boundsSize;

    public override void _Draw()
    {
        Rect2 rect2 = new Rect2(Vector2.Zero - (boundsSize / 2), boundsSize);

        for (int i = 0; i < position.Length; i++)
        {
            DrawCircle(position[i], particleSize, Colors.Blue);
            // DrawString(ThemeDB.FallbackFont, position[i] + new Vector2(2, 0), "Key: " + spatialLookup[i],
            //    HorizontalAlignment.Center, 90, 22);
            // Vector2 cell = new Vector2I((int)(position[i].X / smoothingRadius), (int)(position[i].Y / smoothingRadius));
            // String str = "(" + cell.X + ", " + cell.Y + ")";
            // DrawString(ThemeDB.FallbackFont, position[i] + new Vector2(5, 5), str, HorizontalAlignment.Center, 90, 22);
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
        // HashSet<Vector2I> drawnCells = new HashSet<Vector2I>();
        // foreach (uint particleIndex in spatialLookup)
        // {
        //     Vector2 particlePos = position[particleIndex];
        //     Vector2I cell = PositiontoCellCord(particlePos);

        //     // Only draw once per cell
        //     if (drawnCells.Contains(cell))
        //         continue;
        //     drawnCells.Add(cell);

        //     // Draw centered on cell
        //     Vector2 cellCenter = new Vector2(cell.X * smoothingRadius, cell.Y * smoothingRadius);
        //     Vector2 topLeft = cellCenter - new Vector2(smoothingRadius, smoothingRadius) * 0.5f;
        //     Vector2 size = new Vector2(smoothingRadius, smoothingRadius);

        //     DrawRect(new Rect2(topLeft, size), Colors.Green, false); // outline only
        //     DrawString(ThemeDB.FallbackFont, topLeft, cell + "",
        //        HorizontalAlignment.Center, 90, 22);
        // }
        // }
        Vector2I topLeftCell = PositiontoCellCord(rect2.Position);
        Vector2I bottomRightCell = PositiontoCellCord(rect2.Position + rect2.Size);
        for (int x = topLeftCell.X; x <= bottomRightCell.X; x++)
        {
            for (int y = topLeftCell.Y; y <= bottomRightCell.Y; y++)
            {
                Vector2I cellCoord = new Vector2I(x, y);

                // Center of this cell
                Vector2 cellCenter = new Vector2(cellCoord.X * smoothingRadius, cellCoord.Y * smoothingRadius);

                // Adjust so the rect is centered around cell center
                Vector2 topLeft = cellCenter - new Vector2(smoothingRadius, smoothingRadius) * 0.5f;
                Vector2 size = new Vector2(smoothingRadius, smoothingRadius);
                uint key = getKeyFromHash(HashCell(cellCoord));

                DrawRect(new Rect2(topLeft, size), Colors.LightGray, false); // Outline only
                DrawString(ThemeDB.FallbackFont, topLeft + new Vector2(-5, -5), "" + key,
                   HorizontalAlignment.Center, 90, 8);

            }
        }
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
        Vector2 p = PositiontoCellCord(posit);
        for (int startOffsetRow = -1; startOffsetRow <= 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol <= 1; startOffsetCol++)
            {
                uint key = getKeyFromHash(HashCell(p + new Vector2(startOffsetRow, startOffsetCol)));
                uint cellStartIndex = startIndices[key];
                if (cellStartIndex == uint.MaxValue)
                {
                    // GD.PrintErr($"Skipping cell with key {key} because it is empty (startIndex = {cellStartIndex}).");
                    continue;
                }
                for (uint i = cellStartIndex; i < spatialLookup.Length; i++)
                {
                    if (spatialLookup[i].Y != key) break;
                    uint particleIndex = (uint)spatialLookup[i].X;//X is index
                    float sqrDist = (position[particleIndex] - posit).LengthSquared();

                    if (sqrDist <= squaredRadius)
                    {
                        float dst = (posit - position[particleIndex]).Length();
                        float influence = SmoothingKernel(smoothingRadius, dst);
                        density += mass * influence;
                    }
                }
            }
        }
        return density;
        // float density = 0;
        // foreach (Vector2 pos in position)
        // {
        //     float dst = (posit - pos).Length();
        //     float influence = SmoothingKernel(smoothingRadius, dst);
        //     density += mass * influence;
        // }
        // return density;
    }

    public Vector2 CalculatePressureForce(int particleIndex)
    {
        Vector2 pressureForce = Vector2.Zero;
        Random rng = new Random();
        Vector2 p = PositiontoCellCord(position[particleIndex]);

        for (int startOffsetRow = -1; startOffsetRow <= 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol <= 1; startOffsetCol++)
            {
                uint key = getKeyFromHash(HashCell(p + new Vector2(startOffsetRow, startOffsetCol)));
                uint cellStartIndex = startIndices[key];
                if (cellStartIndex == uint.MaxValue)
                {
                    // GD.PrintErr($"Skipping cell with key {key} because it is empty (startIndex = {cellStartIndex}).");
                    continue;
                }
                for (uint i = cellStartIndex; i < spatialLookup.Length; i++)
                {
                    if (spatialLookup[i].Y != key) break;
                    uint pIndex = (uint)spatialLookup[i].X;//X is index
                    if (pIndex == particleIndex) continue; // Skip self
                    float sqrDist = (predictedPosition[pIndex] - predictedPosition[particleIndex]).LengthSquared();

                    if (sqrDist < squaredRadius)
                    {
                        Vector2 offset = predictedPosition[pIndex] - predictedPosition[particleIndex];
                        float dst = offset.Length();
                        Vector2 dir;
                        // float dst = 1.0f / MathF.Sqrt(offset.X * offset.X + offset.Y * offset.Y);//invDist
                        // Vector2 dir = offset * dst;
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
                        float density = densities[pIndex];
                        float sharedPressure = CalculateSharedPressure(density, densities[particleIndex]);
                        // GD.Print("Mass: " + mass + "|" + "Dir: " + dir + "|" + "Slope: " + slope + "|" + "Pressure: " + ConvertDensityToPressure(density) + "|" + "density: " + density + "|");
                        pressureForce += mass * dir * slope * sharedPressure / density;
                    }
                }
            }
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


    public void updateSpatialLookup(Vector2[] points)
    {
        // Array.Clear(spatialLookup, 0, spatialLookup.Length);
        // Array.Clear(startIndices, 0, startIndices.Length);

        Parallel.For(0, points.Length, i =>
        {
            Vector2I cell = PositiontoCellCord(points[i]);
            uint key = getKeyFromHash(HashCell(cell));
            spatialLookup[i].Y = (int)key;// I get it, I'm sorting this array so the index becomes diasasociated from itself. Lets use a Vector2 with X and Y as the index and key
            spatialLookup[i].X = i;
            startIndices[i] = uint.MaxValue;
        });
        Array.Sort(spatialLookup, (a, b) => a.Y.CompareTo(b.Y));
        // String str = "";
        for (uint i = 0; i < points.Length; i++)
        {
            uint key = (uint)spatialLookup[i].Y;
            uint keyPrevious;
            if (i == 0)
            {
                keyPrevious = uint.MaxValue;
            }
            else
            {
                keyPrevious = (uint)spatialLookup[i - 1].Y;
            }
            if (key != keyPrevious)
            {
                {
                    startIndices[key] = i;
                }
            }
        }
        // String str2 = "Cells Key: ";
        // for (uint i = 0; i < points.Length; i++)
        // {
        //     str += startIndices[i] + " ";
        //     if (startIndices[i] != uint.MaxValue)
        //     {
        //         str2 += spatialLookup[startIndices[i]].Y + " ";
        //     }
        // }
        // // GD.Print(str);
        // GD.Print(str2);
    }
    public Vector2I PositiontoCellCord(Vector2 point)
    {
        return new Vector2I((int)(point.X / smoothingRadius), (int)(point.Y / smoothingRadius));
    }
    public uint getKeyFromHash(uint hash)
    {
        return (uint)(hash % spatialLookup.Length);
    }
    public uint HashCell(Vector2 cell)
    {
        uint x = (uint)(cell.X + 1000000);
        uint y = (uint)(cell.Y + 1000000);

        return ((x * 92837111) ^ (y * 689287499)) & 0x7fffffff;
    }
    public void ForEachPointWithinRadius(Vector2 samplePoint)
    {
        Vector2 p = PositiontoCellCord(samplePoint);
        float squaredRadius = smoothingRadius * smoothingRadius;
        for (int startOffsetRow = -1; startOffsetRow < 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol < 1; startOffsetCol++)
            {
                uint key = getKeyFromHash(HashCell(p + new Vector2(startOffsetRow, startOffsetCol)));
                uint cellStartIndex = startIndices[key];
                for (uint i = cellStartIndex; i < spatialLookup.Length; i++)
                {
                    if (spatialLookup[i].Y != key) break;
                    int particleIndex = spatialLookup[i].Y;//need to look at this
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
}