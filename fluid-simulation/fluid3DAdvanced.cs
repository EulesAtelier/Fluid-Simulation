using Godot;
using System;
using System.Threading.Tasks;

public partial class fluid3DAdvanced : Node3D
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
    private Vector3[] velocity;
    private Vector3[] position;
    public float[] densities;
    private Vector3[] predictedPosition;



    // Optimizations
    public Vector3I[] spatialLookup;
    public uint[] startIndices;
    [Export]
    public float cellSize = 25.0f;
    float squaredRadius;

    public override void _Ready()
    {

        position = new Vector3[numParticles];
        predictedPosition = new Vector3[numParticles];
        velocity = new Vector3[numParticles];
        densities = new float[numParticles];
        spatialLookup = new Vector3I[numParticles];
        startIndices = new uint[numParticles];

        squaredRadius = smoothingRadius * smoothingRadius;


        int cubeDim = (int)Math.Ceiling(Math.Pow(numParticles, 1.0 / 3.0)); // Cube root rounded up
        int particleRow = cubeDim;
        int particleCol = cubeDim;
        int particleDepth = cubeDim;

        float spacing = particleSize * 2 + partSpacing;

        for (int i = 0; i < numParticles; i++)
        {
            int ix = i % particleRow;
            int iy = (i / particleRow) % particleCol;
            int iz = i / (particleRow * particleCol);

            float x = (ix - (particleRow - 1) / 2f) * spacing;
            float y = (iy - (particleCol - 1) / 2f) * spacing;
            float z = (iz - (particleDepth - 1) / 2f) * spacing;

            position[i] = new Vector3(x, y, z);
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
                velocity[i] += Vector3.Down * (gravity * deltaTime);
                predictedPosition[i] = position[i] + (velocity[i] * 1 / 120);
            });
            Parallel.For(0, numParticles, i =>
            {
                densities[i] = CalculateDensity(predictedPosition[i]);
            });

            Parallel.For(0, numParticles, i =>
            {
                Vector3 pressureForce = CalculatePressureForce(i);
                Vector3 pressureAcceleration = pressureForce / densities[i];
                velocity[i] -= pressureAcceleration * deltaTime;
                // velocity[i] -= CalculateViscocityForces(i) * deltaTime;
            });
            Parallel.For(0, numParticles, i =>
            {
                position[i] += velocity[i] * deltaTime;
                checkBounds(ref position[i], ref velocity[i]);
            });
            //            QueueRedraw();
        }
    }
    [Export]
    public Vector3 boundsSize;
    public override void _Process(double delta)
    {
        Parallel.For(0, numParticles, i =>
            {
                DebugDraw3D.DrawSphere(position[i], particleSize, Colors.Blue, 1f);
            });
        DebugDraw2D.SetText("TPS", Engine.PhysicsTicksPerSecond);
        DebugDraw2D.SetText("Frames drawn", Engine.GetFramesDrawn());
        DebugDraw2D.SetText("FPS", Engine.GetFramesPerSecond());
        DebugDraw2D.SetText("TPS", Engine.PhysicsTicksPerSecond);
        DebugDraw2D.SetText("delta", delta);
        DebugDraw3D.DrawBox(Vector3.Zero, Quaternion.Identity, boundsSize, Colors.Red, true, 1);
        if (Input.IsKeyPressed(Key.R))
        {
            _Ready();
        }
    }

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
    public float CalculateDensity(Vector3 posit)
    {
        float density = 0;
        Vector3 p = PositiontoCellCord(posit);
        for (int startOffsetRow = -1; startOffsetRow <= 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol <= 1; startOffsetCol++)
            {
                for (int startOffsetDep = -1; startOffsetDep <= 1; startOffsetDep++)
                {
                    uint key = getKeyFromHash(HashCell(p + new Vector3(startOffsetRow, startOffsetCol, startOffsetDep)));
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
        }
        return density;
        // float density = 0;
        // foreach (Vector3 pos in position)
        // {
        //     float dst = (posit - pos).Length();
        //     float influence = SmoothingKernel(smoothingRadius, dst);
        //     density += mass * influence;
        // }
        // return density;
    }

    public Vector3 CalculatePressureForce(int particleIndex)
    {
        Vector3 pressureForce = Vector3.Zero;
        Random rng = new Random();
        Vector3 p = PositiontoCellCord(position[particleIndex]);

        for (int startOffsetRow = -1; startOffsetRow <= 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol <= 1; startOffsetCol++)
            {
                for (int startOffsetDep = -1; startOffsetDep <= 1; startOffsetDep++)
                {
                    uint key = getKeyFromHash(HashCell(p + new Vector3(startOffsetRow, startOffsetCol, startOffsetDep)));
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
                            Vector3 offset = predictedPosition[pIndex] - predictedPosition[particleIndex];
                            float dst = offset.Length();
                            Vector3 dir;
                            // float dst = 1.0f / MathF.Sqrt(offset.X * offset.X + offset.Y * offset.Y);//invDist
                            // Vector3 dir = offset * dst;
                            if (dst <= 0.5)
                            {
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
                            float density = densities[pIndex];
                            float sharedPressure = CalculateSharedPressure(density, densities[particleIndex]);
                            // GD.Print("Mass: " + mass + "|" + "Dir: " + dir + "|" + "Slope: " + slope + "|" + "Pressure: " + ConvertDensityToPressure(density) + "|" + "density: " + density + "|");
                            pressureForce += mass * dir * slope * sharedPressure / density;
                        }
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
    Vector3 CalculateViscocityForces(int partIndex)
    {
        Vector3 viscoucityForce = Vector3.Zero;
        Vector3 pos = position[partIndex];
        Parallel.For(0, numParticles, i =>
        {
            float dst = (pos - position[i]).Length();
            float influence = ViscositySmoothingKernel(dst, smoothingRadius);
            viscoucityForce += (velocity[i] - velocity[partIndex]) * influence;
        });
        return viscoucityForce * viscoucityStrength;
    }
    static float ViscositySmoothingKernel(float dst, float radius)
    {
        float volume = (float)(Math.PI * Math.Pow(radius, 8) / 4);
        float val = Math.Max(0, radius * radius - dst * dst);
        return val * val * val / volume;
    }


    // --- Optimization Functions ---


    public void updateSpatialLookup(Vector3[] points)
    {
        // Array.Clear(spatialLookup, 0, spatialLookup.Length);
        // Array.Clear(startIndices, 0, startIndices.Length);

        Parallel.For(0, points.Length, i =>
        {
            Vector3I cell = PositiontoCellCord(points[i]);
            uint key = getKeyFromHash(HashCell(cell));
            spatialLookup[i].Y = (int)key;// I get it, I'm sorting this array so the index becomes diasasociated from itself. Lets use a Vector3 with X and Y as the index and key
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
    public Vector3I PositiontoCellCord(Vector3 point)
    {
        return new Vector3I((int)(point.X / smoothingRadius), (int)(point.Y / smoothingRadius), (int)(point.Z / smoothingRadius));
    }
    public uint getKeyFromHash(uint hash)
    {
        return (uint)(hash % spatialLookup.Length);
    }
    public uint HashCell(Vector3 cell)
    {
        uint x = (uint)(cell.X + 1000000);
        uint y = (uint)(cell.Y + 1000000);

        return ((x * 92837111) ^ (y * 689287499)) & 0x7fffffff;
    }
    public void ForEachPointWithinRadius(Vector3 samplePoint)
    {
        Vector3 p = PositiontoCellCord(samplePoint);
        float squaredRadius = smoothingRadius * smoothingRadius;
        for (int startOffsetRow = -1; startOffsetRow < 1; startOffsetRow++)
        {
            for (int startOffsetCol = -1; startOffsetCol < 1; startOffsetCol++)
            {
                for (int startOffsetDep = -1; startOffsetDep < 1; startOffsetDep++)
                {
                    uint key = getKeyFromHash(HashCell(p + new Vector3(startOffsetRow, startOffsetCol, startOffsetDep)));
                    uint cellStartIndex = startIndices[key];
                    for (uint i = cellStartIndex; i < spatialLookup.Length; i++)
                    {
                        if (spatialLookup[i].Y != key) break;
                        int particleIndex = spatialLookup[i].Y;//need to look at this
                        Vector3 temp = (position[i] - samplePoint);
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
}