#include "raylib.h"
#include "vector.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#define MAX_BODIES 10000

int main(void)
{
    const int screenWidth = 1600;
    const int screenHeight = 1000;

    InitWindow(screenWidth, screenHeight, "N-Body Simulation");
    if (!IsWindowReady())
    {
        fprintf(stderr, "Failed to initialize graphics window. Check GPU/OpenGL support and display session.\n");
        return 1;
    }

    double G = 60.0;
    ParticleSystem ps;
    ps.count = 3000;
    ps.x = malloc(sizeof(double) * ps.count);
    ps.y = malloc(sizeof(double) * ps.count);
    ps.vx = malloc(sizeof(double) * ps.count);
    ps.vy = malloc(sizeof(double) * ps.count);
    ps.fx = malloc(sizeof(double) * ps.count);
    ps.fy = malloc(sizeof(double) * ps.count);
    ps.mass = malloc(sizeof(double) * ps.count);
    ps.color = malloc(sizeof(Color) * ps.count);
    Body *bodies = malloc(sizeof(Body) * ps.count);
    int count = 0;

    vector leftGalaxyCenter = {400, 600};
    vector leftGalaxyVelocity = {8, 0};

    vector rightGalaxyCenter = {1200, 600};
    vector rightGalaxyVelocity = {-8, 0};

    int particlesPerGalaxy = 1500;
    double centralMass = 500.0;

    double simulationSpeed = 1.0;

    // LEFT GALAXY
    for (int i = 0; i < particlesPerGalaxy; i++)
    {
        if (i == 0)
        {
            // Central mass moves only with galaxy drift.
            ps.x[count] = leftGalaxyCenter.x;
            ps.y[count] = leftGalaxyCenter.y;
            ps.vx[count] = leftGalaxyVelocity.x;
            ps.vy[count] = leftGalaxyVelocity.y;
            ps.mass[count] = centralMass;
            ps.color[count] = BLUE;
            count++;
        }
        else
        {
            double angle = (double)GetRandomValue(0, 359) * (PI / 180.0);
            double radius = (double)GetRandomValue(80, 250);

            ps.x[count] = leftGalaxyCenter.x + cos(angle) * radius;
            ps.y[count] = leftGalaxyCenter.y + sin(angle) * radius;

            double orbitalSpeed = sqrt(G * centralMass / radius) * 0.95;

            ps.vx[count] = leftGalaxyVelocity.x + (-sin(angle) * orbitalSpeed);
            ps.vy[count] = leftGalaxyVelocity.y + (cos(angle) * orbitalSpeed);

            ps.mass[count] = (double)GetRandomValue(1, 3) * 0.1;
            ps.color[count] = (Color){100, 150, 255, 255};
            count++;
        }
        count++;
    }

    // RIGHT GALAXY
    for (int i = 0; i < particlesPerGalaxy; i++)
    {
        if (i == 0)
        {
            // Central mass moves only with galaxy drift.
            ps.x[count] = rightGalaxyCenter.x;
            ps.y[count] = rightGalaxyCenter.y;
            ps.vx[count] = rightGalaxyVelocity.x;
            ps.vy[count] = rightGalaxyVelocity.y;
            ps.mass[count] = centralMass;
            ps.color[count] = RED;
            count++;
        }
        else
        {
            // Orbiting particles
            double angle = (double)GetRandomValue(0, 359) * (PI / 180.0);
            double radius = (double)GetRandomValue(80, 250);

            ps.x[count] = rightGalaxyCenter.x + cos(angle) * radius;
            ps.y[count] = rightGalaxyCenter.y + sin(angle) * radius;

            double orbitalSpeed = sqrt(G * centralMass / radius) * 0.95;

            ps.vx[count] = rightGalaxyVelocity.x + (-sin(angle) * orbitalSpeed);
            ps.vy[count] = rightGalaxyVelocity.y + (cos(angle) * orbitalSpeed);

            ps.mass[count] = (double)GetRandomValue(1, 3) * 0.1;
            ps.color[count] = (Color){255, 150, 100, 255};
        }
        count++;
    }

    SetTargetFPS(200);

    static cArena arena = {0}; // static storage avoids stack overflow

    while (!WindowShouldClose())
    {
        // Runtime speed controls.
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD))
            simulationSpeed += 0.25;
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT))
            simulationSpeed = fmax(0.1, simulationSpeed - 0.25);
        if (IsKeyPressed(KEY_R))
            simulationSpeed = 1.0; // Reset to normal

        // Use dynamic time step based on actual frame time
        double frameTime = GetFrameTime();
        if (frameTime > 0.02)
            frameTime = 0.02; // Change from 0.05 to 0.02
        double delta_time = frameTime * simulationSpeed;

        for (int i = 0; i < count; i++)
        {
            bodies[i].force = (vector){0, 0};
        }

        // Tight bounds cut tree depth in empty space and reduce traversal work.
        double minX = bodies[0].position.x;
        double maxX = bodies[0].position.x;
        double minY = bodies[0].position.y;
        double maxY = bodies[0].position.y;
        for (int i = 1; i < count; i++)
        {
            if (bodies[i].position.x < minX)
                minX = bodies[i].position.x;
            if (bodies[i].position.x > maxX)
                maxX = bodies[i].position.x;
            if (bodies[i].position.y < minY)
                minY = bodies[i].position.y;
            if (bodies[i].position.y > maxY)
                maxY = bodies[i].position.y;
        }

        double padding = 200.0;
        float worldX = (float)(minX - padding);
        float worldY = (float)(minY - padding);
        float worldW = (float)((maxX - minX) + padding * 2.0);
        float worldH = (float)((maxY - minY) + padding * 2.0);
        if (worldW < 400.0f)
            worldW = 400.0f;
        if (worldH < 400.0f)
            worldH = 400.0f;

        // Reset arena in O(1) before rebuilding the tree.
        arena.index = 0;
        Node *root = create_node(&arena, (Rectangle){worldX, worldY, worldW, worldH});
        if (root == NULL)
        {
            break;
        }
        for (int i = 0; i < count; i++)
        {
            insert_body(root, &bodies[i], &arena);
        }

        for (int i = 0; i < count; i++)
        {
            calculate_force_from_tree(root, &bodies[i], G, 0.5);
        }

        for (int i = 0; i < count; i++)
        {

            vector accel = {bodies[i].force.x / bodies[i].mass,
                            bodies[i].force.y / bodies[i].mass};

            bodies[i].velocity.x += accel.x * delta_time;
            bodies[i].velocity.y += accel.y * delta_time;

            bodies[i].position.x += bodies[i].velocity.x * delta_time;
            bodies[i].position.y += bodies[i].velocity.y * delta_time;
        }

        BeginDrawing();

        ClearBackground(BLACK);

        // Draw bodies.
        for (int i = 0; i < count; i++)
        {
            float radius = 3.0f + (float)sqrtf((float)bodies[i].mass) * 0.35f;
            Vector2 pos = {(float)bodies[i].position.x, (float)bodies[i].position.y};

            double speed = bodies[i].velocity.x * bodies[i].velocity.x +
                           bodies[i].velocity.y * bodies[i].velocity.y;
            float brightness = fmin(1.0f, (float)(speed / 1000.0f));

            if (bodies[i].mass >= 50.0)
            {
                // Keep glow for heavy bodies only; gradient draws are expensive.
                Color glowColor = bodies[i].color;
                glowColor.a = 40;
                DrawCircleGradient((int)pos.x, (int)pos.y, radius * 1.5f,
                                   (Color){0, 0, 0, 0}, glowColor);
            }

            // Brighten color based on velocity
            Color coreColor = bodies[i].color;
            coreColor.r = (unsigned char)fmin(255, coreColor.r + brightness * 80);
            coreColor.g = (unsigned char)fmin(255, coreColor.g + brightness * 80);
            coreColor.b = (unsigned char)fmin(255, coreColor.b + brightness * 80);

            // Draw main body
            DrawCircleV(pos, radius, coreColor);
        }

        // UI Overlay
        DrawText(TextFormat("Bodies: %d", count), 10, 10, 20, WHITE);
        DrawText(TextFormat("FPS: %d", GetFPS()), 10, 35, 20, WHITE);
        DrawText(TextFormat("Speed: %.1fx", simulationSpeed), 10, 60, 20, WHITE);
        DrawText("Press +/- to adjust speed", 10, 85, 16, WHITE);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}