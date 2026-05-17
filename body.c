#include "raylib.h"
#include "vector.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
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
    // We'll use SOA arrays in `ps` as the primary storage.
    // The old AOS `Body *bodies` is removed.
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
            count++;
        }
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
            ps.fx[i] = 0.0;
            ps.fy[i] = 0.0;
        }

        // Tight bounds cut tree depth in empty space and reduce traversal work.
        double minX = ps.x[0];
        double maxX = ps.x[0];
        double minY = ps.y[0];
        double maxY = ps.y[0];
        for (int i = 1; i < count; i++)
        {
            if (ps.x[i] < minX)
                minX = ps.x[i];
            if (ps.x[i] > maxX)
                maxX = ps.x[i];
            if (ps.y[i] < minY)
                minY = ps.y[i];
            if (ps.y[i] > maxY)
                maxY = ps.y[i];
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
            insert_body(root, i, &ps, &arena);
        }

        for (int i = 0; i < count; i++)
        {
            calculate_force_from_tree(root, i, &ps, G, 0.5);
        }

        for (int i = 0; i < count; i++)
        {
            double ax = ps.fx[i] / ps.mass[i];
            double ay = ps.fy[i] / ps.mass[i];

            ps.vx[i] += ax * delta_time;
            ps.vy[i] += ay * delta_time;

            ps.x[i] += ps.vx[i] * delta_time;
            ps.y[i] += ps.vy[i] * delta_time;
        }

        BeginDrawing();

        ClearBackground(BLACK);

        // Draw bodies.
        for (int i = 0; i < count; i++)
        {
            float radius = 3.0f + (float)sqrtf((float)ps.mass[i]) * 0.35f;
            Vector2 pos = {(float)ps.x[i], (float)ps.y[i]};

            double speed = ps.vx[i] * ps.vx[i] + ps.vy[i] * ps.vy[i];
            float brightness = fmin(1.0f, (float)(speed / 1000.0f));

            if (ps.mass[i] >= 50.0)
            {
                Color glowColor = ps.color[i];
                glowColor.a = 40;
                DrawCircleGradient((int)pos.x, (int)pos.y, radius * 1.5f,
                                   (Color){0, 0, 0, 0}, glowColor);
            }

            Color coreColor = ps.color[i];
            coreColor.r = (unsigned char)fmin(255, coreColor.r + brightness * 80);
            coreColor.g = (unsigned char)fmin(255, coreColor.g + brightness * 80);
            coreColor.b = (unsigned char)fmin(255, coreColor.b + brightness * 80);

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