#include "raylib.h"
#include "vector.h"
#include <math.h>
#include <stddef.h>
#define MAX_BODIES 10000

int main(void)
{
    const int screenWidth = 1600;
    const int screenHeight = 1200;

    InitWindow(screenWidth, screenHeight, "N-Body Simulation");

    double G = 100.0;
    double delta_time = 0.1;
    Body bodies[MAX_BODIES];
    int count = 0;

    // Large central body
    bodies[count].position = (vector){500, 400};
    bodies[count].velocity = (vector){0, 0};
    bodies[count].mass = 10200.0f;
    bodies[count].color = GREEN;
    count++;

    for (int i = 1; i < 3500; i++)
    {
        bodies[i].position.x = GetRandomValue(300, screenWidth - 300);
        bodies[i].position.y = GetRandomValue(200, screenHeight - 200);

        bodies[i].velocity.x = (float)GetRandomValue(-5, 50) / 10.0f;
        bodies[i].velocity.y = (float)GetRandomValue(-5, 10) / 10.0f;

        bodies[i].mass = (float)GetRandomValue(10, 40);
        bodies[i].color = WHITE;
        count++;
    }

    SetTargetFPS(60);

    while (!WindowShouldClose())
    {

        for (int i = 0; i < count; i++)
        {
            bodies[i].force = (vector){0, 0};
        }

        Node *root = create_node((Rectangle){0, 0, screenWidth, screenHeight});
        for (int i = 0; i < count; i++)
        {
            insert_body(root, &bodies[i]);
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

        for (int i = 0; i < count; i++)
        {
            // we are using sqrt scale so big masses look noticeably larger.
            float radius = 3.0f + (float)sqrtf((float)bodies[i].mass) * 0.35f;

            DrawCircleV((Vector2){(float)bodies[i].position.x, (float)bodies[i].position.y}, radius, bodies[i].color);
        }

        EndDrawing();
        free_tree(root);
    }

    CloseWindow();

    return 0;
}