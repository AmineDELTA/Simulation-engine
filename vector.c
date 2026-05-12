#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "vector.h"

#define MIN_NODE_SIZE 1.0f

vector add_vectors(vector v1, vector v2)
{
    vector result;
    result.x = v1.x + v2.x;
    result.y = v1.y + v2.y;
    return result;
}

vector subtract_vectors(vector v1, vector v2)
{
    vector result;
    result.x = v1.x - v2.x;
    result.y = v1.y - v2.y;
    return result;
}

vector multiply_vector_by_scalar(vector v, double scalar)
{
    vector result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    return result;
}

vector calculate_force(Body body1, Body body2)
{
    vector force;
    force.x = 0.0;
    force.y = 0.0;
    return force;
}

Node *create_node(cArena *arena, Rectangle bounds)
{
    if (arena == NULL || arena->index >= MAX_NODES)
    {
        return NULL;
    }

    Node *node = &arena->buffer[arena->index++];
    node->bounds = bounds;
    node->totalMass = 0.0;
    node->centerOfMass = (vector){0, 0};
    node->nw = node->ne = node->sw = node->se = NULL;
    node->bodyIndex = -1;
    return node;
}

void split_node(Node *n, cArena *arena)
{
    float x = n->bounds.x;
    float y = n->bounds.y;
    float w = n->bounds.width / 2.0f;
    float h = n->bounds.height / 2.0f;

    n->nw = create_node(arena, (Rectangle){x, y, w, h});
    n->ne = create_node(arena, (Rectangle){x + w, y, w, h});
    n->sw = create_node(arena, (Rectangle){x, y + h, w, h});
    n->se = create_node(arena, (Rectangle){x + w, y + h, w, h});
}

void insert_body(Node *tree, int bodyIndex, ParticleSystem *ps, cArena *arena)
{
    if (tree == NULL || ps == NULL || bodyIndex < 0 || bodyIndex >= ps->count)
    {
        return;
    }

    Vector2 pos = {(float)ps->x[bodyIndex], (float)ps->y[bodyIndex]};
    if (!CheckCollisionPointRec(pos, tree->bounds))
    {
        return;
    }

    if (tree->bodyIndex == -1 && tree->nw == NULL) // empty leaf
    {
        tree->centerOfMass = (vector){ps->x[bodyIndex], ps->y[bodyIndex]};
        tree->totalMass = ps->mass[bodyIndex];
        tree->bodyIndex = bodyIndex;
    }
    else if (tree->nw != NULL) // internal node
    {
        double old_m = tree->totalMass;
        double new_m = ps->mass[bodyIndex];
        tree->totalMass += new_m;

        // Update Center of Mass
        tree->centerOfMass.x = (tree->centerOfMass.x * old_m + ps->x[bodyIndex] * new_m) / tree->totalMass;
        tree->centerOfMass.y = (tree->centerOfMass.y * old_m + ps->y[bodyIndex] * new_m) / tree->totalMass;

        // Push index down to correct quadrant
        float midX = tree->bounds.x + tree->bounds.width / 2.0f;
        float midY = tree->bounds.y + tree->bounds.height / 2.0f;

        if (ps->x[bodyIndex] < midX)
        {
            if (ps->y[bodyIndex] < midY)
                insert_body(tree->nw, bodyIndex, ps, arena);
            else
                insert_body(tree->sw, bodyIndex, ps, arena);
        }
        else
        {
            if (ps->y[bodyIndex] < midY)
                insert_body(tree->ne, bodyIndex, ps, arena);
            else
                insert_body(tree->se, bodyIndex, ps, arena);
        }
    }
    else // occupied leaf
    {
        if (tree->bodyIndex == bodyIndex)
        {
            return; // same body, do nothing
        }

        // Split the node and reinsert the existing body
        int existingBodyIndex = tree->bodyIndex;
        tree->bodyIndex = -1; // mark as internal node
        split_node(tree, arena);
        insert_body(tree, existingBodyIndex, ps, arena);
        insert_body(tree, bodyIndex, ps, arena);
    }
}

void calculate_force_from_tree(Node *tree, int targetIndex, ParticleSystem *ps, double G, double theta)
{
    if (tree == NULL || tree->totalMass == 0 || tree->bodyIndex == targetIndex)
        return;

    // Get vector from target particle to tree node center of mass
    vector direction = {tree->centerOfMass.x - ps->x[targetIndex],
                        tree->centerOfMass.y - ps->y[targetIndex]};

    // Softening factor (+100.0) prevents division by zero
    double distSq = direction.x * direction.x + direction.y * direction.y + 100.0;
    double widthSq = (double)tree->bounds.width * (double)tree->bounds.width;

    // Barnes-Hut Criterion
    if (widthSq < (theta * theta * distSq) || tree->nw == NULL)
    {
        double invDist = 1.0 / sqrt(distSq);
        double forceMag = (G * ps->mass[targetIndex] * tree->totalMass) / distSq;

        // Accumulate into the flat force arrays
        ps->fx[targetIndex] += direction.x * forceMag * invDist;
        ps->fy[targetIndex] += direction.y * forceMag * invDist;
    }
    else
    {
        calculate_force_from_tree(tree->nw, targetIndex, ps, G, theta);
        calculate_force_from_tree(tree->ne, targetIndex, ps, G, theta);
        calculate_force_from_tree(tree->sw, targetIndex, ps, G, theta);
        calculate_force_from_tree(tree->se, targetIndex, ps, G, theta);
    }
}