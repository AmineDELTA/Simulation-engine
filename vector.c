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
    node->body = NULL;
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

void insert_body(Node *tree, Body *body, cArena *arena)
{
    if (tree == NULL || body == NULL)
    {
        return;
    }

    Vector2 pos = {(float)body->position.x, (float)body->position.y};
    if (!CheckCollisionPointRec(pos, tree->bounds))
    {
        return;
    }

    if (tree->body == NULL && tree->nw == NULL) // empty leaf
    {
        tree->centerOfMass = body->position;
        tree->totalMass = body->mass;
        tree->body = body;
    }
    else if (tree->nw != NULL) // internal node
    {
        double old_m = tree->totalMass;
        tree->totalMass += body->mass;
        tree->centerOfMass.x = (tree->centerOfMass.x * old_m + body->position.x * body->mass) / tree->totalMass;
        tree->centerOfMass.y = (tree->centerOfMass.y * old_m + body->position.y * body->mass) / tree->totalMass;

        float midX = tree->bounds.x + tree->bounds.width / 2.0;
        float midY = tree->bounds.y + tree->bounds.height / 2.0;

        if (body->position.x < midX)
        {
            if (body->position.y < midY)
                insert_body(tree->nw, body, arena);
            else
                insert_body(tree->sw, body, arena);
        }
        else
        {
            if (body->position.y < midY)
                insert_body(tree->ne, body, arena);
            else
                insert_body(tree->se, body, arena);
        }
    }
    else // occupied leaf
    {
        Body *old_b = tree->body;

        // If two bodies are effectively at the same position, do not keep subdividing forever.
        if (fabs(old_b->position.x - body->position.x) < 1e-9 &&
            fabs(old_b->position.y - body->position.y) < 1e-9)
        {
            double total = old_b->mass + body->mass;
            tree->centerOfMass.x = (old_b->position.x * old_b->mass + body->position.x * body->mass) / total;
            tree->centerOfMass.y = (old_b->position.y * old_b->mass + body->position.y * body->mass) / total;
            tree->totalMass = total;
            tree->body = NULL;
            return;
        }

        // Stop subdividing once the cell is too small to represent more detail.
        if (tree->bounds.width <= MIN_NODE_SIZE || tree->bounds.height <= MIN_NODE_SIZE)
        {
            double total = old_b->mass + body->mass;
            tree->centerOfMass.x = (old_b->position.x * old_b->mass + body->position.x * body->mass) / total;
            tree->centerOfMass.y = (old_b->position.y * old_b->mass + body->position.y * body->mass) / total;
            tree->totalMass = total;
            tree->body = NULL;
            return;
        }

        tree->body = NULL;

        tree->totalMass = 0;
        tree->centerOfMass = (vector){0, 0};

        split_node(tree, arena);

        if (tree->nw == NULL || tree->ne == NULL || tree->sw == NULL || tree->se == NULL)
        {
            // Arena exhausted: fallback to aggregate mass in this node.
            double total = old_b->mass + body->mass;
            tree->centerOfMass.x = (old_b->position.x * old_b->mass + body->position.x * body->mass) / total;
            tree->centerOfMass.y = (old_b->position.y * old_b->mass + body->position.y * body->mass) / total;
            tree->totalMass = total;
            tree->body = NULL;
            tree->nw = tree->ne = tree->sw = tree->se = NULL;
            return;
        }

        insert_body(tree, old_b, arena);
        insert_body(tree, body, arena);
    }
}

void calculate_force_from_tree(Node *tree, Body *body, double G, double theta)
{
    if (tree == NULL || tree->totalMass == 0)
        return;

    vector direction = subtract_vectors(tree->centerOfMass, body->position);
    double distSq = direction.x * direction.x + direction.y * direction.y + 100.0;
    double widthSq = (double)tree->bounds.width * (double)tree->bounds.width;
    double thetaSq = theta * theta;

    if (tree->body == body)
        return;

    if (widthSq < (thetaSq * distSq) || tree->nw == NULL)
    {
        double invDist = 1.0 / sqrt(distSq);
        double forceMagnitude = (G * body->mass * tree->totalMass) / distSq;
        vector force = multiply_vector_by_scalar(direction, forceMagnitude * invDist);
        body->force = add_vectors(body->force, force);
    }
    else
    {
        calculate_force_from_tree(tree->nw, body, G, theta);
        calculate_force_from_tree(tree->ne, body, G, theta);
        calculate_force_from_tree(tree->sw, body, G, theta);
        calculate_force_from_tree(tree->se, body, G, theta);
    }
}
