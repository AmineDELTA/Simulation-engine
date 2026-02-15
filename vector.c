#include <stddef.h>
#include <stdlib.h>
#include <math.h>
#include "vector.h"

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

Node *create_node(Rectangle bounds)
{
    Node *node = (Node *)malloc(sizeof(Node));
    node->bounds = bounds;
    node->totalMass = 0.0;
    node->centerOfMass = (vector){0, 0};
    node->nw = NULL;
    node->ne = NULL;
    node->sw = NULL;
    node->se = NULL;
    node->body = NULL;
    return node;
}

void split_node(Node *n)
{
    float x = n->bounds.x;
    float y = n->bounds.y;
    float w = n->bounds.width / 2.0f;
    float h = n->bounds.height / 2.0f;

    n->nw = create_node((Rectangle){x, y, w, h});
    n->ne = create_node((Rectangle){x + w, y, w, h});
    n->sw = create_node((Rectangle){x, y + h, w, h});
    n->se = create_node((Rectangle){x + w, y + h, w, h});
}

void free_tree(Node *n)
{
    if (n == NULL)
        return;
    free_tree(n->nw);
    free_tree(n->ne);
    free_tree(n->sw);
    free_tree(n->se);
    free(n);
}

void insert_body(Node *tree, Body *body)
{
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
                insert_body(tree->nw, body);
            else
                insert_body(tree->sw, body);
        }
        else
        {
            if (body->position.y < midY)
                insert_body(tree->ne, body);
            else
                insert_body(tree->se, body);
        }
    }
    else // occupied leaf
    {
        Body *old_b = tree->body;
        tree->body = NULL;

        tree->totalMass = 0;
        tree->centerOfMass = (vector){0, 0};

        split_node(tree);

        insert_body(tree, old_b);
        insert_body(tree, body);
    }
}

void calculate_force_from_tree(Node *tree, Body *body, double G, double theta)
{
    if (tree == NULL || tree->totalMass == 0)
        return;

    vector direction = subtract_vectors(tree->centerOfMass, body->position);
    double distance = sqrt(direction.x * direction.x + direction.y * direction.y + 25.0);

    if (tree->body == body)
        return;

    if ((tree->bounds.width / distance) < theta || tree->nw == NULL)
    {
        double forceMagnitude = (G * body->mass * tree->totalMass) / (distance * distance);
        vector force = multiply_vector_by_scalar(direction, forceMagnitude / distance);
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