#ifndef VECTOR_H
#define VECTOR_H
#include <raylib.h>

typedef struct
{
    double x;
    double y;
} vector;

typedef struct
{
    double mass;
    vector position;
    vector velocity; // vector2???
    vector acceleration;
    vector force; //??
    Color color;
} Body;

typedef struct node
{
    Rectangle bounds;
    double totalMass;
    vector centerOfMass;
    struct node *nw;
    struct node *ne;
    struct node *sw;
    struct node *se;
    Body *body; // Pointer to body if it's a leaf node, otherwise NULL
} Node;

vector add_vectors(vector v1, vector v2);
vector subtract_vectors(vector v1, vector v2);
vector multiply_vector_by_scalar(vector v, double scalar);
vector calculate_force(Body body1, Body body2);
Node *create_node(Rectangle bounds);
void insert_body(Node *tree, Body *body);
void split_node(Node *n);
void calculate_force_from_tree(Node *tree, Body *body, double G, double theta);
void free_tree(Node *tree);

#endif