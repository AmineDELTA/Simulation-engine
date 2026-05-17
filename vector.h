#ifndef VECTOR_H
#define VECTOR_H
#include <raylib.h>

#define MAX_NODES 200000
// Softening constant to avoid singularities in force calculations
#define SOFTENING_EPSILON 100.0

// Forward declarations
struct node;
typedef struct node Node;

typedef struct
{
    double x;
    double y;
} vector;

typedef struct
{
    double mass;
    vector position;
    vector velocity;
    vector acceleration;
    vector force;
    Color color;
} Body;

typedef struct node
{
    Rectangle bounds;
    double totalMass;
    vector centerOfMass;
    struct node *nw, *ne, *sw, *se;
    int bodyIndex; // -1 if the node is empty or internal
} Node;

typedef struct
{
    double *x;
    double *y;
    double *vx;
    double *vy;
    double *fx;
    double *fy;
    double *mass;
    Color *color;
    int count;
} ParticleSystem;

typedef struct
{
    Node buffer[MAX_NODES];
    int index;
} cArena;

vector add_vectors(vector v1, vector v2);
vector subtract_vectors(vector v1, vector v2);
vector multiply_vector_by_scalar(vector v, double scalar);
Node *create_node(cArena *arena, Rectangle bounds);
void insert_body(Node *tree, int bodyIndex, ParticleSystem *ps, cArena *arena);
void split_node(Node *n, cArena *arena);
void calculate_force_from_tree(Node *tree, int targetIndex, ParticleSystem *ps, double G, double theta);

#endif
