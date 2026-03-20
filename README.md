# N-Body Simulation (raylib + C)

Real-time 2D galaxy collision simulation using a Barnes-Hut quadtree and an arena allocator for fast per-frame tree rebuilds.

## Features

- Two colliding galaxies with central massive bodies and orbiting particles
- Barnes-Hut force approximation for scalable gravity calculations
- Arena-based quadtree node allocation (no per-frame malloc/free churn)
- Runtime simulation speed control
- Velocity-based color brightening and glow for heavy bodies

## Project Files

- `body.c`: App entry point, initialization, update loop, rendering
- `vector.h`: Data structures and function declarations
- `vector.c`: Vector utilities, quadtree build/insert, force traversal

## Requirements

- GCC (MSYS2 MinGW64 recommended on Windows)
- raylib 5.x headers and libraries
- OpenGL/Windows linker dependencies (`opengl32`, `gdi32`, `winmm`, `m`)

## Build

```bash
gcc body.c vector.c \
  -I /c/raylib-5.5_win64_mingw-w64/include \
  -L /c/raylib-5.5_win64_mingw-w64/lib \
  -lraylib -lopengl32 -lgdi32 -lwinmm -lm \
  -o body.exe
```

## Run

```bash
./body.exe
```

## Controls

- `+` / numpad `+`: Increase simulation speed
- `-` / numpad `-`: Decrease simulation speed
- `R`: Reset speed to `1.0x`
- Close window or `Esc`: Exit

## Notes on Performance and Stability

- Quadtree nodes are allocated from a fixed arena (`MAX_NODES`) each frame.
- The arena is reset by setting its index to `0` before rebuilding the tree.
- If the arena cannot split further, insertion falls back to aggregated mass at that node.
- Dynamic world bounds are recomputed each frame from current body positions to reduce empty-space traversal.

## Tuning

Useful constants to tweak:

- In `body.c`:
  - `MAX_BODIES`
  - `particlesPerGalaxy`
  - `centralMass`
  - `G`
  - `SetTargetFPS(...)`
- In `vector.h`:
  - `MAX_NODES`
- In `vector.c`:
  - `MIN_NODE_SIZE`
