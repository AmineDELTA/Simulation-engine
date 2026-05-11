# N-Body Simulation (raylib + C)

Real-time 2D galaxy collision simulation using a Barnes-Hut quadtree and an arena allocator for fast per-frame tree rebuilds.

> [!NOTE]  
> **Performance Note:** Please disregard the FPS counter in the recording. The framerate is throttled by the screen capture software and hardware constraints; the underlying simulation logic runs significantly faster.

https://github.com/user-attachments/assets/95679dd0-ef0f-4b37-819c-9ff9b6489314

To handle thousands of particles in real-time, the engine implements the Barnes-Hut algorithm. Instead of calculating forces between every individual pair of particles—an $O(n^2)$ operation—we use a Quadtree to recursively divide the 2D space. By grouping distant particles into internal nodes of the tree, the engine treats them as a single center of mass. This reduces the force calculation complexity to $O(n \log n)$, allowing for the simulation of over 9,000+ particles.

<table align="center">
  <tr>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/7a2d3396-8f04-4c4f-8b6d-7230c8363ff3" width="450">
      <br />
      <sub>Spatial Decomposition</sub>
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/fab04553-4af3-4285-b29e-6667a493f460" width="450">
      <br />
      <sub>Quadtree Structure</sub>
    </td>
  </tr>
</table>



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
