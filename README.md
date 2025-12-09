# SimpleNav3D & Object Pool – Unreal Engine 5 Gameplay Systems

A small Unreal Engine 5 project showcasing **two reusable gameplay systems** implemented in modern C++:

- **SimpleNav3D** – a lightweight 3D grid navigation volume with A* pathfinding and an octree-based blocker query.
- **Object Pool** – a generic pooling system for reusing Actors/objects to avoid repeated allocations and `SpawnActor` calls.

Both systems are written as UE plugins and designed to be dropped into a game project.
This project follows the overall architecture posted in this repo: https://github.com/hpnever/NavAndPool

---

## 1. Overview

This repository is focused on **systems-level gameplay code**, not on building a full game.  

The goal is to demonstrate that I can:

- Design and implement **reusable engine subsystems** in C++.
- Work comfortably with **Unreal’s API**, memory management, and math.
- Apply **classic CS concepts** (A*, BFS, spatial partitioning, pooling) to game problems.

If you’re a hiring manager or gameplay programmer, this project is meant to be a quick, concrete sample of how I write engine-facing gameplay code.

---

## 2. What’s Included

### 🧭 SimpleNav3D – 3D Navigation Volume

A custom 3D navigation system based on a **voxel-like grid**:

- **3D grid volume** defined by `DivisionsX / Y / Z` and `DivisionSize`.
- **NavNode graph**:
  - Each cell is a `NavNode` with integer grid coordinates.
  - Neighbours are precomputed based on a configurable shared-axis rule.
- **A* pathfinding**:
  - Uses a `std::priority_queue` with a custom `NavNodeCompare`.
  - Heuristic based on Euclidean distance in grid space.
- **Octree for spatial queries**:
  - `FOctreeNode` hierarchy built over the navigation volume.
  - Each leaf stores a `bBlocked` flag using UE collision overlap tests.
  - `QueryPointBlocked` quickly rejects nodes inside blocked boxes.
- **Nearest free node search**:
  - BFS (`TQueue`) from a starting node to find the closest valid, non-overlapping node.
  - Avoids paths starting/ending inside walls or other blocking geometry.
- **Debug visualization**:
  - Grid lines rendered at runtime via `UProceduralMeshComponent`.
  - Color and line thickness configurable in the editor.

Core public API lives in `AOctNavVolume3D`:

```cpp
// Convert between world and grid coordinates
FIntVector ConvertWorldLocationToGridCoordinates(const FVector& WorldLocation);
FVector ConvertGridCoordinatesToWorldLocation(const FIntVector& GridCoordinates);

// Compute a navigation path
bool FindPath(
    const FVector& InStart,
    const FVector& InDestination,
    const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
    UClass* InActorClassFilter,
    TArray<FVector>& OutPath,
    AActor* InActor = nullptr,
    float InDetectionRadius = 34.f,
    float InDetectionHalfHeight = 44.f
);
```

---

### ♻️ Object Pool – Reusable Actor/Object Pooling(Working In Progress, Example usages will be added later)

A generic **object pooling system** for Unreal, designed to:

- Reduce repeated `NewObject` / `SpawnActor` calls.
- Reuse existing instances for effects, projectiles, temporary actors, etc.
- Provide a **simple C++ and Blueprint-friendly API**.

High-level features:

- Pre-warm pools with a configurable initial size.
- Borrow / return objects at runtime.
- Optional auto-return based on game logic (e.g. on hit, on lifetime expiry).
- Designed to be **data-driven** and reusable across multiple projects.


---

## 3. Technical Highlights

This project demonstrates:

- **Modern C++ in Unreal**
  - Usage of STL containers (`std::vector`, `std::priority_queue`, `std::unordered_map`, `std::unordered_set`) where it makes sense.
  - Lambda expressions for small, focused helpers (e.g., neighbour updates, quad creation).
  - Clear ownership and cleanup (`new[]` / `delete[]` for `NavNodes`, recursive destructor for `FOctreeNode`).

- **Algorithms & Data Structures**
  - **A*** algorithm on a custom graph (`NavNode` grid).
  - **BFS** for nearest free node search.
  - **Octree** for spatial partitioning and fast blocker queries.
  - **Object pooling** pattern to minimize allocations and improve runtime performance.

- **Unreal Engine 5 Integration**
  - `AActor`-based navigation volume exposed to editor & Blueprints.
  - `UProceduralMeshComponent` used for debug grid rendering.
  - UE collision queries (`OverlapAnyTestByObjectType`, `OverlapMultiByObjectType`) for blocked node / box detection.
  - Configurable properties via `UPROPERTY` with categories and clamping.
  - Editor-only validation logs (`WITH_EDITOR` checks for rotation/scale).

---

## 4. Example Usage

### 4.1 Using the 3D Nav Volume in Gameplay

1. Place an `OctNavVolume3D` actor in the level.
2. Set `DivisionsX / Y / Z` and `DivisionSize` to cover your navigable space.
3. Configure collision channels used to determine blocked cells.
4. At runtime, call `FindPath` from C++ (or a wrapper Blueprint function) to get a list of waypoints:

```cpp
TArray<FVector> Path;
if (NavVolume->FindPath(StartLocation, TargetLocation, ObjectTypes, nullptr, Path, ControlledPawn))
{
    // Follow Path array with your movement logic
}
```

The function internally:

- Snap start/destination to grid.
- Adjusts the goal if it’s inside a blocked region (octree + overlap check).
- Runs A* on the 3D grid graph.
- Returns a list of world-space positions for the AI to follow.

---

## 6. Getting Started

> **Engine Version:** Unreal Engine 5.x  

1. Clone the repository.
2. Generate project files (`.uproject` → “Generate Visual Studio project files”).
3. Build the project in your IDE.
4. Open the project in the Unreal Editor.
5. Enable the **SimpleNav3D** and **ObjectPool** plugins if they aren’t already enabled.
6. Open the example map (if provided) or:
   - Drag an `OctNavVolume3D` into the level.
   - Set grid/division settings.
   - Hook pathfinding and pooling into your AI / gameplay code.

---

## 7. What This Project Says About Me

- Comfortable with **low-level gameplay systems** (navigation, pooling, spatial structures).
- Practical experience combining **CS fundamentals** with **UE5 engine APIs**.
- Focus on **readable, documented, and reusable C++** suitable for production codebases.
- Interested in **gameplay programming**, **AI navigation**, and **engine-adjacent tools** rather than only one-off game jam prototypes.

---


