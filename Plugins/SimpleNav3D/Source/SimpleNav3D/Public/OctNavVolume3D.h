#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "OctNavVolume3D.generated.h"

class UProceduralMeshComponent;
class NavNode;

/**
 * Lightweight octree node used for coarse 3D occupancy / blockage queries.
 * Each node stores a bounding box, leaf flag, blocked flag, and up to 8 children.
 */
struct FOctreeNode
{
	/** Axis-aligned bounds represented by this node in world space. */
	FBox Bounds;

	/** Whether this node is a leaf in the octree. */
	bool bIsLeaf = true;

	/** Whether this node's region is considered blocked (e.g., overlaps geometry). */
	bool bBlocked = false;

	/** Child pointers for the 8 octants (nullptr if not present / leaf). */
	FOctreeNode* Children[8] = { nullptr };

	FOctreeNode(const FBox& InBounds)
		: Bounds(InBounds)
	{
	}

	~FOctreeNode()
	{
		for (int i = 0; i < 8; ++i)
		{
			if (Children[i])
			{
				delete Children[i];
				Children[i] = nullptr;
			}
		}
	}
};

/**
 * Path preference enum for potential future routing strategies.
 */
UENUM(BlueprintType)
enum class EPathPreference : uint8
{
	/** Prefer ground-based navigation (e.g., walking on surfaces). */
	EPP_Ground  UMETA(DisplayName = "Ground"),

	/** Prefer flying navigation (e.g., ignoring slopes/height constraints). */
	EPP_Fly     UMETA(DisplayName = "Fly"),

	/** Prefer paths that stay near a reference point (e.g., shortest / localized). */
	EPP_Near    UMETA(DisplayName = "Near"),
};

/**
 * 3D Navigation Volume
 *
 * - Builds a regular 3D grid of NavNodes in local space.
 * - Visualizes the grid with a procedural debug mesh.
 * - Builds an octree over the volume for coarse blockage queries.
 * - Provides A* pathfinding and nearest-free-node search in 3D.
 *
 * Designed to be dropped into a level as an axis-aligned navigation volume.
 */
UCLASS()
class SIMPLENAV3D_API AOctNavVolume3D : public AActor
{
	GENERATED_BODY()

public:
	/** Default constructor. Initializes components and loads debug material. */
	AOctNavVolume3D();

	//~ Begin AActor Interface
	/** Called when the actor is constructed or a property is changed in the editor. */
	virtual void OnConstruction(const FTransform& Transform) override;

	/** Per-frame update. Currently unused but available for future debug/visualization. */
	virtual void Tick(float DeltaTime) override;

protected:
	/** Called when the game starts or when the actor is spawned. */
	virtual void BeginPlay() override;

	/** Called when the actor is being removed from the world; used for cleanup. */
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	//~ End AActor Interface

public:
	// --------------------------------------------------------------------
	// Grid Metrics / Accessors
	// --------------------------------------------------------------------

	/** Returns the grid X-extent in local units (DivisionsX * DivisionSize). */
	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE float GetGridXBound() { return DivisionsX * DivisionSize; }

	/** Returns the grid Y-extent in local units (DivisionsY * DivisionSize). */
	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE float GetGridYBound() { return DivisionsY * DivisionSize; }

	/** Returns the grid Z-extent in local units (DivisionsZ * DivisionSize). */
	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE float GetGridZBound() { return DivisionsZ * DivisionSize; }

	/** Returns total number of grid cells (DivisionsX * DivisionsY * DivisionsZ). */
	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE int32 GetTotalDivisions() { return DivisionsX * DivisionsY * DivisionsZ; }

	/**
	 * Returns a pointer to the NavNode at the given grid coordinates (clamped to volume).
	 * @param Coordinates  Integer grid coordinates (X, Y, Z).
	 */
	NavNode* GetNode(FIntVector Coordinates);

	// --------------------------------------------------------------------
	// World <-> Grid Conversion
	// --------------------------------------------------------------------

	/**
	 * Converts a world-space location to clamped integer grid coordinates.
	 *
	 * @param WorldCoordinate  World-space position to convert.
	 * @return                 Grid coordinates (0..DivisionsX-1, etc.).
	 */
	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	FIntVector ConvertWorldLocationToGridCoordinates(const FVector& WorldCoordinate);

	/**
	 * Converts integer grid coordinates to a world-space position at the cell center.
	 *
	 * @param GridCoordinates  Grid-space indices (X, Y, Z).
	 * @return                 World-space location at the center of that cell.
	 */
	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	FVector ConvertGridCoordinatesToWorldLocation(const FIntVector& GridCoordinates);

	// --------------------------------------------------------------------
	// Pathfinding
	// --------------------------------------------------------------------

	/**
	 * Finds a path between a start and destination in world space using A* on the 3D grid.
	 *
	 * @param InStart               World-space start location.
	 * @param InDestination         World-space goal location.
	 * @param InObjectTypes         Object types to consider as obstacles (for overlap checks).
	 * @param InActorClassFilter    Optional actor class filter (can be nullptr).
	 * @param OutPath               Output array of world-space points forming the path.
	 * @param InActor               Optional actor to ignore and/or use for agent size.
	 * @param InDetectionRadius     Agent capsule radius used for overlap checks.
	 * @param InDetectionHalfHeight Agent capsule half-height used for overlap checks.
	 *
	 * @return true if a valid path was found, false otherwise.
	 */
	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
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

private:
	// --------------------------------------------------------------------
	// Internal Drawing Helpers (debug grid)
	// --------------------------------------------------------------------

	/**
	 * Creates a thick 3D line segment between two points and appends its geometry
	 * as quads into the provided vertex and triangle arrays.
	 *
	 * @param InStart       Start point of the line in local space.
	 * @param InEnd         End point of the line in local space.
	 * @param InUpNormal    Up direction used to construct the line's cross-section.
	 * @param OutVertices   Accumulated vertex buffer for the procedural mesh.
	 * @param OutTriangles  Accumulated index buffer for the procedural mesh.
	 */
	void CreateLine(const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles);

	// --------------------------------------------------------------------
	// Grid Coordinate Helpers
	// --------------------------------------------------------------------

	/**
	 * Returns true if the specified grid coordinates are within the volume.
	 */
	bool AreCoordinatesValid(const FIntVector& Coordinates) const;

	/**
	 * Clamps the specified grid coordinates to the valid grid range.
	 */
	void ClampCoordinatesToGrid(FIntVector& Coordinates);

	// --------------------------------------------------------------------
	// Octree Lifetime / Bounds
	// --------------------------------------------------------------------

	/**
	 * Destroys the current octree and frees all associated memory.
	 */
	void DestroyOctree();

	/**
	 * Returns the volume bounds in world space, aligned to the world axes.
	 * Ignores actor rotation and scale for simplicity.
	 */
	FBox GetWorldAlignedVolumeBox();

	// --------------------------------------------------------------------
	// Collision / Overlap Helpers
	// --------------------------------------------------------------------

	/**
	 * Tests whether a capsule at a given world location is overlapping any blocking objects.
	 *
	 * @param InAgentRadius     Capsule radius.
	 * @param AgentHalfHeight   Capsule half-height.
	 * @param IgnoreActor       Actor to ignore during the query (optional).
	 * @param WorldLocation     World-space center of the capsule.
	 * @param InObjectTypes     Object types to test against.
	 * @param InActorClassFilter Optional actor class filter (not actively used here).
	 *
	 * @return true if any overlap is detected, false otherwise.
	 */
	bool IsActorOverlapping(
		float InAgentRadius,
		float AgentHalfHeight,
		AActor* IgnoreActor,
		const FVector& WorldLocation,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter) const;

	// --------------------------------------------------------------------
	// Nearest Free Node Search
	// --------------------------------------------------------------------

	/**
	 * Performs a breadth-first search starting from InFromNode to find the nearest
	 * grid node that is not blocked by static geometry (octree) and not overlapping
	 * dynamic obstacles (via capsule overlap checks).
	 *
	 * @param InFromNode            Starting NavNode.
	 * @param IgnoredActor          Actor to ignore for overlap tests (typically the agent).
	 * @param InObjectTypes         Object types for overlap tests.
	 * @param InActorClassFilter    Optional actor class filter.
	 * @param InDetectionRadius     Capsule radius for overlap tests.
	 * @param InDetectionHalfHeight Capsule half-height for overlap tests.
	 *
	 * @return Pointer to the nearest free NavNode, or nullptr if none found.
	 */
	NavNode* FindNearestFreeNode(
		NavNode* InFromNode,
		AActor* IgnoredActor,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter,
		float InDetectionRadius,
		float InDetectionHalfHeight
	);

	// --------------------------------------------------------------------
	// Octree Construction / Query
	// --------------------------------------------------------------------

	/**
	 * Recursively builds an octree node for the given box, splitting until either
	 * the minimum cell size or maximum depth is reached.
	 *
	 * @param InBox             World-space bounds of this node.
	 * @param InDepth           Current recursion depth.
	 * @param InObjectTypes     Object types for blockage tests.
	 * @param InActorClassFilter Optional actor class filter (unused at the moment).
	 *
	 * @return Newly allocated FOctreeNode representing this region.
	 */
	FOctreeNode* BuildOctree(
		const FBox& InBox,
		int32 InDepth,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter);

	/**
	 * Tests whether a given box is considered blocked by performing an overlap query.
	 *
	 * @param InBox             World-space box to test.
	 * @param InObjectTypes     Object types to test against.
	 * @param InActorClassFilter Optional actor class filter (unused).
	 *
	 * @return true if the box overlaps any blocking object, false otherwise.
	 */
	bool IsBoxBlocked(
		const FBox& InBox,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter);

	/**
	 * Queries the octree to determine if a specific world point lies within a blocked node.
	 *
	 * @param WorldPoint World-space position to test.
	 * @return true if the point resides in a blocked leaf, false otherwise.
	 */
	bool QueryPointBlocked(const FVector& WorldPoint) const;

private:
	// --------------------------------------------------------------------
	// Components
	// --------------------------------------------------------------------

	/** Root scene component for the navigation volume. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D", meta = (AllowPrivateAccess = "true"))
	USceneComponent* DefaultRootComponent = nullptr;

	/** Procedural mesh used to render the debug grid. */
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D", meta = (AllowPrivateAccess = "true"))
	UProceduralMeshComponent* ProceduralMeshComponent = nullptr;

	// --------------------------------------------------------------------
	// Grid Settings
	// --------------------------------------------------------------------

	/** Number of divisions along the X-axis of the volume. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsX = 10;

	/** Number of divisions along the Y-axis of the volume. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsY = 10;

	/** Number of divisions along the Z-axis of the volume. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsZ = 10;

	/** Size of each grid cell along one axis, in Unreal units. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1.0))
	float DivisionSize = 100.0f;

	/**
	 * Minimum number of shared axes required to link neighbouring nodes.
	 * 0 = full 26-neighbour connectivity, 1 or 2 = more restricted connectivity.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 0, ClampMax = 2))
	int32 MinSharedNeighborAxes = 0;

	// --------------------------------------------------------------------
	// Drawing Settings (debug grid)
	// --------------------------------------------------------------------

	/** Line thickness used when drawing the debug grid. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true", ClampMin = 0))
	float LineThickness = 2.0f;

	/** Debug grid color and opacity. Alpha controls overall opacity. */
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true"))
	FLinearColor Color = FLinearColor(0.0f, 0.0f, 0.0f, 0.5f);

	// --------------------------------------------------------------------
	// Octree Settings
	// --------------------------------------------------------------------

	/**
	 * Minimum side length of an octree leaf node.
	 * By default, equal to DivisionSize so leaves are at most one grid cell in size.
	 */
	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1.0))
	float OctreeMinCellSize = 100.0f;

	/** Root node of the navigation octree (nullptr if not built). */
	FOctreeNode* OctreeRoot = nullptr;

	/** Maximum recursion depth allowed for the octree (1..10). */
	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1, ClampMax = 10))
	int32 OctreeMaxDepth = 5;

	// --------------------------------------------------------------------
	// Runtime Data
	// --------------------------------------------------------------------

	/** Contiguous array of NavNodes representing the 3D grid, used by A*. */
	NavNode* NavNodes = nullptr;
};
