#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "OctNavVolume3D.generated.h"


class UProceduralMeshComponent;
class NavNode;

// Octave Tree Node
struct FOctreeNode
{
	FBox Bounds;
	bool bIsLeaf = true;
	bool bBlocked = false;
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


UENUM(BlueprintType)
enum class EPathPreference : uint8
{
	EPP_Ground  UMETA(DisplayName = "Ground"),
	EPP_Fly   UMETA(DisplayName = "Fly"),
	EPP_Near  UMETA(DisplayName = "Near"),
};


UCLASS()
class SIMPLENAV3D_API AOctNavVolume3D :public AActor
{
	GENERATED_BODY()

public:
	AOctNavVolume3D();

public:
	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void Tick(float DeltaTime) override;


	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE  float GetGridYBound()  { return DivisionsY * DivisionSize; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE  float GetGridXBound()  { return DivisionsX * DivisionSize; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE  float GetGridZBound()  { return DivisionsZ * DivisionSize; }

	UFUNCTION(BlueprintPure, Category = "SimpleOctaNavVolume3D")
	FORCEINLINE  int32 GetTotalDivisions()  { return DivisionsX * DivisionsY * DivisionsZ; }

	NavNode* GetNode(FIntVector Coordinates);

	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	FIntVector ConvertWorldLocationToGridCoordinates(const FVector& WorldCoordinate);

	UFUNCTION(BlueprintCallable, Category = "SimpleOctaNavVolume3D")
	FVector ConvertGridCoordinatesToWorldLocation(const FIntVector& GridCoordinates);

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

protected:
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
	void CreateLine(const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal, TArray<FVector>& OutVertices, TArray<int32>& outTriangles);
	bool AreCoordinatesValid(const FIntVector& Coordinates) const;
	void ClampCoordinatesToGrid(FIntVector& Coordinates);

	void DestroyOctree();
	FBox GetWorldAlignedVolumeBox();

	bool IsActorOverlapping(float InAgentRadius, 
		float AgentHalfHeight,
		AActor* IgnoreActor,
		const FVector& WorldLocation,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter) const;

	NavNode* FindNearestFreeNode(
		NavNode* InFromNode,
		AActor* IgnoredActor,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter,
		float InDetectionRadius,
		float InDetectionHalfHeight
	);



private:
	FOctreeNode* BuildOctree(const FBox& InBox, int32 InDepth, 
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter);

	bool IsBoxBlocked(const FBox& InBox,
		const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
		UClass* InActorClassFilter);

	bool QueryPointBlocked(const FVector& WorldPoint) const;

	
private:
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D", meta = (AllowPrivateAccess = "true"))
	USceneComponent* DefaultRootComponent = nullptr;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D", meta = (AllowPrivateAccess = "true"))
	UProceduralMeshComponent* ProceduralMeshComponent = nullptr;

	/*Grid Settings*/
private:
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsX = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsY = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1))
	int32 DivisionsZ = 10;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "OctNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 1.0))
	float DivisionSize = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Pathfinding", meta = (AllowPrivateAccess = "true", ClampMin = 0, ClampMax = 2))
	int32 MinSharedNeighborAxes = 0;

	//Drawing Settings
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true", ClampMin = 0))
	float LineThickness = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "SimpleOctaNavVolume3D|Aesthetics", meta = (AllowPrivateAccess = "true"))
	FLinearColor Color = FLinearColor(0.0f, 0.0f, 0.0f, 0.5f);

	// Same with the division size by default, the leaf's size is at least this size
	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1.0))
	float OctreeMinCellSize = 100.0f; 

private:
	FOctreeNode* OctreeRoot = nullptr;

	UPROPERTY(EditAnywhere, Category = "SimpleOctaNavVolume3D|Octree", meta = (ClampMin = 1, ClampMax = 10))
	int32 OctreeMaxDepth = 5;

private:
	//Nodes for A*
	NavNode* NavNodes = nullptr;
};


