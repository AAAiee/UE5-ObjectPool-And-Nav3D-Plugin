#include "OctNavVolume3D.h"
#include "NavNode.h"
#include "ProceduralMeshComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Containers/Queue.h" // TQueue

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <utility>


#include "Kismet/KismetMathLibrary.h"
#include "Kismet/KismetSystemLibrary.h"

#include "Components/CapsuleComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/Actor.h"
#include "Engine/OverlapResult.h"
#include "Engine/World.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"

static UMaterial* GridMaterial = nullptr;


AOctNavVolume3D::AOctNavVolume3D()
{
	PrimaryActorTick.bCanEverTick = true;

	DefaultRootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("DefaultRootComponent"));
	SetRootComponent(DefaultRootComponent);

	ProceduralMeshComponent = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
	ProceduralMeshComponent->SetupAttachment(DefaultRootComponent);
	ProceduralMeshComponent->SetCastShadow(false);
	ProceduralMeshComponent->SetEnableGravity(false);
	ProceduralMeshComponent->bApplyImpulseOnDamage = false;
	ProceduralMeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	ProceduralMeshComponent->SetGenerateOverlapEvents(false);
	ProceduralMeshComponent->CanCharacterStepUpOn = ECanBeCharacterBase::ECB_No;

	//Only shows the box
	ProceduralMeshComponent->SetHiddenInGame(false);

	static ConstructorHelpers::FObjectFinder<UMaterial> MaterialFinder(TEXT("Material'/SimpleNav3D/M_Nav.M_Nav'"));

	checkf(MaterialFinder.Succeeded(), TEXT("Could not find grid material for SimpleNav3D plugin. Make sure the SimpleNav3D plugin is correctly installed."));

	GridMaterial = MaterialFinder.Object;
}

void AOctNavVolume3D::BeginPlay()
{
	Super::BeginPlay();

	//Allocate Nodes here
	const int32 TotalNodes = GetTotalDivisions();
	NavNodes = new NavNode[TotalNodes];


	static const TArray<FIntVector> NeighbourOffsets = {
		// Above (z + 1)
	   { 1, -1,  1}, { 1,  0,  1}, { 1,  1,  1},
	   { 0, -1,  1}, { 0,  0,  1}, { 0,  1,  1},
	   {-1, -1,  1}, {-1,  0,  1}, {-1,  1,  1},

	   // Middle (z)
	   { 1, -1,  0}, { 1,  0,  0}, { 1,  1,  0},
	   { 0, -1,  0},               { 0,  1,  0},
	   {-1, -1,  0}, {-1,  0,  0}, {-1,  1,  0},

	   // Below (z - 1)
	   { 1, -1, -1}, { 1,  0, -1}, { 1,  1, -1},
	   { 0, -1, -1}, { 0,  0, -1}, { 0,  1, -1},
	   {-1, -1, -1}, {-1,  0, -1}, {-1,  1, -1}
	};

	auto AddNeighbour = [this](NavNode* Node, const FIntVector Offset)
		{
			const  FIntVector NCoord = Node->Coordinates + Offset;

			if (!this->AreCoordinatesValid(NCoord))
			{
				return;
			}

			int8 SharedAxes = 0;
			if (Node->Coordinates.X == NCoord.X) SharedAxes++;
			if (Node->Coordinates.Y == NCoord.Y) SharedAxes++;
			if (Node->Coordinates.Z == NCoord.Z) SharedAxes++;

			if (SharedAxes >= this->MinSharedNeighborAxes && SharedAxes < 3)
			{
				Node->Neighbours.push_back(GetNode(NCoord));
			}
		};


	// Populate all nodes and add their corresponding neighbors
	for (int32 Z = 0; Z < DivisionsZ; ++Z)
	for (int32 Y = 0; Y < DivisionsY; ++Y)
	for (int32 X = 0; X < DivisionsX; ++X)
	{
		NavNode* Node = GetNode({ X,Y,Z });
		Node->Coordinates = { X, Y, Z };

		for (const FIntVector& Offset : NeighbourOffsets)
		{
			// Add all neighbors that share at least MinSharedNeighborAxes axes
			AddNeighbour(Node, Offset);
		}
	}

	//Build the Octree for coarse collision testing
	DestroyOctree(); // first delete the original tree 
	OctreeMinCellSize = FMath::Max(OctreeMinCellSize, DivisionSize);
	const FBox EntireGridBoxInWorld = GetWorldAlignedVolumeBox();
	OctreeRoot = BuildOctree(EntireGridBoxInWorld, 0, TArray<TEnumAsByte<EObjectTypeQuery>>(), nullptr);
}

void AOctNavVolume3D::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	DestroyOctree();
	
	delete[] NavNodes;
	NavNodes = nullptr;

	Super::EndPlay(EndPlayReason);
}

void AOctNavVolume3D::CreateLine(const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles)
{

	const float HalfThickness = LineThickness * 0.5f;

	FVector LineDir = InEnd - InStart;
	if (!LineDir.Normalize())
	{
		// Degenerate line : start almost equals to end, skip drawing 
		return;
	}

	FVector SideDir1 = FVector::CrossProduct(LineDir, InUpNormal);
	if (!SideDir1.Normalize())
	{
		SideDir1 = FVector::CrossProduct(LineDir, FVector::UpVector).GetSafeNormal();
	}

	FVector SideDir2 = FVector::CrossProduct(LineDir, SideDir1).GetSafeNormal();

	auto AddQuad = [&OutVertices, &OutTriangles, &InStart, &InEnd, HalfThickness](const FVector& ThicknessDirection)
		{
			const int32 BaseIndex = OutVertices.Num();

			OutVertices.Add(InStart + ThicknessDirection * HalfThickness);
			OutVertices.Add(InEnd + ThicknessDirection * HalfThickness);
			OutVertices.Add(InStart - ThicknessDirection * HalfThickness);
			OutVertices.Add(InEnd - ThicknessDirection * HalfThickness);

			OutTriangles.Add(BaseIndex + 2);
			OutTriangles.Add(BaseIndex + 1);
			OutTriangles.Add(BaseIndex + 0);

			OutTriangles.Add(BaseIndex + 2);
			OutTriangles.Add(BaseIndex + 3);
			OutTriangles.Add(BaseIndex + 1);
		};

	AddQuad(SideDir1);
	AddQuad(SideDir2);
}

bool AOctNavVolume3D::AreCoordinatesValid(const FIntVector& Coordinates) const
{
	return Coordinates.X >=0 && Coordinates.X < DivisionsX
		&& Coordinates.Y >= 0 && Coordinates.Y < DivisionsY
		&& Coordinates.Z >= 0 && Coordinates.Z < DivisionsZ;
}

void AOctNavVolume3D::ClampCoordinatesToGrid(FIntVector& Coordinates)
{
	Coordinates.X = FMath::Clamp(Coordinates.X, 0, DivisionsX - 1);
	Coordinates.Y = FMath::Clamp(Coordinates.Y, 0, DivisionsY - 1);
	Coordinates.Z = FMath::Clamp(Coordinates.Z, 0, DivisionsZ - 1);
}

void AOctNavVolume3D::DestroyOctree()
{
	if (OctreeRoot)
	{
		delete OctreeRoot;
		OctreeRoot = nullptr;
	}
}

FBox AOctNavVolume3D::GetWorldAlignedVolumeBox() 
{
	// Local grid bounds
	FBox LocalBox(
		FVector::ZeroVector,
		FVector(GetGridXBound(), GetGridYBound(), GetGridZBound())
	);

	// Only apply translation -- ignore rotation & scale
	const FVector WorldOffset = GetActorLocation();

	return LocalBox.ShiftBy(WorldOffset);
}

FIntVector AOctNavVolume3D::ConvertWorldLocationToGridCoordinates(const FVector& WorldCoordinate)
{
	FTransform GridTransform = GetActorTransform();
	const FVector GridSpacePos = UKismetMathLibrary::InverseTransformLocation(GridTransform, WorldCoordinate);


	FIntVector GridSpaceCoords;

	GridSpaceCoords.X = FMath::Clamp(FMath::FloorToInt(GridSpacePos.X / DivisionSize), 0, DivisionsX - 1);
	GridSpaceCoords.Y = FMath::Clamp(FMath::FloorToInt(GridSpacePos.Y / DivisionSize), 0, DivisionsY - 1);
	GridSpaceCoords.Z = FMath::Clamp(FMath::FloorToInt(GridSpacePos.Z / DivisionSize), 0, DivisionsZ - 1);

	return GridSpaceCoords;
}

FVector AOctNavVolume3D::ConvertGridCoordinatesToWorldLocation(const FIntVector& GridCoordinates) 
{
	FIntVector GridCoordsCopy = GridCoordinates;
	ClampCoordinatesToGrid(GridCoordsCopy);

	FVector GridSpacePos(0.0f, 0.0f, 0.0f);
	const float EdgeToCenterOffset = DivisionSize * 0.5f;
	GridSpacePos.X = (GridCoordsCopy.X * DivisionSize) + EdgeToCenterOffset;
	GridSpacePos.Y = (GridCoordsCopy.Y * DivisionSize) + EdgeToCenterOffset;
	GridSpacePos.Z = (GridCoordsCopy.Z * DivisionSize) + EdgeToCenterOffset;

	return UKismetMathLibrary::TransformLocation(GetActorTransform(), GridSpacePos);
}

bool AOctNavVolume3D::IsActorOverlapping(float InAgentRadius, float InAgentHalfHeight,
	AActor* IgnoreActor, const FVector& InWorldLocation, const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes, UClass* InActorClassFilter) const
{
	FCollisionObjectQueryParams ObjectParams;
	for (TEnumAsByte<EObjectTypeQuery> ObjType : InObjectTypes)
	{
		ObjectParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjType));
	}

	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(IsActorOverlappingTest), false);
	QueryParams.bFindInitialOverlaps = true;
	if (IgnoreActor)
	{
		QueryParams.AddIgnoredActor(IgnoreActor);
	}

	const FCollisionShape CapsuleShape =
		FCollisionShape::MakeCapsule(InAgentRadius, InAgentHalfHeight);

	TArray<FOverlapResult> OverlapResults;
	const bool bHit = GetWorld()->OverlapMultiByObjectType(
		OverlapResults,
		InWorldLocation,
		FQuat::Identity,
		ObjectParams,
		CapsuleShape,
		QueryParams
	);

	if (!bHit)
	{
		return false;
	}
	return true;
}

NavNode* AOctNavVolume3D::FindNearestFreeNode(NavNode* InFromNode,
	AActor* IgnoredActor,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter,
	float InDetectionRadius, float InDetectionHalfHeight)
{
	if (!InFromNode || AreCoordinatesValid(InFromNode->Coordinates) == false)
	{
		return nullptr;
	}

	TQueue<NavNode*> Queue;
	TSet<NavNode*> Visited;

	Queue.Enqueue(InFromNode);
	Visited.Add(InFromNode);

	while (!Queue.IsEmpty())
	{
		NavNode* CurNode;
		Queue.Dequeue(CurNode);

		FVector NodeWorldLocation = ConvertGridCoordinatesToWorldLocation(CurNode->Coordinates);

		if (OctreeRoot && QueryPointBlocked(NodeWorldLocation))
		{

			
		} 
		else
		{
			if (!IsActorOverlapping(InDetectionRadius, InDetectionHalfHeight, IgnoredActor, NodeWorldLocation
				, InObjectTypes, InActorClassFilter))
			{
				return CurNode;
			}

		}

		for (NavNode* Neighbour : CurNode->Neighbours)
		{
			if (!Visited.Contains(Neighbour))
			{
				Queue.Enqueue(Neighbour);
				Visited.Add(Neighbour);
			}
		}
	}

	return nullptr;
}

FOctreeNode* AOctNavVolume3D::BuildOctree(const FBox& InBox,
	int32 InDepth, const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter)
{
	// create node 
	FOctreeNode* TreeNode = new FOctreeNode(InBox);

	// Leaf Condition
	const FVector BoxSize = InBox.GetSize();
	const float MaxSideLength = FMath::Max3(BoxSize.X, BoxSize.Y, BoxSize.Z);
	
	const bool bSmallEnough = (MaxSideLength <= OctreeMinCellSize + KINDA_SMALL_NUMBER);
	const bool bMaxDepthReached = (InDepth >= OctreeMaxDepth);

	// if this node is a leaf node, update its blocked status
	if (bSmallEnough || bMaxDepthReached)
	{
		TreeNode->bIsLeaf = true;
		TreeNode->bBlocked = IsBoxBlocked(InBox, InObjectTypes, InActorClassFilter);
		return TreeNode;
	}

	TreeNode->bIsLeaf = false;
	const FVector C = InBox.GetCenter();
	const FVector Min = InBox.Min;
	const FVector Max = InBox.Max;

	FBox ChildBoxes[8] = {
	FBox(FVector(Min.X, Min.Y, Min.Z), FVector(C.X,   C.Y,   C.Z)),   // 0
	FBox(FVector(C.X,  Min.Y, Min.Z), FVector(Max.X,  C.Y,   C.Z)),   // 1
	FBox(FVector(Min.X, C.Y,  Min.Z), FVector(C.X,    Max.Y,  C.Z)),  // 2
	FBox(FVector(C.X,  C.Y,  Min.Z), FVector(Max.X,   Max.Y,  C.Z)),  // 3
	FBox(FVector(Min.X, Min.Y, C.Z), FVector(C.X,     C.Y,    Max.Z)),// 4
	FBox(FVector(C.X,  Min.Y, C.Z), FVector(Max.X,    C.Y,    Max.Z)),// 5
	FBox(FVector(Min.X, C.Y,  C.Z), FVector(C.X,      Max.Y,  Max.Z)),// 6
	FBox(FVector(C.X,  C.Y,  C.Z), FVector(Max.X,     Max.Y,  Max.Z)) // 7
	};

	bool bAllBlocked = true;
	for (int i = 0; i < 8; ++i)
	{
		TreeNode->Children[i] = BuildOctree(ChildBoxes[i], InDepth + 1, InObjectTypes, InActorClassFilter);
		bAllBlocked &= (TreeNode->Children[i]->bIsLeaf && TreeNode->Children[i]->bBlocked);
	}

	// TODO::If all children are blocked leaves, we can mark this node as a blocked leaf and delete its children
	return TreeNode;
}

bool AOctNavVolume3D::IsBoxBlocked(const FBox& InBox,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter)
{

	// Overlap Tests, what kind of object channels we want to check against
	FCollisionObjectQueryParams ObjectCollisionParams;
	for (auto ObjectType : InObjectTypes)
	{
		ObjectCollisionParams.AddObjectTypesToQuery(UEngineTypes::ConvertToCollisionChannel(ObjectType));
	}

	// get the box's center and half side length
	const FVector Center = InBox.GetCenter();
	const FVector Extent = InBox.GetExtent();

	// check if the box overlaps with any object type specified
	FCollisionQueryParams QueryParams(SCENE_QUERY_STAT(OctreeBoxTest), false);
	bool bHit = GetWorld()->OverlapAnyTestByObjectType(
		Center, FQuat::Identity, ObjectCollisionParams, 
		FCollisionShape::MakeBox(Extent), QueryParams);

	return bHit;

	//TODO::Trace for blocked only (now all overlaped) if we want to -> 
}

bool AOctNavVolume3D::QueryPointBlocked(const FVector& WorldPoint) const
{
	if (!OctreeRoot)
	{
		return false;
	}

	const FOctreeNode* CurrentNode = OctreeRoot;
	while (CurrentNode && !CurrentNode->bIsLeaf)
	{
		const FVector Center = CurrentNode->Bounds.GetCenter();
		const bool bIsHighX = WorldPoint.X >= Center.X;
		const bool bIsHighY = WorldPoint.Y >= Center.Y;
		const bool bIsHighZ = WorldPoint.Z >= Center.Z;

		int CorrectChildIndex = (bIsHighX ? 1: 0) | ((bIsHighY ? 1: 0) << 1) | ((bIsHighZ ? 1: 0) << 2);
		CurrentNode = CurrentNode->Children[CorrectChildIndex];
	}

	return (CurrentNode) ? CurrentNode->bBlocked : false;
}

void AOctNavVolume3D::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);

#if WITH_EDITOR
	if (!Transform.GetRotation().IsIdentity())
	{
		UE_LOG(LogTemp, Warning, TEXT("OctNavVolume3D: Rotation is ignored. Please keep this actor unrotated."));
	}

	if (!Transform.GetScale3D().Equals(FVector(1.f, 1.f, 1.f), KINDA_SMALL_NUMBER))
	{
		UE_LOG(LogTemp, Warning,
			TEXT("OctNavVolume3D: Scale is ignored. Please keep scale = (1,1,1)."));
	}
#endif
	

	TArray<FVector> Vertices;
	TArray<int32> TrianglesIndexArray;

	const uint32 EstimateLineCount = 
		(DivisionsX + 1) * (DivisionsY + 1) * 2 // XY planes
		+ (DivisionsY + 1) * (DivisionsZ + 1) * 2 // YZ planes
		+ (DivisionsX + 1) * (DivisionsZ + 1) * 2; // XZ planes


	const uint32 EstimateVertexCount = EstimateLineCount * 4;
	const uint32 EstimateTriangleIndexCount = EstimateLineCount * 6;

	Vertices.Reserve(EstimateVertexCount);
	TrianglesIndexArray.Reserve(EstimateTriangleIndexCount);


	const float GridXBound = GetGridXBound();
	const float GridYBound = GetGridYBound();
	const float GridZBound = GetGridZBound();


	auto AddLine = [this,&Vertices, &TrianglesIndexArray](const FVector& InStart, const FVector& InEnd, const FVector& InUpNormal)
		{
			this->CreateLine(InStart, InEnd, InUpNormal, Vertices, TrianglesIndexArray);
		};


	FVector Start = FVector::ZeroVector;
	FVector End = FVector::ZeroVector;

	// Line Parallel to Y Axis
	for (int32 Z = 0; Z<= DivisionsZ; ++Z)
	{
		Start.Z  =  DivisionSize * Z;
		End.Z = DivisionSize *Z;

		for (int32 X = 0; X <= DivisionsX; ++X)
		{
			Start.X = X  * DivisionSize;
			End.X = X * DivisionSize;

			Start.Y = 0.0f;
			End.Y = GridYBound;

			AddLine(Start, End, FVector::UpVector);
		}
	}

	// Line Parallel to X Axis
	for (int32 Z = 0; Z <= DivisionsZ; ++Z)
	{
		Start.Z = DivisionSize * Z;
		End.Z = DivisionSize * Z;

		for (int32 Y = 0; Y <= DivisionsY; ++Y)
		{
			Start.Y = Y * DivisionSize;
			End.Y = Y * DivisionSize;

			Start.X = 0.0f;
			End.X = GridXBound;

			AddLine(Start, End, FVector::UpVector);
		}
	}

	// Line Parallel to Z Axis
	for (int32 X = 0; X <= DivisionsX; ++X)
	{
		Start.X = End.X = X * DivisionSize;

		for (int32 Y = 0; Y <= DivisionsY; ++Y)
		{
			Start.Y = End.Y = Y * DivisionSize;

			Start.Z = 0.f;
			End.Z = GridZBound;

			AddLine(Start, End, FVector::ForwardVector);
		}
	}

	ProceduralMeshComponent->CreateMeshSection(
		0,
		Vertices,                     //Vertices
		TrianglesIndexArray,          //Indices
		TArray<FVector>(),            //Normals
		TArray<FVector2D>(),          //UVs
		TArray<FColor>(),             //Vertex Colors
		TArray<FProcMeshTangent>(),   //Tangents
		false                         //bCreateCollision
	);

	if (GridMaterial)
	{
		UMaterialInstanceDynamic* MID = UMaterialInstanceDynamic::Create(GridMaterial, this);

		if (MID)
		{
			MID->SetVectorParameterValue(TEXT("Color"), Color);
			MID->SetScalarParameterValue(TEXT("Opacity"), Color.A);
			ProceduralMeshComponent->SetMaterial(0, MID);
		}
	}
}

void AOctNavVolume3D::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

NavNode* AOctNavVolume3D::GetNode(FIntVector Coordinates) 
{
	// First clamp coordinates to grid
	ClampCoordinatesToGrid(Coordinates);

	const int32 DivisionPerLevel = DivisionsX * DivisionsY;
	const int32 Index = (Coordinates.Z * DivisionPerLevel) + (Coordinates.Y * DivisionsX) + Coordinates.X;
	if (Index < 0 || Index >= GetTotalDivisions())
	{
		return nullptr;
	}
	return  &NavNodes[Index];
}

bool AOctNavVolume3D::FindPath(const FVector& InStart,
	const FVector& InDestination,
	const TArray<TEnumAsByte<EObjectTypeQuery>>& InObjectTypes,
	UClass* InActorClassFilter,
	TArray<FVector>& OutPath,
	AActor* InActor /*= nullptr */,
	float InDetectionRadius /*= 34.f*/, float InDetectionHalfHeight /*= 44.f */)
{
	OutPath.Reset();


	NavNode* StartNode = GetNode(ConvertWorldLocationToGridCoordinates(InStart));
	NavNode* GoalNode = GetNode(ConvertWorldLocationToGridCoordinates(InDestination));

	if (!StartNode || !GoalNode)
	{
#if WITH_EDITOR
		UE_LOG(LogTemp, Warning, TEXT("Start or End node not found"));
#endif 
		return false;
	}

	bool HasGoalFinalized = false;
	// automatically find nearest free node if start or goal is blocked
	if (OctreeRoot && QueryPointBlocked(ConvertGridCoordinatesToWorldLocation(GoalNode->Coordinates)))
	{
		if (NavNode* NewGoal = FindNearestFreeNode(GoalNode, InActor, InObjectTypes,
			InActorClassFilter, InDetectionRadius, InDetectionHalfHeight))
		{
			GoalNode = NewGoal;
			HasGoalFinalized = true;
		}
		else
		{
			if(GEngine)
			{
				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red,
					TEXT("No free goal node found near destination"));
			}
			return false;
		}
	}

	if (!HasGoalFinalized && IsActorOverlapping(InDetectionRadius, InDetectionHalfHeight, InActor,
		ConvertGridCoordinatesToWorldLocation(GoalNode->Coordinates), InObjectTypes, InActorClassFilter))
	{
		if (NavNode* NewGoal = FindNearestFreeNode(GoalNode, InActor, InObjectTypes,
			InActorClassFilter, InDetectionRadius, InDetectionHalfHeight))
		{
			GoalNode = NewGoal;
		}
		else
		{
			if(GEngine)
			{
				GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Red,
					TEXT("No free goal node found near destination"));
			}
			return false;
		}
	}


	//A* Setup

	std::priority_queue<NavNode*, std::vector<NavNode*>, NavNodeCompare> OpenSet;
	std::unordered_map<NavNode*, NavNode*> CameFrom;
	std::unordered_map<NavNode*, float> GScores;
	std::unordered_set<NavNode*> Visited;

	auto GetHeuristic = [&GoalNode](NavNode* InNode)
		{
			return FVector::Distance(FVector(InNode->Coordinates), FVector(GoalNode->Coordinates));
		};

	auto GetDistance = [](NavNode* FromNode, NavNode* ToNode)
		{
			return FVector::Distance(FVector(FromNode->Coordinates), FVector(ToNode->Coordinates));
		};


	auto GetGScore = [&GScores](NavNode* InNode)
		{
			auto It = GScores.find(InNode);
			if (It != GScores.end())
			{
				return It->second;
			}
			return FLT_MAX;
		};

	StartNode->FScore = GetHeuristic(StartNode);
	OpenSet.push(StartNode);
	GScores[StartNode] = 0.0f;


	while (!OpenSet.empty())
	{
		NavNode* CurrentNavNode = OpenSet.top(); OpenSet.pop();
		Visited.insert(CurrentNavNode);

		if (CurrentNavNode == GoalNode)
		{
			OutPath.Add(ConvertGridCoordinatesToWorldLocation(CurrentNavNode->Coordinates));
			while (CameFrom.contains(CurrentNavNode))
			{
				OutPath.Insert(ConvertGridCoordinatesToWorldLocation(CurrentNavNode->Coordinates), 0);
				CurrentNavNode = CameFrom[CurrentNavNode];
			}
			OutPath.Insert(ConvertGridCoordinatesToWorldLocation(StartNode->Coordinates), 0);
			return true;
		}

		const float CurrentGScore = GetGScore(CurrentNavNode);

		for (NavNode* Neighbour : CurrentNavNode->Neighbours)
		{
			const FVector NeighbourWorldPos = ConvertGridCoordinatesToWorldLocation(Neighbour->Coordinates);
			if (OctreeRoot && QueryPointBlocked(NeighbourWorldPos))
			{
				continue;
			}

			const float TentativeG = CurrentGScore + GetDistance(CurrentNavNode, Neighbour);
			const float ExistingScore = GetGScore(Neighbour);
			if (TentativeG < ExistingScore)
			{

				if (IsActorOverlapping(InDetectionRadius, InDetectionHalfHeight,
					InActor, NeighbourWorldPos, InObjectTypes, InActorClassFilter))
				{
					continue;
				}

				CameFrom[Neighbour] = CurrentNavNode;
				GScores[Neighbour] = TentativeG;

				Neighbour->FScore = TentativeG + GetHeuristic(Neighbour);
				if (!Visited.contains(Neighbour))
				{
					OpenSet.push(Neighbour);
				}
			}
		}
	}
	return false;

}




