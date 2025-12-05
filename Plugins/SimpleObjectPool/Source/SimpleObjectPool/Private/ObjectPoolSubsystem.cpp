#include "ObjectPoolSubsystem.h"
#include "AIController.h"


void UObjectPoolSubsystem::InitializePool(TSubclassOf<AActor> ActorClass, int32 InitialSize)
{
	// Meaningless Size
	checkf(InitialSize > 0, TEXT("ObjectPoolSubsystem:: InitialSize must be greater than zero"));

	// ActorClass is null
	checkf(ActorClass, TEXT("ObjectPoolSubsystem:: ActorClass is null"));

	// Already Initialized, Ignore
	ensureMsgf(!Pool.Contains(ActorClass), TEXT("ObjectPoolSubsystem:: %s is already initialized and in the pool"), *ActorClass->GetName());

	checkf(GetWorld(), TEXT("ObjectPoolSubsystem:: World is null"));

	UWorld* World = GetWorld();

	TArray<FPoolItem> PoolItems;
	PoolItems.Reserve(InitialSize);

	static const FTransform SpawnedItemHiddenTransform
	{
		FRotator::ZeroRotator,
			FVector(0.f, 0.f, -50000.f),
			FVector::OneVector
	};
	

	for (int32 i = 0; i < InitialSize; ++i)
	{

		AActor* SpawnedActor = World->SpawnActor(ActorClass, &SpawnedItemHiddenTransform);

		// Ensure the actor was spawned successfully, otherwise crash to highlight the critical error
		checkf(SpawnedActor, TEXT("ObjectPoolSubsystem: SpawnActor failed during pool initialization for class %s"), *ActorClass->GetName());

		DeactivateActor(SpawnedActor);
	
		PoolItems.Add((FPoolItem{ SpawnedActor, false }));
	}

	Pool.Add(ActorClass, MoveTemp(PoolItems));
	
	UE_LOG(LogTemp, Log,
		TEXT("ObjectPoolSubsystem:: Initialized pool for %s with %d actors."),
		*ActorClass->GetName(), InitialSize);
}

AActor* UObjectPoolSubsystem::GetPooledActor(TSubclassOf<AActor> ActorClass, FTransform SpawnTransform, bool bShouldAutomaticallyReturnPool /*= true*/, float RecyclingTime /*= 1.f*/)
{
	checkf(ActorClass, TEXT("ObjectPoolSubsystem:: ActorClass is null"));


	UWorld* World = GetWorld();
	checkf(World, TEXT("ObjectPoolSubsystem:: World is null"));

	TArray<FPoolItem>* TargetPool = Pool.Find(ActorClass);
	checkf(TargetPool, TEXT("ObjectPoolSubsystem:: No pool found for class %s. Did you forget to initialize it?"), *ActorClass->GetName());

	for (FPoolItem& Item : *TargetPool)
	{
		if (!Item.bInUse && Item.ActorInstance)
		{
			Item.bInUse = true;
			AActor* FreeActor = Item.ActorInstance;

			FreeActor->SetActorTickEnabled(true);
			FreeActor->SetActorHiddenInGame(false);
			FreeActor->SetActorEnableCollision(true);

			if (APawn* PawnActor = Cast<APawn>(FreeActor))
			{
				if (PawnActor->AIControllerClass && PawnActor->GetController() == nullptr)
				{
					AAIController* PawnAIController = World->SpawnActor<AAIController>(PawnActor->AIControllerClass);
				}
			}

		}
	}
	return nullptr;
}

void UObjectPoolSubsystem::GetPooledActorOnMulticast_Implementation(TSubclassOf<AActor> ActorClass, FRotator SpawnRotator, FVector Spawnlocation, bool bAutomaticallyReturnPool /*= true*/, float RecyclingTime /*= 1.f*/)
{

}

void UObjectPoolSubsystem::ReturnActorToPool(AActor* Actor)
{

}

void UObjectPoolSubsystem::DelayActor(AActor* INActor, float DelayTime, bool bAutomaticallyReturnPool)
{

}

void UObjectPoolSubsystem::DeactivateActor(AActor* SpawnedActor)
{
	// Make sure the actor stays hidden and inactive
	SpawnedActor->SetActorTickEnabled(false);
	SpawnedActor->SetActorHiddenInGame(true);
	SpawnedActor->SetActorEnableCollision(false);

	// If the spawned actor is a pawn, unpossess it to avoid any controller conflicts
	if (APawn* PawnActor = Cast<APawn>(SpawnedActor))
	{
		if (AController* PawnController = PawnActor->GetController())
		{
			PawnController->UnPossess();
		}
	}
}
