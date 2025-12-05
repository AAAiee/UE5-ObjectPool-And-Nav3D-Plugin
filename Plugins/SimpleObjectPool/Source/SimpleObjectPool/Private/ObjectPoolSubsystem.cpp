#include "ObjectPoolSubsystem.h"
#include "AIController.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/Controller.h"


UObjectPoolSubsystem::UObjectPoolSubsystem()
	:HiddenTransform(FTransform
		{
			FRotator::ZeroRotator,
			FVector(0.f, 0.f, -50000.f),
			FVector::OneVector
		}) { }

void UObjectPoolSubsystem::InitializePool(TSubclassOf<AActor> InActorClass, int32 InInitialSize)
{
	// Validate input parameters
	checkf(InInitialSize > 0, TEXT("ObjectPoolSubsystem:: InitialSize must be greater than zero"));
	checkf(InActorClass, TEXT("ObjectPoolSubsystem:: ActorClass is null"));

	ensureMsgf(!Pool.Contains(InActorClass), TEXT("ObjectPoolSubsystem:: %s is already initialized and in the pool"), *InActorClass->GetName());

	UWorld* World = GetWorld();
    checkf(World, TEXT("ObjectPoolSubsystem:: World is null"));

	// Preallocate pool items
	TArray<FPoolItem> PoolItems;
	PoolItems.Reserve(InInitialSize);
	for (int32 i = 0; i < InInitialSize; ++i)
	{
		AActor* SpawnedActor = World->SpawnActor(InActorClass, &HiddenTransform);

		// Ensure the actor was spawned successfully, otherwise crash to highlight the critical error
		checkf(SpawnedActor, TEXT("ObjectPoolSubsystem: SpawnActor failed during pool initialization for class %s"), *InActorClass->GetName());

		DeactivateActor(SpawnedActor);
		PoolItems.Add((FPoolItem{ SpawnedActor, false }));
	}
	Pool.Add(InActorClass, MoveTemp(PoolItems));

#if WITH_EDITOR
	UE_LOG(LogTemp, Log,
		TEXT("ObjectPoolSubsystem:: Initialized pool for %s with %d actors."),
		*InActorClass->GetName(), InInitialSize);
#endif
}

AActor* UObjectPoolSubsystem::GetPooledActor(TSubclassOf<AActor> InActorClass, FTransform InSpawnTransform, bool bInShouldAutomaticallyReturnPool /*= true*/, float InRecycleDelayTime /*= 1.f*/)
{
	// Validate input parameters
	checkf(InActorClass, TEXT("ObjectPoolSubsystem:: ActorClass is null"));
	TArray<FPoolItem>* TargetPool = Pool.Find(InActorClass);
	checkf(TargetPool, TEXT("ObjectPoolSubsystem:: No pool found for class %s. Did you forget to initialize it?"), *InActorClass->GetName());
	UWorld* World = GetWorld();
	checkf(World, TEXT("ObjectPoolSubsystem:: World is null"));

	// If there is a free actor in the pool, return it
	for (FPoolItem& Item : *TargetPool)
	{
		if (!Item.bInUse && Item.ActorInstance)
		{
			Item.bInUse = true;
			AActor* FreeActor = Item.ActorInstance;

			ActivateActor(FreeActor, InSpawnTransform, bInShouldAutomaticallyReturnPool, InRecycleDelayTime);

#if WITH_EDITOR
			UE_LOG(LogTemp, Verbose,
				TEXT("ObjectPoolSubsystem: Reused actor: %s"),
				*Item.ActorInstance->GetName());
#endif

			return Item.ActorInstance;
		}
	}

	// If All actors are in use, expands the pool 
	static constexpr float GrowthFactor = 0.5f;
	FActorSpawnParameters SpawnParams;
	SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
	const int32 CurrentCount = TargetPool->Num();
	const int32 NumToSpawn = FMath::CeilToInt(CurrentCount * GrowthFactor);

	AActor* SpawnedActorToReturn = nullptr;
	
	for (int i = 0; i < NumToSpawn; ++i)
	{
		AActor* CurrentSpawnedActor = World->SpawnActor(InActorClass, &HiddenTransform);
		ensureMsgf(CurrentSpawnedActor, TEXT("ObjectPoolSubsystem: SpawnActor failed during pool expansion for class %s"), *InActorClass->GetName());
		if (CurrentSpawnedActor == nullptr)
		{
			continue;
		}

		if (!SpawnedActorToReturn) 
		{
			SpawnedActorToReturn = CurrentSpawnedActor;
			ActivateActor(SpawnedActorToReturn, InSpawnTransform, bInShouldAutomaticallyReturnPool, InRecycleDelayTime);
			TargetPool->Add(FPoolItem{ SpawnedActorToReturn, true });
		}
		else
		{
			DeactivateActor(CurrentSpawnedActor);
			TargetPool->Add(FPoolItem{ CurrentSpawnedActor, false });
		}
	}


#if WITH_EDITOR
	UE_LOG(LogTemp, Log,
		TEXT("ObjectPoolSubsystem:: Expanded pool for %s by %d actors."),
		*InActorClass->GetName(), NumToSpawn);
#endif

	return SpawnedActorToReturn;

}

void UObjectPoolSubsystem::GetPooledActorOnMulticast_Implementation(TSubclassOf<AActor> InActorClass, FRotator InSpawnRotator, FVector InSpawnlocation, bool bInAutomaticallyReturnPool /*= true*/, float bInRecyclingTime /*= 1.f*/)
{

}

void UObjectPoolSubsystem::ReturnActorToPool(AActor* InActor)
{
	// Input Validation
	if (!InActor)
	{
		return;
	}
	TSubclassOf<AActor> ActorClass = InActor->GetClass();
	TArray<FPoolItem>* TargetPool = Pool.Find(ActorClass);
	if (!TargetPool)
	{
#if WITH_EDITOR
		UE_LOG(LogTemp, Warning,
			TEXT("ObjectPoolSubsystem:: No pool found for class %s when returning actor %s. Did you forget to initialize it?"),
			*ActorClass->GetName(),
			*InActor->GetName());
#endif
		return;
	}

	// Find the actor in pool, deactivate it 
	for (FPoolItem& Item : *TargetPool)
	{
		if (Item.ActorInstance == InActor)
		{
			Item.bInUse = false;
			DeactivateActor(InActor);
#if WITH_EDITOR
			GEngine->AddOnScreenDebugMessage(
				-1,
				5.f,
				FColor::Green,
				FString::Printf(TEXT("ObjectPoolSubsystem:: Returned actor %s to pool."), *InActor->GetName())
			);
#endif
			return;
		}
	}

	// not found, the actor returned is not in pool
	ensureMsgf(false, TEXT("ObjectPoolSubsystem:: The actor you return is not a actor in the pool!"));
}

void UObjectPoolSubsystem::DelayActor(AActor* InActor, float InDelayTime, bool bInAutomaticallyReturnPool)
{
	//Validation
	checkf(InActor, TEXT("ObjectPoolSubsystem:: DelayActor received a null actor"));
	checkf(InDelayTime >= 0.f, TEXT("ObjectPoolSubsystem:: DelayTime must be non-negative"));
	UWorld* World = GetWorld();
	checkf(World, TEXT("ObjectPoolSubsystem:: World is null"));

	// Set up a timer, after specified delay time, return the actor to pool
	if (bInAutomaticallyReturnPool)
	{
		//Use Weak Pointers to avoid level destroyed and other unexpected situations, avoid pointers dangling 
		TWeakObjectPtr<UObjectPoolSubsystem> WeakThisSubsystem(this);
		TWeakObjectPtr<AActor> WeakActor(InActor);
		FTimerHandle TimeHandle;
#if WITH_EDITOR
		UE_LOG(LogTemp, Verbose,
			TEXT("ObjectPoolSubsystem:: Setting delay of %f seconds to return actor %s to pool."),
			InDelayTime,
			*InActor->GetName());
#endif

		World->GetTimerManager().SetTimer(
			TimeHandle,
			[WeakThisSubsystem, WeakActor]()
			{

				if (WeakThisSubsystem.IsValid() && WeakActor.IsValid())
				{
					WeakThisSubsystem->ReturnActorToPool(WeakActor.Get());
				}
			},
			InDelayTime,
			false
		);
	}
}

void UObjectPoolSubsystem::DeactivateActor(AActor* SpawnedActor)
{
#if WITH_EDITOR
	UE_LOG(LogTemp, Verbose,
		TEXT("ObjectPoolSubsystem:: Deactivating actor: %s"),
		*SpawnedActor->GetName());
#endif

	// Move back underground, This works when they return to the pool, newly initialized will be at this location anyway.
	SpawnedActor->SetActorTransform(HiddenTransform);

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

void UObjectPoolSubsystem::ActivateActor(AActor* FreeActor, const FTransform& SpawnTransform, bool bShouldAutomaticallyReturnPool, float RecycleDelayTime)
{

#if WITH_EDITOR
	UE_LOG(LogTemp, Verbose,
		TEXT("ObjectPoolSubsystem:: Activating actor: %s"),
		*FreeActor->GetName());
#endif
	// Set the actor's transform to the desired spawn location and rotation
	FreeActor->SetActorTransform(SpawnTransform);
	FreeActor->SetActorTickEnabled(true);
	FreeActor->SetActorHiddenInGame(false);
	FreeActor->SetActorEnableCollision(true);

	// If specified, set a delay to automatically return the actor to the pool
	if (bShouldAutomaticallyReturnPool)
	{
		DelayActor(FreeActor, RecycleDelayTime, bShouldAutomaticallyReturnPool);
	}

	// If the actor is a pawn and has an AIController class, spawn and possess it
	if (APawn* PawnActor = Cast<APawn>(FreeActor))
	{
		UWorld* World = GetWorld(); 
		checkf(World, TEXT("ObjectPoolSubsystem:: World is null"));

		if (PawnActor->AIControllerClass && PawnActor->GetController() == nullptr)
		{
			AAIController* PawnAIController = World->SpawnActor<AAIController>(PawnActor->AIControllerClass);
			PawnAIController->Possess(PawnActor);
		}
	}
}

