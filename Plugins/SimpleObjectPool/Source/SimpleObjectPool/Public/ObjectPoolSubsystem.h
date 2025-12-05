#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "ObjectPoolSubSystem.generated.h"

/**
 * A single pooled item entry used by the object pool system.
 * Stores the actor instance and whether it is currently in use.
 */
USTRUCT(BlueprintType)
struct SIMPLEOBJECTPOOL_API FPoolItem
{
	GENERATED_BODY()

	/** The actor instance managed by the pool. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ObjectPool|PoolItem")
	AActor* ActorInstance = nullptr;

	/** Whether this actor is currently active and in use. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ObjectPool|PoolItem")
	bool bInUse = false;
};


/**
 * UObjectPoolSubsystem
 *
 * A centralized object pooling system used for reusing actor instances at runtime.
 *
 * This subsystem:
 *   - Pre-spawns a configurable number of actors for a given class.
 *   - Provides already-spawned actors when requested, avoiding SpawnActor cost.
 *   - Expands pools dynamically when necessary.
 *   - Supports automatic return of actors to the pool after a delay.
 *
 * This subsystem lives for the lifetime of the GameInstance.
 */
UCLASS(BlueprintType)
class SIMPLEOBJECTPOOL_API UObjectPoolSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:

	/** Constructor ¡ª initializes the default hidden transform used for pooled actors. */
	UObjectPoolSubsystem();

	/**
	 * Initializes a pool for the specified actor class.
	 *
	 * @param ActorClass			The class of actor to pool.
	 * @param InitialSize			Number of actors to pre-spawn into the pool.
	 *
	 * Must be called before requesting pooled actors of this class.
	 */
	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	void InitializePool(TSubclassOf<AActor> ActorClass, int32 InitialSize);

	/**
	 * Retrieves an available actor from the pool.
	 * If no actor is free, the pool will expand automatically.
	 *
	 * @param ActorClass						The class type to retrieve.
	 * @param SpawnTransform					The transform applied before activation.
	 * @param bShouldAutomaticallyReturnPool	Whether the actor should automatically return after a delay.
	 * @param RecycleDelayTime					Time in seconds before automatic return (if enabled).
	 *
	 * @return A valid actor instance ready for use.
	 */
	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	AActor* GetPooledActor(TSubclassOf<AActor> ActorClass,
		FTransform SpawnTransform,
		bool bShouldAutomaticallyReturnPool = true,
		float RecycleDelayTime = 1.f);

	/**
	 * Multicast version of GetPooledActor for networked games.
	 * Spawns or retrieves an actor from the pool across all clients.
	 *
	 * @param ActorClass						The class to spawn/pool.
	 * @param SpawnRotator						Rotation to apply.
	 * @param Spawnlocation						Location to apply.
	 * @param bAutomaticallyReturnPool			If true, returns actor to pool after delay.
	 * @param RecycleDelayTime					Time before automatic recycling.
	 */
	UFUNCTION(Reliable, NetMulticast)
	void GetPooledActorOnMulticast(TSubclassOf<AActor> ActorClass,
		FRotator SpawnRotator,
		FVector Spawnlocation,
		bool bAutomaticallyReturnPool = true,
		float RecycleDelayTime = 1.f);

	/**
	 * Returns an actor instance back into the pool.
	 *
	 * @param Actor		The actor instance to return.
	 *
	 * Ensures the actor is deactivated and marked available for reuse.
	 */
	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	void ReturnActorToPool(AActor* Actor);

private:

	/**
	 * Schedules an actor to be returned to the pool after a delay.
	 *
	 * @param InActor					Actor to schedule for return.
	 * @param DelayTime					Delay in seconds before returning.
	 * @param bAutomaticallyReturnPool	Whether auto-return should happen.
	 */
	UFUNCTION()
	void DelayActor(AActor* INActor, float DelayTime, bool bAutomaticallyReturnPool);

	/**
	 * Deactivates an actor so it becomes hidden, disabled, and safe for reuse.
	 *
	 * @param SpawnedActor	The actor instance to deactivate.
	 */
	void DeactivateActor(AActor* SpawnedActor);

	/**
	 * Prepares and activates an actor for Gameplay use.
	 *
	 * @param FreeActor						The actor instance being reactivated.
	 * @param SpawnTransform				Transform to apply.
	 * @param bShouldAutomaticallyReturnPool Whether actor auto-returns after delay.
	 * @param RecycleDelayTime				Time before automatic recycling.
	 */
	void ActivateActor(AActor* FreeActor,
		const FTransform& SpawnTransform,
		bool bShouldAutomaticallyReturnPool,
		float RecycleDelayTime);

private:

	/** Mapping of actor class ¡ú array of pooled actor entries. */
	TMap<TSubclassOf<AActor>, TArray<FPoolItem>> Pool;

	/** Transform used to hide inactive actors underground and out of view. */
	const FTransform HiddenTransform;
};
