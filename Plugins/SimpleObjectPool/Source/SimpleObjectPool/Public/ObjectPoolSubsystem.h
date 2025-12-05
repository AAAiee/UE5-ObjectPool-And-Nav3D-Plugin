#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "ObjectPoolSubSystem.generated.h"


USTRUCT(BlueprintType)
struct SIMPLEOBJECTPOOL_API FPoolItem
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ObjectPool|PoolItem")
	AActor* ActorInstance = nullptr;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "ObjectPool|PoolItem")
	bool bInUse = false;
};


UCLASS(BlueprintType)
class SIMPLEOBJECTPOOL_API UObjectPoolSubsystem : public UGameInstanceSubsystem
{
	GENERATED_BODY()

public:
	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	void InitializePool(TSubclassOf<AActor> ActorClass, int32 InitialSize);


	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	AActor* GetPooledActor(TSubclassOf<AActor> ActorClass,FTransform SpawnTransform, bool bShouldAutomaticallyReturnPool = true, float RecyclingTime = 1.f);

	UFUNCTION(Reliable, NetMulticast)
	void GetPooledActorOnMulticast(TSubclassOf<AActor> ActorClass, FRotator SpawnRotator, FVector Spawnlocation, bool bAutomaticallyReturnPool = true, float RecyclingTime = 1.f);

	
	UFUNCTION(BlueprintCallable, Category = "ObjectPool")
	void ReturnActorToPool(AActor* Actor);
	
private:
	UFUNCTION()
	void DelayActor(AActor* INActor, float DelayTime, bool bAutomaticallyReturnPool);

	void DeactivateActor(AActor* SpawnedActor);

private:
	TMap<TSubclassOf<AActor>, TArray<FPoolItem>> Pool;
};






