#pragma once 
#include "CoreMinimal.h"

#include <vector>


class NavNode
{
public:
	FIntVector Coordinates;
	std::vector<NavNode*> Neighbours;
	float FScore = 0.0f;
};

struct NavNodeCompare
{
	bool operator()(const NavNode* A, const NavNode* B) const
	{
		return A->FScore > B->FScore;
	}
};
