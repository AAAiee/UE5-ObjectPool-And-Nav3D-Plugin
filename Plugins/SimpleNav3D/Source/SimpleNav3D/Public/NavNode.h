#pragma once 

#include "CoreMinimal.h"

#include <vector>

/**
 * NavNode
 *
 * Represents a single cell in the 3D navigation grid.
 * - Stores integer grid coordinates.
 * - Keeps a list of neighbouring nodes for graph traversal.
 */
class NavNode
{
public:
	/** Discrete grid coordinates for this node (X, Y, Z). */
	FIntVector Coordinates;

	/** Direct neighbours of this node in the navigation graph. */
	std::vector<NavNode*> Neighbours;
};

