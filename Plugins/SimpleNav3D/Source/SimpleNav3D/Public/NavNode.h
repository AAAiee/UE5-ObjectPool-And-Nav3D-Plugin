#pragma once 

#include "CoreMinimal.h"

#include <vector>

/**
 * NavNode
 *
 * Represents a single cell in the 3D navigation grid.
 * - Stores integer grid coordinates.
 * - Keeps a list of neighbouring nodes for graph traversal.
 * - Carries an FScore value used by A* (G + H or similar).
 */
class NavNode
{
public:
	/** Discrete grid coordinates for this node (X, Y, Z). */
	FIntVector Coordinates;

	/** Direct neighbours of this node in the navigation graph. */
	std::vector<NavNode*> Neighbours;

	/**
	 * Score used by the A* priority queue.
	 * Typically FScore = GScore + Heuristic.
	 */
	float FScore = 0.0f;
};

/**
 * Comparison functor for NavNode pointers used in std::priority_queue.
 * Orders nodes by FScore in ascending order (lower FScore has higher priority).
 */
struct NavNodeCompare
{
	bool operator()(const NavNode* A, const NavNode* B) const
	{
		return A->FScore > B->FScore;
	}
};

