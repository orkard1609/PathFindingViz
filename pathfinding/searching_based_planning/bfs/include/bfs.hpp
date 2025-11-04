/**
 * @file bfs.hpp
 * @brief Breadth-First Search pathfinding algorithm implementation
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file implements the BFS algorithm for finding shortest paths on
 * unweighted grids using level-order traversal.
 */

#ifndef BFS_HPP
#define BFS_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

using namespace std;

class BFS : public PathFindingAlgorithm {
    public:
        /**
         * @brief Inherits constructor from the PathFindingAlgorithm base class.
         * 
         * Uses the base class constructor to initialize grid, start position,
         * goal position, and algorithm name.
         */
        using PathFindingAlgorithm::PathFindingAlgorithm;
        
        /**
         * @brief Implements BFS pathfinding algorithm.
         * 
         * Explores the grid level by level using a queue data structure, ensuring
         * that the shortest path (in terms of steps) is found. All cells at distance
         * d from the start are explored before any cell at distance d+1.
         * 
         * @param grid 2D vector representing the grid state
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of (x, y) coordinate pairs representing the shortest path,
         *         or empty vector if no path exists
         */
        vector<pair<int, int>> findPath(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) override;
};

#endif // BFS_HPP