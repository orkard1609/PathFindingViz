/**
 * @file dfs.hpp
 * @brief Depth-First Search pathfinding algorithm implementation
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file implements the DFS algorithm for pathfinding using recursive
 * depth-first exploration with goal-directed optimization.
 */

#ifndef DFS_HPP
#define DFS_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

using namespace std;

class DFS : public PathFindingAlgorithm {
    private:
        bool goalisFound = false;  ///< Flag to enable early termination when goal is reached
        
    public:
        /**
         * @brief Inherits constructor from the PathFindingAlgorithm base class.
         * 
         * Uses the base class constructor to initialize grid, start position,
         * goal position, and algorithm name.
         */
        using PathFindingAlgorithm::PathFindingAlgorithm;

        /**
         * @brief Implements DFS pathfinding algorithm.
         * 
         * Initiates the recursive depth-first search from the start position.
         * Uses goal-directed neighbor prioritization to improve path quality.
         * 
         * @param grid 2D vector representing the grid state
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of (x, y) coordinate pairs representing a path (not necessarily shortest),
         *         or empty vector if no path exists
         */
        vector<pair<int, int>> findPath(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) override;

        /**
         * @brief Recursive DFS helper function.
         * 
         * Recursively explores neighbors in depth-first manner with goal-directed
         * prioritization. Updates distance and parent arrays for path reconstruction.
         * Uses early termination when goal is found to improve performance.
         * 
         * @param cur Current position being explored as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @param dist Reference to 2D distance array tracking steps from start
         * @param parent Reference to 2D parent array for path reconstruction
         */
        void dfs(pair<int, int> cur, pair<int, int> goal, 
                 vector<vector<int>>& dist, vector<vector<pair<int, int>>>& parent);
};

#endif // DFS_HPP