/**
 * @file astar.hpp
 * @brief A* (A-Star) pathfinding algorithm implementation
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file implements the A* algorithm for finding optimal shortest paths
 * using informed search with Manhattan distance heuristic.
 */

#ifndef ASTAR_MAIN_HPP
#define ASTAR_MAIN_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

class AStar : public PathFindingAlgorithm {
    private:
        /**
         * @brief Node structure for A* priority queue.
         * 
         * Stores position and cost values for priority queue ordering.
         */
        struct Node {
            pair<int, int> position;  ///< Cell coordinates (x, y)
            int g;  ///< Actual cost from start to this node
            int f;  ///< Total estimated cost: f = g + h
            
            /**
             * @brief Comparison operator for priority queue (min-heap).
             * 
             * @param other Node to compare with
             * @return true if this node has greater f-cost (lower priority)
             */
            bool operator>(const Node& other) const {
                return f > other.f;
            }
        };
        
    public:
        /**
         * @brief Inherits constructor from the PathFindingAlgorithm base class.
         * 
         * Uses the base class constructor to initialize grid, start position,
         * goal position, and algorithm name.
         */
        using PathFindingAlgorithm::PathFindingAlgorithm;

        /**
         * @brief Calculates Manhattan distance heuristic.
         * 
         * Computes the L1 distance (Manhattan distance) between current node and goal.
         * This heuristic is admissible for 4-directional grid movement, ensuring
         * A* finds the optimal path.
         * 
         * @param currentNode Current position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Manhattan distance between current node and goal
         */
        int heuristicCalc(pair<int, int> currentNode, pair<int, int> goal) const {
            return abs(currentNode.first - goal.first) + abs(currentNode.second - goal.second);
        }

        /**
         * @brief Implements A* pathfinding algorithm.
         * 
         * Uses a priority queue to explore nodes in order of their f-cost (g + h).
         * Guarantees finding the shortest path by always expanding the most promising
         * node first, guided by the Manhattan distance heuristic.
         * 
         * @param grid 2D vector representing the grid state
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of (x, y) coordinate pairs representing the optimal shortest path,
         *         or empty vector if no path exists
         */
        vector<pair<int, int>> findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) override;
};

#endif // ASTART_MAIN_HPP