/**
 * @file dijkstra.hpp
 * @brief Dijkstra's pathfinding algorithm implementation
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file implements Dijkstra's algorithm for finding shortest paths on
 * weighted grids using uniform-cost search with a priority queue.
 */

#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

class Dijkstra : public PathFindingAlgorithm {
    private:
        /**
         * @brief Node structure for Dijkstra's priority queue.
         * 
         * Stores position and cumulative cost for priority queue ordering.
         */
        struct Node {
            pair<int, int> position;  ///< Cell coordinates (x, y)
            int cost;  ///< Cumulative cost from start to this node
            
            /**
             * @brief Comparison operator for priority queue (min-heap).
             * 
             * @param other Node to compare with
             * @return true if this node has greater cost (lower priority)
             */
            bool operator>(const Node& other) const {
                return cost > other.cost;
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
         * @brief Implements Dijkstra's pathfinding algorithm.
         * 
         * Uses a priority queue to explore nodes in order of their cumulative cost
         * from the start. Expands the lowest-cost unexplored node at each step,
         * guaranteeing the shortest path when the goal is reached.
         * 
         * In this grid implementation, all traversable cells have uniform cost (1),
         * making Dijkstra's behavior similar to BFS but demonstrating the general
         * weighted pathfinding approach.
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

#endif // DIJKSTRA_HPP