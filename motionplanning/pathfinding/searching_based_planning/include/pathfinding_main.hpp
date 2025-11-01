/**
 * @file pathfinding_main.hpp
 * @brief Base class for pathfinding algorithms
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file provides the abstract PathFindingAlgorithm base class that defines
 * the interface for various pathfinding algorithms (BFS, DFS, Dijkstra, A*).
 * Includes common utilities for path validation, neighbor exploration, and
 * goal-directed search optimization.
 */

#ifndef PATHFINDING_MAIN_HPP
#define PATHFINDING_MAIN_HPP

#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include "grid.hpp"

using namespace std;

// Direction definitions for neighbor exploration
#define UP    pair<int, int>{0, -1}
#define RIGHT pair<int, int>{1, 0}
#define DOWN  pair<int, int>{0, 1}
#define LEFT  pair<int, int>{-1, 0}

class PathFindingAlgorithm {
    private:
        vector<string> algorithms = {"BFS", "DFS", "Dijkstra",  "A*"}; // List of supported algorithms
        string selectedAlgo_ = algorithms[0]; // Default algorithm
    protected:    
        Grid& grid_;
        pair<int, int> startPos_; // Start position
        pair<int, int> goalPos_; // Goal position
        vector<pair<int, int>> foundPath_; // Store the found path
        vector<pair<int, int>> visitedNodes_; // Store visited nodes
        int n, m; // Grid dimensions
        vector<pair<int, int>> neighbors; // Directions for neighbor exploration, except DFS which uses its own member variable
        vector<vector<int>> dist; // Distance array from start to each cell
        vector<vector<pair<int, int>>> parent; // Parent of each cell for path reconstruction
        vector<vector<bool>> visited; // Visited cells
    public:
        /**
         * @brief Constructs a PathFindingAlgorithm object with necessary pathfinding parameters.
         * 
         * Initializes the algorithm with grid reference, start/goal positions, and prepares
         * internal data structures (distance array, parent tracking, visited flags) based on
         * grid dimensions.
         * 
         * @param grid Reference to the Grid object containing the environment
         * @param startPos Starting position as (x, y) coordinates
         * @param goalPos Goal position as (x, y) coordinates
         * @param selectedAlgo Name of the selected algorithm (e.g., "BFS", "DFS", "Dijkstra", "A*")
         */
        PathFindingAlgorithm(Grid& grid, const pair<int, int>& startPos, const pair<int, int>& goalPos, string selectedAlgo) 
                            : grid_(grid), startPos_(startPos), goalPos_(goalPos), selectedAlgo_(selectedAlgo) {
            // Initialize size-dependent members
            n = grid_.getWidth();
            m = grid_.getHeight();
            dist.resize(n, vector<int>(m, INT_MAX));
            parent.resize(n, vector<pair<int, int>>(m, {-1,-1}));
            visited.resize(n, vector<bool>(m, false));
        }
        
        /**
         * @brief Virtual destructor for proper cleanup of derived classes.
         */
        virtual ~PathFindingAlgorithm() {}
        
        /**
         * @brief Factory method to create the appropriate pathfinding algorithm instance.
         * 
         * Creates and returns a unique pointer to a specific pathfinding algorithm
         * implementation (BFS, DFS, Dijkstra, or A*) based on the algorithm name.
         * 
         * @param algo Name of the algorithm to instantiate ("BFS", "DFS", "Dijkstra", "A*")
         * @param grid Reference to the Grid object containing the environment
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return unique_ptr to the created PathFindingAlgorithm instance, or nullptr if algorithm not recognized
         */
        static unique_ptr<PathFindingAlgorithm> selectAlgorithm(const string& algo, Grid& grid, 
                                                                const pair<int, int>& start, 
                                                                const pair<int, int>& goal);

        /**
         * @brief Retrieves the computed path from start to goal.
         * 
         * If the path hasn't been computed yet, this method triggers the pathfinding
         * algorithm execution. The path excludes start and goal positions.
         * 
         * @return Vector of (x, y) coordinate pairs representing the path from start to goal,
         *         or empty vector if no path exists
         */
        vector<pair<int, int>> getFoundPath() {
            if (foundPath_.empty()) {
                foundPath_ = findPath(grid_.getGridStatus(), startPos_, goalPos_);
            }
            return foundPath_;
        }

        /**
         * @brief Retrieves all nodes visited during the pathfinding process.
         * 
         * Returns the complete list of cells explored by the algorithm in the order
         * they were visited. Useful for visualization purposes.
         * 
         * @return Vector of (x, y) coordinate pairs representing visited nodes
         */
        vector<pair<int, int>> getVisitedNodes() const {
            return visitedNodes_;
        };

        /**
         * @brief Calculates prioritized neighbor exploration directions toward the goal.
         * 
         * Computes the angle from start to goal and returns a prioritized list of
         * direction vectors (UP, DOWN, LEFT, RIGHT) that favor movement toward the goal.
         * Used to improve search efficiency in uninformed algorithms like DFS.
         * 
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of direction pairs prioritized toward the goal
         */
        vector<pair<int, int>> goalDirectionCalc(pair<int, int> start, pair<int, int> goal) const;

        /**
         * @brief Retrieves valid neighboring cells for a given position.
         * 
         * Returns all traversable neighbors (not obstacles, within bounds) for the
         * specified cell. Neighbors are ordered based on goal direction for improved
         * search efficiency.
         * 
         * @param x The x-coordinate of the cell
         * @param y The y-coordinate of the cell
         * @return Vector of (x, y) coordinate pairs representing valid neighbors
         */
        vector<pair<int, int>> getNeighbors(int x, int y) const;

        /**
         * @brief Pure virtual method for pathfinding algorithm implementation.
         * 
         * Must be implemented by derived classes to provide specific algorithm logic
         * (BFS, DFS, Dijkstra, A*). Computes the path and tracks visited nodes.
         * 
         * @param grid 2D vector representing the grid state (cell values)
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of (x, y) coordinate pairs representing the path from start to goal,
         *         or empty vector if no path exists
         */
        virtual vector<pair<int, int>> findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) = 0;

        /**
         * @brief Validates and reconstructs the path from start to goal.
         * 
         * Traces back through the parent array from goal to start to reconstruct
         * the path. Validates that a valid path exists and excludes start/goal
         * positions from the returned path.
         * 
         * @param dist 2D vector containing distances from start to each cell
         * @param parent 2D vector containing parent cell for each position
         * @param start Starting position as (x, y) coordinates
         * @param goal Goal position as (x, y) coordinates
         * @return Vector of (x, y) coordinate pairs representing the validated path,
         *         or empty vector if goal is unreachable
         */
        vector<pair<int, int>> pathValidation(vector<vector<int>> dist, vector<vector<pair<int, int>>> parent, 
                                              pair<int, int> start, pair<int, int> goal) const;
};

#endif // PATHFINDING_MAIN_HPP