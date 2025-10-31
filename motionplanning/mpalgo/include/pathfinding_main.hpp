#ifndef PATHFINDING_MAIN_HPP
#define PATHFINDING_MAIN_HPP

#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <memory>
#include "grid.hpp"

using namespace std;

// Direction definitions for neighbor exploration
#define UP    pair<int, int>{0, -1}
#define RIGHT pair<int, int>{1, 0}
#define DOWN  pair<int, int>{0, 1}
#define LEFT  pair<int, int>{-1, 0}

class PathFindingAlgorithm {
    private:
        vector<string> algorithms = {"A*", "Dijkstra", "BFS", "DFS"}; // List of supported algorithms
        string selectedAlgo_ = "Dijkstra"; // Default algorithm
    protected:    
        Grid& grid_;
        pair<int, int> startPos_; // Start position
        pair<int, int> goalPos_; // Goal position
        vector<pair<int, int>> foundPath_; // Store the found path
        vector<pair<int, int>> visitedNodes_; // Store visited nodes
        //vector<pair<int, int>> directions_; // Directions for neighbor exploration
    public:
        // Constructor
        PathFindingAlgorithm(Grid& grid, const pair<int, int>& startPos, const pair<int, int>& goalPos, string selectedAlgo) 
                            : grid_(grid), startPos_(startPos), goalPos_(goalPos), selectedAlgo_(selectedAlgo) {}
        // Destructor for virtual base class
        virtual ~PathFindingAlgorithm() {}

        // Static factory method to create algorithm instances
        static unique_ptr<PathFindingAlgorithm> selectAlgorithm(const string& algo, Grid& grid, 
                                                                const pair<int, int>& start, 
                                                                const pair<int, int>& goal);

        // Main interfaces method, provide found path from selected algorithm
        vector<pair<int, int>> getFoundPath() {
            if (foundPath_.empty()) {
                foundPath_ = findPath(grid_.getGridStatus(), startPos_, goalPos_);
            }
            return foundPath_;
        }

        // Helper methods, provide visited nodes from selected algorithm
        vector<pair<int, int>> getVisitedNodes() const {
            return visitedNodes_;
        };

        // Calculate direction towards goal for neighbor prioritization
        vector<pair<int, int>> goalDirectionCalc(pair<int, int> start, pair<int, int> goal) const;

        // Helper methods, get neighbors of a given cell (x, y)
        vector<pair<int, int>> getNeighbors(int x, int y) const;

        // Call path finding algorithms - virtual function to be overridden by derived classes
        virtual vector<pair<int, int>> findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) = 0;

        // Path validity check
        vector<pair<int, int>> pathValidation(vector<vector<int>> dist, vector<vector<pair<int, int>>> parent, 
                                              pair<int, int> start, pair<int, int> goal) const;
};

#endif // PATHFINDING_MAIN_HPP