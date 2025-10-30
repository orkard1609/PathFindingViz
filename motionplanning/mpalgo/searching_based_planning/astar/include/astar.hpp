#ifndef ASTAR_MAIN_HPP
#define ASTAR_MAIN_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

class AStar : public PathFindingAlgorithm {
    private:
        struct Node {
            pair<int, int> position;
            int g, f; // g, f in cost function are defined as integer for Manhattan distance only
            bool operator>(const Node& other) const {
                return f > other.f;
            }
        };
    public:
        // Constructor
        using PathFindingAlgorithm::PathFindingAlgorithm;

        // f - moving cost from current node to goal calculation
        int heuristicCalc(pair<int, int> currentNode, pair<int, int> goal) const {
            return abs(currentNode.first - goal.first) + abs(currentNode.second - goal.second); // only consider 4 neighbors
        }

        // Override the findPath method of the base class
        vector<pair<int, int>> findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) override;
};

#endif // ASTART_MAIN_HPP