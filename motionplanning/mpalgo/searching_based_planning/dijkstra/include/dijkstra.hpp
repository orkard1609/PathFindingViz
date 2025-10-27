#ifndef DIJKSTRA_HPP
#define DIJKSTRA_HPP

#include <vector>
#include <utility>
#include <string>
#include "pathfinding_main.hpp"

class Dijkstra : public PathFindingAlgorithm {
    private:
        // Node definition with overriding operator for priority queue
        struct Node {
            pair<int, int> position;
            int cost;
            bool operator>(const Node& other) const {
                return cost > other.cost;
            }
        };
    public:
        // Constructor
        using PathFindingAlgorithm::PathFindingAlgorithm;

        // Override the findPath method of the base class
        vector<pair<int, int>> findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) override;
};

#endif // DIJKSTRA_HPP