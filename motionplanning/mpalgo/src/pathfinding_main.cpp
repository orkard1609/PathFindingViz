#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <memory>  // For unique_ptr and make_unique
#include <cmath>
#include "grid.hpp"
#include "pathfinding_main.hpp"
#include "bfs.hpp"
#include "dfs.hpp"
#include "dijkstra.hpp"

using namespace std;

unique_ptr<PathFindingAlgorithm> PathFindingAlgorithm::selectAlgorithm(const string& algo, Grid& grid, 
                                                                            const pair<int, int>& start, 
                                                                            const pair<int, int>& goal) {
    if (algo == "BFS") {
        return make_unique<BFS>(grid, start, goal, algo);
    } else if (algo == "DFS") {
        return make_unique<DFS>(grid, start, goal, algo);
    } else if (algo == "A*") {
        //return make_unique<AStar>(grid, start, goal, algo);
        cout << "A* algorithm is not supported yet." << endl;
    } else if (algo == "Dijkstra") {
        return make_unique<Dijkstra>(grid, start, goal, algo);
    } else if (algo == "RRT") {
        //return make_unique<RRT>(grid, start, goal, algo);
        cout << "RRT algorithm is not supported yet." << endl;
    } else {
        // Fallback for unrecognized algorithm
        cout << "Algorithm " << algo << " is not recognized." << endl;
    }
    // Return nullptr for unsupported or unrecognized algorithms
    return nullptr;
}

vector<pair<int, int>> PathFindingAlgorithm::goalDirectionCalc(pair<int, int> start, pair<int, int> goal) const {
    /*
    ### Improvement for DFS
    Idea: Calculate the direction vector from start to goal
            Use this vector to prioritize neighbor exploration order
    As we only have 4 directions (up, down, left, right) so we can just decide the direction based on 4 quadrant
    If angle is between:
        -45 to 45 degrees: prioritize right
        45 to 135 degrees: prioritize down
        135 to 225 degrees: prioritize left
        225 to 315 degrees: prioritize up 
    */
    // Stat-goal vector angle
    double sgAngle = atan2(goal.second - start.second, goal.first - start.first) * 180.0 / 3.14159265358979323846;
    // Angle correction to be within [0, 360)
    if (sgAngle < 0) {
        sgAngle = -sgAngle;
    } else if (sgAngle > 0 && sgAngle < 180) {
        sgAngle = 360 - sgAngle;
    }
    vector<pair<int, int>> directions_;
    // First quadrant
    if ((sgAngle >= 0 && sgAngle < 45) || 
        (sgAngle >= 315 && sgAngle < 360)) {
        // Prioritize right
        directions_ = {RIGHT, UP, DOWN, LEFT}; 
    }
    // Second quadrant
    else if (sgAngle >= 45 && sgAngle < 135) {
        // Prioritize up
        directions_ = {UP, RIGHT, LEFT, DOWN}; 
    }
    // Third quadrant
    else if (sgAngle >= 135 && sgAngle < 225) {
        // Prioritize left
        directions_ = {LEFT, UP, DOWN, RIGHT};
    }
    // Fourth quadrant
    else if (sgAngle >= 225 && sgAngle < 315) {
        // Prioritize down
        directions_ = {DOWN, RIGHT, LEFT, UP};
    } else {
        // Default order if angle is exactly on the axes
        directions_ = {UP, RIGHT, DOWN, LEFT};
    }
    return directions_;
}

vector<pair<int, int>> PathFindingAlgorithm::getNeighbors(int x, int y) const {
    vector<pair<int, int>> neighbors;
    vector<pair<int, int>> directions = goalDirectionCalc(startPos_, goalPos_);

    for (const auto& dir : directions) {
        int newX = x + dir.first;
        int newY = y + dir.second;
        // Check if the new position is within grid bounds and not an obstacle
        if (newX >= 0 && newX < grid_.getWidth() && newY >= 0 && newY < grid_.getHeight() && 
            grid_.getCellState(newX, newY) != Grid::OBSTACLE) {
            neighbors.push_back({newX, newY});
        }
    }
    return neighbors;
}