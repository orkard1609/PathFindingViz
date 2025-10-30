#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <queue>
#include <algorithm>
#include "pathfinding_main.hpp"
#include "astar.hpp"

using namespace std;

vector<pair<int, int>> AStar::findPath(const vector<vector<int>>& grid, 
                                                pair<int, int> start, 
                                                pair<int, int> goal) {
    vector<pair<int, int>> neighbors;
    int n = grid.size(), m = grid[0].size();

    vector<vector<int>> dist(n, vector<int>(m, INT_MAX)); // Distance from start to each cell
    vector<vector<pair<int, int>>> parent(n, vector<pair<int, int>>(m, {-1, -1})); // Parent of each cell
    vector<vector<bool>> visited(n, vector<bool>(m, false)); // Visited cells
    // initialize distance for start node
    dist[start.first][start.second] = 0; // set as 0 because the first cell of grid can be an obstacle

    // priority queue for A* algorithm
    priority_queue<Node, vector<Node>, greater<Node>> prioQueue;
    prioQueue.push({start, dist[start.first][start.second], heuristicCalc(start, goal)});

    // main loop
    while (!prioQueue.empty()) {
        // Priority node is using greater<Node> so the smallest cost node will be on top
        Node cur = prioQueue.top();
        prioQueue.pop(); // pop out the smallest cost node for further processing
        pair<int, int> curPos = cur.position;
        // Skip if this node has been visited with a shorter path
        if (visited[curPos.first][curPos.second]) continue;
        // Mark the current node as visited
        visited[curPos.first][curPos.second] = true;
        // Stop when reaching the goal
        if (curPos == goal) break;
        neighbors = getNeighbors(curPos.first, curPos.second);

        for (auto neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;
            int newGCost = cur.g + 1; // update G cost, all directions have 1 as moving cost
            if (newGCost < dist[nx][ny]) {
                dist[nx][ny] = newGCost; // update G cost for the node
                parent[nx][ny] = curPos; // update parent for path reconstruction
                int fCost = newGCost + heuristicCalc({nx, ny}, goal);
                prioQueue.push({{nx, ny}, newGCost, fCost});
                visitedNodes_.push_back({nx, ny});
            }
        }
    }

    return pathValidation(dist, parent, start, goal);
}
