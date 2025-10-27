#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <queue>
#include <algorithm>
#include "pathfinding_main.hpp"
#include "dijkstra.hpp"

using namespace std;

vector<pair<int, int>> Dijkstra::findPath(const vector<vector<int>>& grid, 
                                            pair<int, int> start, 
                                            pair<int, int> goal) {
    vector<pair<int, int>> path;
    vector<pair<int, int>> neighbors;
    int n = grid.size(), m = grid[0].size();
                                                
    vector<vector<int>> dist(n, vector<int>(m, INT_MAX)); // Distance array from start to each cell
    vector<vector<pair<int, int>>> parent(n, vector<pair<int, int>>(m, {-1,-1})); // Store parent to reconstruct path
    vector<vector<bool>> visited(n, vector<bool>(m, false)); // Visited array
    // Initialize distance for start node, unweighted grid so all possible moves will have same cost
    dist[start.first][start.second] =  grid[start.first][start.second];           
    
    // Priority queue for Dijkstra's algorithm
    priority_queue<Node, vector<Node>, greater<Node>> prioQueue;
    prioQueue.push({start, dist[start.first][start.second]});
    
    // Dijkstra's main loop
    while (!prioQueue.empty()) {
        // Priority node is using greater<Node> so the smallest cost node will be on top
        Node cur = prioQueue.top();
        prioQueue.pop();
        pair<int, int> curPos = cur.position;
        // Skip if this node has been visited with a shorter path
        if (visited[curPos.first][curPos.second]) continue;
        // Mark the current node as visited
        visited[curPos.first][curPos.second] = true;
        // Stop when reaching the goal
        if (curPos == goal) break;
        // Get neighbors
        neighbors = getNeighbors(curPos.first, curPos.second);

        for (auto neighbor : neighbors) {
            int nx = neighbor.first;
            int ny = neighbor.second;
            // Calculate new cost to reach neighbor
            int newCost = dist[curPos.first][curPos.second] + grid[nx][ny];
            // If new cost is lower, update distance and parent, and push to priority queue
            if (newCost < dist[nx][ny]) {
                // Set current smaller cost as neighbor cost
                dist[nx][ny] = newCost;
                parent[nx][ny] = curPos; // store parent
                prioQueue.push({{nx, ny}, newCost});
                visitedNodes_.push_back({nx, ny}); // store for visited node visualization
            }
        }
    }
    // If goal is unreachable then there is not path
    if (dist[goal.first][goal.second] == -1) {
        return vector<pair<int, int>>(); // return empty path
    }
    // Trace back the path from goal to start using parent array
    for (pair<int, int> p = goal; p.first != -1; p = parent[p.first][p.second]) {
        // Remove start and goal cells out of found path
        if (p != start && p != goal) {
            path.push_back(p);
        }
    }
    reverse(path.begin(), path.end()); // reverse to get path from start to goal
    
    return path;
}