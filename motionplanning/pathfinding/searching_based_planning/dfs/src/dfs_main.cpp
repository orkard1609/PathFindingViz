#include <vector>
#include <utility>
#include <string>
#include <iostream>
#include <algorithm>
#include "dfs.hpp"
#include "pathfinding_main.hpp"

using namespace std;

void DFS::dfs(pair<int, int> cur, pair<int, int> goal, 
              vector<vector<int>>& dist, vector<vector<pair<int, int>>>& parent) {
    // DFS neighbor exploration
    vector<pair<int, int>> neighbors_ = getNeighbors(cur.first, cur.second);
    // Stop when reaching the goal
    if (cur == goal) {
        goalisFound = true;
        return;
    }
    // Check all possible directions
    for (int i = 0; i < neighbors_.size() && !goalisFound; i++) {
        int nx = neighbors_[i].first;
        int ny = neighbors_[i].second;
        if (dist[nx][ny] == INT_MAX) {
            dist[nx][ny] = dist[cur.first][cur.second] + 1;      // update distance
            parent[nx][ny] = cur;                                // store parent
            visitedNodes_.push_back({nx, ny});                   // store for visited node visualization
            dfs(pair<int, int>{nx, ny}, goal, dist, parent);     // recursive call to keep exploring current neighbor
        }
    }
}

vector<pair<int, int>> DFS::findPath(const vector<vector<int>>& grid, pair<int, int> start, pair<int, int> goal) {
    // Push start point into stack
    dist[start.first][start.second] = 0;
    pair<int, int> cur = start;
    dfs(cur, goal, dist, parent);

    return pathValidation(dist, parent, start, goal);
}