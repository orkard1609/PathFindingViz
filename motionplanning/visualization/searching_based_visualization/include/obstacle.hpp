/**
 * @file obstacle.hpp
 * @brief Obstacle management for grid-based pathfinding
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file provides the Obstacle class for managing obstacle placement,
 * removal, and storage in grid environments using a LIFO stack structure.
 */

#ifndef OBSTACLE_CLASS
#define OBSTACLE_CLASS
#include <vector>
#include <utility>

using namespace std;

/**
 * @brief Obstacle manager for grid-based pathfinding environments.
 */
class Obstacle {
    private:
        //obstaclePos: LIFO stack to store the positions of obstacles
        vector<pair<int, int>> obstaclePositions;
    public:
        /**
         * @brief Default constructor for the Obstacle class.
         * 
         * Creates an Obstacle object with an empty list of obstacle positions.
         */
        Obstacle() {}
        
        /**
         * @brief Constructs an Obstacle object with predefined positions.
         * 
         * Initializes the obstacle manager with a list of existing obstacle
         * positions, useful for loading saved configurations.
         * 
         * @param positions Vector of (x, y) coordinate pairs representing obstacle locations
         */
        Obstacle(vector<pair<int, int>> positions) : obstaclePositions(positions) {}
        
        /**
         * @brief Destructor for the Obstacle class.
         */
        ~Obstacle() {}
        
        /**
         * @brief Adds a new obstacle at the specified position.
         * 
         * Appends the given (x, y) coordinates to the obstacle stack. This method
         * is typically called when a user clicks on a grid cell while in obstacle
         * placement mode. The stack structure enables undo functionality.
         * 
         * @param obstaclePositions Reference to the vector storing obstacle positions (LIFO stack)
         * @param x The x-coordinate of the new obstacle
         * @param y The y-coordinate of the new obstacle
         * @return Updated vector of obstacle positions including the new obstacle
         */
        vector<pair<int, int>> addObstacle(vector<pair<int, int>>& obstaclePositions, int x, int y);
        
        /**
         * @brief Removes the most recently added obstacle.
         * 
         * Pops the last obstacle from the stack, effectively undoing the most recent
         * obstacle placement. This allows users to correct mistakes during obstacle
         * configuration without clearing all obstacles.
         * 
         * @param obstaclePositions Reference to the vector storing obstacle positions (LIFO stack)
         * @return Updated vector of obstacle positions with the last obstacle removed
         */
        vector<pair<int, int>> undoObstacle(vector<pair<int, int>>& obstaclePositions);
        
        /**
         * @brief Retrieves all currently defined obstacle positions.
         * 
         * Returns a copy of the obstacle positions vector, typically used when
         * finalizing obstacle placement or for rendering purposes.
         * 
         * @return Vector of (x, y) coordinate pairs representing all obstacle locations
         */
        vector<pair<int, int>> getObstacle() const { return obstaclePositions; }
};  

#endif //OBSTACLE_CLASS
