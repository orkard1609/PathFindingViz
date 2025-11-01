/**
 * @file grid.hpp
 * @brief Grid environment representation for pathfinding algorithms
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file provides the Grid class for managing 2D grid environments with
 * support for obstacles, start/goal positions, and pathfinding visualization.
 */

#ifndef GRID_CLASS
#define GRID_CLASS

#include <vector>
#include <utility>

using namespace std;

// Forward declaration to avoid circular dependency
class Obstacle;

//Grid class to handle grid related stuffs
class Grid {
    private:
        //Width: number of cells in x axis
        int width;
        //Height: number of cells in y axis
        int height;
        //cellPos: vector to store cell positions, redundant variable, can be removed later
        vector<pair<int, int>> cellPos;
        //gridStatus: vector to store grid status
        vector<vector<int>> gridStatus; // 2D vector to represent the grid
    public:
        /**
         * @brief Enumeration defining possible states for grid cells.
         * 
         * EMPTY: Traversable cell with no special designation
         * OBSTACLE: Non-traversable cell that blocks movement
         * START: Starting position for pathfinding
         * GOAL: Target position for pathfinding
         * PATH: Cell that is part of the computed path
         * VISITED: Cell that was explored during pathfinding
         */
        enum cellStatus { EMPTY, OBSTACLE, START, GOAL, PATH, VISITED }; 

        /**
         * @brief Constructs a Grid object with specified dimensions.
         * 
         * Initializes the grid with the given width and height, creating a 2D
         * representation where all cells are initially empty.
         * 
         * @param width Number of cells along the x-axis (default: 256)
         * @param height Number of cells along the y-axis (default: 256)
         */
        Grid(int width = 256, int height = 256);
        
        /**
         * @brief Destructor for the Grid class.
         */
        ~Grid() {}
        
        /**
         * @brief Retrieves the width of the grid.
         * 
         * @return Number of cells along the x-axis
         */
        int getWidth() const { return width;}
        
        /**
         * @brief Retrieves the height of the grid.
         * 
         * @return Number of cells along the y-axis
         */
        int getHeight() const { return height; }
        
        /**
         * @brief Resizes the grid to new dimensions.
         * 
         * Creates a new grid with the specified dimensions. All existing cell
         * states are cleared and reset to EMPTY. This operation is typically
         * triggered by user input in the visualization interface.
         * 
         * @param newWidth New number of cells along the x-axis
         * @param newHeight New number of cells along the y-axis
         */
        void gridResize(int newWidth, int newHeight);
        
        /**
         * @brief Validates grid dimensions.
         * 
         * Checks if the provided width and height are positive values,
         * which is required for a valid grid.
         * 
         * @param width Width to validate
         * @param height Height to validate
         * @return true if both dimensions are positive, false otherwise
         */
        bool isValid(int width, int height) const { return (width > 0 && height > 0);}
        
        /**
         * @brief Sets the state of a specific cell in the grid.
         * 
         * Updates the cell at the given coordinates to the specified status
         * (EMPTY, OBSTACLE, START, GOAL, PATH, or VISITED).
         * 
         * @param x The x-coordinate of the cell
         * @param y The y-coordinate of the cell
         * @param status The new status to assign to the cell
         */
        void setCellState(int x, int y, cellStatus status);
        
        /**
         * @brief Retrieves the current state of a specific cell.
         * 
         * @param x The x-coordinate of the cell
         * @param y The y-coordinate of the cell
         * @return The current cellStatus of the specified cell
         */
        Grid::cellStatus getCellState(int x, int y) const;
        
        /**
         * @brief Provides read-only access to the complete grid representation.
         * 
         * Returns a const reference to the 2D vector representing the entire grid
         * state. Each cell value corresponds to a cellStatus enum value.
         * 
         * @return Const reference to the 2D vector grid representation
         */
        const vector<vector<int>>& getGridStatus() const { return gridStatus; }
};

#endif // GRID_CLASS