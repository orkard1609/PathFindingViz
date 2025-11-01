/**
 * @file visualization.hpp
 * @brief Visualization interface for motion planning and pathfinding algorithms
 * @author Nhat Diep
 * @date November 2025
 * @version 1.0
 * 
 * This file provides the Visualizer class for rendering grid-based motion planning
 * environments and animating pathfinding algorithm execution using SFML.
 */

#ifndef VISUALIZATION_CLASS
#define VISUALIZATION_CLASS

#include <vector>
#include <utility>
#include <string>
#include <map>
#include "grid.hpp"
#include "obstacle.hpp"
#include "pathfinding_main.hpp"
#include <SFML/Graphics.hpp>

using namespace std;

// Struct to define interactive boxes and texts
struct controlButton {
    int x; // Box start positions
    int y;
    int width; // Box size 
    int height; 
    
    string boxLabel; // Box label
    string boxType; // Including "interactiveBox", "textOnly" and "inputBox"
    string boxID; // Unique identifier for each box/button

    // Check if the box is clicked
    bool isBoxClicked(int mouseX, int mouseY) const {
        return (mouseX >= x && mouseX <= x + width && 
                mouseY >= y && mouseY <= y + height);
    }
};

//Visualizer class to handle visualization related stuffs
class Visualizer {
    private:
        // Main visualization members
        sf::RenderWindow window_; 
        Grid& grid_;
        Obstacle& obstacle_;
        vector<pair<int, int>> obstacleStack_;

        // Global states for event handling
        string clickedButton; // Global clicked button state
        vector<pair<int, int>> clickedCell; // Global clicked cell state

        // Visualization layout members
        int cellSize_ = 600/grid_.getWidth(); // Size of each cell in pixels
        unsigned int gridWidth_, gridHeight_;  // Store grid width and height in pixels
        int gridOffsetX_ = 0; // X offset for grid positioning
        int gridOffsetY_ = 0; // Y offset for grid positioning
        int boxAlignmentX_ = 0; // X position for control panel alignment
        int boxAlignmentY_ = 0; // Y position for control panel alignment
        unsigned int windowWidth_ = 0; // Width of the entire window
        unsigned int windowHeight_ = 0; // Height of the entire window
        unsigned int controlPanelWidth_ = 200; // Width of the control panel area
        int mousePos_x = 0; // Current mouse position, x
        int mousePos_y = 0; // Current mouse position, y
        map<string, controlButton> buttons_; // Map to store buttons by ID
        sf::Font font; // Font of text
        sf::Color cellColor; // Color corresponding to cell status

        // Input box members
        sf::Clock cursorBlinkClock_; // Timer for cursor blinking
        bool cursorVisible_ = true; // Current state of cursor visibility
        string inputBoxTextX_ = ""; // Text currently in the active input box X
        string inputBoxTextY_ = ""; // Text currently in the active input box Y

        // Algorithm selection members
        vector<string> algorithms = {"BFS", "DFS", "Dijkstra", "A*"};
        string selectedAlgo_ = algorithms[0]; // Init algorithm from dropdown;
        bool isAlgoDropdownOpen_ = false; // Track if algorithm dropdown is open
        bool clickedOutside_ = false; // Track if user clicked outside dropdown list
        string clickedAlgo_ = ""; // Store the algorithm selected from dropdown list
        bool isObstacleSet_ = false; // Track if user is in obstacle setting mode

        // Start/Goal selection members
        vector<pair<int, int>> selectedStartGoal_; // Store current start/goal positions being set
        bool isStartGoalSet_ = false; // Track if user is in start-goal position setting mode

        // Algorithm visualization members
        bool isPathDisplaying_ = false; // Track if path is displaying
        vector<pair<int, int>> path_ = {}; // Store found path for visualization
        vector<pair<int, int>> visitedNodes_ = {}; // Store visited nodes for visualization
        size_t visitedNodeIndex_ = 0; // Index for visited nodes visualization
        size_t pathNodeIndex_ = 0; // Index for path visualization
        sf::Clock visualizationClock_; // Clock to control visualization speed
        bool isVisualizing_ = false; // Track if visualization is in progress
    public:
        /**
         * @brief Constructs a Visualizer object and initializes the visualization window.
         * 
         * Initializes all visualization components including window dimensions, grid layout,
         * control panel buttons, and loads necessary fonts. The window is created with
         * appropriate padding and control panel space.
         * 
         * @param grid Reference to the Grid object to be visualized
         * @param obstacle Reference to the Obstacle object for obstacle management
         */
        Visualizer(Grid& grid, Obstacle& obstacle);
        
        /**
         * @brief Destructor for the Visualizer class.
         */
        ~Visualizer() {}

        /**
         * @brief Renders and displays all visualization components.
         * 
         * This method performs the complete rendering cycle including:
         * - Clearing the window
         * - Drawing the grid with cells colored according to their states
         * - Drawing the control panel and all buttons
         * - Animating visited nodes and path visualization if they are found
         * - Displaying the final rendered frame
         * 
         * Called once per frame in the main loop.
         */
        void displayWindows();
        
        /**
         * @brief Checks if the visualization window is currently open.
         * 
         * @return true if the window is open, false otherwise
         */
        bool windowIsOpen() const {
            return window_.isOpen();
        }
        
        /**
         * @brief Updates the stored mouse position coordinates.
         * 
         * @param x The x-coordinate of the mouse position in pixels
         * @param y The y-coordinate of the mouse position in pixels
         */
        void setMousePos(int x, int y) {
            mousePos_x = x;
            mousePos_y = y;
        }

        /**
         * @brief Converts mouse click coordinates to grid cell coordinates.
         * 
         * Takes the current mouse position and determines which grid cell (if any)
         * was clicked. Handles coordinate transformation accounting for grid offset
         * and validates that the click is within grid boundaries.
         * 
         * @return A vector containing the (x, y) coordinates of the clicked cell,
         *         or an empty vector if the click was outside the grid area
         */
        vector<pair<int, int>> getCellClick();

        /**
         * @brief Determines which control button (if any) was clicked.
         * 
         * Checks the current mouse position against all registered buttons in the
         * control panel and returns the ID of the clicked button.
         * 
         * @return The string ID of the clicked button, or an empty string if no
         *         button was clicked
         */
        string getButtonClick();

        /**
         * @brief Handles grid resizing functionality and user text input.
         * 
         * Manages the complete grid resize workflow including:
         * - Activating/deactivating input boxes for width and height
         * - Processing keyboard input for numeric values
         * - Handling backspace for text deletion
         * - Confirming and applying the new grid dimensions
         * 
         * @param inputChar The character entered by the user (default: 0 for no input).
         *                  Special values: '\\b' for backspace, digits for grid size
         */
        void handleGridResize(char inputChar = 0);

        /**
         * @brief Manages obstacle placement and removal on the grid.
         * 
         * Handles the obstacle editing workflow:
         * - Toggling obstacle setting mode on/off
         * - Adding obstacles at clicked cell locations
         * - Removing the most recently placed obstacle (undo)
         * - Confirming the final obstacle configuration
         */
        void handleSetObstacle();

        /**
         * @brief Manages start and goal position selection on the grid.
         * 
         * Handles the start/goal selection workflow:
         * - Toggling start/goal setting mode
         * - Setting start position (first click)
         * - Setting goal position (second click)
         * - Validating that start and goal are different cells
         * - Confirming the selected positions
         */
        void handleStartGoalSelection();

        /**
         * @brief Manages the algorithm selection dropdown interface.
         * 
         * Controls the algorithm selection dropdown behavior:
         * - Opening/closing the dropdown list
         * - Tracking mouse hover over algorithm options
         * - Updating the selected algorithm
         * - Handling clicks outside the dropdown to close it
         */
        void handlePathPlanningBox();

        /**
         * @brief Executes the selected pathfinding algorithm and initiates visualization.
         * 
         * This method:
         * - Validates that start and goal positions are set
         * - Creates an instance of the selected pathfinding algorithm
         * - Retrieves the computed path and visited nodes
         * - Initializes the animated visualization sequence
         * - Handles error cases (no path found, missing positions, etc.)
         */
        void handlePathPlanningAlgo();

        /**
         * @brief Renders a single control button with specified properties.
         * 
         * Draws control panel UI elements with different styles based on type.
         * Handles special rendering for active input boxes (blinking cursor)
         * and dropdown lists.
         * 
         * @param x The x-coordinate of the button's top-left corner in pixels
         * @param y The y-coordinate of the button's top-left corner in pixels
         * @param width The width of the button in pixels
         * @param height The height of the button in pixels
         * @param inputText The text label displayed on the button
         * @param boxType The type of button: "textOnly", "inputBox", or "interactiveBox"
         */
        void drawControlButton(int x, int y, int width, int height, const std::string& inputText, const std::string& boxType);
        
        /**
         * @brief Processes and dispatches all window events.
         * 
         * Polls SFML events in a loop and routes them to appropriate handlers:
         * - Window close events
         * - Mouse button press events (clicks on grid cells and buttons)
         * - Text input events (for grid size input boxes)
         * 
         * This method should be called once per frame in the main loop.
         */
        void handleEvents();

        /**
         * @brief Resets the visualization to its initial state.
         * 
         * Clears all user-defined elements and resets the grid:
         * - Resets grid to default or current size
         * - Clears all obstacles from the grid
         * - Clears start and goal positions
         * - Clears any displayed paths and visited nodes
         * - Resets all UI states and input fields
         * - Adds random obstacles for testing purposes
         */
        void resetWindows();

        /**
         * @brief Provides access to the underlying SFML render window.
         * 
         * @return Reference to the sf::RenderWindow object used for rendering
         */
        sf::RenderWindow& getWindow();

    protected:
        /**
         * @brief Determines the display color for a grid cell based on its state.
         * 
         * Maps cell states to their corresponding visual colors:
         * - EMPTY: White
         * - OBSTACLE: Black
         * - START: Blue
         * - GOAL: Red
         * - PATH: Green
         * - VISITED: Magenta
         * 
         * @param x The x-coordinate of the cell in grid coordinates
         * @param y The y-coordinate of the cell in grid coordinates
         * @param[out] cellColor Reference to sf::Color object that will be set to the appropriate color
         */
        void coloringCell(int x, int y, sf::Color& cellColor) const;
};

#endif // VISUALIZATION_CLASS