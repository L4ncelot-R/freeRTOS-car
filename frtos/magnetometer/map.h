//
// Created by junwei on 31/10/23.
//

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef TEST_PROJECT_MAP_H
#define TEST_PROJECT_MAP_H


// Define the grid structure
typedef struct {
    bool **data;  // 2D array to represent the grid
    int rows;     // Number of rows in the grid
    int cols;     // Number of columns in the grid
} Grid;

// Global grid to track the car's path
Grid *car_path_grid;

// Function to create and initialize a grid
Grid *create_grid(int rows, int cols) {
    Grid *grid = (Grid *) malloc(sizeof(Grid));
    grid->rows = rows;
    grid->cols = cols;

    // Allocate memory for the 2D array
    grid->data = (bool **) malloc(rows * sizeof(bool *));
    for (int i = 0; i < rows; i++) {
        grid->data[i] = (bool *) malloc(cols * sizeof(bool));
        for (int j = 0; j < cols; j++) {
            grid->data[i][j] = false;  // Initialize to 'false' (unvisited)
        }
    }

    return grid;
}

// Function to mark a cell as visited
void mark_cell(Grid *grid, int row, int col) {
    if (row >= 0 && row < grid->rows && col >= 0 && col < grid->cols) {
        grid->data[row][col] = true;
    }
}

// Function to check if a cell has been visited
bool is_cell_visited(Grid *grid, int row, int col) {
    if (row >= 0 && row < grid->rows && col >= 0 && col < grid->cols) {
        return grid->data[row][col];
    }
    return false;  // Consider out-of-bounds as unvisited
}

// Function to destroy the grid and free memory
void destroy_grid(Grid *grid) {
    for (int i = 0; i < grid->rows; i++) {
        free(grid->data[i]);
    }
    free(grid->data);
    free(grid);
}

// Function to update the map based on car's current orientation
// Function to update the map based on car's current orientation and position
void update_map(int orientation, int cur_x, int cur_y) {
    // Define offsets for different orientations
    int offset_x = 0;
    int offset_y = 0;

    switch (orientation) {
        case NORTH:
            offset_y = 1;
            break;
        case EAST:
            offset_x = 1;
            break;
        case SOUTH:
            offset_y = -1;
            break;
        case WEST:
            offset_x = -1;
            break;
        case NORTH_EAST:
            offset_x = 1;
            offset_y = 1;
            break;
        case SOUTH_EAST:
            offset_x = 1;
            offset_y = -1;
            break;
        case SOUTH_WEST:
            offset_x = -1;
            offset_y = -1;
            break;
        case NORTH_WEST:
            offset_x = -1;
            offset_y = 1;
            break;
    }

    // Update the map based on the car's current position and orientation
    mark_cell(car_path_grid, cur_x, cur_y);
    mark_cell(car_path_grid, cur_x + offset_x, cur_y + offset_y);
}


// Function to print the map
void print_map() {
    // Invert the map, 0,0 is at the Middle
    // Print 1 for visited cells and 0 for unvisited cells
    for (int i = car_path_grid->rows - 1; i >= 0; i--) {
        for (int j = 0; j < car_path_grid->cols; j++) {
            (car_path_grid->data[j][i]) ? printf("1 ") : printf("0 ");
//                case false:
//                    printf("0 ");
//                    break;
//                case true:
//                    printf("1 ");
//                    break;
//            }
        }
        printf("\n");
    }
}

#endif //TEST_PROJECT_MAP_H
