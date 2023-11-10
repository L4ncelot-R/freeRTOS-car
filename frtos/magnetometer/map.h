//
// Created by junwei on 31/10/23.
//

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#ifndef TEST_PROJECT_MAP_H
#define TEST_PROJECT_MAP_H

// Function to create and initialize a grid
void
map_init(int rows, int cols, grid_t *p_grid, int initial_x, int initial_y)
{
    p_grid->rows      = rows;
    p_grid->cols      = cols;
    p_grid->initial_x = initial_x;
    p_grid->initial_y = initial_y;

    // Allocate memory for the grid
    p_grid->data = (bool **)malloc(rows * sizeof(bool *));
    for (int i = 0; i < rows; i++)
    {
        p_grid->data[i] = (bool *)malloc(cols * sizeof(bool));
        for (int j = 0; j < cols; j++)
        {
            p_grid->data[i][j] = false; // Initialize to 'false' (unvisited)
        }
    }
}

// Function to mark a cell as visited
void
mark_cell(grid_t *grid, int row, int col)
{
    if (row >= 0 && row < grid->rows && col >= 0 && col < grid->cols)
    {
        grid->data[row][col] = true;
    }
}

// Function to check if a cell has been visited
bool
is_cell_visited(grid_t *grid, int row, int col)
{
    if (row >= 0 && row < grid->rows && col >= 0 && col < grid->cols)
    {
        return grid->data[row][col];
    }
    return false; // Consider out-of-bounds as unvisited
}

// Function to destroy the grid and free memory
void
destroy_grid(grid_t *grid)
{
    for (int i = 0; i < grid->rows; i++)
    {
        free(grid->data[i]);
    }
    free(grid->data);
    free(grid);
}

// Function to update the map based on the car's current orientation and
// position
void
update_map(grid_t *car_path_grid, uint32_t direction, float distance)
{
    int cur_x = car_path_grid->initial_x;
    int cur_y = car_path_grid->initial_y;

    // Define offsets for different forward and turn directions
    int offset_x = 0;
    int offset_y = 0;

    // Update the current position based on the forward direction
    switch (direction)
    {
        case DIRECTION_LEFT:
            offset_x = -1;
            break;
        case DIRECTION_RIGHT:
            offset_x = 1;
            break;
        case DIRECTION_FORWARD:
            offset_y = 1;
            break;
        case DIRECTION_BACKWARD:
            offset_y = -1;
            break;
    }

    cur_x += offset_x;
    cur_y += offset_y;

    // Mark the current and next position as visited
    mark_cell(car_path_grid, cur_x, cur_y);
    car_path_grid->initial_x = cur_x;
    car_path_grid->initial_y = cur_y;
    car_path_grid->distance_array[car_path_grid->distance_array_size]
        = distance;
    car_path_grid->distance_array_size++;


}

// Function to print the map
void
print_map(grid_t *car_path_grid)
{
    // Invert the map, 0,0 is at the Middle
    // Print 1 for visited cells and 0 for unvisited cells
    for (int i = car_path_grid->rows - 1; i >= 0; i--)
    {
        for (int j = 0; j < car_path_grid->cols; j++)
        {
            (car_path_grid->data[j][i]) ? printf("1 ") : printf("0 ");
        }
        printf("\n");
    }
}
#endif // TEST_PROJECT_MAP_H
