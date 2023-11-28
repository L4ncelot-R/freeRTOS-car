/**
 * @file mapping.h
 * @brief Map the environment using the line sensor and the ultrasonic sensor
 *
 * Reference:
 * https://stackoverflow.com/questions/37207022/flood-fill-algorithm-maze
 *
 * @author Woon Jun Wei
 */

#ifndef MAPPING_H
#define MAPPING_H

#include <stdio.h>
#include "pico/stdlib.h"
#include "time.h"
#include "pico/rand.h"

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "semphr.h"

#include "car_config.h"

// Function to generate a random number between min and max (inclusive)
int
generate_random(int min, int max)
{
    int num = (get_rand_32() % (max - min + 1)) + min;
    printf("Random number generated: %d\n", num);
    return num;
}

/**
 * Create a map with hardcoded walls, obstacles, and the goal
 * With the start point at the bottom left corner.
 * Ensures there is at least one clear path from start to goal.
 * @param maze
 */
void
create_map(maze_t *maze)
{
    // Create the map based on maze height and width
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            if (i == 0 || i == maze->height - 1 || j == 0
                || j == maze->width - 1)
            {
                maze->mazecells[i][j].type = 'X'; // Walls at the border
            }
            else
            {
                // Randomly place walls and obstacles
                if (generate_random(0, 9)
                    < 2) // Adjust the threshold for more or fewer obstacles
                {
                    maze->mazecells[i][j].type = 'X'; // Obstacle
                }
                else
                {
                    maze->mazecells[i][j].type = ' '; // Empty space
                }
            }
            maze->mazecells[i][j].reachable = 0;
            maze->mazecells[i][j].visited   = 0;
        }
    }

    // Set the start point
    maze->mazecells[0][0].type      = 'S';
    maze->mazecells[0][0].reachable = 1;
    maze->mazecells[0][0].visited   = 1;

    // Set the goal (assuming it's at the top-right corner)
    maze->mazecells[maze->height - 1][maze->width - 1].type = 'G';

    // Ensure there is a clear path from start to goal
    for (int i = 1; i < maze->height - 1; i++)
    {
        maze->mazecells[i][maze->width / 2].type = ' '; // Clear path
    }
}

/**
 * Create a hardcoded map with a clear path from start to goal
 * @param maze
 */
void
create_hardcoded_map(maze_t *maze)
{
    // Set fixed height and width during initialization
    maze->height = 5;
    maze->width  = 5;

    // Create the map with a clear path
    char hardcoded_map[5][5] = {
        { 'S', ' ', ' ', ' ', 'G' }, { ' ', ' ', 'X', ' ', ' ' },
        { ' ', ' ', ' ', ' ', ' ' }, { ' ', 'X', ' ', ' ', ' ' },
        { 'C', ' ', 'X', ' ', ' ' },
    };

    // Copy the hardcoded map to the maze structure
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            maze->mazecells[i][j].type      = hardcoded_map[i][j];
            maze->mazecells[i][j].reachable = 0;
            maze->mazecells[i][j].visited   = 0;
        }
    }
}

/**
 * @brief Mapping Initialization
 */
void
mapping_init(maze_t *p_maze)
{
    printf("Initializing mapping\n");

    // Set fixed height and width during initialization
    p_maze->height = MAX_HEIGHT / 2;
    p_maze->width  = MAX_WIDTH / 2;

    // Create the maze
    printf("Creating maze\n");
    create_hardcoded_map(p_maze);
    printf("Maze created\n");
}

/**
 * @brief Print the map
 * @param maze
 */
void
print_map(maze_t *maze)
{
    for (int i = maze->height - 1; i >= 0; i--)
    {
        for (int j = 0; j < maze->width; j++)
        {
            char cellType = maze->mazecells[j][i].type;

            switch (cellType)
            {
                case 'X':
                    printf("X "); // Wall
                    break;
                case 'O':
                    printf("O "); // Obstacle
                    break;
                case 'S':
                    printf("S "); // Start
                    break;
                case 'G':
                    printf("G "); // Goal
                    break;
                case 'C':
                    printf("C "); // Car
                    break;
                case 'V':
                    printf("V "); // Visited
                    break;
                default:
                    printf("  "); // Empty space
                    break;
            }
        }
        printf("\n");
    }
}

/**
 * @brief Print the map with the reachable cells
 * @param maze
 */
void
print_map_reachable(maze_t *maze)
{
    for (int i = maze->height - 1; i >= 0; i--)
    {
        for (int j = 0; j < maze->width; j++)
        {
            printf("%d ", maze->mazecells[j][i].reachable);
        }
        printf("\n");
    }
}

/**
 * @brief Perform floodfill on the maze
 * @param maze
 * @param x starting position x-coordinate
 * @param y starting position y-coordinate
 * @param value value to fill
 */
void
floodfill(maze_t *maze, int x, int y, int value)
{
    // Check if the current position is within the maze boundaries and not
    // visited
    if (x >= 0 && x < maze->width && y >= 0 && y < maze->height
        && maze->mazecells[x][y].visited == 0)
    {
        maze->mazecells[x][y].reachable = value;
        maze->mazecells[x][y].visited   = 1;

        // Recursive floodfill for neighboring cells
        floodfill(maze, x + 1, y, value + 1); // right
        floodfill(maze, x - 1, y, value + 1); // left
        floodfill(maze, x, y + 1, value + 1); // up
        floodfill(maze, x, y - 1, value + 1); // down
    }
}

/**
 * @brief Task to simulate the car moving in the maze and perform floodfill
 * @param pvParameters
 */
void
combined_task(void *pvParameters)
{
    maze_t *maze     = (maze_t *)pvParameters;
    int     currentX = 0; // Initial X position
    int     currentY = 0; // Initial Y position

    for (;;)
    {
        // Reset maze before floodfill
        for (int i = 0; i < maze->height; i++)
        {
            for (int j = 0; j < maze->width; j++)
            {
                maze->mazecells[j][i].visited = 0;
            }
        }

        // Simulate car movement (you can replace this logic with your actual
        // movement algorithm)
        mapping_direction_t moveDirection
            = (mapping_direction_t)(get_rand_32()
                                    % 4); // Randomly choose a direction

        // Update the previously visited position before moving
        maze->mazecells[currentX][currentY].type = 'V'; // 'V' for visited

        switch (moveDirection)
        {
            case up:
                if (currentY < maze->height - 1
                    && maze->mazecells[currentX][currentY + 1].type != 'X')
                {
                    currentY++;
                }
                break;
            case down:
                if (currentY > 0
                    && maze->mazecells[currentX][currentY - 1].type != 'X')
                {
                    currentY--;
                }
                break;
            case left:
                if (currentX > 0
                    && maze->mazecells[currentX - 1][currentY].type != 'X')
                {
                    currentX--;
                }
                break;
            case right:
                if (currentX < maze->width - 1
                    && maze->mazecells[currentX + 1][currentY].type != 'X')
                {
                    currentX++;
                }
                break;
        }

        // Update the car's position in the maze
        // (you might want to clear the previous position before updating)
        maze->mazecells[currentX][currentY].type = 'C'; // 'C' for car

        // Print the map with the car's position
        printf("Map with the car's position:\n");
        print_map(maze);

        // Floodfill the maze after each movement
        floodfill(maze, maze->width - 1, 0, 0);

        // Check if the car has reached the goal
        if (maze->mazecells[currentX][currentY].type == 'G')
        {
            printf("Goal reached! Stopping the task.\n");
            // Stop the task
            vTaskSuspend(NULL);
            break;
        }

        vTaskDelay(
            pdMS_TO_TICKS(100)); // Delay to simulate time between movements
    }
}

/**
 * @brief Initialise tasks for the Maze
 * @param maze
 */
void
mapping_tasks_init(maze_t *maze)
{
    TaskHandle_t combined_task_handle = NULL;
    xTaskCreate(combined_task,
                "combined_task",
                configMINIMAL_STACK_SIZE,
                (void *)maze,
                PRIO,
                &combined_task_handle);
}

#endif /* MAPPING_H */
