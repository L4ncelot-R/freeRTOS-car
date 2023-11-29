/**
 * @file mapping.h
 * @author Woon Jun Wei             2200624     (Team 39)
 * @author Wang Rongqi Richie       2201942     (Team 39)
 * @author Poon Xiang Yuan          2200559     (Team 39)
 * @author Benjamin Loh Choon How   2201590     (Team 78)
 * @author Low Hong Sheng Jovian    2203654     (Team 78)
 * @brief Simulated Mapping of the Maze
 *
 * @details This file contains the functions to simulate the mapping of the
 *          maze. Maze is created with hardcoded walls, obstacles, and the goal.
 *
 *          Maze is then mapped by simulating the car's movement and performing
 *          floodfill to mark the reachable cells.
 *
 *          The car will backtrack to the start once the entire maze is
 *          explored.
 *
 *          The shortest path from start to goal is then found using BFS.
 *
 *          BFS is also used to demonstrate the car's movement from start to
 *          goal.
 *
 *          A Queue Data Structure is used to perform BFS.
 *
 * Reference:
 * https://stackoverflow.com/questions/37207022/flood-fill-algorithm-maze
 *
 */

#ifndef MAPPING_H
#define MAPPING_H

#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>
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
 * Queue Node Structure
 */
typedef struct
{
    int x;
    int y;
} QueueNode;

/**
 * Queue Structure
 */
typedef struct
{
    QueueNode *array;
    int        front, rear, size;
    unsigned   capacity;
} Queue;

/**
 * Queue Creation
 * @param capacity
 * @return
 */
Queue *
createQueue(unsigned capacity)
{
    Queue *queue    = (Queue *)malloc(sizeof(Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear                = capacity - 1;
    queue->array = (QueueNode *)malloc(capacity * sizeof(QueueNode));
    return queue;
}

/**
 * @brief Check if the queue is empty
 * @param queue
 * @return true if the queue is empty, false otherwise
 */
bool
isEmpty(Queue *queue)
{
    return (queue->size == 0);
}

/**
 * @brief Check if the queue is full
 * @param queue
 * @return true if the queue is full, false otherwise
 */
bool
isFull(Queue *queue)
{
    return (queue->size == queue->capacity);
}

/**
 * @brief Enqueue a cell to the queue
 * @param queue
 * @param x
 * @param y
 */
void
enqueue(Queue *queue, int x, int y)
{
    if (isFull(queue))
        return;
    queue->rear                 = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear].x = x;
    queue->array[queue->rear].y = y;
    queue->size                 = queue->size + 1;
}

/**
 * @brief Dequeue a cell from the queue
 * @param queue
 * @return
 */
QueueNode
dequeue(Queue *queue)
{
    QueueNode cell = queue->array[queue->front];
    queue->front   = (queue->front + 1) % queue->capacity;
    queue->size    = queue->size - 1;
    return cell;
}

/**
 * @brief BFS to find the shortest path from start to goal
 * @param maze      pointer to the maze
 * @param startX    starting position x-coordinate
 * @param startY    starting position y-coordinate
 */
void
bfs_shortest_path(maze_t *maze, int startX, int startY)
{
    // Create a queue for BFS
    Queue *queue = createQueue(maze->height * maze->width);

    // Initialize visited array
    bool visited[maze->height][maze->width];
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            visited[i][j] = false;
        }
    }

    // Mark the starting cell as visited and enqueue it
    visited[startY][startX] = true;
    enqueue(queue, startX, startY);

    // Define directions (up, down, left, right)
    int dx[] = { -1, 1, 0, 0 };
    int dy[] = { 0, 0, -1, 1 };

    // Perform BFS
    while (!isEmpty(queue))
    {
        // Dequeue a cell and process it
        QueueNode current = dequeue(queue);
        int       x       = current.x;
        int       y       = current.y;

        maze->mazecells[y][x].type = 'P'; // 'P' for path

        // Explore adjacent cells
        for (int i = 0; i < 4; i++)
        {
            int newX = x + dx[i];
            int newY = y + dy[i];

            // Check if the new position is within the maze boundaries
            if (newX >= 0 && newX < maze->width && newY >= 0
                && newY < maze->height)
            {
                // Check if the cell is not a wall and hasn't been visited
                if (maze->mazecells[newY][newX].type != 'X'
                    && !visited[newY][newX])
                {
                    // Mark the new cell as visited and enqueue it
                    visited[newY][newX] = true;
                    enqueue(queue, newX, newY);
                }
            }
        }
    }

    // Free the allocated memory for the queue
    free(queue);
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
 * Create a hardcoded map with a clear path from start to goal
 * @param maze
 */
void
create_hardcoded_map(maze_t *maze)
{
    // Set fixed height and width during initialization
    maze->height = 5;
    maze->width  = 5;

    // Create the map based on the image
    char hardcoded_map[5][5] = { { ' ', ' ', ' ', ' ', ' ' },
                                 { 'S', 'X', 'X', 'X', ' ' },
                                 { ' ', 'X', ' ', ' ', ' ' },
                                 { ' ', 'X', ' ', 'X', ' ' },
                                 { ' ', ' ', ' ', 'X', 'G' } };

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

    printf("Here is the hardcoded map:\n");
    print_map(maze);
}

/**
 * @brief Mapping Initialization
 */
void
mapping_init(maze_t *p_maze)
{
    printf("Initializing mapping\n");

    // Set fixed height and width during initialization
    p_maze->height = 5;
    p_maze->width  = 5;

    // Create the maze
    printf("Creating maze\n");
    create_hardcoded_map(p_maze);
    printf("Maze created\n");
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
 * @brief Function to check if the entire map is filled
 * @param maze
 * @return true if the entire map is filled, false otherwise
 */
bool
maze_explored(const maze_t *maze)
{
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            if (maze->mazecells[j][i].type != 'X'
                && maze->mazecells[j][i].type != 'V'
                && maze->mazecells[j][i].type != 'C'
                && maze->mazecells[j][i].type != 'G'
                && maze->mazecells[j][i].type != 'S')
            {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief Function to find the shortest path from start to goal
 * @param maze
 */
void
find_shortest_path(maze_t *maze)
{
    // Assuming the starting point is the bottom-left corner (0, 0)
    int startX = 0;
    int startY = 0;

    // Perform BFS to find the shortest path
    bfs_shortest_path(maze, startX, startY);
}

/**
 * @brief Function to backtrack to the start from the goal iteratively
 * @param maze
 * @param currentX pointer to the current X position
 * @param currentY pointer to the current Y position
 */
void
backtrack_to_start(maze_t *maze, int *currentX, int *currentY)
{
    printf("Backtracking to the start...\n");

    // Continue backtracking until reaching the start
    while (*currentX != 0 || *currentY != 0)
    {
        printf("Backtracking...\n");
        print_map(maze);

        // Update the current cell as part of the backtracking path
        maze->mazecells[*currentX][*currentY].type = 'P'; // 'P' for path

        // Move the car towards the starting point
        if (*currentX > 0)
        {
            (*currentX)--;
        }
        else if (*currentY > 0)
        {
            (*currentY)--;
        }

        // Print the map after updating the current cell during backtracking
        printf("Map after updating current cell during backtracking:\n");
        print_map(maze);

        // Print the car's position in the map
        printf("Car's position during backtracking: (%d, %d)\n",
               *currentX,
               *currentY);

        vTaskDelay(
            pdMS_TO_TICKS(100)); // Delay to simulate time between movements
    }

    printf("Backtracking completed. Reached the start!\n");
}

/**
 * @brief Task to demonstrate the car following the shortest path from start to
 * goal
 * @param pvParameters
 */
void
demo_shortest_path_task(void *pvParameters)
{
    maze_t *maze = (maze_t *)pvParameters;

    // Assuming the starting point is the bottom-left corner (0, 0)
    int currentX = 0;
    int currentY = 0;

    // Find the shortest path using BFS
    bfs_shortest_path(maze, currentX, currentY);

    printf("Shortest path found. Demonstrating the car's movement...\n");

    // Iterate through the path and demonstrate the car's movement
    for (int i = maze->height - 1; i >= 0; i--)
    {
        for (int j = 0; j < maze->width; j++)
        {
            if (maze->mazecells[i][j].type == 'P')
            {
                // Move the car to the cell in the shortest path
                currentX = j;
                currentY = i;

                // Print the map with the car's position
                printf("Map with the car's position (BFS):\n");
                print_map(maze);

                // Delay to simulate the car's movement
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }

    printf("Car reached the goal following the shortest path!\n");

    vTaskDelete(NULL); // Delete the demonstration task
}

/**
 * @brief Task to perform mapping of the maze
 * @param pvParameters
 */
void
mapping_task(void *pvParameters)
{
    maze_t *maze     = (maze_t *)pvParameters;
    int     currentX = 0; // Initial X position
    int     currentY = 0; // Initial Y position

    // Reset maze before mapping
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            maze->mazecells[j][i].visited = 0;
        }
    }

    // Explore the maze and perform floodfill
    for (;;)
    {
        // Simulate car movement (you can replace this logic with your actual
        // movement algorithm)

        mapping_direction_t moveDirection
            = (mapping_direction_t)(get_rand_32() % 4);

        // Update the previously visited position before moving
        if (maze->mazecells[currentX][currentY].type != 'S'
            && maze->mazecells[currentX][currentY].type != 'G')
        {
            maze->mazecells[currentX][currentY].type = 'V'; // 'V' for visited
        }

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
        if (maze->mazecells[currentX][currentY].type != 'S')
        {
            maze->mazecells[currentX][currentY].type = 'C'; // 'C' for car
        }

        // Print the map with the car's position
        printf("Map with the car's position:\n");
        print_map(maze);

        // Floodfill the maze after each movement
        floodfill(maze, maze->width - 1, 0, 0);

        // Check if the car has explored the entire maze
        printf("%d\n", maze_explored(maze));
        if (maze_explored(maze))
        {
            printf("Entire maze explored!\n");

            // Continue with backtracking, BFS, and demonstration of the
            // shortest path
            printf("Now Backtracking...\n");
            backtrack_to_start(maze, &currentX, &currentY);

            printf("Map after backtracking:\n");
            print_map(maze);

            // Find the shortest path after backtracking
            printf("Finding the shortest path...\n");
            find_shortest_path(maze);

            // Create a task to demonstrate the shortest path
            xTaskCreate(demo_shortest_path_task,
                        "demo_shortest_path_task",
                        configMINIMAL_STACK_SIZE,
                        (void *)maze,
                        PRIO,
                        NULL);
        }

        vTaskDelay(
            pdMS_TO_TICKS(100)); // Delay to simulate time between movements
    }
}

/**
 * @brief Task to perform backtracking from the goal to the start
 * @param pvParameters
 */
void
backtracking_task(void *pvParameters)
{
    maze_t *maze = (maze_t *)pvParameters;

    int currentX = 0; // Initial X position
    int currentY = 0; // Initial Y position

    printf("Backtracking to the start...\n");
    backtrack_to_start(maze, &currentX, &currentY);

    printf("Map after backtracking:\n");
    print_map(maze);

    vTaskDelete(NULL); // Delete the backtracking task
}

/**
 * @brief Task to show the movement from start to goal
 * @param pvParameters
 */
void
movement_task(void *pvParameters)
{
    maze_t *maze = (maze_t *)pvParameters;

    int currentX = 0; // Initial X position
    int currentY = 0; // Initial Y position

    for (;;)
    {
        // Simulate car movement (you can replace this logic with your actual
        // movement algorithm)
        mapping_direction_t moveDirection
            = (mapping_direction_t)(get_rand_32() % 4);

        // Update the previously visited position before moving
        if (maze->mazecells[currentX][currentY].type != 'S'
            && maze->mazecells[currentX][currentY].type != 'G')
        {
            maze->mazecells[currentX][currentY].type = 'V'; // 'V' for visited
        }

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
        if (maze->mazecells[currentX][currentY].type != 'S')
        {
            maze->mazecells[currentX][currentY].type = 'C'; // 'C' for car
        }

        // Print the map with the car's position
        printf("Map with the car's position:\n");
        print_map(maze);

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
    // Task handles
    TaskHandle_t mapping_task_handle       = NULL;
    TaskHandle_t backtracking_task_handle  = NULL;
    TaskHandle_t demo_shortest_path_handle = NULL;
    TaskHandle_t movement_task_handle      = NULL;

    // Create tasks
    xTaskCreate(mapping_task,
                "mapping_task",
                configMINIMAL_STACK_SIZE,
                (void *)maze,
                PRIO,
                &mapping_task_handle);

    xTaskCreate(backtracking_task,
                "backtracking_task",
                configMINIMAL_STACK_SIZE,
                (void *)maze,
                PRIO,
                &backtracking_task_handle);

    // Shortest path task demo
    xTaskCreate(demo_shortest_path_task,
                "demo_shortest_path_task",
                configMINIMAL_STACK_SIZE,
                (void *)maze,
                PRIO,
                &demo_shortest_path_handle);

    xTaskCreate(movement_task,
                "movement_task",
                configMINIMAL_STACK_SIZE,
                (void *)maze,
                PRIO,
                &movement_task_handle);

    // Suspend all tasks except the mapping task
    vTaskSuspend(backtracking_task_handle);
    vTaskSuspend(demo_shortest_path_handle);
    vTaskSuspend(movement_task_handle);
}

#endif /* MAPPING_H */
