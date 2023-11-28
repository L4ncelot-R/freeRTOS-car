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

// Define a queue structure for BFS
typedef struct {
    int x;
    int y;
} QueueNode;

typedef struct {
    QueueNode *array;
    int front, rear, size;
    unsigned capacity;
} Queue;

// Function to create a new queue
Queue* createQueue(unsigned capacity) {
    Queue* queue = (Queue*)malloc(sizeof(Queue));
    queue->capacity = capacity;
    queue->front = queue->size = 0;
    queue->rear = capacity - 1;
    queue->array = (QueueNode*)malloc(capacity * sizeof(QueueNode));
    return queue;
}

// Function to check if the queue is empty
bool isEmpty(Queue* queue) {
    return (queue->size == 0);
}

// Function to check if the queue is full
bool isFull(Queue* queue) {
    return (queue->size == queue->capacity);
}

// Function to enqueue a cell in the queue
void enqueue(Queue* queue, int x, int y) {
    if (isFull(queue))
        return;
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->array[queue->rear].x = x;
    queue->array[queue->rear].y = y;
    queue->size = queue->size + 1;
}

// Function to dequeue a cell from the queue
QueueNode dequeue(Queue* queue) {
    QueueNode cell = queue->array[queue->front];
    queue->front = (queue->front + 1) % queue->capacity;
    queue->size = queue->size - 1;
    return cell;
}

// Function to perform BFS and find the shortest path
void bfs_shortest_path(maze_t *maze, int startX, int startY) {
    // Create a queue for BFS
    Queue* queue = createQueue(maze->height * maze->width);

    // Initialize visited array
    bool visited[maze->height][maze->width];
    for (int i = 0; i < maze->height; i++) {
        for (int j = 0; j < maze->width; j++) {
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
    while (!isEmpty(queue)) {
        // Dequeue a cell and process it
        QueueNode current = dequeue(queue);
        int x = current.x;
        int y = current.y;

        // Process the cell (you can customize this part based on your needs)
        // Here, we mark the cell with a special character to indicate it's part of the shortest path
        maze->mazecells[y][x].type = 'P'; // 'P' for path

        // Explore adjacent cells
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];

            // Check if the new position is within the maze boundaries
            if (newX >= 0 && newX < maze->width && newY >= 0 && newY < maze->height) {
                // Check if the cell is not a wall and hasn't been visited
                if (maze->mazecells[newY][newX].type != 'X' && !visited[newY][newX]) {
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
 * @brief Function to check if the entire maze has been explored
 * @param maze
 * @return true if all cells are visited, false otherwise
 */
bool
maze_explored(const maze_t *maze)
{
    for (int i = 0; i < maze->height; i++)
    {
        for (int j = 0; j < maze->width; j++)
        {
            if (maze->mazecells[j][i].type != 'X'
                && maze->mazecells[j][i].type != 'V')
            {
                return false;
            }
        }
    }
    return true;
}

// Update the find_shortest_path function with the newly created bfs_shortest_path function
void find_shortest_path(maze_t *maze) {
    // Assuming the starting point is the bottom-left corner (0, 0)
    int startX = 0;
    int startY = 0;

    // Perform BFS to find the shortest path
    bfs_shortest_path(maze, startX, startY);
}

void
backtrack_to_start(maze_t *maze, int *currentX, int *currentY)
{
    // Get the current cell type
    char currentCellType = maze->mazecells[*currentX][*currentY].type;

    // Base case: Stop if the current cell is the start
    if (currentCellType == 'S')
    {
        printf("Backtracking completed. Reached the start!\n");
        return;
    }

    // Update the current cell as part of the backtracking path
    maze->mazecells[*currentX][*currentY].type = 'P'; // 'P' for path

    // Initialize newX and newY
    int newX = *currentX;
    int newY = *currentY;

    // Explore adjacent cells in all directions
    for (int i = 0; i < 4; i++)
    {
        // Adjust the new position based on the movement direction
        switch ((mapping_direction_t)i)
        {
            case up:
                newY++;
                break;
            case down:
                newY--;
                break;
            case left:
                newX--;
                break;
            case right:
                newX++;
                break;
        }

        // Check if the new position is within the maze boundaries
        if (newX >= 0 && newX < maze->width && newY >= 0 && newY < maze->height)
        {
            // Check if the new cell is part of the backtracking path
            if (maze->mazecells[newX][newY].type == 'V'
                || maze->mazecells[newX][newY].type == 'P')
            {
                // Move to the new position
                *currentX = newX;
                *currentY = newY;

                // Recursively backtrack from the new position
                backtrack_to_start(maze, currentX, currentY);

                // If backtracking is successful, stop exploring other
                // directions
                return;
            }
        }

        // Reset newX and newY to the original values
        newX = *currentX;
        newY = *currentY;
    }

    // If no valid adjacent cells are found, backtrack to the previous position
    switch (currentCellType)
    {
        case 'C':
            maze->mazecells[*currentX][*currentY].type
                = ' '; // Clear the car's position
            break;
        default:
            maze->mazecells[*currentX][*currentY].type
                = 'V'; // Mark as visited during backtracking
            break;
    }

    // Print the map during backtracking
    printf("Map during backtracking:\n");
    print_map(maze);

    // Move back to the previous position (if not at the start)
    if (currentCellType != 'S')
    {
        // Update the current position to the previous position
        *currentX = newX;
        *currentY = newY;
    }

    // Print the map after moving back during backtracking
    printf("Map after moving back during backtracking:\n");
    print_map(maze);
}

/**
 * @brief Task to explore the maze, find the shortest path, and reach the goal
 * @param pvParameters
 */
void
combined_task(void *pvParameters)
{
    maze_t *maze     = (maze_t *)pvParameters;
    int     currentX = 0; // Initial X position
    int     currentY = 0; // Initial Y position

    // Reset maze before floodfill
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
        if (maze->mazecells[currentX][currentY].type
            != 'S') // Check if it's not the start position
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
        if (maze->mazecells[currentX][currentY].type
            != 'S') // Check if it's not the start position
        {
            maze->mazecells[currentX][currentY].type = 'C'; // 'C' for car
        }

        // Print the map with the car's position
        printf("Map with the car's position:\n");
        print_map(maze);

        // Floodfill the maze after each movement
        floodfill(maze, maze->width - 1, 0, 0);

        // Check if the car has explored the entire maze
        if (maze_explored(maze))
        {
            printf("Entire maze explored! Now finding the shortest path.\n");

            backtrack_to_start(maze, &currentX, &currentY);

            find_shortest_path(maze);
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
