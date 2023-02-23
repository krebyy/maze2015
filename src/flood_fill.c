/* -*- flood_fill.c -*- ***************************************************** *
 *
 *   Created on:  13/06/2015
 *   Author:      Frederico Fernandes de Oliveira
 *
 * ************************************************************************** */


#include "flood_fill.h"

/******************************************************************************
**                    GLOBAL INTERNAL VARIABLE DEFINITIONS
******************************************************************************/
struct cell
{
    int16_t dist;
    bool visited;
    bool up_wall;
    bool right_wall;
    bool down_wall;
    bool left_wall;
};
struct cell maze[MAZE_SIZE][MAZE_SIZE];
uint8_t orientation = NORTH;
uint8_t currentGoal_i = GOAL_I;
uint8_t currentGoal_j = GOAL_J;
uint8_t currentCell_i = START_I;
uint8_t currentCell_j = START_J;

/******************************************************************************
**                     INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
void floodFill(void);
void resetMaze(void);
int16_t getLowestNeighbourCell(uint8_t i, uint8_t j);
uint8_t getNextCell(void);
void checkNorthCell(uint8_t *nextCell);
void checkEastCell(uint8_t *nextCell);
void checkSouthCell(uint8_t *nextCell);
void checkWestCell(uint8_t *nextCell);

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/
void floodFill(void)
{
    resetMaze();

    while (maze[currentCell_i][currentCell_j].dist == -1)
    {
        for (uint8_t i = 0; i < MAZE_SIZE && maze[currentCell_i][currentCell_j].dist == -1; i++)
        {
            for (uint8_t j = 0; j < MAZE_SIZE; j++)
            {
                if (maze[i][j].dist == -1)
                {
                    maze[i][j].dist = getLowestNeighbourCell(i, j) + 1;
                }
            }
        }
    }
}

void resetMaze(void)
{
    for (uint8_t i = 0; i < MAZE_SIZE; i++)
    {
        for (uint8_t j = 0; j < MAZE_SIZE; j++)
        {
            maze[i][j].dist = -1;
        }
    }

    maze[currentGoal_i][currentGoal_j].dist = 0;
}

int16_t getLowestNeighbourCell(uint8_t i, uint8_t j)
{
    int16_t lowestNeighbourCell = -2;

    /* if there's a traversable up cell and its has already been checked */
    if (i - 1 >= 0 && maze[i][j].up_wall == 0 && maze[i - 1][j].dist != -1)
    {
        lowestNeighbourCell = maze[i - 1][j].dist;
    }

    /* if there's a traversable right cell and its has already been checked */
    if (j + 1 < MAZE_SIZE && maze[i][j].right_wall == 0 && maze[i][j + 1].dist != -1)
    {
        if (lowestNeighbourCell == -2 || maze[i][j + 1].dist < lowestNeighbourCell)
        {
            lowestNeighbourCell = maze[i][j + 1].dist;
        }
    }

    /* if there's a traversable down cell and its has already been checked */
    if (i + 1 < MAZE_SIZE && maze[i][j].down_wall == 0 && maze[i + 1][j].dist != -1)
    {
        if (lowestNeighbourCell == -2 || maze[i + 1][j].dist < lowestNeighbourCell)
        {
            lowestNeighbourCell = maze[i + 1][j].dist;
        }
    }

    /* if there's a traversable left cell and its has already been checked */
    if (j - 1 >= 0 && maze[i][j].left_wall == 0 && maze[i][j - 1].dist != -1)
    {
        if (lowestNeighbourCell == -2 || maze[i][j - 1].dist < lowestNeighbourCell)
        {
            lowestNeighbourCell = maze[i][j - 1].dist;
        }
    }

    return lowestNeighbourCell;
}

uint8_t getNextCell(void)
{
    uint8_t nextCell = 0;

    switch (orientation)
    {
        case NORTH:
            checkEastCell(&nextCell);
            checkSouthCell(&nextCell);
            checkWestCell(&nextCell);
            checkNorthCell(&nextCell);
            break;

        case EAST:
            checkSouthCell(&nextCell);
            checkWestCell(&nextCell);
            checkNorthCell(&nextCell);
            checkEastCell(&nextCell);
            break;

        case SOUTH:
            checkWestCell(&nextCell);
            checkNorthCell(&nextCell);
            checkEastCell(&nextCell);
            checkSouthCell(&nextCell);
            break;

        case WEST:
            checkNorthCell(&nextCell);
            checkEastCell(&nextCell);
            checkSouthCell(&nextCell);
            checkWestCell(&nextCell);
            break;
    }

    return nextCell;
}

void checkNorthCell(uint8_t *nextCell)
{
    /* if there's a traversable up cell and it's the next one */
    if (currentCell_i - 1 >= 0 && maze[currentCell_i][currentCell_j].up_wall == 0 &&
        maze[currentCell_i - 1][currentCell_j].dist == maze[currentCell_i][currentCell_j].dist - 1)
    {
        if (*nextCell == 0 || maze[currentCell_i - 1][currentCell_j].visited == false)
        {
            *nextCell = NORTH;
        }
    }
}

void checkEastCell(uint8_t *nextCell)
{
    /* if there's a traversable right cell and it's the next one */
    if (currentCell_j + 1 < MAZE_SIZE && maze[currentCell_i][currentCell_j].right_wall == 0 &&
        maze[currentCell_i][currentCell_j + 1].dist == maze[currentCell_i][currentCell_j].dist - 1)
    {
        /* gives preference to an unvisited cell */
        if (*nextCell == 0 || maze[currentCell_i][currentCell_j + 1].visited == false)
        {
            *nextCell = EAST;
        }
    }
}

void checkSouthCell(uint8_t *nextCell)
{
    /* if there's a traversable down cell and it's the next one */
    if (currentCell_i + 1 < MAZE_SIZE && maze[currentCell_i][currentCell_j].down_wall == 0 &&
        maze[currentCell_i + 1][currentCell_j].dist == maze[currentCell_i][currentCell_j].dist - 1)
    {
        /* gives preference to an unvisited cell */
        if (*nextCell == 0 || maze[currentCell_i + 1][currentCell_j].visited == false)
        {
            *nextCell = SOUTH;
        }
    }
}

void checkWestCell(uint8_t *nextCell)
{
    /* if there's a traversable left cell and it's the next one */
    if (currentCell_j - 1 >= 0 && maze[currentCell_i][currentCell_j].left_wall == 0 &&
        maze[currentCell_i][currentCell_j - 1].dist == maze[currentCell_i][currentCell_j].dist - 1)
    {
        /* gives preference to an unvisited cell */
        if (*nextCell == 0 || maze[currentCell_i][currentCell_j - 1].visited == false)
        {
            *nextCell = WEST;
        }
    }
}

int16_t getNextMove(void)
{
    floodFill();
    int16_t nextMove = -1;
    uint8_t currentOrientation = orientation;
    uint8_t nextCell = getNextCell();

    /* if there's a traversable up cell and it's the next one */
    if (nextCell == NORTH)
    {
        currentCell_i--;
        orientation = NORTH;

        if (currentOrientation == NORTH)
        {
            nextMove = 0;
        }
        else if (currentOrientation == EAST)
        {
            nextMove = -90;
        }
        else if (currentOrientation == SOUTH)
        {
            nextMove = 180;
        }
        else if (currentOrientation == WEST)
        {
            nextMove = 90;
        }
    }

    /* if there's a traversable right cell and it's the next one */
    else if (nextCell == EAST)
    {
        currentCell_j++;
        orientation = EAST;

        if (currentOrientation == NORTH)
        {
            nextMove = 90;
        }
        else if (currentOrientation == EAST)
        {
            nextMove = 0;
        }
        else if (currentOrientation == SOUTH)
        {
            nextMove = -90;
        }
        else if (currentOrientation == WEST)
        {
            nextMove = 180;
        }
    }

    /* if there's a traversable down cell and it's the next one */
    else if (nextCell == SOUTH)
    {
        currentCell_i++;
        orientation = SOUTH;

        if (currentOrientation == NORTH)
        {
            nextMove = 180;
        }
        else if (currentOrientation == EAST)
        {
            nextMove = 90;
        }
        else if (currentOrientation == SOUTH)
        {
            nextMove = 0;
        }
        else if (currentOrientation == WEST)
        {
            nextMove = -90;
        }
    }

    /* if there's a traversable left cell and it's the next one */
    else if (nextCell == WEST)
    {
        currentCell_j--;
        orientation = WEST;

        if (currentOrientation == NORTH)
        {
            nextMove = -90;
        }
        else if (currentOrientation == EAST)
        {
            nextMove = 180;
        }
        else if (currentOrientation == SOUTH)
        {
            nextMove = 90;
        }
        else if (currentOrientation == WEST)
        {
            nextMove = 0;
        }
    }

    checkPosition();
    return nextMove;
}

int16_t getNumMoves(void)
{
    if (maze[currentCell_i][currentCell_j].dist == -1)
    {
        floodFill();
    }

    return maze[currentCell_i][currentCell_j].dist;
}

void getPath(int16_t path[])
{
    uint8_t numMoves = getNumMoves();

    for (uint8_t i = 0; i < numMoves; i++)
    {
        path[i] = getNextMove();
        floodFill();
    }

    checkPosition();
}

void updateMaze(uint8_t walls)
{
    if (orientation == NORTH)
    {
        /* updates the current cell wall status */
        maze[currentCell_i][currentCell_j].left_wall = walls & (1 << 2);
        maze[currentCell_i][currentCell_j].up_wall = walls & (1 << 1);
        maze[currentCell_i][currentCell_j].right_wall = walls & 1;

        /* updates right wall status of the current cell's left neighbour, if it exists */
        if (currentCell_j - 1 >= 0)
        {
            maze[currentCell_i][currentCell_j - 1].right_wall = maze[currentCell_i][currentCell_j].left_wall;
        }

        /* updates down wall status of the current cell's up neighbour, if it exists */
        if (currentCell_i - 1 >= 0)
        {
            maze[currentCell_i - 1][currentCell_j].down_wall = maze[currentCell_i][currentCell_j].up_wall;
        }

        /* updates left wall status of the current cell's right neighbour, if it exists */
        if (currentCell_j + 1 < MAZE_SIZE)
        {
            maze[currentCell_i][currentCell_j + 1].left_wall = maze[currentCell_i][currentCell_j].right_wall;
        }
    }

    else if (orientation == EAST)
    {
        /* updates the current cell wall status */
        maze[currentCell_i][currentCell_j].up_wall = walls & (1 << 2);
        maze[currentCell_i][currentCell_j].right_wall = walls & (1 << 1);
        maze[currentCell_i][currentCell_j].down_wall = walls & 1;

        /* updates down wall status of the current cell's up neighbour, if it exists */
        if (currentCell_i - 1 >= 0)
        {
            maze[currentCell_i - 1][currentCell_j].down_wall = maze[currentCell_i][currentCell_j].up_wall;
        }

        /* updates left wall status of the current cell's right neighbour, if it exists */
        if (currentCell_j + 1 < MAZE_SIZE)
        {
            maze[currentCell_i][currentCell_j + 1].left_wall = maze[currentCell_i][currentCell_j].right_wall;
        }

        /* updates up wall status of the current cell's down neighbour, if it exists */
        if (currentCell_i + 1 < MAZE_SIZE)
        {
            maze[currentCell_i + 1][currentCell_j].up_wall = maze[currentCell_i][currentCell_j].down_wall;
        }
    }

    else if (orientation == SOUTH)
    {
        /* updates the current cell wall status */
        maze[currentCell_i][currentCell_j].right_wall = walls & (1 << 2);
        maze[currentCell_i][currentCell_j].down_wall = walls & (1 << 1);
        maze[currentCell_i][currentCell_j].left_wall = walls & 1;

        /* updates left wall status of the current cell's right neighbour, if it exists */
        if (currentCell_j + 1 < MAZE_SIZE)
        {
            maze[currentCell_i][currentCell_j + 1].left_wall = maze[currentCell_i][currentCell_j].right_wall;
        }

        /* updates up wall status of the current cell's down neighbour, if it exists */
        if (currentCell_i + 1 < MAZE_SIZE)
        {
            maze[currentCell_i + 1][currentCell_j].up_wall = maze[currentCell_i][currentCell_j].down_wall;
        }

        /* updates right wall status of the current cell's left neighbour, if it exists */
        if (currentCell_j - 1 >= 0)
        {
            maze[currentCell_i][currentCell_j - 1].right_wall = maze[currentCell_i][currentCell_j].left_wall;
        }
    }

    else if (orientation == WEST)
    {
        /* updates the current cell wall status */
        maze[currentCell_i][currentCell_j].down_wall = walls & (1 << 2);
        maze[currentCell_i][currentCell_j].left_wall = walls & (1 << 1);
        maze[currentCell_i][currentCell_j].up_wall = walls & 1;

        /* updates up wall status of the current cell's down neighbour, if it exists */
        if (currentCell_i + 1 < MAZE_SIZE)
        {
            maze[currentCell_i + 1][currentCell_j].up_wall = maze[currentCell_i][currentCell_j].down_wall;
        }

        /* updates right wall status of the current cell's left neighbour, if it exists */
        if (currentCell_j - 1 >= 0)
        {
            maze[currentCell_i][currentCell_j - 1].right_wall = maze[currentCell_i][currentCell_j].left_wall;
        }

        /* updates down wall status of the current cell's up neighbour, if it exists */
        if (currentCell_i - 1 >= 0)
        {
            maze[currentCell_i - 1][currentCell_j].down_wall = maze[currentCell_i][currentCell_j].up_wall;
        }
    }
}

/**
 * This function must be called after every move to check if it's necessary
 * to invert the start position with the goal position
 */

uint8_t checkPosition(void)
{
    /* if the robot has reached its destination, its goal */
    /* is inverted so it does the way around */
    if (currentCell_i == GOAL_I && currentCell_j == GOAL_J)
    {
        currentGoal_i = START_I;
        currentGoal_j = START_J;
        return 1;
    }
    else if (currentCell_i == START_I && currentCell_j == START_J)
    {
        currentGoal_i = GOAL_I;
        currentGoal_j = GOAL_J;
        return 2;
    }

    return 0;
}
