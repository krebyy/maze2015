/* -*- flood_fill.h -*- ****************************************************** *
 *
 *   Created on:  13/06/2015
 *   Author:      2015/1
 *
 * ************************************************************************** */

#ifndef FLOOD_FILL_H_
#define FLOOD_FILL_H_


#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
**                     EXTERNAL GLOBAL MACRO DEFINITIONS
******************************************************************************/
#define MAZE_SIZE   16
#define NORTH       1
#define EAST        2
#define SOUTH       3
#define WEST        4
#define GOAL_I      8//0
#define GOAL_J      8//MAZE_SIZE - 1
#define START_I     MAZE_SIZE - 1
#define START_J     0


/******************************************************************************
**                      EXTERNAL GLOBAL FUNCTION PROTOTYPES
******************************************************************************/
int16_t getNextMove(void);
void getPath(int16_t path[]);
uint8_t checkPosition(void);
int16_t getNumMoves(void);
void updateMaze(uint8_t walls);
void printRightWallStatus(void);


#endif /* FLOOD_FILL_H_ */
