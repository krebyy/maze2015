/**
  ******************************************************************************
  * @file    ../src/main.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.1
  * @date    15-Julho-2015
  * @brief   Robot uMaRT Lite+ - Desafio Maze Escape 2015
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* External Variables --------------------------------------------------------*/
int32_t i32_bufParams[N_PARAMETERS];

/* Private Variables ---------------------------------------------------------*/
bool b_RunController = false;
uint32_t u32_ticks = 0;
uint8_t u8_walls = 0;


// #############################################################################

// ################################ MAIN () ####################################

/**
  * @brief Main Program
  */
int main (void)
{
	SelectType_t x_select = SEARCH_RUN, x_selectOld = STOP;
	uint8_t u8_position = 0;
	int32_t i32_move = 0;

	// Configuration and Initialization of Peripherals -------------------------
	configPeripherals();
	initParameters();

	// Waiting to Start and Verify UART Commands -------------------------------
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
	while (waitStart() == false) comandosUART();

	// First Cell Move ---------------------------------------------------------
	b_RunController = true;
	if (x_select == SEARCH_RUN)
	{
		updateMaze(0b101);
		getNextMove();
		checkPosition();
		frente(CELL_SIZE / 2);
	}


	// Main Loop ---------------------------------------------------------------
	while (1)
	{
		// Main State Machine --------------------------------------------------
		switch (x_select)
		{
		// Search run ----------------------------------------------------------
		case SEARCH_RUN:	//--------------------------------------------------
			// Update Walls of the Maze
			updateMaze(u8_walls);
			i32_move = getNextMove();
			u8_position = checkPosition();

			// Realize the Move
			doMove(i32_move);

			// Check Goal Cell
			if (u8_position == 1)
			{
				beep(100);
			}

			// Check Finish of Search Run
			else if (u8_position == 2)
			{
				curvaPivot(180);
				targetSpeedX = 0;
				finishSong();

				x_selectOld = x_select;
				x_select = WAIT;

				break;	// break of if
			}
			break;	// break of case SEARCH_RUN

		// Fast Run 1 ----------------------------------------------------------
		case FAST_RUN1:	//------------------------------------------------------
			fastRun(500, TURN_SPEED);

			x_selectOld = x_select;
			x_select = RETURN_TO_HOME;
			break;	// break of case FAST_RUN1

		// Fast Run 2 ----------------------------------------------------------
		case FAST_RUN2:	//------------------------------------------------------
			fastRun(700, TURN_SPEED);

			x_selectOld = x_select;
			x_select = RETURN_TO_HOME;
			break;	// break of case FAST_RUN2

		// Fast Run 3 ----------------------------------------------------------
		case FAST_RUN3:	//------------------------------------------------------
			fastRun(1000, TURN_SPEED);

			x_selectOld = x_select;
			x_select = RETURN_TO_HOME;
			break;	// break of case FAST_RUN3

		// Fast Run 4 ----------------------------------------------------------
		case FAST_RUN4:	//------------------------------------------------------
			fastRun(1500, TURN_SPEED);

			x_selectOld = x_select;
			x_select = RETURN_TO_HOME;
			break;	// break of case FAST_RUN4

		// Fast Run 5 ----------------------------------------------------------
		case FAST_RUN5:	//------------------------------------------------------
			fastRun(2000, TURN_SPEED);

			x_selectOld = x_select;
			x_select = RETURN_TO_HOME;
			break;	// break of case FAST_RUN5

		// Returns to the Start (without mapping) ------------------------------
		case RETURN_TO_HOME:	//----------------------------------------------
			// Always does after a fastRun
			i32_move = getNextMove();
			u8_position = checkPosition();

			doMove(i32_move);

			if (u8_position == 2)
			{
				allLEDs(LOW);

				curvaPivot(180);
				targetSpeedX = 0;
				finishSong();

				if (x_selectOld < FAST_RUN5) x_select = WAIT;
				else x_select = STOP;

				break;	// break of if
			}
			break;	// break of case RETURN_TO_HOME

		// Gyro Calibration ----------------------------------------------------
		case CAL_GYRO:	//------------------------------------------------------
			calGyro();
			x_select = STOP;
			break;	// break of case CAL_GYRO

		// Sensors Calibration -------------------------------------------------
		case CAL_SENSORS:	//--------------------------------------------------
			//moveStraight(180, 0);	// Usar para calibrar com bUseSensors = false
			moveOneCell(0);	// Usar para testar a correção longitudinal
			b_RunController = false;
			setMotores(0, 0);
			printBuffers();
			x_select = STOP;
			break;	// break of case CAL_SENSORS

		// PID Calibration -----------------------------------------------------
		case CAL_PID:	//------------------------------------------------------
			moveStraight(3 * 180, 0);	// Usar para calibrar com bUseSensors = false
			delay_ms(250);
			b_RunController = false;
			setMotores(0, 0);
			x_select = STOP;
			break;	// break of case CAL_PID

		// Turn 180 Calibration ------------------------------------------------
		case CAL_180:	//------------------------------------------------------
			//frontCal();
			getSensoresParede();
			curva(180);
			targetSpeedX = 0;
			x_select = STOP;
			break;	// break of case CAL_180

		// Shows Sensor Readings -----------------------------------------------
		case PRINT_SENSORS:	//--------------------------------------------------
			printf("%d\t%d\t%d\t%d\r\n",
					frontal_esquerdo, lateral_esquerdo, lateral_direito, frontal_direito);
			delay_ms(250);
			break;	// break of case PRINT_SENSORS

		// Tests Movements -----------------------------------------------------
		case TEST_MOVE:	//------------------------------------------------------
			// Simple Curve
			/*frente(CELL_SIZE);
			curva(-90);
			moveStraight(CELL_SIZE, 0);*/

			// Square
			/*frente(CELL_SIZE);
			curva(90);
			curva(90);
			curva(90);
			curva(90);
			moveStraight(180, 0);*/

			// Zig-Zag
			frente(CELL_SIZE);
			curva(-90); curva(90); curva(-90);
			curva(-90); curva(-90); curva(90); curva(-90);
			for (int i = 0; i < 3; i++)
			{
				curva(-90); curva(-90); curva(90); curva(-90);
				curva(-90); curva(-90); curva(90); curva(-90);
			}
			moveStraight(90, 0);
			pivot180();

			x_select = STOP;
			break;	// break of case TEST_MOVE

		// Waiting to Start ----------------------------------------------------
		case WAIT:	//----------------------------------------------------------
			x_select = x_selectOld + 1;	// Changes the fastRun
			setLED(x_select - 1, HIGH);	// Changes the LED indication

			// Waiting the front sensors
			while (waitStart() == false)
			{
				// Changes the fastRun - if the button is pressed
				if (getSW1() == HIGH)
				{
					if (x_select < FAST_RUN5)
					{
						x_selectOld = x_select;
						x_select++;
					}
					else
					{
						x_selectOld = SEARCH_RUN;
						x_select = FAST_RUN1;
					}

					allLEDs(LOW);
					setLED(x_select - 1, HIGH);
					delay_ms(500);
				}
			}
			break;	// break of case WAIT

		// Stop State ----------------------------------------------------------
		case STOP:	//----------------------------------------------------------
			delay_ms(1000);
			toggleLED(LED2);
			break;	// break of case STOP
		}	// end of switch

		delay_ms(1);
	}	// end of while
}	// end of main



// #############################################################################

// ############################## CONTROLLER ###################################

/**
  * @brief This function is called every 1ms for interruption
  */
void systick( void )
{
	u32_ticks++;	// Ticks counts

	if (b_RunController == true)
	{
		u8_walls = getSensoresParede();	// Read Sensors
		speedProfile();					// Speed Profiler
	}
}


// #############################################################################

// ########################## AUXILIARY FUNCTIONS ##############################

/**
  * @brief Configuration and Initialization of Peripherals
  */
void configPeripherals( void )
{
	configUsart1();
	configUSB();
	configBuzzer();
	configLEDs();
	configSW1();
	configMotors();
	configSensors();
	configEncoders();
}


/**
  * @brief Fast Run
  */
void fastRun(int32_t topSpeed, int32_t turnSpeed)
{
	getNextMove();

	uint8_t numMoves = getNumMoves();
	int16_t moves[numMoves];
	setSpeeds(topSpeed, turnSpeed);
	getPath(moves);
	doPath(moves, numMoves);
	setSpeeds(MOVE_SPEED, TURN_SPEED);
}


/**
  * @brief Initialization of parameters from FLASH
  */
void initParameters( void )
{
	readFlash(ADDR_FLASH_SECTOR_11, i32_bufParams, N_PARAMETERS);

#ifdef DEBUG_PRINTS
	printf("Parameters:\r\n");
	for (uint8_t i = 0; i < N_PARAMETERS; i++)
	{
		printf("[%ld]: %ld\r\n", i, i32_bufParams[i]);
	}
#endif
}


/**
  * @brief
  */
void recordPath( void )
{
	//writeFlash(ADDR_FLASH_SECTOR_10, buf_temp, 2 * SIZE_BUFFER_SECTORS);

#ifdef DEBUG_PRINTS
	int32_t buf[SIZE_BUFFER_SECTORS * 2];
	readFlash(ADDR_FLASH_SECTOR_10, buf, SIZE_BUFFER_SECTORS * 2);

	for (uint32_t i = 0; i < SIZE_BUFFER_SECTORS * 2; i++)
	{
		printf("[%ld]: %ld\r\n", i, buf[i]);
	}
#endif
}


/**
  * @brief Gyro Calibration
  */
void calGyro( void )
{
	for (int16_t speedW = 0; speedW <= 1000; speedW += 100)
	{
		int32_t acumulator_aSpeed = 0, numSamplesGyro = 0;

		targetSpeedW = SPEED_TO_COUNTS(speedW);
		delay_ms(500);
		u32_ticks = 0;
		while (u32_ticks < 1000)
		{
			acumulator_aSpeed += getRawGyro();
			numSamplesGyro++;
			delay_ms(1);
		}
		printf("Gyro[%d] = %ld\r\n", speedW, acumulator_aSpeed/numSamplesGyro);
	}

	printf("\r\n");

	for (int16_t speedW = 0; speedW >= -1000; speedW -= 100)
	{
		int32_t acumulator_aSpeed = 0, numSamplesGyro = 0;

		targetSpeedW = SPEED_TO_COUNTS(speedW);
		delay_ms(500);
		u32_ticks = 0;
		while (u32_ticks < 1000)
		{
			acumulator_aSpeed += getRawGyro();
			numSamplesGyro++;
			delay_ms(1);
		}
		printf("Gyro[%d] = %ld\r\n", speedW, acumulator_aSpeed/numSamplesGyro);
	}

	targetSpeedW = 0;
}
