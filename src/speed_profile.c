/**
  ******************************************************************************
  * @file    ../src/speed_profile.c
  * @author  Kleber Lima da Silva (kleber.ufu@hotmail.com)
  * @version V1.0.0
  * @date    15-Julho-2015
  * @brief   Funções para controle de velocidade dos motores
  ******************************************************************************
  */

#include "speed_profile.h"

/* External Variables --------------------------------------------------------*/
int32_t targetSpeedX = 0, targetSpeedW = 0;
float accX = 1, decX = 1, accW = 8, decW = 8;	// TODO: implementar macro
TIM_HandleTypeDef htim3;


/* Private Variables ---------------------------------------------------------*/
int32_t leftEncoderChange = 0, rightEncoderChange = 0;
int32_t encoderChange = 0, encoderCount = 0;
int32_t oldEncoderCount = 0;
int32_t leftEncoderOld = 0, rightEncoderOld = 0;
int32_t leftEncoderCount = 0, rightEncoderCount = 0;
int32_t distance = 0, distanceLeft = ONE_CELL_DISTANCE;
float angle = 0, angleLeft = 0, rotationalChange = 0;
float rotationalFeedback = 0;
int32_t encoderFeedbackW = 0;

float oldPosErrorX = 0, posErrorX = 0;
float oldPosErrorW = 0, posErrorW = 0;
int32_t posPwmX = 0, posPwmW = 0;

float curSpeedX = 0, curSpeedW = 0;
float endSpeedX = 0, endSpeedW = 0;

bool bUseSensors = false;
bool bUseGyro = false;
bool bUseEncoders = true;

int16_t buf_l_sensor[MAX_BUF_SIZE] = {0}, buf_r_sensor[MAX_BUF_SIZE] = {0};
uint16_t index_buffers = 0;

int32_t bufferLFT[201];
uint8_t index_buffer_lft = 1;

int32_t speed_move = MOVE_SPEED, speed_turn = TURN_SPEED;

#define LFT_PRINTS	// Habilita o envio dos valores para o software LFTrakking


void speedProfile(void)
{
	getEncoderStatus();
	updateCurrentSpeed();
	calculateMotorPwm();

	//putBuffers();	// FIXME : verificar se CAL_SENSORS
	//writeLFT();	// FIXME : verificar se CAL_PID
}


void getEncoderStatus(void)
{
	int32_t leftEncoder, rightEncoder;

	leftEncoder = READ_LEFT_ENCODER;
	rightEncoder = READ_RIGHT_ENCODER;

	leftEncoderChange = leftEncoder - leftEncoderOld;
	rightEncoderChange = rightEncoder - rightEncoderOld;
	encoderChange = (rightEncoderChange + leftEncoderChange) / 2;
	//rotationalChange = (rightEncoderChange - leftEncoderChange) / 2;
	rotationalChange = getGyro() / GYRO_RATIO;	// mm/s to counts/s  // Aumetar RATIO = aumentar ângulo real

	leftEncoderOld = leftEncoder;
	rightEncoderOld = rightEncoder;

	leftEncoderCount += leftEncoderChange;
	rightEncoderCount += rightEncoderChange;
	encoderCount = (rightEncoderCount + leftEncoderCount) / 2;

	distance += encoderChange;
	distanceLeft -= encoderChange;// update distanceLeft

	angle += rotationalChange;
	angleLeft -= rotationalChange;
}


void updateCurrentSpeed(void)
{
	if(curSpeedX < targetSpeedX)
	{
		curSpeedX += accX;
		if(curSpeedX > targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	else if(curSpeedX > targetSpeedX)
	{
		curSpeedX -= decX;
		if(curSpeedX < targetSpeedX)
			curSpeedX = targetSpeedX;
	}
	if(curSpeedW < targetSpeedW)
	{
		curSpeedW += accW;
		if(curSpeedW > targetSpeedW)
			curSpeedW = targetSpeedW;
	}
	else if(curSpeedW > targetSpeedW)
	{
		curSpeedW -= decW;
		if(curSpeedW < targetSpeedW)
			curSpeedW = targetSpeedW;
	}
}


void calculateMotorPwm(void) // encoder PD controller
{
	float sensorFeedback;
	int32_t encoderFeedbackX;

    // Simple PD loop to generate base speed for both motors
	encoderFeedbackX = rightEncoderChange + leftEncoderChange;
	encoderFeedbackW = rightEncoderChange - leftEncoderChange;

	rotationalFeedback = 0;
	if (bUseEncoders == true) rotationalFeedback += encoderFeedbackW;
	if (bUseSensors == true)
	{
		// Have sensor error properly scale to fit the system
		sensorFeedback = (float)(getErroSensores() / SENSOR_SCALE);
		rotationalFeedback += sensorFeedback;
	}
	if (bUseGyro == true) rotationalFeedback += (rotationalChange * 2.0);


	posErrorX += curSpeedX - encoderFeedbackX;
	posErrorW += curSpeedW - rotationalFeedback;

	posPwmX = (KP_X * posErrorX) + (KD_X * (posErrorX - oldPosErrorX));
	posPwmW = (KP_W * posErrorW) + (KD_W * (posErrorW - oldPosErrorW));

	oldPosErrorX = posErrorX;
	oldPosErrorW = posErrorW;

	setMotores(posPwmX - posPwmW, posPwmX + posPwmW);
}


//speed are in encoder counts/ms, dist is in encoder counts
float needToDecelerate(float dist, float curSpd, float endSpd)
{
	if (curSpd < 0) curSpd = -curSpd;
	if (endSpd < 0) endSpd = -endSpd;
	if (dist < 0) dist = -dist;
	if (dist == 0) return 9800;	//dist = 1;  //prevent divide by 0

	// calculate deceleration rate needed with input distance, input current
	// speed and input targetSpeed to determine if the deceleration is needed
	// use equation 2*a*S = Vt^2 - V0^2  ==>  a = (Vt^2-V0^2)/(2*S)
	float a = (curSpd*curSpd - endSpd*endSpd) / (4.0 * dist);
	//if (a < 0) a = -a;
	return fabsf(a);//COUNTS_TO_SPEED(a);
}


//resetEverything;
void resetSpeedProfile(void)
{
	//disable sensor data collecting functions running in 1ms interrupt
 	//bUseSensors = false;

	//setMotores(0, 0);

	curSpeedX = 0;
	curSpeedW = 0;
	//targetSpeedX = 0;
	//targetSpeedW = 0;
	posErrorX = 0;
	posErrorW = 0;
	oldPosErrorX = 0;
	oldPosErrorW = 0;
    leftEncoderOld = 0;
	rightEncoderOld = 0;
	leftEncoderCount = 0;
	rightEncoderCount = 0;
	encoderCount = 0;
	oldEncoderCount = 0;

	RESET_LEFT_ENCODER;
	RESET_RIGHT_ENCODER;
}


void moveOneCell(int32_t endSpd)
{
	bool hasLefttWall = false, hasRightWall = false;

	targetSpeedW = 0;
	targetSpeedX = SPEED_TO_COUNTS(speed_move);
	distanceLeft = ONE_CELL_DISTANCE;

	bUseEncoders = true;
	bUseSensors = true;
	bUseGyro = false;

	do
	{
		/* DECELERATION OF PROFILER */
		if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
		{
			targetSpeedX = SPEED_TO_COUNTS(endSpd);
		}
		else
		{
			targetSpeedX = SPEED_TO_COUNTS(speed_move);
		}


		/* CALIBRAÇÃO LONGITUDINAL */
		if (lateral_esquerdo > LATERAL_TH)
		{
			hasLefttWall = true;
		}
		if (lateral_direito > LATERAL_TH)
		{
			hasRightWall = true;
		}
		if (lateral_esquerdo < LATERAL_FADING_OFF && hasLefttWall == true)
		{
			hasLefttWall = false;
			distance = CORRECTION_CELL_DISTANCE;
			distanceLeft = ONE_CELL_DISTANCE - distance;
		}
		if (lateral_direito < LATERAL_FADING_OFF && hasRightWall == true)
		{
			hasRightWall = false;
			distance = CORRECTION_CELL_DISTANCE;
			distanceLeft = ONE_CELL_DISTANCE - distance;
		}


		/* STOP ALIGNED ON DEAD END */
		if (frontal_esquerdo > CENTRO_FRONTAL && frontal_direito > CENTRO_FRONTAL)
		{
			setMotores(0, 0);
			break;
		}

		delay_us(10);
	}
	while (distance < ONE_CELL_DISTANCE);

	//update here for next movement to minimized the counts loss between cells.
	angle = 0;
	distance = 0;
}


void moveStraight(int32_t dist, int32_t endSpd)
{
	targetSpeedW = 0;
	targetSpeedX = SPEED_TO_COUNTS(speed_move);
	endSpeedX = SPEED_TO_COUNTS(endSpd);
	distanceLeft = MM_TO_COUNTS(dist);

	bUseEncoders = true;
	bUseSensors = true;
	bUseGyro = false;

	do
	{
		/* DECELERATION OF PROFILER */
		if (needToDecelerate(distanceLeft, curSpeedX, endSpeedX) > decX)
		{
			targetSpeedX = SPEED_TO_COUNTS(endSpd);
			//beep(50);
		}
		/*else
		{
			targetSpeedX = SPEED_TO_COUNTS(MOVE_SPEED);
		}*/

		/* STOP ALIGNED ON DEAD END */
		if (frontal_esquerdo > ALINHAMENTO_FRONTAL && frontal_direito > ALINHAMENTO_FRONTAL)
		{
			targetSpeedX = 0;
		}

		if (frontal_esquerdo > CENTRO_FRONTAL && frontal_direito > CENTRO_FRONTAL)
		{
			setMotores(0, 0);
			resetSpeedProfile();
			break;
		}

		delay_us(10);
	}
	while (distance < MM_TO_COUNTS(dist));

	//update here for next movement to minimized the counts loss between cells.
	angle = 0;
	distance = 0;
}


void turnCorrection(int16_t dist, int16_t deg)
{
	bool hasFrontWall = false;

	targetSpeedW = 0;
	targetSpeedX = SPEED_TO_COUNTS(speed_turn);
	endSpeedX = SPEED_TO_COUNTS(speed_turn);
	distanceLeft = MM_TO_COUNTS(dist);

	bUseEncoders = true;
	bUseSensors = false;
	bUseGyro = false;

	if (frontal_esquerdo > FRONTAL_TH || frontal_direito > FRONTAL_TH) hasFrontWall = true;
	if (dist != OFFSET_BEFORE_TURN) hasFrontWall = false;

	do
	{
		if (dist == OFFSET_BEFORE_TURN)
		{
			/* LONGITUDINAL CALIBRATION TO START TURN  */
			if (deg > 0)	// Left
			{
				if (frontal_esquerdo > CURVA_FRONTAL) break;
			}

			else	// Right
			{
				if (frontal_direito > CURVA_FRONTAL) break;
			}
		}

		delay_us(10);
	}
	while ((distance < MM_TO_COUNTS(dist)) || (hasFrontWall == true));

	//update here for next movement to minimized the counts loss between cells.
	angle = 0;
	distance = 0;
}


void curveTurn(int16_t deg)
{
	int32_t curve_distance = abs(DEG_TO_COUNTS(deg));

	turnCorrection(OFFSET_BEFORE_TURN, deg);
	//setLED(LED5, HIGH);

	angleLeft = curve_distance;

	if (deg > 0)
	{
		targetSpeedW = SPEED_TO_COUNTS(SPEED_W);//RAD_S_TO_COUNTS(3.3333);

		do
		{
			/* DECELERATION OF PROFILER */
			if (needToDecelerate((float)curve_distance - angle, curSpeedW, endSpeedW) > decW)
			{
				targetSpeedW = 0;
			}

			delay_us(10);
		}
		while (angle < curve_distance);//(!(targetSpeedW == 0 && curSpeedW == 0));//(angle < curve_distance);
	}
	else
	{
		targetSpeedW = -SPEED_TO_COUNTS(SPEED_W);//-RAD_S_TO_COUNTS(3.3333);

		do
		{
			/* DECELERATION OF PROFILER */
			if (needToDecelerate((float)curve_distance + angle, curSpeedW, endSpeedW) > decW)
			{
				targetSpeedW = 0;
			}

			delay_us(10);
		}
		while (-angle < curve_distance);//(!(targetSpeedW == 0 && curSpeedW == 0));//(angle < curve_distance);
	}

	//update here for next movement to minimized the counts loss between cells.
	angle = 0;
	distance = 0;

	//moveStraight(CORRECTION_TURN_DISTANCE, TURN_SPEED);
	//setLED(LED5, LOW);
	turnCorrection(OFFSET_AFTER_TURN, deg);
}


void pivot180(void)
{
	bool hasLeftWall = false;//, hasRightWall = false;

	if (lateral_esquerdo > LATERAL_TH) hasLeftWall = true;
	//if (lateral_direito > LATERAL_TH) hasRightWall = true;

	if (frontal_esquerdo > FRONTAL_TH && frontal_direito > FRONTAL_TH) frontCal();
	else moveStraight(CELL_SIZE / 2, 0);

	if (hasLeftWall == true) pivot(90);
	else pivot(-90);

	frontCal();

	if (hasLeftWall == true) pivot(90);
	else pivot(-90);
}



void pivot(int16_t deg)
{
	int32_t curve_distance = abs(DEG_TO_COUNTS(deg)) + 10;

	targetSpeedX = 0;
	angleLeft = curve_distance;

	bUseEncoders = true;
	bUseSensors = false;
	bUseGyro = false;

	if (deg > 0)
	{
		targetSpeedW = SPEED_TO_COUNTS(SPEED_W);//RAD_S_TO_COUNTS(3.3333);

		do
		{
			/* DECELERATION OF PROFILER */
			if (needToDecelerate((float)curve_distance - angle, curSpeedW, endSpeedW) > decW)
			{
				targetSpeedW = 0;
			}

			delay_us(10);
		}
		while (angle < curve_distance);//(!(targetSpeedW == 0 && curSpeedW == 0));//(angle < curve_distance);
	}
	else
	{
		targetSpeedW = -SPEED_TO_COUNTS(SPEED_W);//-RAD_S_TO_COUNTS(3.3333);

		do
		{
			/* DECELERATION OF PROFILER */
			if (needToDecelerate((float)curve_distance + angle, curSpeedW, endSpeedW) > decW)
			{
				targetSpeedW = 0;
			}

			delay_us(10);
		}
		while (-angle < curve_distance);//(!(targetSpeedW == 0 && curSpeedW == 0));//(angle < curve_distance);
	}

	//update here for next movement to minimized the counts loss between cells.
	delay_ms(100);

	angle = 0;
	distance = 0;
}


void frente(int32_t mm)
{
	if (mm != CELL_SIZE) moveStraight(mm, speed_turn);
	else moveOneCell(speed_turn);
}

void curva(int16_t deg)
{
	bUseEncoders = true;
	bUseSensors = false;
	bUseGyro = false;

	if (abs(deg) == 180)
	{
		//beep(100);
		//moveStraight(CELL_SIZE / 2, 0);
		pivot180();
		moveStraight(CELL_SIZE / 2, speed_turn);
	}
	else
	{
		curveTurn(deg);
	}
}

void curvaPivot(int16_t deg)
{
	bUseSensors = false;

	if (abs(deg) == 180) pivot180();
}


void frontCal(void)
{
	uint32_t timeout = 0;
	targetSpeedX = 0;

	if (frontal_esquerdo > FRONTAL_TH && frontal_direito > FRONTAL_TH)
	{
		do
		{
			targetSpeedX = (CENTRO_FRONTAL - ((frontal_esquerdo + frontal_direito) / 2)) / 20;
			targetSpeedW = (frontal_esquerdo - frontal_direito) / 20;

			delay_us(10);
			timeout++;
		} while (((targetSpeedX != 0) || (targetSpeedW != 0)) && (timeout < 50000));
	}
}


void doMove(int32_t move)
{
	if (move == 0)
	{
		frente(CELL_SIZE);
	}
	else
	{
		curva(-move);
	}
}


void doPath( int16_t moves[] , uint8_t num_moves )
{
	uint8_t actual_move = 0;
	int32_t count = 0;

	//for (int i = 0; i < num_moves; i++) printf("%d\r\n", moves[i]);

	if (moves[0] == 0)  moveStraight(CELL_SIZE / 2, MOVE_SPEED);
	else moveStraight(CELL_SIZE / 2, TURN_SPEED);

	while (actual_move < num_moves)
	{
		while (moves[actual_move] == 0)
		{
			count++;
			if (++actual_move == num_moves) break;
		}

		if (count != 0)
		{
			if (actual_move == num_moves)
			{
				moveStraight(count * CELL_SIZE, 0);
			}
			else moveStraight(count * CELL_SIZE, TURN_SPEED);
		}
		else
		{
			curva(-moves[actual_move]);
			actual_move++;
		}

		count = 0;
	}

	targetSpeedX = 0;
}


void setSpeeds(int32_t spd_move, int32_t spd_turn)
{
	speed_move = spd_move;
	speed_turn = spd_turn;
}


void putBuffers(void)
{
	buf_l_sensor[index_buffers] = lateral_esquerdo;
	buf_r_sensor[index_buffers] = lateral_direito;

	index_buffers++;

	if (index_buffers >= MAX_BUF_SIZE) index_buffers = 0;
}


void printBuffers(void)
{
	for (uint16_t i = 0; i < index_buffers; i++)
	{
		printf("%d %d\r\n", buf_l_sensor[i], buf_r_sensor[i]);
	}
}


// Envio de dados ao software LFTrakking
void writeLFT(void)
{
#ifdef LFT_PRINTS	// Buffer de envio ao software LFTrakking
	// Envia as velocidades (pacotes de 100 contagens - a cada 100ms)
	//bufferLFT[index_buffer_lft] = leftEncoderChange;
	//bufferLFT[index_buffer_lft + 1] = rightEncoderChange;

	// Envia a resposta da velocidade SpeedX
	//bufferLFT[index_buffer_lft] = curSpeedX;
	//bufferLFT[index_buffer_lft + 1] = 2 * encoderChange;

	// Envia a resposta da velocidade SpeedW
	//bufferLFT[index_buffer_lft] = curSpeedW;
	//bufferLFT[index_buffer_lft + 1] = rotationalFeedback;

	bufferLFT[index_buffer_lft] = 2 * encoderChange;
	bufferLFT[index_buffer_lft + 1] = rotationalFeedback;

	index_buffer_lft += 2;
	if (index_buffer_lft == 201)// && c_aux < 100)
	{
		bufferLFT[0] = 0xAAAAAAAA;
		HAL_UART_DMAResume(&huart1);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)bufferLFT, 804);
		index_buffer_lft = 1;

		//c_aux++;
	}
#endif
}
