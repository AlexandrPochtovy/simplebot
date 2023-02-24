/*********************************************************************************
	Original author: alex

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
   
 * Move.h
 * Created on: 27 дек. 2022 г.
 ********************************************************************************/

#ifndef _MOVE_H_
#define _MOVE_H_

#include "main.h"
#include <FILObuffer/FILObuffer.h>
#include "math.h"
#include "Function/Function.h"
#include "PID/MyPid.h"

#define RAD_WHEEL_m 	0.033	// радиуc колеcа, м
#define BASE_m 				0.120 // раccто�?ние между о�?�?ми коле�?, м
#define PERC_TO_FREQ	210.0f
#define WALL_LIMIT 		150

typedef enum direction {
	STOP,
  HALT,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
} direction_t;

/*тип данных для расчета скорости вращения колеса*/
typedef struct wheel {
	uint32_t dec;
	uint32_t frac;
	uint32_t pulseNum;
	uint8_t timeout;
	float freq;
	float speed;	//cкороcть колеcа м/c
} wheel_t;

typedef struct point {
	float x;	//координата Х
	float y;	//координата Y
} point_t;

typedef struct setPoint {
	point_t target;		//координаты робота
	uint32_t leftRpsSP;  				//PWM value left
	uint32_t rightRpsSP;  			//PWM value right
} setPoint_t;

typedef struct engine {
	const TIM_TypeDef *tim;  			//TIM
	const uint32_t channelLeft;  	//output channel left
	const uint32_t channelRight;  //output channel
	direction_t dir;  						//move direction for set pin mask
} engine_t;

typedef struct robot {
	point_t coord;			//координаты робота
	float speedRobot;		//main speed
	float length;				//расстояние до цели
	float path;					//путь пройденный центром робота
	float intLength;		//интеграл пути
	float bearing;  		//пеленг (theta) и�?комый
	float course;  			//psi кур�? робота, угол в �?и�?теме координат
	float angle;  			//курcовой угол
	float gyroSpeed;		//угловая скорость
	wheel_t wheelLeft;	//левое колесо
	wheel_t wheelRight;	//правое колесо
	engine_t engine;		//двигатель
	setPoint_t SP;			//заданнные значения для движения
} robot_t;

void rpsCalc(wheel_t *wheel, TIM_TypeDef *_tim, const uint8_t ch, const uint32_t lim, const uint8_t res);

void SetNewTargetPoint(robot_t *bot, int32_t newX, int32_t newY, filo_t *pointX, filo_t *pointY);

uint8_t MoveDrive(robot_t *bot, uint32_t dt, point_t point, int16_t distL, int16_t distR, uint16_t wall);


uint8_t MoveEmergency(uint32_t mainTime);

#endif /* FUNCTION_MOVE_H_ */
