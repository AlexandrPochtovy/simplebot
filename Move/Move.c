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

 * Move.c
 * Created on: 27 дек. 2022 г.
 ********************************************************************************/
#include "Move.h"

#define K_I						10  	// ко�?ффициент интегральной �?о�?тавл�?ющей линейной �?коро�?ти
#define ERROR_m 			0.05		// точно�?ть до�?тижени�? целевой точки м

static int32_t x[32] = {0.0};
static int32_t y[32] = {0.0};
static filo_t pointX = {x, 32, 0, 0, FREE};
static filo_t pointY = {y, 32, 0, 0, FREE};


__STATIC_INLINE void Set_Direction_STOP(void) {
	LL_GPIO_ResetOutputPin(AIN1_GPIO_Port, AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin);
}

__STATIC_INLINE void Set_Direction_HALT(void) {
	LL_GPIO_SetOutputPin(AIN1_GPIO_Port, AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin);
}

__STATIC_INLINE void Set_Direction_FORWARD(void) {
	LL_GPIO_ResetOutputPin(AIN1_GPIO_Port, AIN2_Pin | BIN1_Pin);
	LL_GPIO_SetOutputPin(AIN1_GPIO_Port, AIN1_Pin | BIN2_Pin);
}

__STATIC_INLINE void Set_Direction_BACKWARD(void) {
	LL_GPIO_ResetOutputPin(AIN1_GPIO_Port, AIN1_Pin | BIN2_Pin);
	LL_GPIO_SetOutputPin(AIN1_GPIO_Port, AIN2_Pin | BIN1_Pin);
}

__STATIC_INLINE void Set_Direction_LEFT(void) {
	LL_GPIO_ResetOutputPin(AIN1_GPIO_Port, AIN2_Pin | BIN2_Pin);
	LL_GPIO_SetOutputPin(AIN1_GPIO_Port, AIN1_Pin | BIN1_Pin);
}

__STATIC_INLINE void Set_Direction_RIGHT(void) {
	LL_GPIO_ResetOutputPin(AIN1_GPIO_Port, AIN1_Pin | BIN1_Pin);
	LL_GPIO_SetOutputPin(AIN1_GPIO_Port, AIN2_Pin | BIN2_Pin);
}

__STATIC_INLINE int32_t float_to_int_mm(float value) {
	return (int32_t)value * 1000;
}

__STATIC_INLINE float int_mm_to_float(int value) {
	return ((float)value) / 1000.0f;
}

/*функция гистерезиса для устранения дребезжания колеса на скорости около нуля*/
uint32_t Hyst(uint32_t act, const uint32_t deadzone, const uint32_t trig) {
	if (act > deadzone) {
		return act;
	}
	else if (act < (deadzone - trig)) {
		return 0;
	}
	else {
		return deadzone;
	}
}

/*функция рассчитывает частоту вращения колес*/
void rpsCalc(wheel_t *wheel, TIM_TypeDef *_tim, const uint8_t ch, const uint32_t lim, const uint8_t res) {
	volatile uint32_t act;
	switch (ch) {
	case 1:
		act = _tim->CCR1; break;
	case 2:
		act = _tim->CCR2; break;
	case 3:
		act = _tim->CCR3; break;
	case 4:
		act = _tim->CCR4; break;
	default:
		act = 0; break;
	}
	volatile uint32_t total;
	if (act >= wheel->frac) {
		total = wheel->dec * (_tim->ARR + 1) + (act - wheel->frac);
	} else {
		total = (wheel->dec - 1) * (_tim->ARR + 1) + ((_tim->ARR + 1) - wheel->frac + act);
	}
		//total pulses from measure timer
	uint32_t timFreq = SystemCoreClock / (_tim->PSC + 1);
	wheel->freq = (float)timFreq * wheel->pulseNum / (total * res);
	wheel->timeout = 0;
	wheel->dec = 0;
	wheel->pulseNum = 0;
	wheel->frac = act;
}


direction_t DirEnable(int32_t leftRpsSP, int32_t rightRpsSP) {
	if ((leftRpsSP > 0) 		& (rightRpsSP > 0))		{
		Set_Direction_FORWARD();
		return FORWARD;
	}
	if ((leftRpsSP < 0) 		& (rightRpsSP < 0)) 		{
		Set_Direction_BACKWARD();
		return BACKWARD;
	}
	if ((leftRpsSP > 0) 	& (rightRpsSP <= 0))	{
		Set_Direction_RIGHT();
		return RIGHT;
	}
	if ((leftRpsSP <= 0) 	& (rightRpsSP > 0)) 	{
		Set_Direction_LEFT();
		return LEFT;
	}
	else 	{
		Set_Direction_STOP();
		return STOP;
	}
}

void ResetTrace(robot_t *bot) {
	bot->coord.x = 0;
	bot->coord.y = 0;
	bot->length = 0;
	bot->intLength = 0;
}

void SetNewTargetPoint(robot_t *bot, int32_t newX, int32_t newY, filo_t *pointX, filo_t *pointY) {
	FILO_PutOne(pointX, newX);
	FILO_PutOne(pointY, newY);
}

uint8_t AddIntermediatePoint(robot_t *bot, filo_t *pointX, filo_t *pointY) {
	int32_t tmpX_mm = float_to_int_mm(bot->coord.x + bot->length * cosf(bot->course) / 2);
	int32_t tmpY_mm = float_to_int_mm(bot->coord.y + bot->length * sinf(bot->course) / 2);
	FILO_PutOne(pointX, tmpX_mm);
	FILO_PutOne(pointY, tmpY_mm);
	return 1;
}

uint8_t GetNextTargetPoint(robot_t *bot, filo_t *pointX, filo_t *pointY) {
	int32_t tmp;
	FILO_GetOne(pointX, &tmp);
	bot->SP.target.x = int_mm_to_float(tmp);
	FILO_GetOne(pointY, &tmp);
	bot->SP.target.y = int_mm_to_float(tmp);
	return 1;
}

uint8_t MoveDrive(robot_t *bot, uint32_t dt, point_t point, int16_t distL, int16_t distR, uint16_t wall) {
	float delthaX = bot->SP.target.x - bot->coord.x;//расстояние до точки по X
	float delthaY = bot->SP.target.y - bot->coord.y;//расстояние до точки по Y
	bot->length = sqrtf(delthaY * delthaY + delthaX * delthaX);//обща�? длина до точки назначени�?
	bot->bearing = atan2f(delthaY, delthaX);  //пеленг - угол вектора из текущего положени�? к заданному

	bot->wheelLeft.speed = M_TWOPI * RAD_WHEEL_m * bot->wheelLeft.freq;
	bot->wheelRight.speed = M_TWOPI * RAD_WHEEL_m * bot->wheelRight.freq;
	bot->speedRobot = (bot->wheelLeft.speed + bot->wheelRight.speed) / 2;
	float delthaPath_m = bot->speedRobot * dt / 1000;
	bot->path += delthaPath_m;
	bot->intLength += delthaPath_m;
	if (bot->intLength > 10) { bot->intLength = 10; }

	bot->gyroSpeed = (bot->wheelRight.speed - bot->wheelLeft.speed) / BASE_m;  //углова�? �?коро�?ть (рад/cек)
	bot->course += bot->gyroSpeed * dt / 1000;  //ра�?чет кур�?а - текущий угол поворота робота, рад
	if (bot->course > M_TWOPI) {
		bot->course = bot->course - M_TWOPI;
	}
	else if (bot->course < -M_TWOPI) {
		bot->course = bot->course + M_TWOPI;
	}
	bot->coord.x += delthaPath_m * cosf(bot->course);
	bot->coord.y += delthaPath_m * sinf(bot->course);

	int16_t distMax = distR >= distL ? distR : distL;
	int16_t kDist = signum_t(distL - distR);//положение ближайшего преп�?т�?тви�?, �?права или �?лева

	float kd = kDist * distMax * M_TWOPI / WALL_LIMIT;
	bot->angle = bot->bearing - bot->course - kd;  //new
	//if (fabsf(angle) > M_PI) angle = angle - signum_t(angle) * M_TWOPI;
	if ((bot->angle > M_PI_2)) {
		bot->angle = M_PI_2;
		/*препятствие ровно справа формируем новую промежуточную точку*/
	}
	else if ((bot->angle < -M_PI_2)) {
		bot->angle = -M_PI_2;
		/*препятствие ровно слева формируем новую промежуточную точку*/
	}

	float vector = tanhf(bot->length + K_I * bot->intLength);
	int32_t baseSpeed = (int32_t)(220 * cosf(bot->angle) * vector);
	int32_t control = (int32_t)(120 * sinhf(bot->angle) * vector);
	int32_t pwmLeft = baseSpeed - control + 280 - wall;
	int32_t pwmRight = baseSpeed + control + 280 - wall;
	bot->engine.dir = DirEnable(pwmLeft, pwmRight);//set direction move
	//bot->SP.leftRpsSP = Hyst(abs(pwmLeft), 100, 20);
	//bot->SP.rightRpsSP = Hyst(abs(pwmRight), 100, 20);
	//bot->wheelLeft.freq = (float)abs(pwmLeft) / PERC_TO_FREQ;
	//bot->wheelRight.freq = (float)abs(pwmRight) / PERC_TO_FREQ;
	__NOP();
	if (bot->length < ERROR_m) {  // выход при до�?тижении обла�?ти точки
		//думаем куда ехать дальше

		return 1;
	}
	return 0;
}
