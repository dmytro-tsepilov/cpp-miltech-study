#pragma once

#include "config/DroneConfig.h"

struct SimStep {
	Coord pos;          	// позиція дрона
	float direction;    	// напрямок (рад)
	int8_t state;        	// стан автомата (0-4)
	int   targetIdx;    	// індекс поточної цілі
	Coord dropPoint;    	// точка скиду (куди летить дрон)
	Coord aimPoint;     	// куди впаде бомба (якщо скинути зараз)
	Coord predictedTarget;  // прогнозована позиція цілі
};
