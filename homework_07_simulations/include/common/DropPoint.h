#pragma once

#include "drone/DroneConfig.h"

struct DropPoint {
    Coord position;    // точка скидання (fire point)
    Coord aimPoint;    // куди впаде бомба (bomb aim point)
    Coord predictedTarget; // прогнозована позиція цілі
    double time;       // час обчислення
    int targetIndex;   // індекс цілі
    bool valid;        // чи є валідне рішення

    DropPoint() : position{0, 0}, aimPoint{0, 0}, predictedTarget{0, 0}, time(0), targetIndex(-1), valid(false) {}
};
