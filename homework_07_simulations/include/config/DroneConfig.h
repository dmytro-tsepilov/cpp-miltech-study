#pragma once
#include <cmath>
#include <ostream>

struct AmmoType
{
    std::string name;
	float mass = 0; 	// маса (кг)
	float drag = 0; 	// коефіцієнт опору
	float lift = 0; 	// коефіцієнт підйому
};

struct Coord
{
    double x;
    double y;

	// Додавання координат
	Coord operator+(const Coord& other) const {
    	Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
	}

	// Віднімання координат
	Coord operator-(const Coord& other) const {
    	Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
	}

	// Множення на скаляр
	Coord operator*(float s) const {
    	Coord result;
        result.x = x * s;
        result.y = y * s;
        return result;
	}

	// Ділення на скаляр
	Coord operator/(double s) const {
    	Coord result;
        result.x = x / s;
        result.y = y / s;
        return result;
	}

    double length() const {
        return std::hypot(x, y);
    }

    Coord normalize() const {
        double len = length();
        if (len > 1e-9) {
            return *this / len;
        }
        return {0, 0};
    }

    bool operator==(const Coord& other) const {
        return x == other.x && y == other.y;
    }

    // Helper function for hypot()
    double distanceTo(const Coord& other) const {
        return std::hypot(x - other.x, y - other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const Coord& c) {
        os << "(x: " << c.x << "; y:" << c.y << ")";
        return os;
    }
};

struct Target : Coord {

};

struct DroneConfig {
	Coord startPos;     	// початкова позиція (x, y)
	float altitude;     	// висота
	float initialDir;   	// початковий напрямок (рад)
	float attackSpeed;  	// швидкість атаки (м/с)
	float accelPath;    	// шлях розгону (м)
	std::string ammoName; 	// обрані боєприпаси
	float arrayTimeStep;	// крок часу масиву цілей
	float simTimeStep;  	// крок симуляції
	float hitRadius;    	// радіус влучення
	float angularSpeed; 	// кутова швидкість (рад/с)
	float turnThreshold;	// поріг повороту (рад)
};
