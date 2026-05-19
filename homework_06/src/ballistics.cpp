#include "ballistics.hpp"

#include <strings.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <regex>
#include <cmath>

int processCalculations(const char* line) {
    float xd, yd, zd = 0;
    float targetX, targetY = 0;
    float attackSpeed = 0;
    float accelerationPath = 0;
    char ammo_name[50] = "";

    float ammoMass, ammoDrag, ammoLift = 0;
    bool ammoFly = false; // false - Free fall, true - Gliding

    // Debug output
    std::cout << "file:" << line << std::endl;

    std::smatch match;
    // Parse the line using regex, where format is: `xd yd zd targetX targetY attackSpeed accelerationPath ammo_name`
    std::regex pattern(R"(([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+(.+))");
    std::string lineStr(line);
    if (std::regex_match(lineStr, match, pattern)) {
        xd = std::stof(match[1]);
        yd = std::stof(match[2]);
        zd = std::stof(match[3]);
        targetX = std::stof(match[4]);
        targetY = std::stof(match[5]);
        attackSpeed = std::stof(match[6]);
        accelerationPath = std::stof(match[7]);
        std::string ammo = match[8];
        strcpy(ammo_name, ammo.c_str());
    } else {
        std::cout << "Error parsing line" << std::endl;
    }

    std::cout << "data:" << ammo_name << " " << yd << " " << zd << std::endl;

    // Detect Ammo Type
    if (!strcasecmp(ammo_name, "VOG-17")) {
        ammoMass = 0.35f;
        ammoDrag = 0.07f;
    } else if (!strcasecmp(ammo_name, "M67")) {
        ammoMass = 0.6f;
        ammoDrag = 0.1f;
    } else if (!strcasecmp(ammo_name, "RKG-3")) {
        ammoMass = 1.2f;
        ammoDrag = 0.1f;
    } else if (!strcasecmp(ammo_name, "GLIDING-VOG")) {
        ammoMass = 0.45f;
        ammoDrag = 0.1f;
        ammoLift = 1.0f;
        ammoFly = true;
    } else if (!strcasecmp(ammo_name, "GLIDING-RKG")) {
        ammoMass = 1.4f;
        ammoDrag = 0.1f;
        ammoLift = 1.0f;
        ammoFly = true;
    } else {
        std::cout << "Ammo Type: Unknown" << std::endl;
        return 1;
    }

    // Calculate distance to target
    float distance = sqrt(pow(targetX - xd, 2) + pow(targetY - yd, 2));
    std::cout << "Distance to target: " << distance << std::endl;

    if (distance <= 0) {
        std::cout << "Target is at the same position as the shooter" << std::endl;
        return 1;
    }

    std::cout << "ammoMass: " << ammoMass << std::endl;
    std::cout << "ammoDrag: " << ammoDrag << std::endl;
    std::cout << "ammoLift: " << ammoLift << std::endl;

    // Calculate time to target
    float a = ammoDrag * g * ammoMass - 2 * pow(ammoDrag, 2) * ammoLift * attackSpeed;
    float b = -3 * g * pow(ammoMass, 2) + 3 * ammoDrag * ammoLift * ammoMass * attackSpeed;
    float c = 6 * pow(ammoMass, 2) * zd;

    std::cout << "a: " << a << std::endl;
    std::cout << "b: " << b << std::endl;
    std::cout << "c: " << c << std::endl;


    // Calculate Kardano method
    float p = -pow(b, 2) / (3 * pow(a, 2));
    float q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

    float phi = acos(3 * q / (2 * p) * sqrt(-3 / p));

    std::cout << "p: " << p << std::endl;
    std::cout << "q: " << q << std::endl;
    std::cout << "phi: " << phi << std::endl;

    float t = 2 * sqrt(-p / 3) * cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    if (t <= 0) {
        std::cout << "No valid time solution" << std::endl;
        return 1;
    }

    std::cout << "t: " << t << std::endl;

    // Calclulate horizontal distance to target
    // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
    //     + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³) 
    //     + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)


                       // h = V₀t − t²d·V₀/(2m) +
    float horizontalDistance = attackSpeed * t - pow(t, 2) * ammoDrag * attackSpeed / (2 * ammoMass) + 
        // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
        pow(t, 3) * (6 * ammoDrag * g * ammoLift * ammoMass - 6 * pow(ammoDrag, 2) * (pow(ammoLift, 2) -1) * attackSpeed) / (36 * pow(ammoMass, 2)) +
        // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³) +
        pow(t, 4) * (
            -6 * pow(ammoDrag, 2) * g * ammoLift * (1 + pow(ammoLift, 2) + pow(ammoLift, 4)) * ammoMass + 
            3 * pow(ammoDrag, 3) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed + 
            6 * pow(ammoDrag, 3) * pow(ammoLift, 4) * (1 + pow(ammoLift, 2)) * attackSpeed) / (36 * pow(1 + pow(ammoLift, 2), 2) * pow(ammoMass, 3)) + 
        // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
        pow(t, 5) * (
            3 * pow(ammoDrag, 3) * g * pow(ammoLift, 3) * ammoMass - 
            3 * pow(ammoDrag, 4) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed
        ) / (36 * (1 + pow(ammoLift, 2)) * pow(ammoMass, 4));

    if (horizontalDistance <= 0) {
        std::cout << "Calculated horizontal distance is zero or negative, which is invalid" << std::endl;
        return 1;
    }

    std::cout << "Horizontal distance to target: " << horizontalDistance << std::endl;


    char buffer[100];
    std::ofstream outputFile("output.txt");
    if (horizontalDistance + accelerationPath > distance) {
        std::cout << "Target is out of range" << std::endl;

        float xd_prime = targetX - (targetX - xd) * (horizontalDistance + accelerationPath) / distance;
        float yd_prime = targetY - (targetY - yd) * (horizontalDistance + accelerationPath) / distance;

        std::snprintf(buffer, sizeof(buffer), "%.3f %.3f", xd_prime, yd_prime);
        outputFile << buffer << " ";
    }

    // Calculate targert coordinates after acceleration path
    float ratio = (distance - horizontalDistance) / distance;
    float fireX = xd + (targetX - xd) * ratio;
    float fireY = yd + (targetY - yd) * ratio;

    std::cout << "Fire coordinates: (" << fireX << ", " << fireY << ")" << std::endl;

    // Write results to file
    std::snprintf(buffer, sizeof(buffer), "%.3f %.3f", fireX, fireY);
    outputFile << buffer << std::endl;
    //writeOutputFile(std::string(buffer).c_str() );

    return 0;
}

bool readInputFile(std::vector<std::string>& lines, std::string filename) {
    lines.clear();
    std::ifstream inputFile(filename);
    if (!inputFile) {
        return false;
    }

    std::string line;
    size_t totalSize = 0;
    while (std::getline(inputFile, line)) {
        size_t lineSize = line.size() + 1;
        if (totalSize + lineSize > 1024) {
            break;
        }
        totalSize += lineSize;
        lines.push_back(line);
    }
    return true;
}

bool writeOutputFile(const char* line, std::string filename) {
    std::ofstream outputFile(filename);
    if (!outputFile) {
        return false;
    }

    outputFile << line << std::endl;
    return true;
}
