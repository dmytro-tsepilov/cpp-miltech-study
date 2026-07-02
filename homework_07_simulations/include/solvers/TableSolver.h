#pragma once

#include <string>
#include <vector>
#include "common/macros.h"
#include "interfaces/IBallisticSolver.h"

class TableSolver : public IBallisticSolver {
private:
    std::string filePath_;      // Path to ballistic_table.txt

    // 5 осей — кожна зі своїм набором вузлів (нерівномірний крок)
    std::vector<float> axisZ0;  // висота
    std::vector<float> axisV0;  // швидкість
    std::vector<float> axisM;   // маса
    std::vector<float> axisD;   // опір
    std::vector<float> axisL;   // підйомна сила
 
    // Результат в кожному вузлі сітки
    struct Result : BallisticResult { 
        // double t;      // час польоту
        // double hDist;  // горизонтальна дистанція
    };
 
    // Плоский масив розміром |Z0| * |V0| * |M| * |D| * |L|
    std::vector<Result> data;
 
    // Індекс і коефіцієнт для одного виміру
    struct Interp {
        int lo;      // нижній індекс в осі
        float frac;  // коефіцієнт [0..1]
    };

    Result lerp(const Result& a, const Result& b, float t) const;
    Interp findInterp(float val, const std::vector<float>& axis) const;
    Result lookup(float Z0, float V0, float m, float d,  float l) const;
    const Result& at(int iz, int iv, int im, int id, int il) const;
    size_t index(int iz, int iv, int im, int id, int il) const;

public:
    TableSolver(const std::string &path = "") {
        filePath_ = path;
        load(path);
    }
    BallisticResult calcluateTimeAndDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd) override;
    bool load(const std::string &path);
};
