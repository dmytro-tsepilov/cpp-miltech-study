#include <cmath>
#include "solvers/AnalyticalSolver.h"

double AnalyticalSolver::calculateHorizontalDistance(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const double &time)
{
    // Calclulate horizontal distance to target
    // h = V₀t − t²d·V₀/(2m) + t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
    //     + t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³)
    //     + t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)

    // h = V₀t − t²d·V₀/(2m) +
    double horizontalDistance = attackSpeed * time - pow(time, 2) * ammoDrag * attackSpeed / (2 * ammoMass) +
                               // t³(6d·g·l·m − 6d²(l²-1)·V₀)/(36m²) +
                               pow(time, 3) * (6 * ammoDrag * g * ammoLift * ammoMass - 6 * pow(ammoDrag, 2) * (pow(ammoLift, 2) - 1) * attackSpeed) / (36 * pow(ammoMass, 2)) +
                               // t⁴ (−6d²g·l·(1+l²+l⁴)m + 3d³l²(1+l²)V₀ + 6d³l⁴(1+l²)V₀)  / (36(1+l²)²m³) +
                               pow(time, 4) * (-6 * pow(ammoDrag, 2) * g * ammoLift * (1 + pow(ammoLift, 2) + pow(ammoLift, 4)) * ammoMass + 3 * pow(ammoDrag, 3) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed + 6 * pow(ammoDrag, 3) * pow(ammoLift, 4) * (1 + pow(ammoLift, 2)) * attackSpeed) / (36 * pow(1 + pow(ammoLift, 2), 2) * pow(ammoMass, 3)) +
                               // t⁵(3d³g·l³m − 3d⁴l²(1+l²)V₀) / (36(1+l²)m⁴)
                               pow(time, 5) * (3 * pow(ammoDrag, 3) * g * pow(ammoLift, 3) * ammoMass - 3 * pow(ammoDrag, 4) * pow(ammoLift, 2) * (1 + pow(ammoLift, 2)) * attackSpeed) / (36 * (1 + pow(ammoLift, 2)) * pow(ammoMass, 4));

    return horizontalDistance;
}

double AnalyticalSolver::calculateTimeToTarget(const float &attackSpeed, const float &ammoDrag, const float &ammoMass, const float &ammoLift, const float &zd)
{
    // Calculate time to target
    double a = ammoDrag * g * ammoMass - 2 * pow(ammoDrag, 2) * ammoLift * attackSpeed;
    double b = -3 * g * pow(ammoMass, 2) + 3 * ammoDrag * ammoLift * ammoMass * attackSpeed;
    double c = 6 * pow(ammoMass, 2) * zd;

    // Degenerate case: a ≈ 0 → use simple free fall formula
    if (std::abs(a) < 1e-12)
    {
        return std::sqrt(2 * zd / g);
    }

    // Calculate Kardano method
    double p = -pow(b, 2) / (3 * pow(a, 2));
    double q = (2 * pow(b, 3)) / (27 * pow(a, 3)) + c / a;

    // If p >= 0, use fallback formula
    if (p >= 0)
    {
        return std::sqrt(2 * zd / g);
    }

    double arg = 3 * q / (2 * p) * std::sqrt(-3 / p);

    // If arg outside [-1, 1], use fallback formula
    if (std::abs(arg) > 1)
    {
        return std::sqrt(2 * zd / g);
    }

    double phi = std::acos(arg);
    double t = 2 * std::sqrt(-p / 3) * std::cos((phi + 4 * M_PI) / 3) - b / (3 * a);

    // If computed t is invalid, use fallback
    if (t <= 0 || !std::isfinite(t))
    {
        return std::sqrt(2 * zd / g);
    }

    return t;
}
