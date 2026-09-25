#pragma once
#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <cstdio>

/*
 * 3D Linkage Mechanism Solver (utils.hpp)
 * Header-only, no external dependencies.
 *
 * Forward:  (rot_x, rot_y) → (theta_C, theta_D)
 * Inverse:  (theta_C, theta_D) → (rot_x, rot_y)   [Newton-Raphson]
 * Velocity: uses numerical Jacobian inverse
 *
 * Port of linkage_3d.py.
 */

namespace linkage {

// ============================================================
// Basic 3D vector math (no library needed)
// ============================================================
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator*(double s)      const { return {x * s, y * s, z * s}; }
    double dot(const Vec3& v)     const { return x*v.x + y*v.y + z*v.z; }
    double length_sq()            const { return dot(*this); }
    double length()               const { return std::sqrt(length_sq()); }
};

// 3x3 matrix stored row-major
struct Mat3 {
    double m[3][3] = {};

    Vec3 operator*(const Vec3& v) const {
        return {
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z
        };
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }
};

// ============================================================
// Constants (matching Python code)
// ============================================================
namespace detail {

constexpr double DEG2RAD    = 3.14159265358979323846 / 180.0;
constexpr double OFFSET_RAD = 98.5836 * DEG2RAD;

// Initial positions
constexpr double A0x = -41.0, A0y =  26.5, A0z = -13.5;
constexpr double B0x = -41.0, B0y = -26.5, B0z = -13.5;
constexpr double C0x = -41.0, C0y = -26.5, C0z = 102.0;
constexpr double D0x = -41.0, D0y =  26.5, D0z = 169.0;

constexpr double O1x = -41.0, O1y = 0.0, O1z = 106.0;
constexpr double O2x = -41.0, O2y = 0.0, O2z = 173.0;

inline double compute_L_BC() {
    double dy = C0y - B0y, dz = C0z - B0z, dx = C0x - B0x;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
inline double compute_L_AD() {
    double dy = D0y - A0y, dz = D0z - A0z, dx = D0x - A0x;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

inline double compute_R_c() {
    double dy = C0y - O1y, dz = C0z - O1z;
    return std::sqrt(dy*dy + dz*dz);
}
inline double compute_R_d() {
    double dy = D0y - O2y, dz = D0z - O2z;
    return std::sqrt(dy*dy + dz*dz);
}

} // namespace detail

// ============================================================
// Rotation matrices
// ============================================================
inline Mat3 rotation_matrix_x(double angle_rad) {
    double a = angle_rad;
    double ca = std::cos(a), sa = std::sin(a);
    Mat3 r;
    r.m[0][0] = 1;  r.m[0][1] = 0;   r.m[0][2] = 0;
    r.m[1][0] = 0;  r.m[1][1] = ca;  r.m[1][2] = -sa;
    r.m[2][0] = 0;  r.m[2][1] = sa;  r.m[2][2] = ca;
    return r;
}

inline Mat3 rotation_matrix_y(double angle_rad) {
    double a = -angle_rad;  // reversed sign, matching Python
    double ca = std::cos(a), sa = std::sin(a);
    Mat3 r;
    r.m[0][0] = ca;  r.m[0][1] = 0;  r.m[0][2] = sa;
    r.m[1][0] = 0;   r.m[1][1] = 1;  r.m[1][2] = 0;
    r.m[2][0] = -sa;  r.m[2][1] = 0; r.m[2][2] = ca;
    return r;
}

// ============================================================
// C and D positions on their circular orbits
// ============================================================
inline Vec3 get_C_position(double theta_c_rad, double R_c) {
    return {
        detail::O1x,
        detail::O1y + R_c * std::sin(theta_c_rad),
        detail::O1z + R_c * std::cos(theta_c_rad)
    };
}

inline Vec3 get_D_position(double theta_d_rad, double R_d) {
    return {
        detail::O2x,
        detail::O2y + R_d * std::sin(theta_d_rad),
        detail::O2z + R_d * std::cos(theta_d_rad)
    };
}

// ============================================================
// Result structs
// ============================================================
struct LinkageResult {
    bool   ok;
    double theta_c_rad;  // final angle C (negated, minus offset)
    double theta_d_rad;  // final angle D (minus offset)
    Vec3   A, B, C, D;
    double error_BC;
    double error_AD;
};

struct InverseLinkageResult {
    bool   ok;
    double rot_x_rad;
    double rot_y_rad;
    double error;        // max residual in radians
    int    iterations;
};

struct VelocityConvertResult {
    bool   ok;
    double vel_rot_x;    // velocity in rot_x space
    double vel_rot_y;    // velocity in rot_y space
};

// Precomputed ankle linkage input limits (rot space):
//   rot_y = ankle pitch direction → ±30° = ±0.524 rad
//   rot_x = ankle roll  direction → ±15° = ±0.262 rad
constexpr double kAnklePitchRotLimitRad = 0.524;
constexpr double kAnkleRollRotLimitRad  = 0.262;

// ============================================================
// Internal helpers
// ============================================================
namespace detail {

struct TwoSolutions { double s[2]; int count; };

inline TwoSolutions solve_circle_constraint(
    double Ox, double Oy, double Oz, double R,
    const Vec3& Q, double L2)
{
    TwoSolutions result = {{0, 0}, 0};
    double dx = Ox - Q.x;
    double dy = Oy - Q.y;
    double dz = Oz - Q.z;
    double rhs = (L2 - dx*dx - dy*dy - dz*dz - R*R) / (2.0 * R);

    double A = dy;
    double B = dz;
    double mag = std::sqrt(A*A + B*B);

    if (mag < 1e-15) return result;

    double ratio = rhs / mag;
    if (ratio > 1.0)  ratio = 1.0;
    if (ratio < -1.0) ratio = -1.0;

    double base = std::atan2(A, B);
    double delta = std::acos(ratio);

    result.s[0] = base + delta;
    result.s[1] = base - delta;
    result.count = 2;
    return result;
}

inline double pick_closest(const TwoSolutions& sols, double guess) {
    constexpr double PI  = 3.14159265358979323846;
    constexpr double PI2 = 2.0 * PI;
    auto angle_dist = [&](double a, double b) {
        double d = std::fmod(a - b, PI2);
        if (d > PI)  d -= PI2;
        if (d < -PI) d += PI2;
        return std::fabs(d);
    };
    if (sols.count == 0) return guess;
    if (sols.count == 1) return sols.s[0];
    return (angle_dist(sols.s[0], guess) <= angle_dist(sols.s[1], guess))
           ? sols.s[0] : sols.s[1];
}

} // namespace detail

// ============================================================
// Forward solver: (rot_x, rot_y) → (theta_C, theta_D)
// ============================================================
inline LinkageResult solve_linkage(double rot_x_rad, double rot_y_rad)
{
    using namespace detail;

    const double L_BC  = compute_L_BC();
    const double L_AD  = compute_L_AD();
    const double L_BC2 = L_BC * L_BC;
    const double L_AD2 = L_AD * L_AD;
    const double R_c   = compute_R_c();
    const double R_d   = compute_R_d();

    Mat3 R = rotation_matrix_x(rot_x_rad) * rotation_matrix_y(rot_y_rad);
    Vec3 A_new = R * Vec3(A0x, A0y, A0z);
    Vec3 B_new = R * Vec3(B0x, B0y, B0z);

    double tc0 = std::atan2(C0y - O1y, C0z - O1z);
    double td0 = std::atan2(D0y - O2y, D0z - O2z);

    auto sols_c = detail::solve_circle_constraint(O1x, O1y, O1z, R_c, B_new, L_BC2);
    auto sols_d = detail::solve_circle_constraint(O2x, O2y, O2z, R_d, A_new, L_AD2);

    bool ok_c = (sols_c.count > 0);
    bool ok_d = (sols_d.count > 0);

    double tc_sol = ok_c ? detail::pick_closest(sols_c, tc0) : 0.0;
    double td_sol = ok_d ? detail::pick_closest(sols_d, td0) : 0.0;

    LinkageResult res;
    res.ok = ok_c && ok_d;
    res.A  = A_new;
    res.B  = B_new;

    if (res.ok) {
        res.C = get_C_position(tc_sol, R_c);
        res.D = get_D_position(td_sol, R_d);
        res.theta_c_rad = -tc_sol - OFFSET_RAD;
        res.theta_d_rad =  td_sol - OFFSET_RAD;
        res.error_BC = std::fabs((res.C - res.B).length() - L_BC);
        res.error_AD = std::fabs((res.D - res.A).length() - L_AD);

        if (res.error_BC > 0.01 || res.error_AD > 0.01) {
            res.ok = false;
        }
    } else {
        res.theta_c_rad = 0;
        res.theta_d_rad = 0;
        res.C = {};
        res.D = {};
        res.error_BC = -1;
        res.error_AD = -1;
    }

    return res;
}

// ============================================================
// Inverse solver: (theta_C, theta_D) → (rot_x, rot_y)
//
// Uses Newton-Raphson on the forward solver.
// f(rx, ry) = (theta_c(rx,ry), theta_d(rx,ry))
// We want f(rx, ry) = (target_tc, target_td)
// Jacobian computed numerically with central differences.
// ============================================================
inline InverseLinkageResult solve_linkage_inverse(
    double target_theta_c_rad,
    double target_theta_d_rad,
    double guess_rx = 0.0,
    double guess_ry = 0.0,
    int    max_iter = 50,
    double tol      = 1e-6,
    double eps      = 1e-4)
{
    InverseLinkageResult res;
    res.ok = false;
    res.rot_x_rad = guess_rx;
    res.rot_y_rad = guess_ry;
    res.error = 1e9;
    res.iterations = 0;

    double rx = guess_rx;
    double ry = guess_ry;

    for (int iter = 0; iter < max_iter; iter++) {
        res.iterations = iter + 1;

        // Evaluate f at current point
        auto f0 = solve_linkage(rx, ry);
        if (!f0.ok) break;

        double ec = f0.theta_c_rad - target_theta_c_rad;
        double ed = f0.theta_d_rad - target_theta_d_rad;

        double err = std::fmax(std::fabs(ec), std::fabs(ed));
        res.error = err;

        if (err < tol) {
            res.ok = true;
            res.rot_x_rad = rx;
            res.rot_y_rad = ry;
            return res;
        }

        // Jacobian via central differences
        auto fxp = solve_linkage(rx + eps, ry);
        auto fxm = solve_linkage(rx - eps, ry);
        auto fyp = solve_linkage(rx, ry + eps);
        auto fym = solve_linkage(rx, ry - eps);

        if (!fxp.ok || !fxm.ok || !fyp.ok || !fym.ok) break;

        double dTc_drx = (fxp.theta_c_rad - fxm.theta_c_rad) / (2.0 * eps);
        double dTc_dry = (fyp.theta_c_rad - fym.theta_c_rad) / (2.0 * eps);
        double dTd_drx = (fxp.theta_d_rad - fxm.theta_d_rad) / (2.0 * eps);
        double dTd_dry = (fyp.theta_d_rad - fym.theta_d_rad) / (2.0 * eps);

        // Invert 2x2 Jacobian: J = [[a,b],[c,d]], J_inv = 1/det * [[d,-b],[-c,a]]
        double det = dTc_drx * dTd_dry - dTc_dry * dTd_drx;
        if (std::fabs(det) < 1e-15) break;  // singular

        double inv_det = 1.0 / det;
        double drx = inv_det * ( dTd_dry * ec - dTc_dry * ed);
        double dry = inv_det * (-dTd_drx * ec + dTc_drx * ed);

        rx -= drx;
        ry -= dry;
    }

    // Final check
    auto ff = solve_linkage(rx, ry);
    if (ff.ok) {
        double ec = std::fabs(ff.theta_c_rad - target_theta_c_rad);
        double ed = std::fabs(ff.theta_d_rad - target_theta_d_rad);
        res.error = std::fmax(ec, ed);
        if (res.error < tol * 10.0) {  // slightly relaxed for final
            res.ok = true;
        }
    }
    res.rot_x_rad = rx;
    res.rot_y_rad = ry;
    return res;
}

// ============================================================
// Velocity conversion: (vel_theta_C, vel_theta_D) → (vel_rot_x, vel_rot_y)
//
// Uses the inverse Jacobian at the current position.
// J * [vel_rx, vel_ry]^T = [vel_tc, vel_td]^T
// => [vel_rx, vel_ry]^T = J_inv * [vel_tc, vel_td]^T
//
// Position (rot_x, rot_y) must be known (from inverse position solve).
// ============================================================
inline VelocityConvertResult convert_velocity_to_rot(
    double rot_x_rad, double rot_y_rad,
    double vel_theta_c, double vel_theta_d,
    double eps = 1e-4)
{
    VelocityConvertResult res;
    res.ok = false;
    res.vel_rot_x = 0.0;
    res.vel_rot_y = 0.0;

    auto fxp = solve_linkage(rot_x_rad + eps, rot_y_rad);
    auto fxm = solve_linkage(rot_x_rad - eps, rot_y_rad);
    auto fyp = solve_linkage(rot_x_rad, rot_y_rad + eps);
    auto fym = solve_linkage(rot_x_rad, rot_y_rad - eps);

    if (!fxp.ok || !fxm.ok || !fyp.ok || !fym.ok) return res;

    double dTc_drx = (fxp.theta_c_rad - fxm.theta_c_rad) / (2.0 * eps);
    double dTc_dry = (fyp.theta_c_rad - fym.theta_c_rad) / (2.0 * eps);
    double dTd_drx = (fxp.theta_d_rad - fxm.theta_d_rad) / (2.0 * eps);
    double dTd_dry = (fyp.theta_d_rad - fym.theta_d_rad) / (2.0 * eps);

    double det = dTc_drx * dTd_dry - dTc_dry * dTd_drx;
    if (std::fabs(det) < 1e-15) return res;

    double inv_det = 1.0 / det;
    res.vel_rot_x = inv_det * ( dTd_dry * vel_theta_c - dTc_dry * vel_theta_d);
    res.vel_rot_y = inv_det * (-dTd_drx * vel_theta_c + dTc_drx * vel_theta_d);
    res.ok = true;
    return res;
}

} // namespace linkage

#endif // UTILS_HPP