// CADRC.cpp
#include "CADRC.h"
#include <cmath>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>

using matrix::Matrix;
using matrix::SquareMatrix;

// ---------- geninv：Cholesky + DLS ----------
template<typename Type, size_t N>
static bool cholesky_spd(const SquareMatrix<Type, N>& A, SquareMatrix<Type, N>& L)
{
    constexpr Type eps = Type(1e-18);
    L.setZero();
    SquareMatrix<Type, N> S = (A + A.transpose()) * Type(0.5);

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j <= i; ++j) {
            Type sum = S(i, j);
            for (size_t k = 0; k < j; ++k) sum -= L(i, k) * L(j, k);

            if (i == j) {
                if (sum <= eps) return false;
                L(i, i) = std::sqrt(sum);
            } else {
                L(i, j) = sum / L(j, j);
            }
        }
    }
    return true;
}

template<typename Type, size_t N, size_t K>
static bool cholesky_solve_inplace(const SquareMatrix<Type, N>& L, Matrix<Type, N, K>& B)
{
    for (size_t col = 0; col < K; ++col) {
        // forward
        for (size_t i = 0; i < N; ++i) {
            Type sum = B(i, col);
            for (size_t k = 0; k < i; ++k) sum -= L(i, k) * B(k, col);
            B(i, col) = sum / L(i, i);
        }
        // backward
        for (size_t ii = 0; ii < N; ++ii) {
            size_t i = N - 1 - ii;
            Type sum = B(i, col);
            for (size_t k = i + 1; k < N; ++k) sum -= L(k, i) * B(k, col);
            B(i, col) = sum / L(i, i);
        }
    }
    return true;
}

template<typename Type, size_t M, size_t N>
static bool geninv_c(const Matrix<Type, M, N>& G, Matrix<Type, N, M>& res, Type lambda = Type(1e-8))
{
    if (lambda <= Type(0)) lambda = Type(1e-12);

    if (M <= N) {
        SquareMatrix<Type, M> A = G * G.transpose();
        SquareMatrix<Type, M> I; I.setIdentity();
        A += lambda * I;

        SquareMatrix<Type, M> L;
        if (!cholesky_spd<Type, M>(A, L)) {
            res.setZero();                 // 不要用默认构造
            return false;
        }

        Matrix<Type, M, N> X = G;
        cholesky_solve_inplace<Type, M, N>(L, X);
        res = X.transpose();
        return true;

    } else {
        SquareMatrix<Type, N> A = G.transpose() * G;
        SquareMatrix<Type, N> I; I.setIdentity();
        A += lambda * I;

        SquareMatrix<Type, N> L;
        if (!cholesky_spd<Type, N>(A, L)) {
            res.setZero();                 // 不要用默认构造
            return false;
        }

        Matrix<Type, N, M> X = G.transpose();
        cholesky_solve_inplace<Type, N, M>(L, X);
        res = X;
        return true;
    }
}


// ---------- ADRC ----------
ADRCController::ADRCController(float J0, float kp1, float omega_o1, float kp2, float omega_o2, float alpha)
{
    _J0 = J0;
    _Jc_inv = 1.0f / _J0;

    _kp1 = kp1;
    _omega_o1 = omega_o1;
    _kp2 = kp2;
    _omega_o2 = omega_o2;
    _alpha = alpha;

    initWeightMatrix();
    updateBetaGains();
    reset();
}

void ADRCController::setJ0(float J0)
{
    _J0 = math::max(J0, 0.001f);
    _Jc_inv = 1.0f / _J0;
}

void ADRCController::setGains(float kp1, float omega_o1, float kp2, float omega_o2)
{
    _kp1 = kp1;
    _omega_o1 = omega_o1;
    _kp2 = kp2;
    _omega_o2 = omega_o2;
    updateBetaGains();
}

void ADRCController::reset()
{
    _Z1.setZero();
    _Z2.setZero();
    _Wx_vec.setZero();
    _DeltaXe1_vec.setZero();
    _Gamma1.setZero();
    _A.setZero();
    _A_inv.setZero();

    _nn = 0;
    _Xe1 = 0.f;
    _Xe2 = 0.f;
    _Wx_meas = 0.f;
    _Wx_p = 0.f;
    _Uc = 0.f;
    _Z1_dot_prev[0] = 0.f;
    _Z1_dot_prev[1] = 0.f;
}

void ADRCController::updateBetaGains()
{
    _beta11 = 2.f * _omega_o1;
    _beta12 = _omega_o1 * _omega_o1;
    _Lc1(0, 0) = _beta11;
    _Lc1(1, 0) = _beta12;

    _beta21 = 2.f * _omega_o2;
    _beta22 = _omega_o2 * _omega_o2;
    _Lc2(0, 0) = _beta21;
    _Lc2(1, 0) = _beta22;
}

void ADRCController::initWeightMatrix()
{
    _Q1.setZero();
    float sum_log = 0.f;
    for (int i = 1; i <= m; i++) sum_log += std::log((float)i);
    for (int i = 0; i < m; i++) _Q1(i, i) = std::log((float)(i + 1)) / sum_log;
}

void ADRCController::update(float P_current, float P_target, float Wx_meas, float dt)
{
    const float tau = math::constrain(dt, 0.0005f, 0.02f);

    ++_nn;
    _Wx_meas = Wx_meas;
    _Xe1 = P_target - P_current;

    // 位置环 ESO
    Matrix<float, 2, 2> Ap1{}; Ap1.setZero(); Ap1(0, 1) = 1.f;
    Matrix<float, 2, 1> Bp1{}; Bp1.setZero(); Bp1(0, 0) = -1.f;
    Matrix<float, 1, 2> Cp1{}; Cp1.setZero(); Cp1(0, 0) = 1.f;

    Matrix<float, 2, 1> Z1_dot{};
    Z1_dot = (Ap1 - _Lc1 * Cp1) * _Z1 + Bp1 * ((_nn > 1) ? _Wx_meas : 0.f) + _Lc1 * _Xe1;

    _Z1_dot_prev[0] = Z1_dot(0, 0);
    _Z1_dot_prev[1] = Z1_dot(1, 0);

    _Z1 = _Z1 + tau * Z1_dot;
    const float Z11 = _Z1(0, 0);
    const float Z12 = _Z1(1, 0);

    // 速度历史
    for (int i = m - 2; i >= 0; i--) _Wx_vec(0, i + 1) = _Wx_vec(0, i);
    _Wx_vec(0, 0) = _Wx_meas;

    // 自适应项（alpha=0 可关闭）
    if (fabsf(_alpha) > 1e-6f && _nn > m) {
	_DeltaXe1_vec = -tau * _Wx_vec * _Q1;
	_A = _DeltaXe1_vec.transpose() * _DeltaXe1_vec;

	bool ok = geninv_c(_A, _A_inv, 1e-6f);

	if (ok) {
		_Gamma1 = (-_alpha * _A_inv * _DeltaXe1_vec.transpose() * _Xe1);
	} else {
		_A_inv.setZero();
		_Gamma1.setZero();         // 键：失败就关闭自适应补偿
		_DeltaXe1_vec.setZero();
	}

	} else {
	_Gamma1.setZero();
	_DeltaXe1_vec.setZero();
	}

    // 期望角速度 Wx_p
    if (fabsf(_alpha) <= 1e-6f) {
        _Wx_p = _kp1 * Z11 + Z12;
    } else {
        const float corr = (_DeltaXe1_vec * _Gamma1)(0, 0);
        const float Wx_ADRC = _kp1 * (Z11 + corr) + Z12;
        const float Wx_IM   = (_Wx_vec * _Gamma1)(0, 0);
        _Wx_p = Wx_ADRC + Wx_IM;
    }

    // 速度环 ESO
    _Xe2 = _Wx_p - _Wx_meas;

    Matrix<float, 2, 2> Ap2{}; Ap2.setZero(); Ap2(0, 1) = 1.f;
    Matrix<float, 2, 1> Bp2{}; Bp2.setZero(); Bp2(0, 0) = -_J0;
    Matrix<float, 1, 2> Cp2{}; Cp2.setZero(); Cp2(0, 0) = 1.f;

    Matrix<float, 2, 2> Dp2{}; Dp2.setZero(); Dp2(0, 0) = _kp1; Dp2(0, 1) = 1.f;

    Matrix<float, 2, 1> Z1_dot_vec{};
    Z1_dot_vec(0, 0) = _Z1_dot_prev[0];
    Z1_dot_vec(1, 0) = _Z1_dot_prev[1];

    Matrix<float, 2, 1> Z2_dot{};
    Z2_dot = (Ap2 - _Lc2 * Cp2) * _Z2
           + Bp2 * ((_nn > 1) ? _Uc : 0.f)
           + _Lc2 * _Xe2
           + Dp2 * Z1_dot_vec;

    _Z2 = _Z2 + tau * Z2_dot;

    if (_Z2(1, 0) > 50) _Z2(1, 0) = 50;
    if (_Z2(1, 0) < -50) _Z2(1, 0) = -50;

    const float Z21 = _Z2(0, 0);
    const float Z22 = _Z2(1, 0);



    _Uc = _Jc_inv * (_kp2 * Z21 + Z22);

    if (!PX4_ISFINITE(_Uc) || !PX4_ISFINITE(_Wx_p)) {
    reset();
    _Uc = 0.f;
    _Wx_p = 0.f;
    }
}
