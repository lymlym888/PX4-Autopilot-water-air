#!/usr/bin/env python3
"""Independent double-precision oracle for the user-authorized z3 cap.

Prints eight-sample golden vectors. Does not import/run firmware controller
implementations or write files. Clip ONLY z3, AFTER simultaneous Euler, before
control. Original unbounded vectors remain available in git history.
"""
import json
import math

LIMIT = 0.25  # rad/s^2, user request 2026-09-12
DT = 0.004
B0, A, K1, K2 = 10.1746, -0.5, 26.0, 9.8


def clip(x, lo, hi):
    return min(hi, max(lo, x))


def sign(x):
    return (x > 0) - (x < 0)


def fal(e, alpha, delta):
    return e / delta**(1-alpha) if abs(e) <= delta else abs(e)**alpha * sign(e)


def fhan(x1, x2, r, h):
    d = r*h*h
    a0 = h*x2
    y = x1+a0
    a2 = a0 + sign(y)*(math.sqrt(d*(d+8*abs(y)))-d)/2
    sy = (sign(y+d)-sign(y-d))/2
    a = (a0+y-a2)*sy+a2
    sa = (sign(a+d)-sign(a-d))/2
    return -r*(a/d-sign(a))*sa-r*sign(a)


def vector(kind):
    z1, z2, z3, z4 = 0.05, -0.02, 0.0, 0.0
    v1, v2, rho = 0.0, 0.0, 0.16
    result = []
    for i in range(8):
        phi = 0.05 + 0.01*i + (0.003 if i % 2 else -0.002)
        previous = -0.08+0.025*i
        ref, ref_dot, ref_ddot = 0.12, 0.015, -0.03
        eo = phi-z1
        if kind == 'Nadrc':
            acceleration = fhan(v1-ref, v2, 20.0, 2*DT)
            v1, v2 = v1+DT*v2, v2+DT*acceleration
            d1 = z2-60*(z1-phi)
            d2 = z3-1200*fal(z1-phi, 0.5, 0.01)+B0*previous
            d3 = -8000*fal(z1-phi, 0.25, 0.01)
            d4 = 0.0
        else:
            wo = 25.0 if kind == 'Ladrc' else 22.0
            if kind == 'Amceo':
                sigma_e = clip(abs(z1-ref)/0.0523598776+abs(z2-ref_dot)/0.698131701, 0, 1)
                sigma_u = clip(abs(previous)/0.4, 0, 1)
                rho_star = clip(0.16+0.7*sigma_e-0.01*sigma_u, 0, 1)
                rho = clip(rho+DT/0.055*(rho_star-rho), 0, 1)
                wo = 18+28*rho
            if kind == 'Ladrc':
                d1, d2, d3, d4 = z2+3*wo*eo, B0*previous+z3+3*wo**2*eo, wo**3*eo, 0.0
            else:
                d1, d2 = z2+4*wo*eo, A*z2+B0*previous+z3+6*wo**2*eo
                d3, d4 = z4+4*wo**3*eo, wo**4*eo
        z1, z2, z3, z4 = z1+DT*d1, z2+DT*d2, clip(z3+DT*d3, -LIMIT, LIMIT), z4+DT*d4
        if kind == 'Nadrc':
            e1, e2 = v1-z1, v2-z2
            u0 = K1*fal(e1, 0.8, 0.01)+K2*fal(e2, 1.2, 0.01)
            row = [v1, v2, acceleration, z1, z2, z3, e1, e2, u0, (u0-z3)/B0]
        else:
            e1, e2 = z1-ref, z2-ref_dot
            u = (ref_ddot-K1*e1-K2*e2-(0 if kind == 'Ladrc' else A*z2)-z3)/B0
            row = [z1, z2, z3]
            if kind != 'Ladrc':
                row += [z4]
            if kind == 'Amceo':
                row += [rho, rho_star, sigma_e, sigma_u, wo]
            row += [e1, e2, u]
        assert all(math.isfinite(x) for x in row)
        result.append(row)
    return result


if __name__ == '__main__':
    print(json.dumps({kind: vector(kind) for kind in ['Ladrc', 'Mceo', 'Amceo', 'Nadrc']}, indent=2))
