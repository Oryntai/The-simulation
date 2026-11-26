"""Кинематика эскиза руки Claude: та же, что в controller/src/main.rs.

Координаты стола: начало на оси поворота, z вверх от столешницы, x вперёд.
Наклон инструмента от вертикали: 0 — вверх, 90° — вперёд, 180° — вниз.
"""
import math

H0, A1, A2, D = 0.10, 0.12, 0.12, 0.07
LIMITS_DEG = [(-150, 150), (-15, 95), (-20, 150), (-110, 110), (-170, 170)]


# Точка захвата схвата: фланец 70 мм от оси запястья плюс 37.5 мм до середины губ.
D_GRIP = 0.1075


def forward(q, d=D):
    a1, a2 = q[1], q[1] + q[2]
    a3 = a2 + q[3]
    r = A1 * math.sin(a1) + A2 * math.sin(a2) + d * math.sin(a3)
    z = H0 + A1 * math.cos(a1) + A2 * math.cos(a2) + d * math.cos(a3)
    return (r * math.cos(q[0]), r * math.sin(q[0]), z)


def inverse(x, y, z, tilt_deg=180.0, roll_deg=0.0, d=D):
    """Углы суставов в радианах или None, если точка вне досягаемости или пределов."""
    D = d
    t = math.radians(tilt_deg)
    q0 = math.atan2(y, x)
    r = math.hypot(x, y)
    wr = r - D * math.sin(t)
    wz = z - D * math.cos(t) - H0
    c2 = (wr * wr + wz * wz - A1 * A1 - A2 * A2) / (2 * A1 * A2)
    if abs(c2) > 1:
        return None
    q2 = math.acos(c2)
    q1 = math.atan2(wr, wz) - math.atan2(A2 * math.sin(q2), A1 + A2 * math.cos(q2))
    q = [q0, q1, q2, t - q1 - q2, math.radians(roll_deg)]
    if all(lo - 1e-9 <= math.degrees(v) <= hi + 1e-9 for v, (lo, hi) in zip(q, LIMITS_DEG)):
        return q
    return None


if __name__ == "__main__":
    # Карта досягаемости с инструментом вниз: границы радиуса на разной высоте.
    print("высота, мм | радиус от оси поворота, мм (инструмент вертикально вниз)")
    for z in range(0, 260, 20):
        rs = [r for r in range(0, 400, 2) if inverse(r / 1000, 0, z / 1000)]
        print(f"{z:4d} | " + (f"{min(rs)}–{max(rs)}" if rs else "недостижимо"))
