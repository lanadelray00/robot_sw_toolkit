import serial
import time
import math

# === 설정 ===
# 링크 길이 (cm 등 단위 통일)
L1 = 1.4   # base → shoulder 높이
L2 = 8.5  # upper arm
L3 = 9.0  # lower arm

# 조인트 가동 범위 (degree)
joint_limits = {
    'theta1': (0, 180),     # base 회전
    'theta2': (15, 165),    # shoulder
    'theta3': (0, 180),     # elbow
}

# === 함수 ===

def ik_3dof(x, y, z):
    # 1. base 회전각
    theta1 = math.atan2(y, x)  # rad
    theta1_deg = math.degrees(theta1)

    # 2. 평면 거리 계산
    r = math.hypot(x, y)
    dz = z - L1
    d = math.hypot(r, dz)

    if d > (L2 + L3):
        return None, "Unreachable: 목표 좌표가 팔 길이보다 멉니다."

    try:
        # 3. shoulder (theta2)
        angle_a = math.acos((L2**2 + d**2 - L3**2) / (2 * L2 * d))
        angle_b = math.atan2(dz, r)
        theta2 = angle_a + angle_b
        theta2_deg = math.degrees(theta2)

        # 4. elbow (theta3)
        angle_c = math.acos((L2**2 + L3**2 - d**2) / (2 * L2 * L3))
        theta3_deg = 180 - math.degrees(angle_c)

        # 5. 범위 검사
        for name, angle in zip(['theta1', 'theta2', 'theta3'], [theta1_deg, theta2_deg, theta3_deg]):
            min_angle, max_angle = joint_limits[name]
            if not (min_angle <= angle <= max_angle):
                return None, f"{name}={angle:.1f}°: 가동 범위 초과"

        return (theta1_deg, theta2_deg, theta3_deg), None

    except ValueError:
        return None, "역삼각법 계산 불가 (acos domain 오류)"

# === 사용 예 ===

def test_move(x, y, z):
    result, error = ik_3dof(x, y, z)
    if error:
        print(f"[오류] {error}")
    else:
        t1, t2, t3 = result
        print(f"목표 좌표 ({x}, {y}, {z}) → 각도 θ1={t1:.1f}, θ2={t2:.1f}, θ3={t3:.1f}")

# 테스트
test_move(15, 5, 10)
test_move(25, 0, 10)  # 범위 밖
