import numba as nb
# 用于限制一个数的范围


@nb.jit()
def clamp(num, limit1, limit2):
    return max(min(num, max(limit1, limit2)), min(limit1, limit2))


# PID控制器
class PID:
    def __init__(self, kp, ki, kd, integral_output_limit = 1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral_output_limit = integral_output_limit
        self.integral = 0
        self.error_prev = 0

    def update(self, error, dt):
        # 计算 P 分量
        p = error * self.kp
        # 计算 I 分量
        self.integral += error * dt * self.ki
        self.integral = clamp(self.integral, self.integral_output_limit, -self.integral_output_limit)
        i = self.integral
        # 计算 D 分量
        d = (error - self.error_prev) / dt * self.kd
        self.error_prev = error
        # 加起来得到结果
        return p + i + d

    def reset(self):
        self.integral = 0
        self.error_prev = 0
