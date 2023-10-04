import numpy as np
import scipy.linalg
import cv2

# 卡尔曼滤波
class KalmanFilter(object):
    """
    A simple Kalman filter for tracking bounding boxes in image space.

    The 8-dimensional state space

        x, y, a, h, vx, vy, va, vh

    contains the bounding box center position (x, y), aspect ratio a, height h,
    and their respective velocities.

    Object motion follows a constant velocity model. The bounding box location
    (x, y, a, h) is taken as direct observation of the state space (linear
    observation model).

    """

    def __init__(self):
        ndim, dt = 4, 1.

        # Create Kalman filter model matrices.
        self._motion_mat = np.eye(2 * ndim, 2 * ndim)  # 初始化动态转移矩阵, shape(8, 8)
        for i in range(ndim):
            self._motion_mat[i, ndim + i] = dt
        self._update_mat = np.eye(ndim, 2 * ndim)   # 初始化映射矩阵，shape(4, 8)

        # Motion and observation uncertainty are chosen relative to the current
        # state estimate. These weights control the amount of uncertainty in
        # the model. This is a bit hacky.
        self._std_weight_position = 1. / 20
        self._std_weight_velocity = 1. / 160

    def initiate(self, measurement):
        """Create track from unassociated measurement.

        Parameters
        ----------
        measurement : ndarray
            Bounding box coordinates (x, y, a, h) with center position (x, y),
            aspect ratio a, and height h.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector (8 dimensional) and covariance matrix (8x8
            dimensional) of the new track. Unobserved velocities are initialized
            to 0 mean.

        """
        mean_pos = measurement
        mean_vel = np.zeros_like(mean_pos)
        mean = np.r_[mean_pos, mean_vel]   # 状态向量，shape(1, 8)

        std = [
            2 * self._std_weight_position * measurement[3],
            2 * self._std_weight_position * measurement[3],
            1e-2,
            2 * self._std_weight_position * measurement[3],
            10 * self._std_weight_velocity * measurement[3],
            10 * self._std_weight_velocity * measurement[3],
            1e-5,
            10 * self._std_weight_velocity * measurement[3]]
        covariance = np.diag(np.square(std))    # 状态向量协方差矩阵， shape(8, 8)
        return mean, covariance

    def predict(self, mean, covariance):
        """Run Kalman filter prediction step.

        Parameters
        ----------
        mean : ndarray
            The 8 dimensional mean vector of the object state at the previous
            time step.
        covariance : ndarray
            The 8x8 dimensional covariance matrix of the object state at the
            previous time step.

        Returns
        -------
        (ndarray, ndarray)
            Returns the mean vector and covariance matrix of the predicted
            state. Unobserved velocities are initialized to 0 mean.

        """
        std_pos = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-2,
            self._std_weight_position * mean[3]]
        std_vel = [
            self._std_weight_velocity * mean[3],
            self._std_weight_velocity * mean[3],
            1e-5,
            self._std_weight_velocity * mean[3]]
        motion_cov = np.diag(np.square(np.r_[std_pos, std_vel]))

        mean = np.dot(self._motion_mat, mean)  # 动态转移矩阵*状态向量
        covariance = np.linalg.multi_dot((
            self._motion_mat, covariance, self._motion_mat.T)) + motion_cov   # 动态转移矩阵*状态向量协方差矩阵 + 噪声矩阵

        return mean, covariance

    def project(self, mean, covariance):
        """Project state distribution to measurement space.

        Parameters
        ----------
        mean : ndarray
            The state's mean vector (8 dimensional array).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).

        Returns
        -------
        (ndarray, ndarray)
            Returns the projected mean and covariance matrix of the given state
            estimate.

        """
        std = [
            self._std_weight_position * mean[3],
            self._std_weight_position * mean[3],
            1e-1,
            self._std_weight_position * mean[3]]
        innovation_cov = np.diag(np.square(std))

        mean = np.dot(self._update_mat, mean)   # 映射矩阵*状态向量
        covariance = np.linalg.multi_dot((
            self._update_mat, covariance, self._update_mat.T))
        return mean, covariance + innovation_cov         # 映射矩阵*状态向量 + 噪声矩阵

    def update(self, mean, covariance, measurement):
        """Run Kalman filter correction step.

        Parameters
        ----------
        mean : ndarray
            The predicted state's mean vector (8 dimensional).
        covariance : ndarray
            The state's covariance matrix (8x8 dimensional).
        measurement : ndarray
            The 4 dimensional measurement vector (x, y, a, h), where (x, y)
            is the center position, a the aspect ratio, and h the height of the
            bounding box.

        Returns
        -------
        (ndarray, ndarray)
            Returns the measurement-corrected state distribution.

        """
        projected_mean, projected_cov = self.project(mean, covariance)

        chol_factor, lower = scipy.linalg.cho_factor(
            projected_cov, lower=True, check_finite=False)    # Cholesky分解
        kalman_gain = scipy.linalg.cho_solve(
            (chol_factor, lower), np.dot(covariance, self._update_mat.T).T,
            check_finite=False).T                             # 求解卡尔曼增益矩阵
        innovation = measurement - projected_mean

        new_mean = mean + np.dot(innovation, kalman_gain.T)    # 预测值和测量值融合后，新的状态向量
        new_covariance = covariance - np.linalg.multi_dot((
            kalman_gain, projected_cov, kalman_gain.T))        # 预测值和测量值融合后，新状态向量的协方差矩阵
        return new_mean, new_covariance

    def gating_distance(self, mean, covariance, measurements,
                        only_position=False):
        """Compute gating distance between state distribution and measurements.

        A suitable distance threshold can be obtained from `chi2inv95`. If
        `only_position` is False, the chi-square distribution has 4 degrees of
        freedom, otherwise 2.

        Parameters
        ----------
        mean : ndarray
            Mean vector over the state distribution (8 dimensional).
        covariance : ndarray
            Covariance of the state distribution (8x8 dimensional).
        measurements : ndarray
            An Nx4 dimensional matrix of N measurements, each in
            format (x, y, a, h) where (x, y) is the bounding box center
            position, a the aspect ratio, and h the height.
        only_position : Optional[bool]
            If True, distance computation is done with respect to the bounding
            box center position only.

        Returns
        -------
        ndarray
            Returns an array of length N, where the i-th element contains the
            squared Mahalanobis distance between (mean, covariance) and
            `measurements[i]`.

        """
        mean, covariance = self.project(mean, covariance)
        if only_position:
            mean, covariance = mean[:2], covariance[:2, :2]
            measurements = measurements[:, :2]

        cholesky_factor = np.linalg.cholesky(covariance)
        d = measurements - mean
        z = scipy.linalg.solve_triangular(
            cholesky_factor, d.T, lower=True, check_finite=False,
            overwrite_b=True)
        squared_maha = np.sum(z * z, axis=0)
        return squared_maha


if __name__ == "__main__":
    kf = KalmanFilter()
    img = cv2.imread("./car3.png")
    #
    # < xmin > 1 < / xmin >
    # < ymin > 225 < / ymin >
    # < xmax > 223 < / xmax >
    # < ymax > 381 < / ymax >

    cv2.rectangle(img, (1, 225),
                  (223, 381), (0, 255, 0), 5)   # 用绿色框，绘制小车移动后的真实位置

    # 小车的初始化测量值
    m = np.array([112, 303, 1.423, 156]) 
    mean_ini, covariance_ini = kf.initiate(m)  # 初始化卡尔曼的动态转移矩阵，映射矩阵

    for i in range(15):
        mean_pre, covariance_pre = kf.predict(mean_ini, covariance_ini)  # 预测状态变量
        dx = np.random.randint(0, 240)
        # dy = np.random.randint(0, 120)
        dy = np.random.randint(-10, 30)
        # dx = np.random.randint(-30, 30)
        m = m + np.array([dx, dy, 0, 0])   # 随机向左，和上下移动
        cv2.rectangle(img, (int(m[0]-m[2]*m[3]/2), int(m[1]-m[3]/2)),
                      (int(m[0]+m[2]*m[3]/2), int(m[1]+m[3]/2)), (0, 255, 0), 5)  # 用绿色框，绘制小车移动后的真实位置

        mean_upd, covariance_upd = kf.update(mean_pre, covariance_pre, m+np.random.randn(4))   # 利用测量值，更新状态变量
        mean_ini = mean_upd
        covariance_ini = covariance_upd

        # 用红色框，绘制小车移动后的估计位置
        cv2.rectangle(img, (int(mean_ini[0] - mean_ini[2] * mean_ini[3] / 2), int(mean_ini[1] - mean_ini[3] / 2)),
                      (int(mean_ini[0] + mean_ini[2] * mean_ini[3] / 2), int(mean_ini[1] + mean_ini[3] / 2)), (0, 0, 255), 5)

        # cv2.waitKey(30)

    cv2.namedWindow("img", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("img", 960, 540)
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()