import rospy
import numpy as np
import quaternion
import tf
from sensor_msgs.msg import Imu

class Attitude:
    def __init__(self):
        self.it = 0
        self.time_prev = None
        self.omega_prev = None
        self.data = None
        self.attitude_est = self.identity_quaternion()
        self.br = tf.TransformBroadcaster()
        self.bias = [[], [], []]
        self.b = [0, 0, 0]

    @staticmethod
    def cross_matrix(w):
        return np.array([
            [    0, -w[2],  w[1]],
            [ w[2],     0, -w[0]],
            [-w[1],  w[0],     0]
        ])

    @staticmethod
    def bigOmegaMatrix(w):
        m = np.zeros((4, 4))
        m[1:, 0] = w
        m[0, 1:] = -1 * np.array(w)
        m[1:, 1:] = -1 * Attitude.cross_matrix(w)
        return m

    @staticmethod
    def identity_quaternion():
        return np.quaternion(1, 0, 0, 0)

    @staticmethod
    def q2r(q):
        return np.asarray([
            [q.w**2 + q.x**2 - q.y**2 - q.z**2,             2*(q.x*q.y - q.w*q.z),             2*(q.x*q.z + q.w*q.y)],
            [            2*(q.x*q.y + q.w*q.z), q.w**2 - q.x**2 + q.y**2 - q.z**2,             2*(q.y*q.z - q.w*q.x)],
            [            2*(q.x*q.z - q.w*q.y),             2*(q.y*q.z + q.w*q.x), q.w**2 - q.x**2 - q.y**2 + q.z**2]
        ])

    def data_callback(self, data):
        vel = data.angular_velocity
        if vel.x == 0 and vel.y == 0 and vel.z == 0:
            return
        print(vel)
        self.data = data
        if self.it < 500:
            self.bias[0].append(vel.x)
            self.bias[1].append(vel.y)
            self.bias[2].append(vel.z)
            self.after_iteration()
            return
        elif self.it == 500:
            self.b[0] = sum(self.bias[0]) / 500
            self.b[1] = sum(self.bias[1]) / 500
            self.b[2] = sum(self.bias[2]) / 500
        self.data.angular_velocity.x -= self.b[0]
        self.data.angular_velocity.y -= self.b[1]
        self.data.angular_velocity.z -= self.b[2]
        self.update()
        self.publish_estimates()
        print(self.data.linear_acceleration)
        self.after_iteration()

    def after_iteration(self):
        self.time_prev = self.data.header.stamp
        self.omega_prev = self.data.angular_velocity
        self.it += 1

    def publish_estimates(self):
        est = self.attitude_est
        self.br.sendTransform(
            (0, 0, 0),
            np.array([est.x, est.y, est.z, est.w]),
            rospy.Time.now(),
            "to-frame",
            "odom_combined")

    def delta_t(self):
        return (self.data.header.stamp - self.time_prev).to_sec()

    def update(self):
        perturb = self.calc_local_attitude_perturbation()
        self.attitude_est = self.attitude_est * perturb

        print(self.q2r(self.attitude_est).dot(np.array([0, 0, 1])))

    def calc_local_attitude_perturbation(self):
        q_id = self.identity_quaternion()
        q_0 = [q_id.w, q_id.x, q_id.y, q_id.z]

        omega = self.data.angular_velocity
        omega_vec = np.array([omega.x, omega.y, omega.z])
        omega_prev_vec = np.array([self.omega_prev.x, self.omega_prev.y, self.omega_prev.z])

        omega_mean_vec = (omega_vec + omega_prev_vec) / 2

        k_1 = 0.5 * self.bigOmegaMatrix(omega_prev_vec).dot(q_0)
        k_2 = 0.5 * self.bigOmegaMatrix(omega_mean_vec).dot(q_0 + (self.delta_t() / 2)*k_1)
        k_3 = 0.5 * self.bigOmegaMatrix(omega_mean_vec).dot(q_0 + (self.delta_t() / 2)*k_2)
        k_4 = 0.5 * self.bigOmegaMatrix(omega_vec).dot(q_0 + self.delta_t()*k_3)

        perturb_vec = q_0 + self.delta_t() / 6 * (k_1 + 2*k_2 + 2*k_3 + k_4)
        perturb = np.quaternion(*perturb_vec)
        return perturb.normalized()

    def run(self):
        rospy.init_node('orientation', anonymous=True)
        rospy.Subscriber("iosmsg/imu_data", Imu, self.data_callback)
        print('Spin ...')
        rospy.spin()


def main():
    attitude = Attitude()
    attitude.run()


if __name__ == '__main__':
    main()
