import rospy
from std_msgs.msg import Int8, Float64

class log_pub_print():
    def __init__(self):
        self.res_RF_floor = rospy.Publisher('/datalog/res_RF_floor', Float64, queue_size=2)
        self.res_RF_obs = rospy.Publisher('/datalog/res_RF_obs', Float64, queue_size=2)
        self.res_VIO_floor = rospy.Publisher('/datalog/res_VIO_floor', Float64, queue_size=2)
        self.meas_RF = rospy.Publisher('/datalog/meas_RF', Float64, queue_size=2)
        self.bias_VIO_z = rospy.Publisher('/datalog/bias_VIO_z', Float64, queue_size=2)
        self.bias_RF = rospy.Publisher('/datalog/bias_RF', Float64, queue_size=2)
        self.weight_RF_VIO = rospy.Publisher('/datalog/weight_RF_VIO', Float64, queue_size=2)
        self.down_status_prob_floor = rospy.Publisher('/datalog/down_status_prob_floor', Float64, queue_size=2)
        self.down_status_prob_obstacle = rospy.Publisher('/datalog/down_status_prob_obstacle', Float64, queue_size=2)
        self.down_status = rospy.Publisher('/datalog/down_status', Int8, queue_size=2)
        self.meas_used = rospy.Publisher('/datalog/meas_used', Int8, queue_size=2)
        self.time_delay = rospy.Publisher('/datalog/time_delay', Float64, queue_size=2)

    def log_publish_print(self, res_RF_floor, res_RF_obs, res_VIO_floor, meas_RF, bias_VIO_z, bias_RF, weight_RF_VIO, down_status_prob, down_status, meas_used, time_delay, task):
        if task == 1: # publish
            self.res_RF_floor.publish(res_RF_floor)
            self.res_RF_obs.publish(res_RF_obs)
            self.res_VIO_floor.publish(res_VIO_floor)
            self.meas_RF.publish(meas_RF)
            self.bias_VIO_z.publish(bias_VIO_z)
            self.bias_RF.publish(bias_RF)
            self.weight_RF_VIO.publish(weight_RF_VIO)
            self.down_status_prob_floor.publish(down_status_prob[0,0])
            self.down_status_prob_obstacle.publish(down_status_prob[0,1])
            self.down_status.publish(down_status)
            self.meas_used.publish(meas_used)
            self.time_delay.publish(time_delay)
        elif task == 2: # print
            print('---------')
            # residual
            print(['residual_RF_floor  : ', round(res_RF_floor, 3)])
            print(['residual_RF_obstacle  : ', round(res_RF_obs, 3)])
            print(['residual_VIO_floor : ', round(res_VIO_floor, 3)])

            # rangefinder meas
            print(['rangefinder meas : ', round(meas_RF, 3)])

            # bias of VIO & rangefinder
            print(['bias_VIO_z : ', round(bias_VIO_z, 3)])
            print(['bias RF : ', round(bias_RF, 3)])

            # VIO and rangefinder ratio (which is used only when VIO->range transition on the obstacle)
            print(['weight RF vs VIO : ', round(weight_RF_VIO, 3)])

            # downward status estimation
            print(['downward status probability (floor, obstacle) : ', round(down_status_prob[0, 0], 3),
                   round(down_status_prob[0, 1], 3)])
            print(['current downward (floor:0, obstacle:1, transition:2) : ', round(down_status, 0)])

            # measurement source used
            print(['measurement source (rf:0, VIO:1, mixed:2): ', round(meas_used, 0)])

            # time delay
            print(['time delay : ', round(time_delay, 5)])