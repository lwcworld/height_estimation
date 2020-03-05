import numpy as np

class Params(object):
    def __init__(self):
        # ======== problem domain parameters ========
        self.Dim_mode = 2 # [RF mode, VIO mode]
        self.Dim_VIO = 7 # *not used now...
        self.Dim_RF = 1

        # ======== user setting parameters ========
        self.GateLevel_RF = 0.2**2
        self.GateLevel_RF_onobstacle = 0.2**2 # UAV should not change height abruptly (dz < 2 m/s)

        # confusion matrix
        self.confusion_matrix = np.array([[0.8, 0.2], [0.2, 0.8]])

        # VIO to rangefinder transition speed on obstacle
        self.del_weight_rf_vio = 0.01

        # VIO bias buffer
        self.bufflen_VIO_bias = 5

        # moving average buffer length
        self.num_MA_buf = 5

        # minimum height to run algorithm
        self.min_height = 1.0