import numpy as np

class Params(object):
    def __init__(self):
        # ======== problem domain parameters ========
        self.Dim_mode = 2 # [RF mode, VIO mode]
        self.Dim_VIO = 7 # *not used now...
        self.Dim_RF = 1

        # ======== user setting parameters ========
        self.GatePerDim_onfloor = 0.2**2
        self.GatePerDim_onobstacle = 0.2**2 # UAV should not change height abruptly (dz < 2 m/s)
        self.GateLevel_VIO = self.GatePerDim_onfloor**self.Dim_VIO
        self.GateLevel_RF = self.GatePerDim_onfloor**self.Dim_RF
        self.GateLevel_RF_onobstacle = self.GatePerDim_onobstacle**self.Dim_RF

        # confusion matrix
        self.confusion_matrix_user0 = np.array([[0.8, 0.1], [0.2, 0.9]])
        self.confusion_matrix_user1 = np.array([[1.0, 1.0], [0.0, 0.0]])
        self.confusion_matrix_user2 = np.array([[0.0, 0.0], [1.0, 1.0]])
        self.confusion_matrix = self.confusion_matrix_user0

        # VIO to rangefinder transition speed on obstacle
        self.del_weight_rf_vio = 0.05

        # VIO bias buffer
        self.bufflen_VIO_bias = 70

        # moving average buffer length
        self.num_MA_buf = 7

        # measurement selection policy
        self.meas_selec_policy = 0 # 0: automatic // 1: rangefinder only // 2: VIO only

        # minimum height to run algorithm
        self.min_height = 1.0
