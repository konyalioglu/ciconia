#! /usr/bin/python3

import numpy as np


class dynamics:

    g = 9.81
    a = 0.530
    b = 0.635
    m = 25

    Ix = 8.638
    Iy = 9.014
    Iz = 16.738

    TRANSITION_BP_NUMBER = 3
    TRANSITION_BP_VT = np.array([[5], [10], [15]])

    FLIGHT_BP_NUMBER = 1
    FLIGHT_BP_VT = np.array([[18]])

    A_QUADROTOR_FULL = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -g, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype='float32')

    B_QUADROTOR_FULL = np.array([[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [-1/m, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, b/Ix, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, a/Iy, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 1/Iz]], dtype='float32')

    A_QUADROTOR_ATTITUTDE = np.array([[0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1],
                                    [0, 0, 0, 0, 0, 0]])
    
    B_QUADROTOR_ATTITUDE = np.array([[0, 0, 0],
                                    [b/Ix, 0, 0],
                                    [0, 0, 0],
                                    [0, a/Iy, 0],
                                    [0, 0, 0],
                                    [0, 0, 1/Iz]])

    C_QUADROTOR_ATTITUDE = np.eye(A_QUADROTOR_ATTITUTDE.shape[0])
    D_QUADROTOR_ATTITUDE = 0

    A_QUADROTOR_UPPER = np.array([[0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0]])
    B_QUADROTOR_UPPER = np.array([[0, 0, 0],
                        [0, 0, -g],
                        [0, 0, 0],
                        [0, g, 0],
                        [0, 0, 0],
                        [-1/m, 0, 0]])

    C_QUADROTOR_UPPER = np.eye(A_QUADROTOR_UPPER.shape[0])
    D_QUADROTOR_UPPER = 0


    TRANSITION_TRIM_STATES = np.array([[10],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]).reshape(10,1)
    TRANSITION_TRIM_CONTROL_STATES = np.array([[0],[2.776],[181.6],[-4.03348834],[0],[0],[0],[0]]).reshape(8,1)
        
    A_TRANSITION_FULL = np.array([[-0.016454,	0.187203,	0,	-9.8065,	0,	0.0046127,	0,	-1.62038e-06,	0,	0],
                        [-0.508427,	-2.31099,	8.82873,	0,	0,	-0.0808179,	1.62038e-06,	0,	0,	0],
                        [0.0881203,	-1.40677,	-3.93851,	0,	0,	0.00708723,	-7.60669e-08,	-1.42194e-07,	0,	0],
                        [0,	0,	1,	0,	0,	0,	0,	0,	1.22792e-07,	0],
                        [0,	-1,	0,	10,	0,	0,	0,	0,	1.62038e-06,	0],
                        [1.3613e-07,	-9.76549e-08,	0,	0,	0,	-0.0942647,	-0.00964732,	-9.83297,	9.8065,	0],
                        [8.6732e-09,	-1.12744e-07,	9.48618e-08,	0,	0,	0.273087,	-6.28216,	1.77522,	0,	0],
                        [-2.77507e-08,	8.23678e-09,	1.95739e-08,	0,	0,	0.267054,	-0.990437,	-0.2658,	0,	0],
                        [0,	0,	0,	-1.22792e-07,	0,	0,	1,	0,	0,	0],
                        [0,	0,	0,	0,	0,	0,	0,	1,	-5.84652e-05,	0]])
                         
    B_TRANSITION_FULL = np.array([[0,	0.04,	0,	0,	0,	0,	0,	0],
                        [-2.65643,	0,	-0.04,	0,	0,	0,	0,	0],
                        [-4.7999,	0,	0,	0.110939,	0,	0,	0,	0],
                        [0,	0,	0,	0,	0,	0,	0,	0],
                        [0,	0,	0,	0,	0,	0,	0,	0],
                        [0,	0,	0,	0,	0,	-0.700145,	0,	0],
                        [0,	0,	0,	0,	-10.6816,	0.196707,	0.117137,	0.00909773],
                        [0,	0,	0,	0,	-0.909796,	1.87101,	0.00909773,	0.0604509],
                        [0,	0,	0,	0,	0,	0,	0,	0],
                        [0,	0,	0,	0,	0,	0,	0,	0]])


    C_TRANSITION_FULL = np.eye(A_TRANSITION_FULL.shape[0])
    D_TRANSITION_FULL = np.zeros((C_TRANSITION_FULL.shape[0], B_TRANSITION_FULL.shape[1]))  

    A_TRANSITION_LONGITUDINAL = A_TRANSITION_FULL[:5,:5]
    B_TRANSITION_LONGITUDINAL = B_TRANSITION_FULL[:5,:4]
    C_TRANSITION_LONGITUDINAL = np.eye(A_TRANSITION_LONGITUDINAL.shape[0])
    D_TRANSITION_LONGITUDINAL = np.zeros((C_TRANSITION_LONGITUDINAL.shape[0], B_TRANSITION_LONGITUDINAL.shape[1]))  

    A_TRANSITION_LATERAL = A_TRANSITION_FULL[5:,5:]
    B_TRANSITION_LATERAL = B_TRANSITION_FULL[5:,4:]
    C_TRANSITION_LATERAL = np.eye(A_TRANSITION_LATERAL.shape[0])
    D_TRANSITION_LATERAL = np.zeros((C_TRANSITION_LATERAL.shape[0], B_TRANSITION_LATERAL.shape[1])) 

    A_TRANSITION_LONGITUDINAL_ATT = A_TRANSITION_FULL[:4,:4]
    B_TRANSITION_LONGITUDINAL_ATT = B_TRANSITION_FULL[:4,:2]
    C_TRANSITION_LONGITUDINAL_ATT = np.eye(A_TRANSITION_LONGITUDINAL_ATT.shape[0])
    D_TRANSITION_LONGITUDINAL_ATT = np.zeros((C_TRANSITION_LONGITUDINAL_ATT.shape[0], B_TRANSITION_LONGITUDINAL_ATT.shape[1])) 

    A_TRANSITION_LATERAL_ATT = A_TRANSITION_FULL[5:9,5:9]
    B_TRANSITION_LATERAL_ATT = B_TRANSITION_FULL[5:9,2:]
    C_TRANSITION_LATERAL_ATT = np.eye(A_TRANSITION_LATERAL_ATT.shape[0])
    D_TRANSITION_LATERAL_ATT = np.zeros((C_TRANSITION_LATERAL_ATT.shape[0], B_TRANSITION_LATERAL_ATT.shape[1]))  


    FLIGHT_TRIM_STATES = np.array([[18],[0.3468],[0],[1.9266e-02],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
    FLIGHT_TRIM_CONTROL_STATES = np.array([[3.57596876e-02],[4.0],[0],[0]]).reshape(4,1)
        
    FLIGHT_TRIM_STATES = np.array([[18],[0.281],[0],[1.567e-02],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
    FLIGHT_TRIM_CONTROL_STATES = np.array([[4.638e-02],[9.0],[0],[0]]).reshape(4,1)
        
    A_FLIGHT_FULL = np.array([[  -0.0249373,	0.434151,	-0.306821,	-9.80468,	0,	0,	0,	0,	0,	0],
                            [   -1.0119,	-4.03576,	15.9232,	-0.188924,	0,	0.0,	0,	0,	0,	0],
                            [   0.0480231,	-2.5199,	-6.97358,	0,	0,	0,	0,	0,	0,	0],
                            [   0,	0,	1,	0,	0,	0,	0,	0,	0,	0],
                            [   0.0192652,	-0.999814,	0,	18,	0,	0,	0,	0,	-1.49012e-07,	0],
                            [   1.35817e-09,	-3.16569e-09,	0,	0,	0,	-0.172437,	0.329692,	-17.7009,	9.80468,	0],
                            [   -4.43797e-09,	9.09345e-09,	5.28334e-11,	0,	0,	0.505768,	-11.1232,	3.14313,	0,	0],
                            [   -3.85613e-09,	-2.59574e-09,	1.09466e-11,	0,	0,	0.465515,	-1.75368,	-0.470645,	0,	0],
                            [   0,	0,	0,	-6.86224e-11,	0,	0,	1,	0.0192688,	1.24375e-12,	0],
                            [   0,	0,	0,	-1.32203e-12,	0,	0,	0,	1.00019,	6.45594e-11,	0]]) 

    B_FLIGHT_FULL = np.array([[  0.163105,	0.04,	0,	0],
                            [  -8.46473,	0,	0,	0],
                            [  -15.2978,	0,	0,	0],
                            [  0,	0,	0,	0],
                            [  0,	0,	0,	0],
                            [  0,	0,	0,	-2.23143],
                            [  0,	0,	-34.0432,	0.626925],
                            [  0,	0,	-2.89961,	5.96312],
                            [  0,	0,	0,	0],
                            [  0,	0,	0,	0]])

    C_FLIGHT_FULL = np.eye(A_FLIGHT_FULL.shape[0])
    D_FLIGHT_FULL = np.zeros((C_FLIGHT_FULL.shape[0], B_FLIGHT_FULL.shape[1]))  


    A_FLIGHT_LONGITUDINAL = A_FLIGHT_FULL[:5,:5]
    B_FLIGHT_LONGITUDINAL = B_FLIGHT_FULL[:5,:2]
    C_FLIGHT_LONGITUDINAL = np.eye(A_FLIGHT_LONGITUDINAL.shape[0])
    D_FLIGHT_LONGITUDINAL = np.zeros((C_FLIGHT_LONGITUDINAL.shape[0], B_FLIGHT_LONGITUDINAL.shape[1]))

    A_FLIGHT_LATERAL = A_FLIGHT_FULL[5:,5:]
    B_FLIGHT_LATERAL = B_FLIGHT_FULL[5:,2:]
    C_FLIGHT_LATERAL = np.eye(A_FLIGHT_LATERAL.shape[0])
    D_FLIGHT_LATERAL = np.zeros((C_FLIGHT_LATERAL.shape[0], B_FLIGHT_LATERAL.shape[1]))

    A_FLIGHT_LONGITUDINAL_ATT = A_FLIGHT_FULL[:4,:4]
    B_FLIGHT_LONGITUDINAL_ATT = B_FLIGHT_FULL[:4,:2]
    C_FLIGHT_LONGITUDINAL_ATT = np.eye(A_FLIGHT_LONGITUDINAL_ATT.shape[0])
    D_FLIGHT_LONGITUDINAL_ATT = np.zeros((C_FLIGHT_LONGITUDINAL_ATT.shape[0], B_FLIGHT_LONGITUDINAL_ATT.shape[1]))

    A_FLIGHT_LATERAL_ATT = A_FLIGHT_FULL[5:9,5:9]
    B_FLIGHT_LATERAL_ATT = B_FLIGHT_FULL[5:9,2:]
    C_FLIGHT_LATERAL_ATT = np.eye(A_FLIGHT_LATERAL_ATT.shape[0])
    D_FLIGHT_LATERAL_ATT = np.zeros((C_FLIGHT_LATERAL_ATT.shape[0], B_FLIGHT_LATERAL_ATT.shape[1]))

    TRANSITION_CL_TRIM = 0.695
    FLIGHT_CL_TRIM = 0.669
    FLIGHT_CLa_TRIM = 6.079
    TRANSITION_CLq_TRIM = 15.858
    FLIGHT_CLDe = 0.69952
    TRANSITION_CD0 = 0.0288
    TRANSITION_CDa = 0.4688
    TRANSITION_CDq = 0.26
    TRANSITION_Cm0 = 0.1091 
    TRANSITION_Cma = -3.4657
    TRANSITION_Cmq = -49.4246



if __name__ == '__main__':

    eq = dynamics()

    print(eq.TRANSITION_Cmq)
