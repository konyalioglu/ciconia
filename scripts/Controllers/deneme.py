
import numpy as np
import scipy.linalg
from scipy import signal



def stable_unstable_decomposition(A):
    A = np.array(A)
    eigVals, eigVecs = scipy.linalg.eig(A)
    #print(A)
    
    Ahat =  np.linalg.inv(A + np.eye(A.shape[0])) @ (A - np.eye(A.shape[0]))
    #print(Ahat)
    
    
    #Step 1: Determine the location of poles
    u_poles = []
    s_poles = []
    unstability = False
    for eigval in eigVals:
        if abs(eigval) > 1:
            unstability = True
            u_poles.append(eigval)
            
    if unstability == False:
        Talpha = np.eye(A.shape[0])
        Astar  = A
    else:
        alpha = abs(min(u_poles))
        Aahat = np.linalg.inv(2/(1+alpha)*A + np.eye(A.shape[0])) @ (2/(1+alpha)*A - np.eye(A.shape[0]))
        eigVals, eigVecs = scipy.linalg.eig(Aahat)
        print(Aahat)
        print(u_poles)
        T = eigVecs.copy()
        
        print(np.linalg.inv(T)@ A @ T)
             
    return



def infinite_horizon(self, V, J):
    '''
    In order to guarantee the stability or provide stability for the system, 
    MPC problem can be converted into the infinite horizon control problem.
    
    V : Jacobian Transformation Matrix
    J : Diagonally Grouped Eigenvalue Matrix (Sorted regarding stability)
    where
        A = V*J/V
    '''
    
    isstable = True
    ind = 0
    for i in range(J.shape[0]):
        if abs(J[i,i]) > 1:
            isstable = False
            u_poles.append(eigval)
            break
        ind += 1
        
    
    if isstable == True:
        P  = np.transpose(Cd) @ Q @ Cd
        Qb = scipy.linalg.solve_discrete_lyapunov(Ad, P)
        return isstable, Qb, None
    else:
        Ws = V[:,0:ind] 
        Wu = V[:,ind:]
        Js = V[0:ind,0:ind] 
        Ju = V[ind:, ind:]
        
        iV  = np.linalg.inv(V)
        Wbs = V[0:ind,:]
        Wbu = V[ind:,:]
        P   = np.transpose(Ws) @ np.transpose(self.Cd) @ self.Q @ self.Cd @ Ws
        pi  = scipy.linalg.solve_discrete_lyapunov(self.Ad, P)
        Qb  = np.transpose(Wbs) @ pi @ Wbs
        return isstable, Qb, Wbs
    

A_flight = np.array([[  -0.0249373,	0.434151,	-0.306821,	-9.80468,	0,	-0.0134076,	0,	1.5049e-07,	0,	0],
                     [   -1.0119,	-4.03576,	15.9232,	-0.188924,	0,	0.201211,	-1.5049e-07,	0,	0,	0],
                     [   0.0480231,	-2.5199,	-6.97358,	0,	0,	0.0223937,	-4.21831e-11,	-8.0406e-11,	0,	0],
                     [   0,	0,	1,	0,	0,	0,	0,	0,	6.85969e-11,	0],
                     [   0.0192652,	-0.999814,	0,	18,	0,	0,	0,	0,	-1.49012e-07,	0],
                     [   1.35817e-09,	-3.16569e-09,	0,	0,	0,	-0.172437,	0.329692,	-17.7009,	9.80468,	0],
                     [   -4.43797e-09,	9.09345e-09,	5.28334e-11,	0,	0,	0.505768,	-11.1232,	3.14313,	0,	0],
                     [   -3.85613e-09,	-2.59574e-09,	1.09466e-11,	0,	0,	0.465515,	-1.75368,	-0.470645,	0,	0],
                     [   0,	0,	0,	-6.86224e-11,	0,	0,	1,	0.0192688,	1.24375e-12,	0],
                     [   0,	0,	0,	-1.32203e-12,	0,	0,	0,	1.00019,	6.45594e-11,	0]]) 
                     
B_flight = np.array([[  0.163105,	0.04,	0,	0],
                     [  -8.46473,	0,	0,	0],
                     [  -15.2978,	0,	0,	0],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	-2.23143],
                     [  0,	0,	-34.0432,	0.626925],
                     [  0,	0,	-2.89961,	5.96312],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	0]])
                     
C_flight = np.eye(A_flight.shape[0])
D_flight = np.zeros((A_flight.shape[0], B_flight.shape[1]))   


A = np.array([[34.2, 1.8, 8.2, -16.6],[-64.4, -1.6, -16.4, 33.2],
                     [4.2, -0.2, 2.2, -2.6],[61.6, 0.4, 14.6, -32.8]])

sys  = signal.StateSpace(A_flight, B_flight, C_flight, D_flight)
sysd = sys.to_discrete(100)


A = np.array([[34.2, 1.8, 8.2, -16.6],[-64.4, -1.6, -16.4, 33.2],
                     [4.2, -0.2, 2.2, -2.6],[61.6, 0.4, 14.6, -32.8]])
                     
A = np.array([[1.6, 6, 7.4, 6.2, 2.2],[-2.38, -4.45, -7.27, -6.76, -2.66],
                     [4.42, 9.05, 13.43, 11.84, 4.96],[-2.06, -4.65, -6.49, -5.62,-2.42],
                     [-2.28, -8.2, -10.62, -9.06, -2.96]])



Z = scipy.linalg.schur(sysd.A, output='real', sort='ouc')
E,V = scipy.linalg.eig(sysd.A)

soru = Z[1] @ Z[0] @ np.transpose(Z[1])
EigVals,EigVec = scipy.linalg.eig(Z[0])
cevap = sysd.A
#print(E, '\n')
#print(Z)

'''
#print(E)
#stable_unstable_decomposition(A)

A = np.array([[0.6543, -0.5013, 0.2865],[1.0, 0, 0],[0, 1.0, 0]])
Cz = np.diag(np.array([[-0.0536, 0.5775, 0.5188]]))
Q = np.eye(3)

q = -np.transpose(Cz) @ Q @ Cz
xx = scipy.linalg.solve_discrete_lyapunov(A,Q)
print(xx)
'''






