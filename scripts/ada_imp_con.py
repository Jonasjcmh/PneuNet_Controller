class ada_imp_con( ):
    def __init__(self, dof):
        self.DOF = dof
        self.k_mat = np.mat(np.zeros((self.DOF, self.DOF)))
        self.d_mat = np.mat(np.zeros((self.DOF, self.DOF)))
        self.ff_tau_mat = np.mat(np.zeros((self.DOF, 1)))

        self.q = np.mat(np.zeros((self.DOF, 1)))
        self.q_d = np.mat(np.zeros((self.DOF, 1)))

        self.dq = np.mat(np.zeros((self.DOF, 1)))
        self.dq_d = np.mat(np.zeros((self.DOF, 1)))
        self.a = 0.2
        self.b = 44000.0#35000.0#5.0
        self.k = 0.05#

        #minimal jerk trajectory
        self.ini_ang = 0.0   #q_i
        self.end_ang = 60.0  #q_f
        self.dur = 2.0       #D
        self.pre_ang = 0.0
        
        
    def des_tra(self, t, dt):   ####Trajectory planner### minimum jerk trajectory equation 4 iros2021
        des_ang = self.ini_ang + ( self.end_ang - self.ini_ang ) * ( 10 * np.power(t,3 ) / ( self.dur ** 3 ) - 15 * np.power(t, 4 ) / ( self.dur ** 4 ) +  6 * np.power( t, 5 ) / ( self.dur ** 5 ) )
        deg_vel = (des_ang - self.pre_ang)/dt
        self.pre_ang = np.copy(des_ang)
        return des_ang, deg_vel
    
    
    def gen_pos_err(self):   ## 2018 equ. 3
        #print np.mat(self.arm.q).shape, self.task.ref_traj_ang.shape
        return (self.q - self.q_d)

    def gen_vel_err(self):   ## 2018 equ. 3
        #print np.mat(self.arm.dq).shape, self.task.ref_traj_vel.shape
        return (self.dq - self.dq_d)

    def gen_track_err(self):  ## 2018 equ. 3
        return (self.gen_vel_err() + self.k * self.gen_pos_err())

    def gen_for_factor(self):  ## 2018 equ. 9  adaptation rate
        #return la.norm(self.gen_track_err()) * la.norm(self.gen_track_err())
        return self.a/(1.0 + self.b * la.norm(self.gen_track_err()) * la.norm(self.gen_track_err()))
    
    def update_impedance(self, q, q_d, dq, dq_d):   ### main function for impedance control equation 8 and 9 in 2018 paper
        #print(q.shape)
        self.q = np.mat(np.copy(q)).T
        self.q_d = np.mat(np.copy(q_d)).T
        #print(self.q.shape)

        self.dq = np.mat(np.copy(dq)).T
        self.dq_d = np.mat(np.copy(dq_d)).T

        self.k_mat = (self.gen_track_err() * self.gen_pos_err().T)/self.gen_for_factor()

        self.d_mat = (self.gen_track_err() * self.gen_vel_err().T)/self.gen_for_factor()
        #print self.d_mat

        self.ff_tau_mat = self.gen_track_err() / self.gen_for_factor()
        
        #print self.ff_tau_mat
        #print(self.k_mat.shape)
        #print(la.eigvals(self.k_mat))

        #self.tau = -self.ff_tau_mat - self.k_mat * self.gen_pos_err() - self.d_mat * self.gen_vel_err() + self.task.inter_force
        return self.k_mat, self.d_mat, self.ff_tau_mat
