import numpy as np
import kinematics2 as kinematics
import time


import VREP_ROBOT3

class VREP_KUKA_BASIC(VREP_ROBOT3.VREP_ROBOT):
    '''
    This class implements the methods that are particular for the robot at the scene of a single KUKA arm moving amongst obstacles.
    This class inherits from VREP_ROBOT3.VREP_ROBOT which is a class implementing the common functionalities of v-rep.
    '''

    def __init__(self, childscriptname, ipaddress='127.0.0.1', port=19999, clientID=-999):
        VREP_ROBOT3.VREP_ROBOT.__init__(self, childscriptname, ipaddress='127.0.0.1', port=19997, clientID=-999, ik_name_list=['ik'])

        self.joint_handles = [self.get_handle('j1'), self.get_handle('j2'),
                              self.get_handle('j3'), self.get_handle('j4'),
                              self.get_handle('j5'), self.get_handle('j6'),
                              self.get_handle('j7')
                             ]

        #self.get_vrep_IK_handle('ik')

        T_oct = np.eye(4)
        T_oct[2,-1] = 1.5
        self.set_T(T_oct, 'approachDirectionObstacle')

    def IK(self, flag):

        self.useIK(flag, 'ik')

        if flag == 0:
            self.set_joint_mode(self.joint_handles, 'passive')

        if flag == 1:
            self.set_joint_mode(self.joint_handles, 'ik')


    def set_q_KUKA(self, q):
        '''
        Position the robot in the scene given the 7-D joints angles of the arm in q.
        :param q: joint angles for the specific KUKA robot in the scene.
        :return:
        '''
        self.set_q(q, self.joint_handles)



    def motion_planning(self, return_xyz = False):
        '''
        start a motion plan in a threaded script by moving
        the dummy 'start_thread' to a height zero (i.e. a  small hack to communicate between scripts).
        The threaded script in vrep is always running but waiting for the dummy
        'start_thread' to go to zero height, indicating that the motion plan should
        start. When the motion plan finishes, the threaded script puts the dummy at different heights, which is parsed as
        different status of the motion planner to the python client.
        :return:
        '''

        if return_xyz:
            self.set_xyz([0, 0, -0.2], 'start_thread', block=True)
        else:
            self.set_xyz([0, 0, -0.1], 'start_thread', block=True)

        one_message_every_N_seconds = 0.5

        time0  = time.time()
        tprint = time.time()

        # you may not be able to read the state immediately when OMPL is just starting
        print('OMPL working...')
        while self.check_thread_flag() < 0:
            if time.time()-tprint >= one_message_every_N_seconds:
                print('OMPL working...')
                tprint = time.time()

        if np.abs(self.get_T('start_thread')[2, -1] - 0.2) < 0.0001:
            print('Could not find final desired configuration!!!')
            print('Search time took %.2f sec' % (time.time() - time0))
            return -1

        if np.abs(self.get_T('start_thread')[2, -1] - 0.3) < 0.0001:
            print('OMPL could not find path!!!')
            print('Search time took %.2f sec' % (time.time() - time0))
            return -1

        print('OMPL solution found in %.2f sec' %(time.time()-time0))

        time0 = time.time()
        while self.get_T('start_thread')[2, -1] == 0:
            if time.time()-tprint >= one_message_every_N_seconds:
                print('getting data ready...')
                tprint = time.time()

        print('Path retrieved in %.2f sec' % (time.time() - time0))

        pathq = self.retriveOMPLpath(path_name='pathq')

        q = self.parse_stream(pathq, n_states=7)

        if return_xyz:
            pathx = self.retriveOMPLpath(path_name='pathx')
            x = self.parse_stream(pathx, n_states=7)
        else:
            x = []

        mp_sol ={'q': q, 'x': x}

        return mp_sol

def main():

    v = VREP_KUKA_BASIC('utils_non_threaded')
    v.start()

    # set initial robot pose
    v.set_q_KUKA(np.zeros(7))


    # ================================
    # task 1. Motion planning via OMPL in V-REP.
    #         Request motion planning at different end-effector target poses
    # ================================
    targetPoses = []

    # adding target poses (xyz + quaternion)
    targetPoses.append([-0.03699656, -0.79048306, 0.37774992, 0.58503448, 0.66590548, 0.32851783, -0.32616038])
    targetPoses.append([-0.48699659,  0.05951694,  0.37774992,  0.21098073,  0.71290066, -0.20767055, -0.63571434])
    targetPoses.append([0.56300342, -0.09048315, 0.70275015, 0.46241403, -0.02185819, 0.88620061, -0.01854639])

    v.IK(0) # set this to specify joint angles manually

    mp_xyzq = []
    for target in targetPoses:

        v.set_T(kinematics.xyzq2T(target)[0], 'testTarget1')

        # if you call motion_planning  with return_xyz=True
        #     V-REP will run the MP solution in FK mode in order to return
        #     the end-effector trajectory in mp_sol['x']
        # if you call motion_planning  with return_xyz=False
        #     V-REP will run not run the MP solution
        #     and trajectory in mp_sol['x'] will be empty
        mp_sol = v.motion_planning(return_xyz = True)

        if mp_sol != -1: # if solution was found
            q    = mp_sol['q']
            xyzq = mp_sol['x']
            mp_xyzq.append(xyzq)

            n_traj = q.shape[0]

            for t in range(n_traj):

                # visualize solution
                v.set_q_KUKA(q[t,:])

                if xyzq != []:# also, move the aux dummy to check that xyzq represents the end-effector position
                    v.set_T(kinematics.xyzq2T(xyzq[t,:])[0], 'aux')

                time.sleep(0.02)
        else:
            print("Solution was not found for this target.")

    print("MP task finished")


    # ================================
    # task 2. IK
    #         Use the cartesian values in mp_xyzq
    #         and rely on the IK of v-rep to find joint angles
    #         This part is only used if you called v.motion_planning(return_xyz = True)
    #         as it ill return the Cartesian trajectories of the end-effector.
    #         If you called with v.motion_planning(return_xyz = False)
    #         there is no Cartesian trajectory to call IK.
    #
    # OBS: this task is only to show the functionality of IK in this API,
    #      but the actual solution is the one given by OMPL in the
    #      previous step (IK does not care if the links will hit obstacles, but OMPL does.)
    # ================================

    v.IK(1) # set the robot to IK mode
    v.set_IK_damping(400, 0.1, 'ik') # change damping of IK solver to make solutions more stable or faster

    for xyzq in mp_xyzq:

        if xyzq != []:
            n_traj = xyzq.shape[0]
            for t in range(n_traj):
                v.set_T(kinematics.xyzq2T(xyzq[t,:])[0], 'target')
                time.sleep(0.02)

    print("Finished!")


if __name__ == '__main__':

    main(  )
