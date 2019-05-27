import numpy as np
import kinematics2 as kinematics

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


class Handles:
    def __init__(self):
        self.handle =[]
        self.name = []


class VREP_ROBOT:

    def __init__(self, childscriptname, ipaddress='127.0.0.1', port=19999, clientID=-999, ik_name_list=[]):

        self.vrep = vrep
        self.script = childscriptname

        if clientID == -999:
            make_new_client = True
        else:
            make_new_client = False

        self.inputBufferEmpty = bytearray()

        if make_new_client:
            vrep.simxFinish(-1)  # just in case, close all opened connections

            # check connection here
            vrep_ready = -1

            print('VREP attempting to connect...'),

            while vrep_ready != 0:
                self.clientID = self.vrep.simxStart(ipaddress, port, 1, 1, 5000, 5)
                vrep_ready = self.clientID
                print('.')
            print('VREP returned ID')
            print('ClientID is', self.clientID)

        else:
            self.clientID = clientID

        self.start()
        self.handles = Handles()
        self.initialize_handles(ik_name_list=ik_name_list)


    def get_handle(self, handle_name):
        idx = self.handles.name.index(handle_name)
        return self.handles.handle[idx]

    def set_specified_joint_angle(self, jointName, q, flagOneshotWait=0):

        jointHandle = self.get_handle(jointName)

        if flagOneshotWait == 0:  # fast but does not guarantee update from vrep
            self.vrep.simxSetJointPosition(self.clientID, jointHandle, q, self.vrep.simx_opmode_oneshot)
        else:
            self.vrep.simxSetJointPosition(self.clientID, jointHandle, q, self.vrep.simx_opmode_oneshot_wait)

    def initialize_handles(self, ik_name_list=[]):

        vrep = self.vrep
        number_datatype = 0  # 0 is or object names

        # getting dummies
        [return_code, array_handles, a2, a3, array_stringdata] = vrep.simxGetObjectGroupData(self.clientID, vrep.sim_object_dummy_type, number_datatype, vrep.simx_opmode_blocking)

        self.check_return_ok(return_code, 'getHandlesDummy')
        self.handles.handle = array_handles
        self.handles.name = array_stringdata

        # getting joint handles
        [return_code, array_handles, a2, a3, array_stringdata] = vrep.simxGetObjectGroupData(self.clientID, vrep.sim_object_joint_type, number_datatype, vrep.simx_opmode_blocking)

        self.check_return_ok(return_code, 'getHandlesJoint')
        self.handles.handle.extend(array_handles)
        self.handles.name.extend(array_stringdata)

        # shape types
        [returnCode, array_handles, _, _, array_stringdata] = vrep.simxGetObjectGroupData(self.clientID, vrep.sim_object_shape_type, number_datatype, vrep.simx_opmode_blocking)
        self.check_return_ok(return_code, 'getHandlesShapes')
        self.handles.handle.extend(array_handles)
        self.handles.name.extend(array_stringdata)

        # see if I can get camera handles
        [returnCode, array_handles, _, _, array_stringdata] = vrep.simxGetObjectGroupData(self.clientID, vrep.sim_object_camera_type, number_datatype, vrep.simx_opmode_blocking)
        self.check_return_ok(return_code, 'getHandlesShapes')
        self.handles.handle.extend(array_handles)
        self.handles.name.extend(array_stringdata)

        # get IK handles
        if ik_name_list !=[]:

            for ik_name in ik_name_list:

                res, ikHandle, a1, a2, a3 = \
                    self.vrep.simxCallScriptFunction(self.clientID, self.script,
                                                     self.vrep.sim_scripttype_childscript, 'getIKHandle_function',
                                                     [], [], [ik_name], self.inputBufferEmpty, self.vrep.simx_opmode_blocking)

                self.check_return_ok(res, 'toggleIK')
                self.handles.handle.append(ikHandle[0])
                self.handles.name.append(ik_name)


    def check_return_ok(self, return_code, func_name):
        if return_code != self.vrep.simx_return_ok:
            print('Remote function call failed')
            print('function name: ', func_name)

    def get_vrep_IK_handle(self, ik_name = 'ik'):

        res, ikHandle, a1, a2, a3 = \
            self.vrep.simxCallScriptFunction(self.clientID, self.script,
                                             self.vrep.sim_scripttype_childscript, 'getIKHandle_function',
                                             [], [], [ik_name], self.inputBufferEmpty, self.vrep.simx_opmode_blocking)

        self.check_return_ok(res, 'toggleIK')
        self.handles.handle.append(ikHandle[0])
        self.handles.name.append(ik_name)

    def retriveOMPLpath(self, path_name = 'pathq'):
        '''
        *g
        :param path_name:
        :return:
        '''

        res, a0, path, a2, a3 = \
            self.vrep.simxCallScriptFunction(self.clientID, self.script,
                                             self.vrep.sim_scripttype_childscript, 'retriveOMPLpath_function',
                                             [], [], [path_name], self.inputBufferEmpty, self.vrep.simx_opmode_blocking)

        return np.array(path)


    def useIK(self, flag, ik_name):
        '''
        *g
        :param flag: 1 to activate IK, 0 to deactivate IK
        :param ik_name: name of IK entity defined in the v-rep scene
        :return:
        '''

        if flag == 1:
            on_off = 0
        else:
            on_off = 1

        ikHandle = self.get_handle(ik_name)

        res, a0, a1, a2, a3 = \
            self.vrep.simxCallScriptFunction(self.clientID,
                         self.script, self.vrep.sim_scripttype_childscript,
                         'toggleIK_function', [on_off, ikHandle], [], [],
                         self.inputBufferEmpty, self.vrep.simx_opmode_blocking)

        self.check_return_ok(res, 'toggleIK')



    def parse_vrep_T(self, T):

        if type(T) is list:
            T = np.array(T)
        nElements = T.size

        if nElements == 12:
            Tout = np.zeros(shape=(4, 4))
            #   from vrep format to full T
            Tout[0, 0] = T[0]
            Tout[0, 1] = T[1]
            Tout[0, 2] = T[2]
            Tout[0, 3] = T[3]
            Tout[1, 0] = T[4]
            Tout[1, 1] = T[5]
            Tout[1, 2] = T[6]
            Tout[1, 3] = T[7]
            Tout[2, 0] = T[8]
            Tout[2, 1] = T[9]
            Tout[2, 2] = T[10]
            Tout[2, 3] = T[11]
            Tout[3, 3] = 1

        if nElements == 16:
            Tout = []

            Tout = [T[0, 0], T[0, 1], T[0, 2], T[0, 3], T[1, 0], T[1, 1], T[1, 2], T[1, 3], T[2, 0], T[2, 1], T[2, 2], T[2, 3]]

        return Tout

    def close_vrep_connection(self):
        self.vrep.simxFinish(self.clientID)
        print('Closing vrep connection')

    def start(self):
        self.vrep.simxStartSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait)

    def stop(self):
        self.vrep.simxStopSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait)

    def pause(self):
        self.vrep.simxPauseSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait)

    def get_xyzq(self, name, relative_to=[], block=True):
        '''
        *g
        :param name: object name in v-rep scene.
        :param relative_to: is not given, it will be in relation to world frame.
        :param block: usually blocking
        :return: [xyz + quaternion], thus 7 dimensional
        '''

        T = self.get_T(name, relative_to=relative_to, block=block)

        return kinematics.T2xyzq(T)[:,0]


    def get_T(self, name, relative_to=[], block = True):
        '''
        *gjm
        :param name:   name of the object in vrep scene to measure the position
        :param name:   name of the reference object. If not given, then assume it is the world ref frame.
        :param block:  default is true.
        :return: T
        '''
        dummyHandle = [self.get_handle(name)]

        if relative_to == []:
            dummyHandle2 = -1
        else:
            dummyHandle2 = self.get_handle(relative_to)

        dummyHandle.append(dummyHandle2)

        if block:
            [res, retInts, retFloats, retStrings, retBuffer] = \
                self.vrep.simxCallScriptFunction(self.clientID,
                                                 self.script, self.vrep.sim_scripttype_childscript,
                                                 'getT_function',
                                                 dummyHandle, [], ['getT_test'],
                                                 self.inputBufferEmpty,
                                                 self.vrep.simx_opmode_blocking)
        else:
            [res, retInts, retFloats, retStrings, retBuffer] = \
                self.vrep.simxCallScriptFunction(self.clientID,
                                                 self.script, self.vrep.sim_scripttype_childscript,
                                                 'getT_function',
                                                 dummyHandle, [], ['getT_test'],
                                                 self.inputBufferEmpty,
                                                 self.vrep.simx_opmode_oneshot)

        step = 4
        return np.array([retFloats[0:0 + step], retFloats[4:4 + step], retFloats[8:8 + step], [0, 0, 0, 1]])


    def set_q(self, q, joint_handles):
        '''
        Position the robot in the scene given the 7-D joints angles of the arm in q.
        :param joint_handles: [N x 1] vector containing the joint handles
        :param q: [N x 1] the corresponding values of joint angles (radians)
        :return:
        '''

        for j in range(len(joint_handles)):
            self.vrep.simxSetJointPosition(self.clientID, joint_handles[j], q[j], self.vrep.simx_opmode_oneshot)



    def set_xyz(self, xyz, object, rel_object=[], block=False):
        '''
        *gjm
        :param xyz: position
        :param object: object to move
        :param rel_object: name of object to which to move relatively. If not given, then use the world coordinates
        :param block: should be false by default
        :return:
        '''

        if rel_object != []:
            handle_relative = self.get_handle(rel_object)
        else:
            handle_relative = -1 # use world coordinates (absolute)

        if block:
            res = self.vrep.simxSetObjectPosition(self.clientID,
                              self.get_handle(object), handle_relative, xyz, self.vrep.simx_opmode_blocking)
        else:
            res = self.vrep.simxSetObjectPosition(self.clientID,
                              self.get_handle(object), handle_relative, xyz, self.vrep.simx_opmode_oneshot)

        self.check_return_ok(res, 'set xyz')


    def set_T(self, T, dummy_name, flagBlocking=0):
        '''
        *gjm
        :param T: as a 4x4 homog transf matrix
        :param object: object to move w.r.t. the world frame
        :param block: should be false by default
        :return:
        '''
        Tout = self.parse_vrep_T(T)
        try:
            if flagBlocking:
                # [res, retInts, retFloats, retStrings, retBuffer]
                [res, a1, T, a2, a3] = self.vrep.simxCallScriptFunction(self.clientID, self.script, self.vrep.sim_scripttype_childscript,
                                                                        'setPositionViaHomgTransf_function', [self.get_handle(dummy_name)], Tout, [], self.inputBufferEmpty, self.vrep.simx_opmode_blocking)
            else:
                [res, a1, T, a2, a3] = self.vrep.simxCallScriptFunction(self.clientID, self.script, self.vrep.sim_scripttype_childscript,
                                                                        'setPositionViaHomgTransf_function', [self.get_handle(dummy_name)], Tout, [], self.inputBufferEmpty, self.vrep.simx_opmode_oneshot)
        except:
            print('** VREP_AGENT.py.' + dummy_name + ' does not exist in the scene!!!')

    def matrixT2flatT(self, matrixT):
        # This is used to send a flat table_12, truncated homog transf. matrix to vrep
        # Receive a normal homog transf matrix as matrixT [4x4]
        # Return an array [1x12] that can be interpreted by vrep
        Tflat = np.squeeze(np.asarray(matrixT))
        Tflat = Tflat.ravel()
        return Tflat[0:12]


    def parse_stream(self, x, n_states=4):
        '''
        Receives a x = [ (n_traj * n_states) x 1 ] array. Parse it as a [n_traj x n_states]
        :param x: the streamed message from vrep
        :param n_states: the number of states.
        :return: x2  = [n_traj x n_states]
        '''

        n_traj = int(len(x)/n_states)
        x2 = np.empty(shape=(n_traj, n_states))

        for t in range(n_traj):
            for k in range(n_states):
                x2[t,k] =x[(t)*n_states + k]

        return x2

    def check_thread_flag(self):
        '''
        This function is necessary because OMPL seems to block the streaming of messages, even when you use a blocked call.
        To avoid problems, you can check if the return is empty or not.
        :return:
        '''

        while 1:
            #print('\ntrying to read flag')
            flag = self.get_T('start_thread')
            if flag[0] != []:
                #print('flag has value %.2f' %flag[2,-1])
                #print(flag[2,-1])
                return flag[2,-1]
            else:
                #print('flag is empty')
                pass

    def set_joint_mode(self, joint_handles, mode):
        '''
        To control the robot in joint space, set the joints to passive mode.
        To do IK, you need to both set the joints in IK mode, and also enable the IK solver by calling
        the method useIK(self, flag, ik_name):
        :param joint_handles:
        :param mode:
        :return:
        '''

        if mode == 'passive':
            [res, retInts, retFloats, retStrings, retBuffer] = \
                self.vrep.simxCallScriptFunction(self.clientID,
                         self.script, self.vrep.sim_scripttype_childscript,
                         'setJointModePassive_function',
                          joint_handles, [], [], bytearray(), self.vrep.simx_opmode_blocking)
        if mode == 'ik':
            [res, retInts, retFloats, retStrings, retBuffer] = \
                self.vrep.simxCallScriptFunction(self.clientID,
                         self.script, self.vrep.sim_scripttype_childscript,
                         'setJointModeIK_function',
                          joint_handles, [], [], bytearray(), self.vrep.simx_opmode_blocking)


    def set_IK_damping(self, maxIter, damping, ik_name='ik'):
        '''
        Control the parameters of the IK solver in v-rep. See the V-REP IK configuration.
        :param maxIter:
        :param damping:
        :param ik_name: name of the IK group in the v-rep scene.
        :return:
        '''
        ikHandle = self.get_handle(ik_name)

        res, a0, a1, a2, a3 = self.vrep.simxCallScriptFunction(
                            self.clientID, self.script, self.vrep.sim_scripttype_childscript,
                             'setIKparam_function', [ikHandle], [maxIter, damping],
                             [], self.inputBufferEmpty, self.vrep.simx_opmode_oneshot_wait)

        self.check_return_ok(res, 'set_IK_damping')

def main():

    # place holder
    pass

if __name__ == '__main__':

    main(  )
