# PythonOMPL_via_VREP
Simple integration between python 3 and OMPL in V-REP.

The initial motivation of the code here was to enable access to the OMPL demo motionPlanningDemo1.ttt via Python.
As it is, the demo is self-contained in the V-REP scene and all planning is done via the scripts in Lua.

Enabling access via Python allows one to combine other functionalities, or call the motion planner as part of a method that requires motion planning.

To create a successful connection, you need the following files also in the same folder 
1. remoteApi.so
2. vrep.py
3. vrepConst.py

These files are found in the current V-REP distribution you are currently using (see the V-REP documentation).

To run the code, open the scene 
    kuka_python_OMPL.ttt
in vrep (tested with V-REP 3.6.1).

Then on the terminal 
    python3 VREP_KUKA_BASIC.py

The demo shows how you can 
1. keep the robot waiting for a "do motion planning" signal. 
2. start the planning while the python client waits for the solution.
3. retrieve the solution as a path in joint and cartesian coordinates as numpy arrays.
4. change the robot to operate in IK mode, and then run the Cartesian solution while V-REP IK tries to find the joint angles.


