This version of the PathMaker is an upgrade of the original implementation. The main improvement is the use of a state machine wrapped around the original engine that calculates the power in the three degrees of freedom (forward, strafe, turn) of a mecanum wheel drive.

Change log:

- Modifications to PathMakerStateMachine, PathManager as well as small updates to PathDetails and Tele_Robot1:
- added autonomous turn control to field centric driving see function PMSM getGamepadInput
- ...now PM variables autonomous_x, autonomous_y, autonomous_a can be set true/false as needed
- ...to control if a field DOF is driver controlled or autonomous
- ...Also added autoLaneKeeping function to PathMakerStateMachine.
- Moved autoPathList to PathDetails class. Makes more sense.