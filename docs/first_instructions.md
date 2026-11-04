# First Insructions to build Java/Kotlin code for FTC robots

In this folder I am working with code for robots used in First Tech Challenge (FTC) robots. I am coaching a team of High School students. As a coach, I want to help students grow into mastering engineering skills. You can find the code controlled directly by the team in the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder. The student currently working on code has produced this module: AutoOp_BlueClose.java in the op folder. 

You can find code samples provide by the FIRST organization in  https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/e14c2aeb33e84d4ea21697beeea9ac49557be870/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples.

In the past, I developed the PathMaker code (see TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pathmaker). The purpose was to provide students with an easy to use pathing library that also doesn't require much tuning.

Additionally, I want the students to use panles, see https://panels.bylazar.com/, that provides a dashboard to visualize what's going on with the robot and to also be able to change variables without recompiling the code. This cuts sw development time dramatically. 

This year's robot key challenge is to collect 2.8" dia whiffle balls and shoot them into an elevated basket. The shooter is powerd by a dc motor (this model: https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/). 
We are using a Rhino wheel (https://www.gobilda.com/3601-series-rhino-wheel-14mm-bore-72mm-diameter/) to shoot out the balls in connection with a steel flywheel for inertia (https://www.gobilda.com/steel-flywheel-82mm-diameter-152g-1651-g-cm/). to control the rpm of the rhino wheel, the encoders on the rev robotics hub in concert with the builtin PID turned out not to be reliable. Therefore I am planning to use the encoders of the OctoQuad (https://github.com/DigitalChickenLabs/OctoQuad/blob/master/documentation/OctoQuadDatasheet_Rev_3.0C.pdf) and program our own PID controller. 

Can you analyze the codebase and create well documented java code that integrates a PID controller for the rhino wheel? You can freely make changes to the code in this folder that I will use in my communication with the student who has their own code copy on their laptop. we want to shoot up to four balls in quick succession. The internal robot sw loop is updated every 20-40 ms. 

I also want to include panels with this project. It would be useful to expose the PID variables as well as RPM measurement visuals.

Please create a document in the docs folder narrating your findings, recommendations, and strategy with an emphasis on explaining to a high school student that I as a technical coach can use


