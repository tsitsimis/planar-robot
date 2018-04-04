# planaRobot

2D robotic arm with arbitrary number of links and rotational
joints.

Ready implementation of trajectory planning and
kinematic control.

## Examples
Real-time visualization of kinematic control
while the end-effector follows a path.

## Dependencies
* numpy

## Use
<code>from planarobot.planar_arm import PlanarArm

links = np.array([2, 1, 1])  # lengths of links  
robot = PlanarArm(links)

...

</code>
