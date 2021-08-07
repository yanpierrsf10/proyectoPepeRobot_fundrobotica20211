#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from markers import *
from functions import *
from std_msgs.msg import String

global press_key
press_key = "0"

def callback(msg):

    global press_key
    press_key = msg.data

rospy.init_node("keyboardikine")
rospy.Subscriber("/keys", String, callback)
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)
bmarker      = BallMarker(color['RED'])
bmarker_des  = BallMarker(color['GREEN'])
# Joint names
jnames = ['q1', 'q2', 'q3','q4', 'q5', 'q6','q7']
# Desired position
#xd = np.array([0.84, 0.125, 0.249])
xd = np.array([1.7, -0.21, 2])
# Initial configuration
q0 = np.array([0.0, 0.05, 0, 0, -1, 1, 0.0])
# Inverse kinematics
q = ikine(xd, q0)
# Resulting position (end effector with respect to the base link)
T = fkine(q)
# Red marker shows the achieved position
bmarker.xyz(T[0:3,3])
# Green marker shows the desired position
bmarker_des.xyz(xd)
# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Conditional Loop
    paso = 0.005
    if(press_key=="q"):
        xd[0]=xd[0]+paso 
    if(press_key=="a"):
        xd[0]=xd[0]-paso
    if(press_key=="w"):
        xd[1]=xd[1]+paso
    if(press_key=="s"):
        xd[1]=xd[1]-paso
    if(press_key=="e"):
        xd[2]=xd[2]+paso
    if(press_key=="d"):
        xd[2]=xd[2]-paso
    # Inverse kinematics
    q = ikine(xd, q0)
    # Resulting position (end effector with respect to the base link)
    T = fkine(q)
    print('Posicion Obtenida:')
    pos = np.array([[T[0,3]],
                [T[1,3]],
                [T[2,3]]])
    print(np.round(pos,3))
    q0=q
    # Red marker shows the achieved position
    bmarker.xyz(T[0:3,3])
    # Green marker shows the desired position
    bmarker_des.xyz(xd)
    # Add the head joint value (with value 0) to the joints
    jstate.position = q
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()