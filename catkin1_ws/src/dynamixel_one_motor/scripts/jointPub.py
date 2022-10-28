import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint





def deg(degrees):
    angle=degrees*np.pi/180
    return angle



#postura deseada.
home=[0,0,0,0,0]
rest=[0,deg(-110),deg(90),deg(5),0]
pos1=[deg(-20),deg(20),deg(-20),deg(20),0]
pos2=[deg(30),deg(-30),deg(30),deg(-30),0]
pos3=[deg(-90),deg(15),deg(-55),deg(17),0]
pos4=[deg(-90),deg(45),deg(-55),deg(45),deg(10)]
#array de posturas.
postura=[home,pos1,pos2,pos3,pos4,rest]
#posturas de prueba:
pos5=[deg(45),deg(0),deg(0),deg(0),deg(0)]
pos6=[0,deg(45),deg(0),deg(0),deg(0)]
pos7=[deg(0),deg(0),deg(45),deg(0),deg(0)]
pos8=[deg(0),deg(0),deg(0),deg(45),deg(10)]

def joint_publisher():
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)
    rospy.init_node('joint_publisher', anonymous=False)
    
    while not rospy.is_shutdown():
        #control de mov. con teclas.
        key=input()
        if key == 'a':
            postura = pos1
            key = ' '
        elif key == 'w':
            postura = pos2
            key = ' '
        elif key == 's':
            postura = pos3
            key = ' '
        elif key == 'd':
            postura = pos4
            key = ' '
        elif key == 'q':
            postura = home
            key = ' '
        elif key == 'e':
            postura = rest
            key = ' '

        state = JointTrajectory()
        state.header.stamp = rospy.Time.now()
        state.joint_names = ["joint_1","joint_2","joint_3","joint_4","joint_5"]
        point = JointTrajectoryPoint()
        point.positions = postura  
        point.time_from_start = rospy.Duration(0.5)
        state.points.append(point)
        pub.publish(state)
        print('published command')
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        joint_publisher()
    except rospy.ROSInterruptException:
        pass

