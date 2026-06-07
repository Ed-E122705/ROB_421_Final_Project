import math
import numpy as np

from . import geometry
#from . import rrt_shapes

class Joint():
    def __init__(self, name, parent=None, type='fixed', getter=(lambda:0),
                 description='A kinematic joint',
                 qmin=-math.inf, qmax=math.inf,
                 d=0, theta=0, r=0, alpha=0,
                 collision_model=None, ctransform=geometry.identity()): # creates a new joint
        self.name = name
        self.parent = parent
        self.type = type
        if type == 'fixed':
            self.apply_q = self.fixed
        elif type == 'revolute':
            self.apply_q = self.revolute
        elif type == 'prismatic':
            self.apply_q = self.prismatic
        elif type == 'world':
            self.apply_q = self.world_joint
        else:
            raise ValueError("Type must be 'fixed', 'revolute', 'prismatic', or 'world'.")
        self.getter = getter    # function that returns current joint value
        self.description = description
        self.children = []
        self.d = d
        self.theta = theta
        self.r = r
        self.alpha = alpha
        self.children = []
        self.collision_model = collision_model
        self.q = 0      # current joint value
        self.qmin = qmin
        self.qmax = qmax
        self.parent_link_to_this_joint = geometry.dh_matrix(-d,-theta,-r,-alpha)        #transformation matrix
        self.this_joint_to_parent_link = np.linalg.inv(self.parent_link_to_this_joint)

        self.solver = None

    def __repr__(self):     # printing object  for debugging
        if self.type == 'fixed':
            qval = 'fixed'
        elif isinstance(self.q, (int,float)):
            qval = "q=%.2f deg." % (self.q*180/math.pi)
        else:
            qval = ("q=%s" % repr(self.q))
        return "<Joint '%s' %s>" % (self.name, qval)

    def this_joint_to_this_link(self):
        "The link moves by q in the joint's reference frame."
        return self.apply_q()

    def this_link_to_this_joint(self):
        return np.linalg.inv(self.this_joint_to_this_link())

    def revolute(self):     # defines axis of rotation for revolute joint
        return geometry.aboutZ(-self.q)

    def prismatic(self):        # defines axis for prismatic joint
        return geometry.translate(0.,0.,-self.q)

    def fixed(self):
        return geometry.identity()  # fixed joint is identity matrix

    def world_joint(self):      # joint representing robot moving in 2D world
        return geometry.translate(self.q[0],self.q[1]).dot(geometry.aboutZ(self.q[2]))

class Kinematics():
    def __init__(self,joint_list,robot):    #build kinematic model
        self.joints = dict()    # maps names to joints
        for j in joint_list:    # creates kinematic tree
            self.joints[j.name] = j
            if j.parent:
                j.parent.children.append(j)
        self.base = self.joints[joint_list[0].name]     # base joint
        self.robot = robot
        robot.kine = self
        self.get_pose()     # read current joint values

    def joint_to_base(self,joint):      # transform between frames
        if isinstance(joint,str):
            joint = self.joints[joint]
        Tinv = geometry.identity()
        j = joint
        while j is not self.base and j.parent is not None:  # work up kinematic chain
            Tinv = j.parent.this_link_to_this_joint().dot(
                j.this_joint_to_parent_link.dot(Tinv)
                )
            j = j.parent
        if j:
            return Tinv
        else:
            raise Exception('Joint %s has no path to base frame' % joint)

    # these functions are for the transformation between frames
    def base_to_joint(self,joint):
        return np.linalg.inv(self.joint_to_base(joint))

    def joint_to_joint(self,joint1,joint2):
        return self.base_to_joint(joint2).dot(self.joint_to_base(joint1))

    def link_to_base(self,joint):
        if isinstance(joint,str):
            joint = self.joints[joint]
        return self.joint_to_base(joint).dot(joint.this_link_to_this_joint())

    def base_to_link(self,joint):
        return np.linalg.inv(self.link_to_base(joint))

    def link_to_link(self,joint1,joint2):
        return self.base_to_link(joint2).dot(self.link_to_base(joint1))

    def get_pose(self):
        for j in self.joints.values():
            j.q = j.getter()

    def project_to_target(self, target_pose):
        """
        target will have x, y and v_x, v_y
        will need to project out a distance from base of robot to target, then
        use ik to determine how the robot needs to turn, and then push the 
        kicker forward to launch ball to target  
        """


        # # Current robot position
        # target_x, target_y, target_theta, target_vx, target_vy, target_ω = target_pose
        # target_world_frame = geometry.point(target_x, target_y, 0)
        # target_transform = self.base_to_joint('world')

        # target_robot_frame = target_transform.dot(target_world_frame)
        # rob_x = self.robot.pose.x
        # rob_y = self.robot.pose.y
        # rob_theta = self.robot.pose.theta
        

        # x_displacement = target_robot_frame[0, 0]
        # y_displacement = target_robot_frame[1, 0]
        

        target_x, target_y, target_vx, target_vy = target_pose
        rob_x = self.robot.pose.x
        rob_y = self.robot.pose.y
        rob_theta = self.robot.pose.theta
        

        x_displacement = target_x - rob_x
        y_displacement = target_y - rob_y
        distance = np.sqrt(x_displacement**2+y_displacement**2)
        
        

        ball_speed = 7.14 #m/s
        ball_travel_time = distance / ball_speed

        # predict where target will be
        x_predict = target_x + target_vx * ball_travel_time
        y_predict = target_y + target_vy * ball_travel_time

        # vectorize
        x_displacement = x_predict - rob_x
        y_displacement = y_predict - rob_y
        heading = math.atan2(y_displacement, x_displacement)
        angle_to_turn = heading - rob_theta
        
        # limit robot to turn between -π and π
        robot_turn = math.atan2(
            math.sin(angle_to_turn),
            math.cos(angle_to_turn)
            )
        robot_turn = robot_turn * 180/np.pi

        # return heading
        return robot_turn