class IrisSimulator:
    """Skeleton class for the quad simulator.
    You can think about it as an abstract class.
    The real simulators are obtained by inheritance.
    Each inherited simulator must implement the 'update_pose' function, which
    defines the dynamics of the simulator.
    """

    def __init__(self, time, pos, rot, vel):
        """Constructor of the simulator.
        The initialization parameters are:
        - initial time t
        - initial position p
        - initial attitude in the form of a rotation matrix
        - initial velocity
        The state of the quad is represented by pos, rot, vel.
        This is a sufficient description of the state if we control the Iris
        quads in either Stabilize or Acro mode. Moreover, position and velocity
        can be reliably obtained by our mocap, while further derivatives tend to
        be unreliable.
        This function sets the interface between the quad and the controller.
        If you want to change this interface, you will need to change all the
        simulators and the controllers accordingly. 
        """
        
        self.time = time
        self.pos = pos
        self.rot = rot
        self.vel = vel
        

    def update_state(self, control_input, final_time):
        """Template of the 'update_state' method.
        You can think of this as an abstract method.
        Each simulator must reimplement this method, but it does not matter how.
        Most likely, there will be an ODE solver that will integrate the
        dynamics of the quad between the current time and 'final_time'.
        """
        
        new_pos = None
        new_rot = None
        new_vel = None
        new_ang_vel = None
        
        self.time = final_time
        self.pos = new_pos
        self.rot = new_rot
        self.vel = new_vel
        
        return self.time, self.pos, self.rot, self.vel
