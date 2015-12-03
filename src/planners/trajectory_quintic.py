import numpy
from numpy import cos as c
from numpy import sin as s

import planners.trajectory as tj

# for testing
import matplotlib.pyplot as plt

class TrajectoryQuintic(tj.Trajectory):
    """Cubic trajectory.
    Implements a trajectory that starts in q0 = numpy.zeros(4) and ends in a
    given point qf.
    The trajectory has a given duration tf.
    Initial and final veocity are zero.
    """


    def __init__(self, offset, rotation, initial_time, final_time, final_point):
        """Arguments:
        - offset (numpy array, 4)
        - rotation (numpy array, 3-by-3)
        - final point (numpy array, 4)
        - duration (float)
        """
        
        tj.Trajectory.__init__(self, offset, rotation, initial_time, final_time)
        
        t0 = 0.0
        q0 = numpy.zeros(4)
        dq0 = numpy.zeros(4)
        ddq0 = numpy.zeros(4)
        
        tf = final_time - initial_time
        qf = numpy.array(final_point) - numpy.array(offset)
        dqf = numpy.zeros(4)
        ddqf = numpy.zeros(4)

        # compute polynomial coefficients
        
        # known term: [q0, dq0, qf, dqf]
        known_term = numpy.concatenate([q0, dq0, ddq0, qf, dqf, ddqf])
        #print(known_term)
        
        # matrix of the times
        matrix_q0 = numpy.kron(numpy.eye(4), numpy.array([[1.0, t0, t0**2, t0**3, t0**4, t0**5]]))
        matrix_dq0 = numpy.kron(numpy.eye(4), numpy.array([[0.0, 1.0, 2.0*t0, 3.0*t0**2, 4.0*t0**3, 5.0*t0**4]]))
        matrix_ddq0 = numpy.kron(numpy.eye(4), numpy.array([[0.0, 0.0, 2.0, 6.0*t0**2, 12.0*t0**2, 20.0*t0**3]]))
        matrix_qf = numpy.kron(numpy.eye(4), numpy.array([[1.0, tf, tf**2, tf**3, tf**4, tf**5]]))
        matrix_dqf = numpy.kron(numpy.eye(4), numpy.array([[0.0, 1.0, 2.0*tf, 3.0*tf**2, 4.0*tf**3, 5.0*tf**4]]))
        matrix_ddqf = numpy.kron(numpy.eye(4), numpy.array([[0.0, 0.0, 2.0, 6.0*tf**2, 12.0*tf**2, 20.0*tf**3]]))
        matrix = numpy.concatenate([matrix_q0, matrix_dq0, matrix_ddq0, matrix_qf, matrix_dqf, matrix_ddqf], axis=0)
        #print(matrix)

        # polynomial coefficients
        if numpy.fabs(numpy.linalg.det(matrix)) < 0.01:
            self.coeff = numpy.concatenate([numpy.ones(4), numpy.zeros(20)])
        else:
            self.coeff = numpy.linalg.solve(matrix, known_term)
        #print(self.coeff)
        

    def _get_untransformed_point(self, time):

        t = time
        coeff = self.coeff
        
        q = numpy.kron(numpy.eye(4), numpy.array([1.0, t, t**2, t**3, t**4, t**5])).dot(coeff)
        v = numpy.kron(numpy.eye(4), numpy.array([0.0, 1.0, 2.0*t, 3.0*t**2, 4.0*t**3, 5.0*t**4])).dot(coeff)
        a = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 2.0, 6.0*t, 12.0*t**2, 20.0*t**3])).dot(coeff)
        j = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 0.0, 6.0, 24.0*t, 60.0*t**2])).dot(coeff)
        sn = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 0.0, 0.0, 24.0, 120.0*t])).dot(coeff)
        cr = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 0.0, 0.0, 0.0, 120.0])).dot(coeff)
    
        return q, v, a, j, sn, cr
        
        
# test
#quintic = TrajectoryQuintic([0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), [1.0, 2.0, 3.0, numpy.pi], 0.0, 10.0)
#q_record = []
#time_record = [0.01*i for i in range(1000)]
#for time in time_record:
#    q, dq, ddq, dddq, sn, cr = quintic.get_point(time)
#    q_record.append(q)
#    print(q)

#plt.figure()
#plt.plot(time_record, q_record)
#plt.grid()
#plt.show()



