import numpy
from numpy import cos as c
from numpy import sin as s

import planners.trajectory as tj

# for testing
import matplotlib.pyplot as plt

class TrajectoryCubic(tj.Trajectory):
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
        
        q0 = numpy.zeros(4)
        dq0 = numpy.zeros(4)
        qf = numpy.array(final_point) - numpy.array(offset)
        dqf = numpy.zeros(4)

        t0 = 0.0
        tf = final_time-initial_time

        # compute polynomial coefficients
        
        # known term: [q0, dq0, qf, dqf]
        known_term = numpy.concatenate([q0, dq0, qf, dqf])
        #print(known_term)
        
        # matrix of the times
        matrix_q0 = numpy.kron(numpy.eye(4), numpy.array([[1.0, t0, t0**2, t0**3]]))
        matrix_dq0 = numpy.kron(numpy.eye(4), numpy.array([[0.0, 1.0, 2.0*t0, 3.0*t0**2]]))
        matrix_qf = numpy.kron(numpy.eye(4), numpy.array([[1.0, tf, tf**2, tf**3]]))
        matrix_dqf = numpy.kron(numpy.eye(4), numpy.array([[0.0, 1.0, 2.0*tf, 3.0*tf**2]]))
        matrix = numpy.concatenate([matrix_q0, matrix_dq0, matrix_qf, matrix_dqf], axis=0)
        #print(matrix)

        # polynomial coefficients
        if numpy.linalg.det(matrix) < 0.01:
            self.coeff = numpy.concatenate([numpy.ones(4), numpy.array(12)])
        else:
            self.coeff = numpy.linalg.solve(matrix, known_term)
        #print(self.coeff)
        

    def _get_untransformed_point(self, time):
        
        t = time
        coeff = self.coeff
        
        p = numpy.kron(numpy.eye(4), numpy.array([1.0, t, t**2, t**3])).dot(coeff)
        v = numpy.kron(numpy.eye(4), numpy.array([0.0, 1.0, 2.0*t, 3.0*t**2])).dot(coeff)
        a = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 2.0, 6.0*t])).dot(coeff)
        j = numpy.kron(numpy.eye(4), numpy.array([0.0, 0.0, 0.0, 6.0])).dot(coeff)
        sn = numpy.zeros(4)
        cr =numpy.zeros(4)
    
        return p, v, a, j, sn, cr
        
        
# test
#cubic = TrajectoryCubic([0.0, 0.0, 1.0, numpy.pi], numpy.eye(3), [1.0, 2.0, 3.0, numpy.pi], 10.0)
#q_record = []
#time_record = [0.01*i for i in range(1000)]
#for time in time_record:
#    q, dq, ddq, dddq, sn, cr = cubic.get_point(time)
#    q_record.append(q)
#    print(q)

#plt.figure()
#plt.plot(time_record, q_record)
#plt.grid()
#plt.show()



