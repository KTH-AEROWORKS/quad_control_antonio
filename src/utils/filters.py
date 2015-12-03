"""This file implements a velocity filter for estimating the velocity of
a quad from position measurements.
"""

import numpy


class MedianFilter3D:
    """This class implements a median filter.
        Such filters will be used to derive velocity estimates from position
        measurements.
        """

    def __init__(self, order):
        self.order = order
        self.data = [numpy.zeros(3) for i in range(order)]

    def update_data(self, new_data):
        for i in range(self.order - 1):
            self.data[i] = numpy.array(self.data[i + 1])
        self.data[-1] = numpy.array(new_data)

    def output(self):
        # print self.data
        return numpy.median(self.data, axis=0)

    def update_and_output(self, new_data):
        self.update_data(new_data)
        return self.output()


class VelocityFilter:
    """This class implements a filter for estimating a velocity from
    position measurements.
    It is based on the Median_Filter_3D class.
    """

    def __init__(self, order, initial_position, initial_time):
        self.median_filter = MedianFilter3D(order)
        self.old_position = numpy.array(initial_position)
        self.old_time = initial_time

    def update_and_output(self, new_position, new_time):
        dt = new_time - self.old_time
        vel_estimate = (numpy.array(new_position) - self.old_position) / dt
        self.old_position = new_position
        self.old_time = new_time
        return self.median_filter.update_and_output(vel_estimate)


#vel_fil = VelocityFilter(4, [0.0, 0.0, 0.0], 0.0)
#print(vel_fil.update_and_output([1, 2, 3], 0.2))
#print(vel_fil.update_and_output([2, 3, 4], 0.4))
#print(vel_fil.update_and_output([3, 4, 5], 0.6))
