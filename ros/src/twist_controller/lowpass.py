
class LowPassFilter(object):
    '''
    Basic Lowpass Filter
    '''
    def __init__(self, tau, ts):
        '''
        Initialiser with time constant tau and sampling time ts.
        '''
        self.a = 1. / (tau / ts + 1.)
        self.b = tau / ts / (tau / ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        '''
        Basic implementation of alpha x value + (1 - alpha) x last value
        '''
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
