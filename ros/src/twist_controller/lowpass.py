
class LowPassFilter(object):
    '''
    Basic Lowpass Filter
    '''
    def __init__(self, alpha):
        '''
        Initialiser with directly with alpha.
        '''
        self.a = alpha
        self.b = 1 - alpha

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
