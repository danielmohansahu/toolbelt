#! /usr/bin/env python3
"""@package docstring
 # file debug.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date April, 2018

 This class contains debugging tools.
"""

class DummyClass(object):
    """
    This class is intended to be used as a drop in replacement for an API when
    simulation isn't set up properly.
    """

    ############################### BASICS #####################################

    def __init__(self):
        self.value = 0
        self.size = 10
        self.bool = True
        self.index = 0

    def __getattr__(self, arg):
        return self

    ########################### FUNCTION HANDLING ##############################

    def __call__(self, *args):
        return self

    ######################### ITERATION HANDLING ###############################

    def __iter__(self):
        return self

    def __next__(self):
        raise StopIteration

    def __reversed__(self):
        return self

    ############################# SET HANDLING #################################

    def __len__(self):
        return int(self.size)

    def __contains__(self):
        return self.bool

    ############################## DICT HANDLING ###############################

    def __getitem__(self, key):
        return self

    def __setitem__(self, key, value):
        pass

    def __delitem__(self, key, value):
        pass

    def __missing__(self, key, value):
        pass

    ############################## MATH HANDLING ###############################

    def __add__(self, y):
        return (self.value + y)

    def __sub__(self, y):
        return (self.value - y)

    def __mul__(self, y):
        return (self.value * y)

    def __truediv__(self, y):
        return (self.value / y)

    def __floordiv__(self, y):
        return (self.value // y)

    def __mod__(self, y):
        return (self.value % y)

    def __divmod__(self, y):
        return divmod(self.value,y)

    def __pow__(self, y):
        return (self.value ** y)

    ########################### UNARY MATH HANDLING ############################

    def __neg__(self):
        return -self.value

    def __pos__(self):
        return +self.value

    def __abs__(self):
        return abs(self.value)

    def __invert__(self):
        return ~self.value

    def __complex__(self):
        return complex(self.value)

    def __int__(self):
        return int(self.value)

    def __float__(self):
        return float(self.value)

    def __round__(self):
        return round(self.value)

    ########################### COMPARISON HANDLING ############################

    def __eq__(self, y):
        return self.bool

    def __ne__(self, y):
        return self.bool

    def __lt__(self, y):
        return self.bool

    def __le__(self, y):
        return self.bool

    def __gt__(self, y):
        return self.bool

    def __ge__(self, y):
        return self.bool

    def __bool__(self):
        return self.bool

    ############################ CONTEXT HANDLING ##############################

    def __enter__(self):
        pass

    def __exit__(self, *args):
        pass

    ############################ MISC HANDLING ##############################

    def __hash__(self):
        self.index += 1
        return self.index


if __name__ == "__main__":
    import code

    D = DummyClass()

    code.interact(local=locals())
