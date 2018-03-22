#! /usr/bin/env python3
"""@package docstring
 # file decorators.py
 # copyright SoftWear Automation, Inc.
 # author Daniel Sahu <d.sahu@softwearinc.com>
 # date February, 2018
"""

import time
import sys

# Try to import from relative path; if we're calling as main import
if __package__:
    from .misc import SoftwearLogger as sLogger
else:
    from misc import SoftwearLogger as sLogger

def LoggingDecorator(originalMethod):
    """
    Decorator to log function calls:
    """
    _logger = sLogger("LoggingDecorator")
    def wrap(*args, **kwargs):
        _logger.debug("Entering: " + str(originalMethod.__qualname__))
        x = originalMethod(*args, **kwargs)
        _logger.debug("Exiting: " + str(originalMethod.__qualname__))
        return x
    return wrap

def TimingDecorator(originalMethod):
    """
    Decorator to time function calls:
    """
    _logger = sLogger("TimingDecorator")
    def wrap(*args, **kwargs):
        st = time.time()
        x = originalMethod(*args, **kwargs)
        _logger.info(str(originalMethod.__qualname__) + " took " + \
                     str(time.time()-st) + " seconds.")
        return x
    return wrap

def TryExceptDecorator(originalMethod):
    """
    Decorator to wrap function calls in a try/except loop and pass the result
    to sys.excepthook. This is mostly designed to work with threads which don't
    natively pass Exceptions that way.
    """
    _logger = sLogger("TryExceptDecorator")
    def wrap(*args, **kwargs):
        try:
            return originalMethod(*args, **kwargs)
        except (KeyboardInterrupt, SystemExit):
            raise
        except:
            _logger.error("Caught and raised exception in " + str(originalMethod.__qualname__))
            sys.excepthook(*sys.exc_info())
    return wrap

def GenerateClassDecorator(method_wrapper):
    """
    This function takes a decorator that we desire to apply to all methods of a
    given class. It returns a class decorator for that purpose.

    Use: Instantiate with a defined wrapper as input. Something of the following will work:

    >> def method_wrapper(self, original_method):
    ...     def new_method(*args, **kwargs):
    ...         x = original_method(*args, **kwargs)
    ...         return x
    ...     return new_method
    >> class_decorator = generate_class_decorator(method_wrapper)
    >>  @class_decorator
    ... class Test(object): ...

    Note: the class to which we apply a decorator needs to inherit from the object class.

    """

    def class_decorator(InputClass):
        """
        Decorator that applies the desired function
        Basically constructs a new class that calls the desired
        method_wrapper on any method that isn't overriden ( everything
        except __init__ by default ).
        """

        class WrappedClass(InputClass):
            """
            Lorum Ipsum
            """

            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)

            def __getattribute__(self, s):
                """
                this is called whenever any attribute of a NewCls object is accessed.
                This function first tries to get the attribute off NewCls. If it
                fails then it tries to fetch the attribute from self.oInstance (an
                instance of the decorated class). If it manages to fetch the
                attribute from self.oInstance, and the attribute is an instance
                method then `time_this` is applied.
                """

                x = getattr(InputClass, s)

                if len(s) > 4 and (s[:2] + s[-2:] == "____"):
                    # Check if it's a special method (i.e. "__*__") and return
                    # original if so.
                    return x
                elif not callable(x):
                    return x
                else:
                    # It's a callable method; wrap it!
                    return method_wrapper(self, x)

        return WrappedClass

    return class_decorator

if __name__ == "__main__":
    """
    Unit Testing:
    Call this file standalone to run unit tests.
    """

    import code

    def time_one(self, original_method):
        def new_method(*args, **kwargs):
            st = time.time()
            x = original_method(self, *args, **kwargs)
            print("Took " + str(time.time()-st) + " seconds to call.")
            return x
        return new_method

    my_class_decorator = GenerateClassDecorator(time_one)

    @my_class_decorator
    class A(object):
        instances = []
        def __init__(self, test_val):
            self.test_val = test_val
            self.instances.append(self)

        def test1(self):
            print("Running for 1 second.")
            time.sleep(1.0)

        def test2(self):
            print("Running for 2 seconds.")
            time.sleep(2.0)

        def test3(self):
            print("Running for 3 seconds.")
            time.sleep(3.0)

    test = A("Happy?")
    test.test1()
    test.test2()
    test.test3()

    code.interact(local=locals())
