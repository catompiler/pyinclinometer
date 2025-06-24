import time


# https://www.geeksforgeeks.org/python/timing-functions-with-decorators-python/

def utimeit(func):
    # This function shows the execution time of
    # the function object passed
    def wrap_func(*args, **kwargs):
        t1 = time.monotonic()
        result = func(*args, **kwargs)
        t2 = time.monotonic()
        #print(f'Function {func.__name__!r} executed in {(t2-t1):.4f}s')
        print(f'Function {func.__name__!r} executed in {(t2 - t1)}s')
        return result
    return wrap_func
