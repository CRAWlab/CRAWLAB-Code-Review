### Exceptions in Python
For most of the work that we do, we can think of exceptions as handling two cases, either we made a mistake (a typo, syntax error, etc.) or the user of our script, be it another person or the system we're controlling, produced an input or output that we either didn't anticipate or is outright faulty.

In simulation, these type of errors will just result in a script crash. However, if the script is controlling a physical system, these errors can be dangerous. So, we need to do all that we can to stop the system in a safe manner, even in the presence of bugs, malformed input, etc. To do so, we'll use the `try... except... finally` construction.

In the example script in this folder, we use a `ZeroDivisionError` to demonstrate dealing with exceptions. Other common exceptions can be found [here](https://docs.python.org/3/library/exceptions.html#bltin-exceptions). Knowing this list can also help with debugging your scripts.

After our usual file header and imports, we try to divide by zero on line 32 (commented out on the commit from 02/04/18):

``` test_exception = 1/0 ```

If this line were not commented out, when we run the script, it will crash and provide a traceback like:

        ---------------------------------------------------------------------------
        ZeroDivisionError                         Traceback (most recent call last)
        ~/Documents/Research/CRAWLAB Code Review/20180205_Vaughan_Exceptions/20180205_Vaughan_Exceptions.py in <module>()
             31 # below commented out, you should get a traceback to this line of the script and an
             32 # explanation of the error
        ---> 33 test_exception = 1/0
             34
             35 # If we instead, wrap our calculation in a try... except block looking for this error

        ZeroDivisionError: division by zero

Starting on line 36, we wrap this same calculation in a `try... except` block:

``` python 
        try:
            test_exception = 1/0
            print('The result is: {:0.3f}'.format(test_exception))

        except(ZeroDivisionError):
            print('Error: You are trying to divide by zero.')
```

In this case, the script will no longer crash. Instead, we "catch" the `ZeroDivisionError` exception and print `Error: You are trying to divide by zero.`. 

If we want to take some action if an exception occurs, but still cause the exception to crash the script, we can re-raise the exception in by adding `raise` to the end of the `except` block:

``` python 
        try:
            test_exception = 1/0
            print('The result is: {:0.3f}'.format(test_exception))

        except(ZeroDivisionError):
            print('Error: You are trying to divide by zero.')
            raise
```

***Key Point:*** Make your `except` clauses as narrowly-scoped as possible. Only catch exceptions that you are explicitly handling. In other words, do  ***NOT*** do this:

``` python 
        try:
            test_exception = 1/0
            print('The result is: {:0.3f}'.format(test_exception))

        except:
            print('Error: You are trying to divide by zero.')
```

In this case, *any* exception that occurs while this block is running will be treated as if it were a `ZeroDivisionError`.

We can also add a `finally` clause to the `try... except` block to create a `try... except... finally` block. Any code within the `finally` clause of the block will *always* run, whether an exception is raised or not. This is shown in the block starting on line 51: 

``` python
        try:
            test_exception = 1/0
            print('The result is: {:0.3f}'.format(test_exception))

        except(ZeroDivisionError):
            print('Error: You are trying to divide by zero.')

        finally:
            print('Code in the finally block will *always* run.')
```


A common construction for us will have a `while True:` statement containing the system's main control loop inside a `try... except... finally` block. We will most often explicitly catch the `KeyboardInterrupt` and `SystemExit` exceptions. The `KeyboardInterrupt` exception is what is called when we stop a script using `Control-C` from the terminal.

``` python
        try:
             while True:
                print('Running the control loop...')
                time.sleep(0.1)
        
        except (KeyboardInterrupt, SystemExit):
            print('In some cases, we should also stop the system here.')
            print('Typically, we will save data, etc. in the except clause')
    
        finally:
            print('Then, safely stop the system by stopping motors, closing servers, etc.')
```

Whatever we do in the `except` clause needs to happen quickly relative to the system dynamics. It will run prior to the `finally` clause for the `KeyboardInterrupt` and `SystemExit` exceptions. So, if we only have the safe-stop code in the `finally` block, it will wait for this clause ot finish running. One option to combat this is to include the code necessary to stop the system safely in both the `except` and `finally`clauses. The downside to this is that the code is repeated, which is usually bad practice. In some cases, it may cause additional errors, as the code will be attempting to close files, servers, etc. and stop motors, etc. more than once. However, one *large* benefit is that the `finally` block *always* runs, so the system will still stop safely in cases of exceptions other than the `KeyboardInterrupt` and `SystemExit` we are explicitly catching.

For more discussion of this topic, see the links below:
* [SciPy Lecture Notes Section 1.2.8](http://www.scipy-lectures.org/intro/language/exceptions.html)
* [Python Documentation](https://docs.python.org/3/tutorial/errors.html)
* [List of Built-in Exceptions](https://docs.python.org/3/library/exceptions.html#bltin-exceptions)
* [Simple custom exception example](https://dbader.org/blog/python-custom-exceptions)