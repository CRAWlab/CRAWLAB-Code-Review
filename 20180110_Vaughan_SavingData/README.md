### Saving to a csv File with a Header
This simple script demonstrates saving data to a comma-separated-value (csv) file. We make the data more-easily processed by including a header in the csv.

We begin, like most of our scripts by importing numpy. In addition, we'll import the `datetime` module to allow us to create unique filenames using date/timestamps. 

``` python 
import numpy as np
import datetime         # We'll use this to define a unique filename
```

Lines 26-29 of the script create some example data for us to use here. In order to save the data into a csv file, we need to create a single array where each column represents one variable of the data. There are many ways to do this. In this case we are using the NumPy `stack` method:

``` python
data_to_save = np.stack((t, data1, data2, data3), axis=1)
```

For experimental results, we often want to identify the trial using a timestamp. We can do that by:
 
``` python
data_filename = 'Data_' + datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S') + '.csv'
```

In that snippet ```datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S')``` gets the current time of the computer in use and formats is as `YYYY-mm-dd-HHMMSS`. This gets combined with `Data_` and `.csv` to create a unique filename.

Note that it's probably a good idea to include more info in the filename and
to name it more distinctly than `Data_`. You can/should use string formatting 
methods to do this automatically. [This post](https://pyformat.info) has a lot of good examples using the `.format()` method to do so. The new-in-Python-3.6 f-strings are nice too. [This is a good overview of them.](https://www.blog.pythonlibrary.org/2017/02/08/new-in-python-formatted-string-literals/).


Now, we're ready to save the data. First, we'll create the string defining the header, making sure to name the columns to represent the actual data and include units. The names here are obviously *way* too generic.

``` python 
file_header = 'Time (s), Column 1 (units), Data Column 2 (units), Column 3 (units)'
```
    
Now, we're ready to actually we save the file:

```python
np.savetxt(data_filename, data_to_save, header=file_header, delimiter=',')
```

#### A Note on Experimental Data
If you are working with experimental data, do *NOT* write to the file during each time-step of the system. Instead, append/save that time-step's data into (preferably-pre-defined) arrays. Then, once the trial is complete, process and save those arrays, just like we do above.