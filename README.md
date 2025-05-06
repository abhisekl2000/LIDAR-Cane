Written in MicroPython 
Object Oriented programming with threading 
which consists of two scripts: tf_luna.py and main.py

  - tf_luna.py
  - A class for LIDAR module 
  - Decodes bitstream into distance
    
  - main.py
  - Assigns pinouts to LIDAR modules and NPN transistor
  - Creates independent threads for LIDAR module 1 and 2 to run in parallel
  - Runs obstacle detection algorithm by computing OPP1 and OPP2 every second
