# radar-task
Repository for University project of the Real-Time Embedded Systems course.  
The software is intended to simulate the working of a naval radar, by using randomly moving balls on a flat space. The
user can pass a number between 2 and 5 as input parameter, which serves the purpose to dictate the number of samples that the
radar has to take before starting to predict the next position of the balls.

## Requirements and Installation
The program uses [ptask](https://github.com/glipari/ptask), a Period Real-Time Task interface to pthreads and the
[Allegro4.4](https://liballeg.org/index.html) graphics library.  
To install the allegro4.4 library type:  

`sudo apt-get install liballegro4.4 liballegro4.4-dev` on Debian-based systems  
`sudo yum install allegro allegro‐devel` under Red Hat  

### Build
Go in the program folder and type:  

`make clean` to clean the installation folder  
`make` to build the program  

### Run
To run the program you must have superuser privileges. You can launch the program using `sudo ./radar`.
Press the `i` key followed by a number between 2 and 5 and the `Enter` key to select the number of positions that the radar
needs to record before starting to predict the next positions of the balls. Press the `Space` key to launch balls on the
screen and press the `ESC` key to close the application.

## Program Structure  
### Global Data Structures
The program makes use of global variables and data structures. In particular:

* the **`status`** struct type is used to store the information of a single ball (velocity, acceleration, color, ecc...)
* the single **`m`** struct is used to store the interceptions made by the radar; it also contains the condition variable
  `ptrt`, which is used to synchronize the radar task with the radar tracking task
* the **`t`** struct contains the positions computed by the tracking threads

### Tasks
The program makes use of the **ptask** library to create period tasks. In particular, in this program there are three types
of periodic tasks:

* **`radartask`** is the task used to handle the radar rotation and to intercept the position of the balls when they enter
the radar beam
* **`path_randomizer`** is a lower priority task that changes the acceleration of the balls at periodic intervals; the balls
can either increase their longitudinal acceleration, decrease their longitudinal acceleration or change their transversal acceleration (to simulate a plane turn)
* **`ball_task`** is created when the `space` key is pressed and its purpose is to handle the movement of a single ball, by
periodically updating the information stored in the `status` struct and drawing the ball on the underlying screenbuffer (the balls are shown on the screen only when they are intercepted by the radar beam)

The program also uses conventional posix threads named **`tracking_thread`**. Each `tracking_thread` is assigned to a ball
when its position is intercepted by `radartask`and it waits to be awakened by the same task when one of these conditions is
met:

* **case 1**: an object is intercepted inside the 'burst' period in which the radar beam intercepts multiple position of the same ball that are then averaged to a single one when the burst is over
* **case 2**: the burst period is over; the thread is awakened so it can start to compute the mean position of the ball
* **case 3**: when contact with the ball is lost; the thread then becomes available to track a new ball

To predict the next position of the ball the tracking thread uses only the speed of the ball calculated at the last position
if the number of samples given as input(`k_samples`) is equal to 2. If 3 or more samples are given as input the program
can then use the last sample to calculate the acceleration of the ball using the following formula:

given the 4 positions: **_x3 x2 x1 x0_** where **_x3_** is the position to predict, _**v**_ is the speed and _**a**_ is the
acceleration:

**_x3 = x2 + (v2 + a*dt)*dt_**    but if **_v2 = (x2 - x1)/dt_**  and **_a = (v2 - v1)/dt_  _v1 = (x1 - x0)/dt_**  
**_x3 = x2 + (2v2 - v1)*dt_ = _x2 + 2(x2 - x1) - (x1 - x0)_ = _3*x2 - 3*x1 + x0_** 
