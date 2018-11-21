# How to setup Pycharm to work with the ROS-Python-binary + ROS-Modules


###Step 1
-> File  
-> Settings  
-> Project Settings  
-> Project Interpreter  
-> Cogwheel thing  
-> show all  
-> '+' symbol  
-> configure a python interpreter, this step is optional, you can also use a already existing python interpreter

###Step 2
Then again in the project interpreter settings:   
-> cogwheel  
-> show all  
-> select python interpreter you either set up in the previous step or which you want to use  
-> Folder symbol (should be the last symbol on the right)  
-> add custom paths to not importable python modules.   
I added/activated only the following: 
- /opt/ros/$ROS_VERSION/lib/python2.7/dist-packages
- /opt/ros/$ROS_VERSION/lib/python2.7/dist-packages/rospy
- /opt/ros/$ROS_VERSION/lib/python2.7
- /usr/local/lib/python2.7/dist-packages
- /usr/lib/python2.7  

deactivate everything else with the '-' symbol

###Step 3
If a Module is still missing:  
-> open a terminal  
-> source all those ROS scripts  
-> open a python console  
-> try importing your library (`import $LIBRARY`)  
-> if success: `print $LIBRARY.__file__`  
-> you get the path to that library, add this path (See Step 2).