# How to setup Pycharm to work with the ROS-Python-binary + ROS-Modules

->File  
->Settings  
->Project Settings  
->Project Interpreter  
->Cogwheel thing  
->show all  
->'+' symbol  
->configure a python interpreter, this step is optional, you can also use a already existing python interpreter

Then again in the project interpreter settings:   
->cogwheel  
->show all  
->select python interpreter you either set up in the previous step or which you want to use  
->Folder symbol (should be the last symbol on the right)  
->add custom paths to not importable python modules.   
I added/activated only the following: 
- /opt/ros/$ROS_VERSION/lib/python2.7/dist-packages
- /opt/ros/$ROS_VERSION/lib/python2.7/dist-packages/rospy
- /opt/ros/$ROS_VERSION/lib/python2.7
- /usr/local/lib/python2.7/dist-packages
- /usr/lib/python2.7  

deactivate everything else with the '-' symbol
