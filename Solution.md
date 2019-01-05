# Ros Basics 


Ros is an operating system used for robotic research and development. In the context of our project this semester, we used ROS to predict numbers as displayed in pictures. In doing this we learned about some the ROS basics. The first of these basics is the ROS workspace. ROS uses the workspace to combine different spaces, which allows us to work with different packages and versions of ROS. In our case the main advantage was package management, particularly as it pertains to building our application. A further advantage was shell support in the sense that running 
```
rosrun [package_name] [node_name] 
```
was simplified and allowed for automatically finding the package inside of catkin workspace. 
   Everytime we used ROS we started by calling 
   ```
roscore [package_name] [node_name] 
```
    Roscore takes care of starting a ROS Master, a ROS Parameter Server and a rosout logging node. After calling roscore we called 
   ```rosrun [package_name] [node_name]``` 
   Rosrun takes care of starting a rosnode from a given package. A ROS node is at the core of the ROS functionality. It is an executable that uses ROS to communicate with other nodes. A node can be a publisher, subscriber, or a service and uses the client library to communicate with other nodes.  

This is required because ROS relies on the notion of combining spaces using the shell environment. This makes developing against different versions of ROS or against different sets of packages easier. 


# Publisher-Subscriber Principle
Publisher connects via TCP to a central Broker, which in the case of ROS is built into ROSCORE. The publisher notifies the broker that he wants to publish a message N under a topic T.  The broker provides the necessary infrastructure to further distribute this message to several other subscribers of this topic. If a new subscriber subscribes to the topic he will receive the last published message. 



