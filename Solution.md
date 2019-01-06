# Repo
<https://github.com/FlorianSauer/RobotikROSUebung>

# Ros Basics 


Ros is an operating system used for robotic research and development. In the context of our project this semester, we used ROS to predict numbers as displayed in pictures. In doing this we learned about some of the ROS basics. The first of these basics is the ROS workspace. ROS uses the workspace to combine different spaces, which allows us to work with different packages and versions of ROS. In our case the main advantage was package management, particularly as it pertains to building our application. A further advantage was shell support in the sense that running 
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
 
# Publisher and Subscriber
### Publish-Subscriber Principle
The Publisher connects via TCP to a central Broker, which in the case of ROS is built into ROSCORE. The publisher 
notifies the broker that he wants to publish a message N under a topic T.
The broker provides the necessary infrastructure to further distribute this message to several other subscribers of 
this topic. If a new subscriber subscribes to the topic, he will receive the last published message.

### Subscriber and Callbacks

The publish-subscriber pattern works asynchronously. Subscribers therefore require callbacks to handle the received 
messages. Callbacks are functions passed to a subscriber via a reference. This callback gets called, when a new message 
was received from the broker. For this exercise, the passing of a callback to a subscriber was wrapped by the 
RosSubscriber class, where only the handle method has to be implemented.

# Development 

The project was developed using the given exercise template and the 'mnist' dataset example.
The counterparts for the image-publishing-topics in 'camera-pseudo' node were added and implemented inside the 
'prediction' node. The already pre-learned prediction model was taken from the 'mnist' example. To load a pre-learned 
model, a loadMakeModel() method was implemented.
Because the image message received by the 'camera-pseudo' node was not correctly shaped to be processed by the Keras 
Model, a unpackMessageStatic method had to be implemented in the class PredictionCISubscriber (superclass is the 
RosSubscriber class).

Also the checking of a correct prediction for random images was skipped, because given the current state of the project it is impossible 
to link an image-prediction with the test of its correctness.This is also due to the asynchronious communication between nodes, and can be surmounted by providing an identifier to the images sent and the corresponding tests.