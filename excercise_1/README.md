# Excercise ROS + Supervised Learning
This is an excercise for the course `AI in Robotics` at University of Applied Science (HTW) Berlin.

## ROS Publisher/Subscriber:  
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

## Datasets:  
https://keras.io/datasets/

## Camera Subscriber:  
Pay attention, you have to specify the buffer parameter when initilizing a subscriber. The formula is:  
height*width*depth*bytes-per-channel*ros-overhead  
To simplify things, use ros-overhead=2  
So for a simple 28x28 RGB image (greyscale counts as RGB with depth=1) the formula would look like this:  
28*28*1*4*2=6272  
`Note:`  
For the mnist example data, no buffer manual adaption is required.

## Excercise  
Required part to pass course.
### 1)
* Make yourself familiar with all the existing code. But especially with `catkin_ws/src/camera_pseudo/src/CameraPseudo.py` and `ai_train/mnist_cnn_modified.py`
* Write a subscriber to receive image data from the following topic
  * /camera/output/specific/compressed_img_msgs
* Train a model of your choice for the mnist data with keras (or use the predefined in `ai_train/models/`)
* Include your trained model in `catkin_ws/src/prediction/src/Prediction.py` and predict the value based on the subscribed image inside the subscribers callback (pay attention, that predicting in a callback is a different thread: https://stackoverflow.com/questions/46725323/keras-tensorflow-exception-while-predicting-from-multiple-threads)
* Publish your predicted number to (real class number not the one-hot encoded one)
  * /camera/input/specific/number
* Subscribe to the following topic to verify your prediction
  * /camera/output/specific/check

### 2)
* Write a subscriber to receive image data from the following topic
  * /camera/output/random/compressed_img_msgs
* Subscribe for verification to
  * /camera/output/random/number
* Since the images do NOT stay consistent, the image and according value will be published simultaneously. Safe if locally for verification.

### 3)
* Write a `SOLUTION.md` concluding
  * the ROS basics and publish-subscriber principle in your own words
  * describing the process of your implementation
  * the ROS graph (use rqt therefore)

## Excercise
Optional part for further knowledge.

### 4)
* Make use of ROS network capability and use publishing camera data on one device and predicting on another (see ROS network tutorial therefore: http://wiki.ros.org/ROS/Tutorials/MultipleMachines, do not use plain python. Use ROS on both machines/VMs)
* Adapt your application to a more complex problem, like cifar or real images based on webcam input (you can find an implemented publisher for the webcam in `CameraPseudo.py`)
	
