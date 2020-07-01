ROS in 5 Days Notes
===================

Notes (mostly copied verbatim) from robotigniteacademy "ROS in 5 Days (C++)"

## Unit 2: Topics

### Part 1: Publisher

Create a new package named `topic_publisher_pkg`. When creating the package, add as dependencies roscpp and std_msgs.

```bash
catkin_create_pkg topic_publisher_pkg roscpp std_msgs
```

Inside the `src` folder of the package, create a new file named `simple_topic_publisher.cpp`:

```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000);
    ros::Rate loop_rate(2);
    
    std_msgs::Int32 count;
    count.data = 0;
    
    while (ros::ok())
    {
        pub.publish(count);
        ros::spinOnce();
        loop_rate.sleep();
        ++count.data;
    }
    
    return 0;
}
```

Create a launch file for launching this code.

Do the necessary modifications to your `CMakeLists.txt` file, and compile the package:
```cmake
add_executable(simple_topic_publisher src/simple_topic_publisher.cpp)
add_dependencies(simple_topic_publisher ${simple_topic_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_topic_publisher
   ${catkin_LIBRARIES}
 )
```

Execute the launch file to run your executable.

Nothing happens? Well... that's not actually true! You have just created a topic named `/counter`, and published through it as an integer that increases indefinitely. Let's check some things.

A topic is like a pipe. Nodes use topics to publish information for other nodes so that they can communicate.
You can find out, at any time, the number of topics in the system by doing a `rostopic list`. You can also check for a specific topic.

```bash
rostopic list | grep  '/counter'
```
Here, you have just listed all of the topics running right now and filtered with the grep command the ones that contain the word `/counter`. If it appears, then the topic is running as it should.

You can request information about a topic by doing `rostopic info <name_of_topic>`.

Now, type rostopic info /counter.

```rostopic info /counter```

```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>
// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_publisher"); // Initiate a Node named 'topic_publisher'
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000); // Create a Publisher object, that will                                                                                // publish on the /counter topic messages
                                                                         // of type Int32
    ros::Rate loop_rate(2); // Set a publish rate of 2 Hz
    
    std_msgs::Int32 count; // Create a variable of type Int32
    count.data = 0; // Initialize 'count' variable
    
    while (ros::ok()) // Create a loop that will go until someone stops the program execution
    {
        pub.publish(count); // Publish the message within the 'count' variable
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
        ++count.data; // Increment 'count' variable
    }
    
    return 0;
}
```

So basically, what this code does is to initiate a node and create a publisher that keeps publishing into the `/counter` topic a sequence of consecutive integers. Summarizing:

A publisher is a node that keeps publishing a message into a topic. So now... what's a topic?

A topic is a channel that acts as a pipe, where other ROS nodes can either publish or read information. Let's now see some commands related to topics (some of them you've already used).

To get a list of available topics in a ROS system, you have to use the next command: `rostopic list`

```bash
rostopic echo <topic_name>
```

This command will start printing all of the information that is being published into the topic, which sometimes (ie: when there's a massive amount of information, or when messages have a very large structure) can be annoying. In this case, you can read just the last message published into a topic with the next command:
`rostopic echo <topic_name> -n1`

#### Messages

As you may have noticed, topics handle information through messages. There are many different types of messages.

In the case of the code you executed before, the message type was an `std_msgs/Int32`, but ROS provides a lot of different messages. You can even create your own messages, but it is recommended to use ROS default messages when its possible.

Messages are defined in `.msg` files, which are located inside a `msg` directory of a package.

To get information about a message, you use the next command:

```bash
rosmsg show <message>
rosmsg show std_msgs/Int32
```

In this case, the Int32 message has only one variable named data of type int32. This Int32 message comes from the package `std_msgs`, and you can find it in its msg directory. If you want, you can have a look at the Int32.msg file by executing the following command:

`roscd std_msgs/msg/`


### Part 2: Subscriber

You've learned that a topic is a channel where nodes can either write or read information. You've also seen that you can write into a topic using a publisher, so you may be thinking that there should also be some kind of similar tool to read information from a topic. And you're right! That's called a subscriber. A subscriber is a node that reads information from a topic. Let's execute the next code:

**Example 2.3**

Create a new package named `topic_subscriber_pkg`. When creating the package, add as dependencies roscpp and std_msgs.

Inside the src folder of the package, create a new file named simple_topic_subscriber.cpp. Inside this file, copy the contents of `simple_topic_subscriber.cpp`.

```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback);
    
    ros::spin();
    
    return 0;
}
```

What's up? Nothing happened again? Well, that's not actually true... Let's do some checks.

```bash
rostopic echo /counter
```
You should see an output like this:

> $ rostopic echo /counter
>
> WARNING: no messages received and simulated time is active.
>
> Is /clock being published?

And what does this mean? This means that nobody is publishing into the /counter topic, so there's no information to be read. Let's then publish something into the topic and see what happens. For that, let's introduce a new command:

`rostopic pub <topic_name> <message_type> <value>`

This command will publish the message you specify with the value you specify, in the topic you specify.

Open another shell (leave the one with the rostopic echo opened) and type the next command:

```bash
rostopic pub /counter std_msgs/Int32 5
```
Now check the output of the console where you did the rostopic echo again. You should see something like this: .

> $ rostopic echo /counter
>
> WARNING: no messages received and simulated time is active.
>
> Is /clock being published?
>
> data:
>
> 5
>
> ---

This means that the value you published has been received by your subscriber program (which prints the value on the screen).


Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.


```cpp
#include <ros/ros.h>
#include <std_msgs/Int32.h>

void counterCallback(const std_msgs::Int32::ConstPtr& msg) // Define a function called 'callback' that receives a                                                                // parameter named 'msg' 
{
  ROS_INFO("%d", msg->data); // Print the value 'data' inside the 'msg' parameter
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber"); // Initiate a Node called 'topic_subscriber'
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback); // Create a Subscriber object that will                                                                               // listen to the /counter topic and will
                                                                          // call the 'callback' function each time                                                                             // it reads something from the topic
    
    ros::spin(); // Create a loop that will keep the program in execution
    
    return 0;
}
```
So now, let's explain what has just happened. You've basically created a subscriber node that listens to the /counter topic, and each time it reads something, it calls a function that does a print of the msg. Initially, nothing happened since nobody was publishing into the /counter topic, but when you executed the rostopic pub command, you published a message into the /counter topic, so your subscriber has printed that number and you could also see that message in the rostopic echo output. Now everything makes sense, right?

Now let's do some exercises to put into practice what you've learned!

How to Prepare CMakeLists.txt and package.xml for Custom Topic Message Compilation

Now you may be wondering... in case I need to publish some data that is not an Int32, which type of message should I use? You can use all ROS defined (rosmsg list) messages. But, in case none fit your needs, you can create a new one.

In order to create a new message, you will need to do the following steps:

    Create a directory named 'msg' inside your package
    Inside this directory, create a file named Name_of_your_message.msg (more information down)
    Modify CMakeLists.txt file (more information down)
    Modify package.xml file (more information down)
    Compile
    Use in code

For example, let's create a message that indicates age, with years, months, and days.

1) Create a directory msg in your package.

roscd <package_name>

mkdir msg

2) The Age.msg file must contain this:

float32 years

float32 months

float32 days

3) In CMakeLists.txt

You will have to edit four functions inside CMakeLists.txt:

    find_package()
    add_message_files()
    generate_messages()
    catkin_package()

I. find_package()

This is where all the packages required to COMPILE the messages of the topics, services, and actions go. In package.xml, you have to state them as build_depend.

HINT 1: If you open the CMakeLists.txt file in your IDE, you'll see that almost all of the file is commented. This includes some of the lines you will have to modify. Instead of copying and pasting the lines below, find the equivalents in the file and uncomment them, and then add the parts that are missing.

find_package(catkin REQUIRED COMPONENTS

       roscpp

       std_msgs

       message_generation   # Add message_generation here, after the other packages

)

II. add_message_files()

This function includes all of the messages of this package (in the msg folder) to be compiled. The file should look like this.

add_message_files(

      FILES

      Age.msg

    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO

III. generate_messages()

Here is where the packages needed for the messages compilation are imported.

generate_messages(

      DEPENDENCIES

      std_msgs

) # Dont Forget to uncoment here TOO

IV. catkin_package()

State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as exec_depend.

catkin_package(

      CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here

)

Summarizing, this is the minimum expression of what is needed for the CMakaelist.txt to work:

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.

cmake_minimum_required(VERSION 2.8.3)

project(topic_ex)

​

​

find_package(catkin REQUIRED COMPONENTS

  roscpp

  std_msgs

  message_generation

)

​

add_message_files(

  FILES

  Age.msg

)

​

generate_messages(

  DEPENDENCIES

  std_msgs

)

​

catkin_package(

  CATKIN_DEPENDS roscpp std_msgs message_runtime

)

​

include_directories(

  ${catkin_INCLUDE_DIRS}

)

4) Modify package.xml

Just add these 3 lines to the package.xml file.

<build_depend>message_generation</build_depend>

​

<build_export_depend>message_runtime</build_export_depend>

<exec_depend>message_runtime</exec_depend>

This is the minimum expression of the package.xml

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.
```xml
<?xml version="1.0"?>
<package format="2">
  <name>topic_ex</name>
  <version>0.0.0</version>
  <description>The topic_ex package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
  <export>
  </export>
</package>
```
5) Now you have to compile the msgs. To do this, you have to type in a WebShell:

Execute in shell:
```bash
roscd; cd ..
catkin_make
source devel/setup.bash
```
**VERY IMPORTANT**: When you compile new messages, there is still an extra step before you can use the messages. You have to type in the Webshell, in the **catkin_ws**, the following command: **source devel/setup.bash**.

This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.

If you don't do this, it might give you an import error, saying it doesn't find the message generated.

If your compilation goes fine, you should see something similar to this:

HINT 2: To verify that your message has been created successfully, type in your webshell rosmsg show Age. If the structure of the Age message appears, it will mean that your message has been created successfully and it's ready to be used in your ROS programs.

To use Custom Messages in Cpp files

You will have to add to your CMakeLists.txt the following extra lines to compile and link your executable ( in this example its called publish_age.cpp ) :
```cmake
add_executable(publish_age src/publish_age.cpp)

add_dependencies(publish_age ${publish_age_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(publish_age

   ${catkin_LIBRARIES}

 )

add_dependencies(publish_age topic_ex_generate_messages_cpp)
```
As you can see, there is nothing new here except for the last line:

`add_dependencies(publish_age topic_ex_generate_messages_cpp)`

And why do you need to add this extra add_dependencies()? Well, this is to make sure that all the messages contained in the package (topic_ex) are compiled before we create the publish_age executable. In this case, the publish_age executable uses the custom message we have just created: Age.msg. So... what would happen if we try to build the executable before those messages are built? Well, it would fail, of course. Then, with this line, you will make sure that the messages are built before trying to build your executable.

### Unit 3: Services

Part 1

Congratulations! You now know 75% of ROS-basics!
The reason is that, with topics, you can do more or less whatever you want and need for your astromech droid. Many ROS packages only use topics and have the work perfectly done.

Then why do you need to learn about services?
Well, that's because for some cases, topics are insufficient or just too cumbersome to use. Of course, you can destroy the Death Star with a stick, but you will just spend ages doing it. Better tell Luke SkyWalker to do it for you, right? Well, it's the same with services. They just make life easier.
Topics - Services - Actions

To understand what services are and when to use them, you have to compare them with topics and actions.
Imagine you have your own personal BB-8 robot. It has a laser sensor, a face-recognition system, and a navigation system. The laser will use a Topic to publish all of the laser readings at 20hz. We use a topic because we need to have that information available all the time for other ROS systems, such as the navigation system.

The Face-recognition system will provide a Service. Your ROS program will call that service and WAIT until it gives you the name of the person BB-8 has in front of it.
The navigation system will provide an Action. Your ROS program will call the action to move the robot somewhere, and WHILE it's performing that task, your program will perform other tasks, such as complain about how tiring C-3PO is. And that action will give you Feedback (for example: distance left to the desired coordinates) along the process of moving to the coordinates.

So... What's the difference between a Service and an Action?
Services are Synchronous. When your ROS program calls a service, your program can't continue until it receives a result from the service.
Actions are Asynchronous. It's like launching a new thread. When your ROS program calls an action, your program can perform other tasks while the action is being performed in another thread.

Conclusion: Use services when your program can't continue until it receives the result from the service.

Services Introduction

Enough talk for now, let's go play with a robot and launch a prepared demo!

**Example 3.1**

Go to the WebShell and do the following:

Execute in WebShell #1

roslaunch iri_wam_aff_demo start_demo.launch

This will make the Wam robot-arm of the simulation move.
You should get something similar to this:

END **Example 3.1**
What did you do just now?

The launch file has launched two nodes (Yes! You can launch more than one node with a single launch file):

    /iri_wam_reproduce_trajectory
    /iri_wam_aff_demo

The first node provides the /execute_trajectory service. This is the node that contains the service. The second node, performs calls to that service. When the service is called, the robot will execute a given trajectory.
Let's learn more about services.

**Example 3.2**

Let's see a list of the available services in the Wam robot. For that, open another shell.
You have to leave the start_demo.launch running, otherwise the services won't be there to see them.

Execute the following command in a different shell from the one that has the roslaunch start_demo.launch running:

Execute in WebShell #2

rosservice list

You should see something like the following image, listing all the services available:

WebShell #2 Output

user ~ $ rosservice list

/camera/rgb/image_raw/compressed/set_parameters

/camera/rgb/image_raw/compressedDepth/set_parameters

/camera/rgb/image_raw/theora/set_parameters

/camera/set_camera_info

/camera/set_parameters

/execute_trajectory

/gazebo/apply_body_wrench

...

WARNING: If the /execute_trajectory server is not listed, maybe that's because you stopped the start_demo.launch. If that is the case, launch it again and check for the service.

There are a few services, aren't there? Some refer to the simulator system (/gazebo/...), and others refer to the Kinect Camera (/camera/...) or are given by the robot himself (/iri_wam/...). You can see how the service /execute_trajectory is listed there.

You can get more information about any service by issuing the following command:

rosservice info /name_of_your_service

Execute the following command to know more about the service /execute_trajectory

Execute in WebShell #2

rosservice info /execute_trajectory

WebShell #2 Output

user ~ $ rosservice info /execute_trajectory

Node: /iri_wam_reproduce_trajectory

URI: rosrpc://ip-172-31-17-169:35175

Type: iri_wam_reproduce_trajectory/ExecTraj

Args: file

Here you have two relevant parts of data.

    Node: It states the node that provides (has created) that service.
    Type: It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of package_where_the_service_message_is_defined / ** Name_of_the_File_where_Service_message_is_defined. In this case, the package is **iri_wam_reproduce_trajectory, and the file where the Service Message is defined is called ExecTraj.
    Args: Here you can find the arguments that this service takes when called. In this case, it only takes a trajectory file path stored in the variable called file.

END **Example 3.2**
Want to know how this /execute_trajectory service is started?

Here you have an example on how to check the start_demo.launch file through WebShell.

**Example 3.3**

Do you remember how to go directly to a package and where to find the launch files?

Execute in WebShell #2

roscd iri_wam_aff_demo

cd launch/

cat start_demo.launch

You should get something like this:

<launch>

​

  <include file="$(find iri_wam_reproduce_trajectory)/launch/start_service.launch"/>

​

  <node pkg ="iri_wam_aff_demo"

        type="iri_wam_aff_demo_node"

        name="iri_wam_aff_demo"

        output="screen">

  </node>

​

</launch>

Some interesting things here:

1) The first part of the launch file calls another launch file called start_service.launch.

That launch file starts the node that provides the /execute_trajectory service. Note that it's using a special ROS launch file function to find the path of the package given.

<include file="$(find package_where_launch_is)/launch/my_launch_file.launch"/>

2) The second part launches a node just as you learned in the ROS Basics Unit. That node is the one that will call the /execute_trajectory service in order to make the robot move.

How to call a service

You can call a service manually from the console. This is very useful for testing and having a basic idea of how the service works.

rosservice call /the_service_name TAB-TAB

Info: TAB-TAB means that you have to quickly press the TAB key twice. This will autocomplete the structure of the Service message to be sent for you. Then, you only have to fill in the values.

**Example 3.4**

Let's call the service with the name /trajectory_by_name by issuing the following command. But before being able to call this Service, you will have to launch it. For doing so you can execute the following command:

Execute in WebShell #1

roslaunch trajectory_by_name start_service.launch

Now let's call the Service.

Execute in WebShell #2

rosservice call /trajectory_by_name [TAB]+[TAB]

When you [TAB]+[TAB], an extra element appears. ROS autocompletes with the structure needed to input/request the service.
In this case, it gives the following structure:

"traj_name: 'the_name_of_the_trajectory_you_want_to_execute'"

The /trajectory_by_name Service is a service provided by the Robot Ignite Academy as an example, that allows you to execute a specified trajectory with the robotic arm.

Use that service to execute one trajectory with the WAM Arm. The trajectories that are available are the following ones: init_pos, get_food and release_food.

Execute in WebShell #2

rosservice call /trajectory_by_name "traj_name: 'get_food'"

Did it work? You should have seen the robotic arm executing a trajectory, like the one at the beginning of this Chapter:

You can now try to call the service providing different trajectory names (from the ones indicated above), and see how the robot executes each one of them.

END **Example 3.4**
But how do you interact with a service programatically?

**Exercise 3.1**

    Create a new package named service_client_pkg. When creating the package, add as dependency roscpp.

    Inside the src folder of the package, create a new file named simple_service_client.cpp. Inside this file, copy the contents of simple_service_client.cpp

    Create a launch file for launching this code. Keep in mind that, in order to be able to call the /trajectory_by_name service, you need to have it running.

    Do the necessary modifications to your CMakeLists.txt file, and compile the package.

    Execute the launch file to run your executable.

What do you think it will do?

```cpp
#include "ros/ros.h"
#include "trajectory_by_name_srv/TrajByName.h"
// Import the service message used by the service /trajectory_by_name

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_client"); // Initialise a ROS node with the name service_client
  ros::NodeHandle nh;

  // Create the connection to the service /trajectory_by_name
  ros::ServiceClient traj_by_name_service = nh.serviceClient<trajectory_by_name_srv::TrajByName>("/trajectory_by_name");
  trajectory_by_name_srv::TrajByName srv; // Create an object of type TrajByName
  srv.request.traj_name = "release_food"; // Fill the variable traj_name with the desired value
  
  if (traj_by_name_service.call(srv)) // Send through the connection the name of the trajectory to execute
  {
    ROS_INFO("%s", srv.response.status_message.c_str()); // Print the result given by the service called
  }
  else
  {
    ROS_ERROR("Failed to call service /trajectory_by_name");
    return 1;
  }

  return 0;
}
```

How to know the structure of the service message used by the service?

**Example 3.5**

You can do a *rosservice info * to know the type of service message that it uses.

rosservice info /name_of_the_service

This will return you the name_of_the_package/Name_of_Service_message

Then, you can explore the structure of that service message with the following command:

rossrv show name_of_the_package/Name_of_Service_message

Execute the following command to see what is the service message used by the /trajectory_by_name service:

Execute in WebShell #2

rosservice info /trajectory_by_name

You should see something like this:

WebShell #2 Output

Node: /traj_by_name_node

URI: rosrpc://rosdscomputer:38301

Type: trajectory_by_name_srv/TrajByName

Args: traj_name

Now, execute the following command to see the structure of the message TrajByName:

Execute in WebShell #2

rossrv show trajectory_by_name_srv/TrajByName

You should see something like this:

WebShell #2 Output

user catkin_ws $ rossrv show trajectory_by_name_srv/TrajByName

string traj_name

---

bool success

string status_message

Does it seem familiar? It should, because it's the same structure as the Topics messages, with some addons.
1- Service messages have the extension .srv. Remember that Topic messages have the extension .msg
2- Service messages are defined inside a srv directory instead of a msg directory.

You can type the following command to check it.

Execute in WebShell #2

roscd trajectory_by_name_srv; ls srv

3- Service messages have TWO parts:

**REQUEST**

---

**RESPONSE**

In the case of the TrajByName service, REQUEST contains a string called traj_name and RESPONSE is composed of a boolean named success, and a string named status_message.

The Number of elements on each part of the service message can vary depending on the service needs. You can even put none if you find that it is irrelevant for your service. The important part of the message is the three dashes ---, because they define the file as a Service Message.

Summarizing:

The REQUEST is the part of the service message that defines HOW you will do a call to your service. This means, what variables you will have to pass to the Service Server so that it is able to complete its task.

The RESPONSE is the part of the service message that defines HOW your service will respond after completing its functionality. If, for instance, it will return an string with a certaing message saying that everything went well, or if it will return nothing, etc...

END **Example 3.5**

**Exercise 3.2**

    Create a launch that starts the /execute_trajectory service , called my_robot_arm_demo.launch. As explained in the Example 3.3, this service is launched by the launch file start_service.launch, which is in the package iri_wam_reproduce_trajectory.

    Get information of what type of service message does this execute_trajectory service use, as explained in Example 3.5.

    Make the arm robot move following a trajectory, which is specified in a file.
    Modify the previous code of Exercise 3.1, which called the /trajectory_by_name service, to call now the /execute_trajectory service instead.

    Here you have the code necessary to get the path to the trajectory files based on the package where they are. Here, the trajectory file get_food.txt is selected, but you can use any of the available in the config folder of the iri_wam_reproduce_trajectory package.

#include <ros/package.h>

​

// This ros::package::getPath works in the same way as $(find name_of_package) in the launch files.

trajectory.request.file = ros::package::getPath("iri_wam_reproduce_trajectory") + "/config/get_food.txt";

    In order to be able to properly compile the the ros::package::getPath() function, you will need to add dependencies to the roslib library. You can do that by modifying the find_package() function in the CmakeLists.txt file. Like this:

find_package(catkin REQUIRED COMPONENTS

  roscpp

  roslib

)

    Modify the main launch file my_robot_arm_demo.launch, so that now it also launches the new C++ code you have just created.

    Compile again your package.

    Finally, execute the my_robot_arm_demo.launch file and see how the robot performs the trajectory.

END **Exercise 3.2**
Summary

Services provide functionality to other nodes. If a node knows how to delete an object on the simulation, it can provide that functionality to other nodes through a service call, so they can call the service when they need to delete something.

Services allow the specialization of nodes (each node specializes in one thing).

## Part2: How to Give a Service


Until now, you have called services that others provided. Now, you are going to create your own.

**Exercise 3.3**

    Create a new package named service_server_pkg. When creating the package, add roscpp as a dependency.

    Inside the src folder of the package, create a new file named simple_service_server.cpp. Inside this file, copy the contents of simple_service_server.cpp

    Create a launch file for launching this code.

    Do the necessary modifications to your CMakeLists.txt file, and compile the package.

    Execute the launch file to run your executable.

**C++ Program {3.2}: simple_service_server.cpp**

#include "ros/ros.h"

#include "std_srvs/Empty.h"

// Import the service message header file generated from the Empty.srv message

​

// We define the callback function of the service

bool my_callback(std_srvs::Empty::Request  &req,

                 std_srvs::Empty::Response &res)

{  

  // res.some_variable = req.some_variable + req.other_variable;

  ROS_INFO("My_callback has been called"); // We print an string whenever the Service gets called

  return true;

}

​

int main(int argc, char **argv)

{

  ros::init(argc, argv, "service_server");

  ros::NodeHandle nh;

​

  ros::ServiceServer my_service = nh.advertiseService("/my_service", my_callback); // create the Service called                                                                                          // my_service with the defined                                                                                        // callback

  ros::spin(); // mantain the service open.

​

  return 0;

}

**END C++ Program {3.2}: simple_service_server.cpp**

END **Exercise 3.3**

Did something happen?

Of course not! At the moment, you have just created and started the Service Server. So basically, you have made this service available for anyone to call it.

This means that if you do a rosservice list, you will be able to visualize this service on the list of available services.

Execute in WebShell #1

rosservice list

On the list of all available services, you should see the /my_service service.

/base_controller/command_select

/bb8/camera1/image_raw/compressed/set_parameters

/bb8/camera1/image_raw/compressedDepth/set_parameters

/bb8/camera1/image_raw/theora/set_parameters

...

/my_service

...

Now, you have to actually CALL it. So, call the /my_service service manually. Remember the calling structure discussed in the previous chapter and don't forget to TAB-TAB to autocomplete the structure of the Service message.

Execute in WebShell #1

rosservice call /my_service [TAB]+[TAB]

Did it work? You should've seen the message, 'My callback function has been called', printed at the output of the shell where you executed the service server code. Great!

INFO: Note that, in the example, there is a commented line in my_callback function. It gives you an example of how you would access the REQUEST given by the caller of your service. It's always req.variables_in_the_request_part_of_srv_message. In this case, this is not necessary because we are working with the Empty message, which is a special message that doesn't contain anything.

So, for instance, let's do a flashback to the previous chapter. Do you remember Example 3.5? Where you had to perform calls to a service in order to delete an object in the simulation? Well, for that case, you were passing the name of the object to delete to the Service Server in a variable called model_name. So, if you wanted to access the value of that model_name variable in the Service Server, you would have to do it like this:

req.model_name

Quite simple, right?

That commented line also shows you how you would return the RESPONSE of the service. For that, you have to access the variables in the RESPONSE part of the message. It would be like this: res.variables_in_the_response_part_of_srv_message.

And why do we use req and res for accessing the REQUEST and RESPONSE parts of the service message? Well, it's just because we are defining this variables here:

bool my_callback(std_srvs::Empty::Request  &req,

                 std_srvs::Empty::Response &res)

**Exercise 3.4**

    The objective of Exercise 3.4 is to create a service that, when called, will make the BB-8 robot move in a circle-like trajectory.

    You can work on a new package or use one of the ones you have already created.

    Create a Service Server that accepts an Empty service message and activates the circle movement. This service could be called /move_bb8_in_circle.

    You will place the necessary code into a new C++ file named bb8_move_in_circle_service_server.cpp. You can use the C++ file simple_service_server.cpp as an example.

    To move the BB-8 robot, you just have to write into the /cmd_vel topic, as you did in the Topics Units.

    Create a launch file called start_bb8_move_in_square_service_server.launch. Inside it, you have to start a node that launches the bb8_move_in_circle_service_server.cpp.

    Launch start_bb8_move_in_circle_service_server.launch and check that when called through the WebShell, BB-8 moves in a circle.

    Create a new C++ file, called bb8_move_in_square_service_client.cpp, that calls the service /move_bb8_in_circle. Remember how it was done in the previous Chapter: Services Part1.
    Then, generate a new launch file, called call_bb8_move_in_circle_service_server.launch, that executes the code in the bb8_move_in_circle_service_client.cpp file.

    Finally, when you launch this call_bb8_move_in_circle_service_server.launch file, BB-8 should move in a circle.

END **Exercise 3.4**
How to create your own service message

So, what if none of the service messages that are available in ROS fit your needs? Then, you create your own message, as you did with the Topic messages.

In order to create a service message, you will have to follow the next steps:

**Example 3.6**

1) Create a package like this:

Execute in WebShell #1

roscd;cd ..;cd src

catkin_create_pkg my_custom_srv_msg_pkg roscpp

2) Create your own Service message with the following structure. You can put as many variables as you need, of any type supported by ROS: ROS Message Types. Create a srv folder inside your package , as you did with the topics msg folder. Then, inside this srv folder, create a file called MyCustomServiceMessage.srv. You can create it with the IDE or the WebShell, as you wish.

Execute in WebShell #1

roscd my_custom_srv_msg_pkg/

mkdir srv

vim srv/MyCustomServiceMessage.srv

You can also create the MyCustomServiceMessage.srv through the IDE, if you don't feel confortable with vim.

The MyCustomServiceMessage.srv could be something like this:

int32 duration    # The number of times BB-8 has to execute the square movement when the service is called

---

bool success         # Did it achieve it?

How to Prepare CMakeLists.txt and package.xml for Custom Service Compilation

You have to edit two files in the package, in a way similar to how we explained it for Topics:

    CMakeLists.txt
    package.xml

Modification of CMakeLists.txt

You will have to edit four functions inside CMakeLists.txt:

    find_package()
    add_service_files()
    generate_messages()
    catkin_package()

I. find_package()

All the packages needed to COMPILE the messages of topic, services, and actions go here. It's only getting its paths, and not really importing them to be used in the compilation.
The same packages you write here will go in package.xml, stating them as build_depend.

find_package(catkin REQUIRED COMPONENTS

  roscpp

  std_msgs

  message_generation

)

II. add_service_files()

This function contains a list with all of the service messages defined in this package (defined in the srv folder).
For our example:

add_service_files(

  FILES

  MyCustomServiceMessage.srv

)

III. generate_messages()

Here is where the packages needed for the service messages compilation are imported.

generate_messages(

  DEPENDENCIES

  std_msgs

)

IV. catkin_package()

State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml file as <exec_depend>.

catkin_package(

      CATKIN_DEPENDS

      roscpp

)

Once you're done, you should have something similar to this:

cmake_minimum_required(VERSION 2.8.3)

project(my_custom_srv_msg_pkg)

​

​

## Here is where all the packages needed to COMPILE the messages of topic, services, and actions go.

## It's only getting its paths, and not really importing them to be used in the compilation.

## It's only for further functions in CMakeLists.txt to be able to find those packages.

## In package.xml you have to state them as build

find_package(catkin REQUIRED COMPONENTS

  roscpp

  std_msgs

  message_generation

)

​

## Generate services in the 'srv' folder

## In this function will be all the action messages of this package ( in the action folder ) to be compiled.

## You can state that it gets all the actions inside the action directory: DIRECTORY action

## Or just the action messages stated explicitly: FILES my_custom_action.action

## In your case, you only need to do one of two things, as you wish.

add_service_files(

  FILES

  MyCustomServiceMessage.srv

)

​

## Here is where the packages needed for the action messages compilation are imported.

generate_messages(

  DEPENDENCIES

  std_msgs

)

​

## State here all the packages that will be needed by someone that executes something from your package.

## All the packages stated here must be in the package.xml as exec_depend

catkin_package(

  CATKIN_DEPENDS roscpp

)

​

​

include_directories(

  ${catkin_INCLUDE_DIRS}

)

Modification of package.xml:

Just add these 3 lines to the package.xml file.

<build_depend>message_generation</build_depend>

​

<build_export_depend>message_runtime</build_export_depend>

<exec_depend>message_runtime</exec_depend>

You should have something similar to:

<?xml version="1.0"?>

<package format="2">

  <name>my_custom_srv_msg_pkg</name>

  <version>0.0.0</version>

  <description>The my_custom_srv_msg_pkg package</description>

​

  <maintainer email="user@todo.todo">user</maintainer>

​

  <license>TODO</license>

​

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>

  <build_depend>message_generation</build_depend>

  <build_export_depend>roscpp</build_export_depend>

  <exec_depend>roscpp</exec_depend>

  <build_export_depend>message_runtime</build_export_depend>

  <exec_depend>message_runtime</exec_depend>

​

  <export>

  </export>

</package>

Once done, compile your package and source the newly generated messages:

roscd;cd ..

catkin_make

source devel/setup.bash

**Important!!** When you compile new messages through catkin_make, there is an extra step that needs to be done. You have to type in the WebShell, in the **catkin_ws** directory, the following command: **source devel/setup.bash**.

This command executes the bash file that sets, among other things, the newly generated messages created with **catkin_make**.

To check that you have the new service msg in your system prepared for use, type the following:

Execute in WebShell #1

rossrv list | grep MyCustomServiceMessage

It should output something like:

WebShell #1 Output

user ~ $ rossrv list | grep MyCustomServiceMessage

my_custom_srv_msg_pkg/MyCustomServiceMessage

That's it! You have created your own service msg.
Using a Custom Service Message

If you want to use the custom message you have just generated in a Service Server, you will need to create a program similar to the one below:

**C++ Program {3.3}: custom_service_server.cpp**

#include "ros/ros.h"

#include "my_custom_srv_msg_pkg/MyCustomServiceMessage.h"

​

bool my_callback(my_custom_srv_msg_pkg::MyCustomServiceMessage::Request  &req,

                 my_custom_srv_msg_pkg::MyCustomServiceMessage::Response &res)

{  

  ROS_INFO("Request Data==> radius=%f, repetitions=%d", req.radius, req.repetitions); 

  if (req.radius > 5.0)

  {

    res.success = true;

    ROS_INFO("sending back response:true");

  }

  else

  {

    res.success = false;

    ROS_INFO("sending back response:false");

  }

  

  return true;

}

​

int main(int argc, char **argv)

{

  ros::init(argc, argv, "service_server");

  ros::NodeHandle nh;

​

  ros::ServiceServer my_service = nh.advertiseService("/my_service", my_callback); 

  ros::spin();

​

  return 0;

}

**END C++ Program {3.3}: custom_service_server.cpp**

And the CMakeLists.txt needed to make this work:

cmake_minimum_required(VERSION 2.8.3)

project(my_custom_srv_msg_pkg)

​

find_package(catkin REQUIRED COMPONENTS

  roscpp

  std_msgs

  message_generation

)

​

​

add_service_files(

   FILES

   MyCustomServiceMessage.srv

 )

​

generate_messages(

   DEPENDENCIES

   std_msgs

 )

​

​

catkin_package(

  CATKIN_DEPENDS roscpp

)

​

include_directories(include ${catkin_INCLUDE_DIRS})

​

​

add_executable(custom_service_server src/custom_service_server.cpp)

target_link_libraries(custom_service_server ${catkin_LIBRARIES})

add_dependencies(custom_service_server ${my_custom_srv_msg_pkg_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

You also need to know that, if you wanted to use this Custom Service Message in another ROS package, you will need to add the following line to the find_package() function of the CMakeLists.txt file of the package that wants to use this message:

find_package(catkin REQUIRED COMPONENTS

  roscpp

  my_custom_srv_msg_pkg

)

This is because at compilation time, the package that wants to use a custom message needs to know in which ROS package this message is. Otherwise, its compilation will fail. Adding the package that contains the message to the find_package() function ensures that the package (and therefore, the messages that it contains) will be found.

END **Example 3.6**

**Exercise 3.5**

    Create a new C++ file called bb8_move_custom_service_server.cpp. Inside this file, modify the code you used in Exercise 3.4, which contained a Service Server that accepted an Empty Service message to activate the circle movement. This new service will be called /move_bb8_in_circle_custom. This new service will have to be called through a custom service message. The structure of this custom message is presented below:

int32 duration    # The time (in seconds) during which BB-8 will keep moving in circles

---

bool success      # Did it achieve it?

    Use the data passed to this new /move_bb8_in_circle_custom to change the BB-8 behavior.
    During the specified duration time, BB-8 will keep moving in circles. Once this time has ended, BB-8 will then stop its movement and the Service Server will return a True value (in the success variable). Keep in mind that even after BB-8 stops moving, there might still be some rotation on the robot, due to inertia.

    Create a new launch file called start_bb8_move_custom_service_server.launch that launches the new bb8_move_custom_service_server.cpp file.

    Test that when calling this new /move_bb8_in_circle_custom service, BB-8 moves accordingly.

    Create a new C++ file, called call_bb8_move_custom_service_server.cpp that calls the service /move_bb8_in_circle_custom. Remember how it was done in Unit 3 Services Part 1.

    Then, generate a new launch file, called call_bb8_move_custom_service_server.launch, that executes the call_bb8_move_custom_service_server.cpp through a node.

END **Exercise 3.5**
Summary

Let's do a quick summary of the most important parts of ROS Services, just to try to put everything in place.

A ROS Service provides a certain functionality to your robot. A ROS Service is composed of two parts:

    Service Server: It is the one that PROVIDES the functionality. Whatever you want your Service to do, you have to place it in the Service Server.

    Service Client: It is the one that CALLS the functionality provided by the Service Server. That is, it CALLS the Service Server.

ROS Services use a special service message, which is composed of two parts:

    Request: The request is the part of the message that is used to CALL the Service. Therefore, it is sent by the Service Client to the Service Server.
    Response: The response is the part of the message that is returned by the Service Server to the Service Client, once the Service has finished.

ROS Services are synchronous. This means that whenever you CALL a Service Server, you have to wait until the Service has finished (and returns a response) before you can do other stuff with your robot.





## Using C++classes in ROS

Great! So, now that you already know what a C++ Class is, and how it works, let's try to apply this to a ROS code. For instance, we can start by creating a simple class that will move the BB-8 robot in a circular movement, just as you did in the previous unit.

**Example P1**

Below you can have a look at a class that will control the movement of the BB-8 robot.

**C++ File: bb8_move_circle_class.cpp** 
```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class MoveBB8
{
  
    public:

        // ROS Objects
        ros::NodeHandle nh_;

        // ROS Services
        ros::ServiceServer my_service;

        // ROS Publishers
        ros::Publisher vel_pub;
    
        // ROS Messages
        geometry_msgs::Twist vel_msg;
  
        MoveBB8()
        {
            my_service = nh_.advertiseService("/move_bb8_in_circle", &MoveBB8::my_callback, this);
            ROS_INFO("The Service /move_bb8_in_circle is READY");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }
        
        void move_in_circle()
        {
            vel_msg.linear.x = 0.2;
            vel_msg.angular.z = 0.2;
            vel_pub.publish(vel_msg);
        }
        
        bool my_callback(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res)
        {
            ROS_INFO("The Service /move_bb8_in_circle has been called");
            move_in_circle();
            ROS_INFO("Finished service /move_bb8_in_circle");
            return true;
        }
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_bb8_node");
  
  MoveBB8 moveBB8;

  ros::spin();
  
  return 0;
}
```

**END C++ File: bb8_move_circle_class.cpp**

Let's analyze some of the most important parts of the above class.

public:

    

    // ROS Objects

    ros::NodeHandle nh_;

​

    // ROS Services

    ros::ServiceServer my_service;

​

    // ROS Publishers

    ros::Publisher vel_pub;

​

    // ROS Messages

    geometry_msgs::Twist vel_msg;

We are defining all the elements we need as public attributes of the MoveBB8 class. These are:

    A ROS node handler
    A ROS Service
    A ROS Publisher
    A ROS Message of the type Twist

MoveBB8()

{

    my_service = nh_.advertiseService("/move_bb8_in_circle", &MoveBB8::my_callback, this);

    ROS_INFO("The Service /move_bb8_in_circle is READY");

    vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

}

This is the constructor of the class. Here we are initializing a couple of attributes:

    my_service: We create a ROS Service named /move_bb8_in_circle, connected to a callback function named my_callback.
    vel_pub: We create a Publisher for the /cmd_vel topic.

void move_in_circle()

{

    vel_msg.linear.x = 0.2;

    vel_msg.angular.z = 0.2;

    vel_pub.publish(vel_msg);

}

Here we've created a class method named move_in_circle(). This class method will be used to start moving our robot in circles.

bool my_callback(std_srvs::Empty::Request &req,

                std_srvs::Empty::Response &res)

{

    ROS_INFO("The Service /move_bb8_in_circle has been called");

    move_in_circle();

    ROS_INFO("Finished service /move_bb8_in_circle");

    return true;

}

Here we have the callback function of the Service. Inside it, we just perform a call to the move_in_circle() class method so that anytime someone calls our Service, the move_in_circle() method will be triggered, and the robot will start moving in circles.

int main(int argc, char** argv)

{

  ros::init(argc, argv, "move_bb8_node");

  

  MoveBB8 moveBB8;

​

  ros::spin();

  

  return 0;

}

Finally, in the main function, we are doing three things:

    We create a ROS node named move_bb8_node.
    We create an object of the MoveBB8 class, which is stored in a variable named moveBB8.
    We spin our ROS node forever, so that the Service will be available until someone stops our program.

Great! So now, we already have a class that moves our BB-8 robot in a circle movement. Let's test it. In this case, since we are using a ROS node, we are going to create a ROS package in which to place the class.

Execute in WebShell #1

catkin_create_pkg my_cpp_class roscpp

Now, add the bb8_move_circle_class.cpp file to your package and compile it. You will need to add the following to your CMakeLists.txt file:

add_executable(bb8_move_circle_class src/bb8_move_circle_class.cpp)

add_dependencies(bb8_move_circle_class ${bb8_move_circle_class_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bb8_move_circle_class

   ${catkin_LIBRARIES}

 )

Compile your package by running the following commands:

Execute in WebShell #1

roscd; cd ..;

catkin_make

source devel/setup.bash

Finally, let's create a launch file that will start our program.

**Launch File: bb8_move_circle_class.launch**

<launch>

    <!-- Start Service Server for move_bb8_in_circle service -->

    <node pkg="my_cpp_class" type="bb8_move_circle_class" name="move_bb8_node"  output="screen">

    </node>

</launch>

**END Launch File: bb8_move_circle_class.launch**

Finally, let's execute the launch file:

Execute in WebShell #1

roslaunch my_cpp_class bb8_move_circle_class.launch

If everything is OK, you should now have a service named /move_bb8_in_circle among your running services. Check it by running the following command:

Execute in WebShell #1

rosservice list | grep move_bb8_in_circle

Finally, let's call our service and check that everything works as expected. That is, the BB-8 robot starts moving in circles.

Execute in WebShell #1

rosservice call /move_bb8_in_circle "{}"

**END Example P1**

**Exercise P1**

Modify the C++ script you created in Example P1, so that now it includes the custom Service Message you used for Exercise 3.5, in the previous unit.

    You will need to modify the class so that now, the BB-8 robot stops moving after the specified time in the message has passed.

**END Exercise P1**

