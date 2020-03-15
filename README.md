# robotiq_ft300_sensor


--

## Installation and Compilation
  1. Connect all the cables and check that the sensor is properly powered. 
    - Add pictures
  2. Go to the driver folder `/robotiq_ft_sensor_dev_v1.0.1` and follow the compilation instructions stated in the FT 300 sensor [manual](https://assets.robotiq.com/website-assets/support_documents/document/FT_Sensor_Instruction_Manual_PDF_20181218.pdf) in **Section 4.2.2 Linux** page 79. Check that the device is recognized by your PC and the test code works (i.e. data is streaming). 
  - To get an update version of the driver you can download it from [https://robotiq.com/support](https://robotiq.com/support). Search for FT300 Force Torque Sensor -> Software -> PC Integration -> Sensor Development. 
   
  3. If you are using one of robotiq's grippers you probably have the robotiq [stack](https://github.com/ros-industrial/robotiq/tree/indigo-devel) which includes the [robotiq_force_torque_sensor](https://github.com/ros-industrial/robotiq/tree/indigo-devel/robotiq_force_torque_sensor) and you can skip this step. Otherwise:
  - Delete the CATKIN_IGNORE file in `robotiq_ft300_sensor/robotiq_force_torque_sensor/.`
  - Compile package:
    ```bash
    $ cd ~/catkin_ws
    $ catkin_make
    $ source devel/setup.bash
    $ catkin_make
    ```
---
## Usage    
- To stream the custom FT sensor message, `/robotiq_force_torque_sensor`, which has of the following format:
    ```bash
    float32 fx
    float32 fy
    float32 fz
    float32 Mx
    float32 My
    float32 Mz
     ```
     or `geometry_msgs/WrenchStamped` named `/robotiq_force_torque_wrench`. You must run the following node:
    ```bash
    $ rosrun robotiq_force_torque_sensor rq_sensor
    ```
 - To listen to subscribed to the topics or visualize the messages from command line:
   ```bash
    $ rostopic echo /robotiq_force_torque_sensor
   ```
   or simply subscribe to the message:
    ```bash
    $ rostopic echo /robotiq_force_torque_wrench
    ```
 - To bias the sensor and test the readings you can run the following node:
    ```bash
    $ rosrun robotiq_force_torque_sensor rq_test_sensor
    ```
 - If there is a tool attached to the sensor you can calibrate using the following something like this...[force_torque_tools](https://github.com/kth-ros-pkg/force_torque_tools)
