# **HIKROBOTcam-ROS-IDMVS**
Allows the HIKROBOT MV-SC2005AM to run in ROS1.

## 🎬 **Demo VOD**
Link: https://www.youtube.com/watch?v=uk1ZsSm9Oko
## 🧭 **Overview**
This package is developed using the **Strategy Pattern**, enabling different communication modes with unique features.

## ⚙️ **Features**
- **Switch Mode**:  
The camera has two switchable modes, and it receives the mode-switch command through a ROS topic.

- **SDK Mode**:  
The camera does not provide the coordinate transformation between the camera and tag centers.Users can import the intrinsic camera parameters to compute this transformation.

- **UDP Mode**:  
The camera provides the coordinate transformation between the camera and tag centers, but image display is not available.

## 🔧 **Parameters**
| Parameter | Type | Description |
|------------|------|-------------|
| mode | int | Init Mode. |
| show_debug_info | bool | Show info to debug |
| image_show_info | bool | Display reading data on image. |
| file_path | string | camera_info_manager::CameraInfoManager yaml |

## 📡 **Subscribe Topic**
- **switch_hikcam_mode**(std_msgs::Int32):  
Switch Mode(UDP:1 / SDK:2)

- **restart_camera**(std_msgs::Bool):  
Restart Camera(Restart:True)