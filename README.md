# wit_node

This is the ROS nodelet package for wit motion company imu and gps sensor. Providing driver, ros driver and nodelet intergrating program.



## Dependencies and Install

1. ROS
2. ros-<distro>-ecl

Install ecl by

```bash
sudo apt install ros-<distro>-ecl
```



## Usage

Launch the only ROS launch file:

```bash
roslaunch wit_node wit.launch
```

About parameter:

-port

This is the port that device name in Linux system, for example the default port name is "/dev/ttyUSB0"



## Msg

### ImuGpsRaw

> Header header
>
> float64 time
>
> float64[] acc
>
> float64[] gyro
>
> float64[] rpy
>
> float64[] mag
>
> uint16[]  ps #port state
>
> float64   temperature
>
> float64   altitude
>
> float64   ap #atmosphere pressure
>
> float64   latitude
>
> float64   longtitude
>
> float64   gpsh #gps height
>
> float64   gpsy #gps yaw
>
> float64   gpsv #gps velocity
>
> float64[] quarternion
>
> uint8     sn #satelites number
>
> float64[] dop

## Published Topics

### /imu (sensor_msgs/Imu)

The standard ROS imu sensor msg which include orientation by filtered RPY.

### /gps (sensor_msgs/NavSatFix)

The standard ROS gps or navigation satellites msg.

### /wit/raw_data (wit_node/ImuGpsRaw)

All raw data provided by the wit device, including nine axises data, atmosphere pressure, temperature, latitude,longitude, altitude, satellites number .etc

### /wit/related_yaw (std_msgs/Float64)

The offseted imu yaw data, which means the zero direction is start direction.



## Subscribed Topics

### /wit/reset_offset (std_msgs/Empty)

Reset the offset yaw angle to current yaw,  so the zero direction is turned to current direction.



