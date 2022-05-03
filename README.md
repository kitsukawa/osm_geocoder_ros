# osm_geocoder_ros

osm_geocoder_ros is a ROS package that extracts a address from latitude and longitude contained in nmea messages.
This package utilizes [Geocoder](https://geocoder.readthedocs.io/), a geocoding library in Python.

## Prerequisite
```
pip install geocoder
```

## Build

```
cd <YOUR_CATKIN_WS>/src/
git clone https://gitlab.com/MapIV/osm_geocoder_ros.git
cd ../
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Run
```
source devel/setup.bash
rosrun osm_geocoder_ros nmea_geocoder.py <CONFIG> <LOG(Optional)>
# ex) rosrun osm_geocoder_ros nmea_geocoder.py src/osm_geocoder_ros/scripts/nmea_geocoder_config.yaml /home/map4/nmea_geocoder_log.txt
# You can run 'nmea_geocoder.py' instead of using rosrun.
```

You need specify configuration file(yaml) and output log(txt). In the configuration file, the topic (type: nmea_msgs/Sentence) used for (reverse) geocoding and interval(in seconds) to send request to OSM server are set.
