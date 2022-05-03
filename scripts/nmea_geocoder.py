#!/usr/bin/python

import sys
import rospy
import geocoder
import math
import yaml
import datetime

from std_msgs.msg import Header
from nmea_msgs.msg import Sentence

class nmea_gga:
    def __init__(self, utc_time, lat, north_south, lon, east_west, fix_type, num_satellites, hdop, elevation, elevation_unit, geoid, geoid_unit, age, correction_station_id):
        self.utc_time = utc_time
        self.lat = set_degree(float(lat))
        self.north_south = north_south
        self.lon = set_degree(float(lon))
        self.east_west = east_west
        self.fix_type = fix_type
        self.num_satellites = num_satellites
        self.hdop = hdop
        self.elevation = elevation
        self.elevation_unit = elevation_unit
        self.geoid = geoid
        self.geoid_unit = geoid_unit
        self.age = age
        self.correction_station_id = correction_station_id

def set_degree(nmea_deg):
    nmea_dec, nmea_int = math.modf(nmea_deg)
    integer = math.floor(nmea_int / 100.0)
    decimal = (nmea_int - integer * 100) / 60.0 + nmea_dec / 100.0
    return integer + decimal


prev_stamp = rospy.Time()
interval = rospy.Duration()
prev_geo = geocoder.osm([0.0, 0.0], method='reverse')
log_file = None

def nmea_callback(msg):
    #rospy.loginfo(format(msg))

    global prev_stamp
    global interval
    global prev_geo
    global log

    if(msg.header.stamp - prev_stamp >= interval):
        s = msg.sentence
        l = s.split(',')
        
        if l[0].endswith('GGA'):
            gga = nmea_gga(l[1], l[2], l[3], l[4], l[5], l[6], l[7], l[8], l[9], l[10], l[11], l[12], l[+13], l[14])
            geo = geocoder.osm([gga.lat, gga.lon], method='reverse')

            prev_stamp = msg.header.stamp

            if geo.address != prev_geo.address:

                #item = geo.address.split(',')
                #for str in item:
                #    if str.endswith(('県', '都', '府', '道')):
                #        print(str)

                date_time = datetime.datetime.fromtimestamp(int(msg.header.stamp.to_sec()))
                print(date_time, 'lat:', '{:.9f}'.format(gga.lat), ' lon:', '{:.9f}'.format(gga.lon), geo.address)
                
                log = open(log_file, 'a')
                log.write(str(date_time) + ' lat: ' + '{:.9f}'.format(gga.lat) + ' lon: ' + '{:.9f}'.format(gga.lon) + ' ' + geo.address + '\n')
                log.close()

                prev_geo = geo

def main():

    global log_file
    yaml_file = None
    if len(sys.argv) < 2 or len(sys.argv) > 3:
        print('$ rosrun geocoder_ros nmea_geoder.py <config_yaml> <output_file(Optional)>')
        quit()
    elif len(sys.argv) == 2:
        yaml_file = sys.argv[1]
    else:
        yaml_file = sys.argv[1]
        log_file = sys.argv[2]

    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)

    topic = config['topic']
    interval_sec = float(config['request_interval'])
    global interval
    interval = rospy.Duration.from_sec(interval_sec)

    print('NMEA Topic:', topic)
    print('Request interval:', interval.to_sec())

    rospy.init_node("nmea_geocoder")
    rospy.Subscriber(topic, Sentence, nmea_callback)
    rospy.spin()

if __name__ =="__main__":
    main()