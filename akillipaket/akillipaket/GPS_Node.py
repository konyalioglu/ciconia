import rclpy
from rclpy.node import Node
from sys import platform
from io import BufferedReader
from threading import Thread, Lock
from std_msgs.msg import String, Float64MultiArray
import time
import math
import numpy as np
from serial import Serial
from pynmeagps import (
    NMEAMessage,
    NMEAReader,
    POLL,
    NMEA_MSGIDS,
)


class GPS_Node(Node):

    def __init__(self):
        super().__init__('GPS')
        self.gps_fix_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/GPS/fix', 4)
        self.gps_set_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/GPS/ref', 4)
        self.gps_info_publisher_ = self.create_publisher(String, '/akillipaket/GPS/info', 4)

        self.reading = False
        self.baudrate = 9600
        self.timeout = 1
        self.port = "/dev/ttyS0"
        self.r_earth = 6371000
        self.a  = 6378137.0
        self.b  = 6356752.3

        self.set_lat = 41.038113
        self.set_lon = 28.992101

        self.flag = False
        self.flag_counter = 0
        self.flag_stop_count = 20

        self.lat_ref = 0
        self.lon_ref = 0
        self.alt_ref = 0
        self.lat_ref_sum = 0
        self.lon_ref_sum = 0
        self.alt_ref_sum = 0

        self.time = 0
        self.isReady = False

        self.data = 0
        self.store = np.array([[0.0,0.0,0.0,0.0,0.0]])

        self.fix = Float64MultiArray()
        self.set = Float64MultiArray()
        self.info= String()


    def data_handler(self, data):
        if isinstance(data.lat, str):
            return
        if self.flag == False:
            self.lat_ref = data.lat
            self.lon_ref = data.lon
            self.alt_ref = data.alt
            self.flag_counter += 1
            self.lat_ref_sum += self.lat_ref
            self.lon_ref_sum += self.lon_ref
            self.alt_ref_sum += self.alt_ref
            if self.flag_counter == self.flag_stop_count:
                self.lat_ref = self.lat_ref_sum / self.flag_counter
                self.lon_ref = self.lon_ref_sum / self.flag_counter
                self.alt_ref = self.alt_ref_sum / self.flag_counter
                self.flag = True

        else:
            lat = data.lat - self.lat_ref
            lon = data.lon - self.lon_ref

            x = self.a * math.sin(math.radians(lat))
            y = self.b * math.sin(math.radians(lon)) * math.cos(math.radians(self.lat_ref))
            z = data.alt - self.alt_ref

            x_set = self.a * math.sin(math.radians(self.set_lat))
            y_set = self.b * math.sin(math.radians(self.set_lon)) * math.cos(math.radians(self.lat_ref))

            self.fix.data  = [x, y, z]
            self.info.data = 'Quality: ' + str(data.quality) + 'Number of Satellites: ' + str(data.numSV) + 'HDOP: '  + str(data.HDOP)
            self.set.data = [x_set, y_set]
            self.gps_set_publisher_.publish(self.set)
            self.gps_fix_publisher_.publish(self.fix)
            self.gps_info_publisher_.publish(self.info)
            store_vector = np.array([[x, y, z, lat, lon]])
            self.store = np.concatenate((self.store, store_vector))
            var_x = np.var(self.store[:,0])
            var_y = np.var(self.store[:,1])
            var_z = np.var(self.store[:,2])
            print(var_x, '\t', var_y, '\t', var_z)
        print(data.lat, data.lon)



    def read_messages(self, stream, lock, nmeareader):
        """
        Reads, parses and prints out incoming UBX messages
        """
        # pylint: disable=unused-variable, broad-except

        while self.reading:
            if stream.in_waiting:
                try:
                    lock.acquire()
                    (raw_data, parsed_data) = nmeareader.read()
                    lock.release()
                    if parsed_data and parsed_data.msgID == 'GGA':
                        self.isReady = True
                        self.data = parsed_data
                except Exception as err:
                    print(f"\n\nSomething went wrong {err}\n\n")
                    continue


    def main_thread(self, stream, lock, nmeareader):
        """
        Start read thread
        """
        thr = Thread(target=self.read_messages, args=(stream, lock, nmeareader), daemon=True)
        thr.start()
        return thr



def main(args=None):
    rclpy.init(args=args)

    gps = GPS_Node()
    with Serial(gps.port, gps.baudrate, timeout=gps.timeout) as serial:
        nmr = NMEAReader(BufferedReader(serial))
        gps.reading = True
        serial_lock = Lock()
        read_thread = gps.main_thread(serial, serial_lock, nmr)
        print("\nStarting threads...\n")
        while rclpy.ok:
            if gps.isReady:
                    print("Check")
                    #print(gps.data.lon)
                    gps.isReady = False
                    gps.data_handler(gps.data)
                    print(gps.data)
        gps.reading = False
        read_thread.join()



    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

