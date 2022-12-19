import rclpy
from rclpy.node import Node
from sys import platform
from io import BufferedReader
from threading import Thread, Lock
from std_msgs.msg import String, Float64MultiArray
import time
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
        self.gps_vel_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/GPS/vel', 4)
        self.gps_info_publisher_ = self.create_publisher(String, '/akillipaket/GPS/info', 4)

        self.reading = False
        self.baudrate = 9600
        self.timeout = 1
        self.port = "/dev/ttyS0"
        self.r_earth = 6371000
        self.a  = 6378137.0
        self.b  = 6356752.3

        self.lat_ref = 49.8999999974
        self.lon_ref = 8.90000000309

        self.lon = self.lat_ref
        self.lat = self.lon_ref
        self.alt = 0
        self.time = 0
        self.isReady = False

        self.data = 0

    def gps_pos_data_handler(self, msg):
        z = -msg.altitude

        latitude  = msg.lat - self.lat_ref
        longitude = msg.lon - self.lon_ref

        x = self.a * math.sin(math.radians(latitude))
        y = self.b * math.sin(math.radians(longitude)) * math.cos(math.radians(self.lat_ref))

        return x, y, z


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


    def check_func(self, serial, serial_lock):
        try:
            if self.isReady:
                msgid = 'GGA'
                print(f"\n\nSending a GNQ message to poll for an {msgid} response...\n\n")
                msg = NMEAMessage("GP", "GGA", msgId=msgid)
                self.send_message(serial, serial_lock, msg)
                print("Check")
                self.isReady = False
        except Exception:
            print("Error occured!")
        return


    def main_thread(self, stream, lock, nmeareader):
        """
        Start read thread
        """
        thr = Thread(target=self.read_messages, args=(stream, lock, nmeareader), daemon=True)
        thr.start()
        return thr


    def check_thread(self, stream, lock):
        thr2 = Thread(target=self.check_func, args=(stream, lock))
        thr2.start()
        return thr2


    def send_message(self, stream, lock, message):
        """
        Send message to device
        """

        lock.acquire()
        stream.write(message.serialize())
        lock.release()


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
                    #msgid = 'GSV'
                    #print(f"\n\nSending a GNQ message to poll for an {msgid} response...\n\n")
                    #msg = NMEAMessage("EI", "GNQ", POLL, msgId=msgid)
                    #gps.send_message(serial, serial_lock, msg)
                    print("Check")
                    print(gps.data)
                    gps.isReady = False

        gps.reading = False
        read_thread.join()



    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

