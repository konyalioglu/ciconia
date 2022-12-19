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
        self.gps_rate = self.create_rate(1)
        self.reading = False
        self.baudrate = 9600
        self.timeout = 3
        self.port = "/dev/ttyS0"

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
                    print('Hello')
                    if parsed_data:
                        print(parsed_data)
                except Exception as err:
                    print(f"\n\nSomething went wrong {err}\n\n")
                    continue


    def start_thread(self, stream, lock, nmeareader):
        """
        Start read thread
        """
        thr = Thread(target=self.read_messages, args=(stream, lock, nmeareader), daemon=True)
        thr.start()
        return thr


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

        # create NMEAReader instance
        nmr = NMEAReader(BufferedReader(serial))

        print("\nStarting read thread...\n")
        gps.reading = True
        serial_lock = Lock()
        read_thread = gps.start_thread(serial, serial_lock, nmr)

        # DO OTHER STUFF HERE WHILE THREAD RUNS IN BACKGROUND...
        #for msgid in NMEA_MSGIDS:
        while rclpy.ok:
            msgid = 'GSV'
            print(f"\n\nSending a GNQ message to poll for an {msgid} response...\n\n")
            msg = NMEAMessage("EI", "GNQ", POLL, msgId=msgid)
            gps.send_message(serial, serial_lock, msg)
            print(msg)
            time.sleep(1)

        print("\nPolling complete. Pausing for any final responses...\n")
        sleep(1)
        print("\nStopping reader thread...\n")
        gps.reading = False
        read_thread.join()
        print("\nProcessing Complete")

    gps.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

