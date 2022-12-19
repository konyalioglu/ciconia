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
    GET,
    NMEA_MSGIDS,
)

# initialise global variables
reading = False


def read_messages(stream, lock, nmeareader):
    """
    Reads, parses and prints out incoming UBX messages
    """
    # pylint: disable=unused-variable, broad-except

    while reading:
        if stream.in_waiting:
            try:
                lock.acquire()
                (raw_data, parsed_data) = nmeareader.read()
                lock.release()
                if parsed_data:
                    print(parsed_data.lat)
            except Exception as err:
                print(f"\n\nSomething went wrong {err}\n\n")
                continue


def start_thread(stream, lock, nmeareader):
    """
    Start read thread
    """
    thr = Thread(target=read_messages, args=(stream, lock, nmeareader), daemon=True)
    thr.start()
    return thr


def send_message(stream, lock, message):
    """
    Send message to device
    """

    lock.acquire()
    stream.write(message.serialize())
    lock.release()


if __name__ == "__main__":
    port = "/dev/ttyS0"
    baudrate = 9600
    timeout = 3

    with Serial(port, baudrate, timeout=timeout) as serial:

        # create NMEAReader instance
        nmr = NMEAReader(BufferedReader(serial))

        print("\nStarting read thread...\n")
        reading = True
        serial_lock = Lock()
        read_thread = start_thread(serial, serial_lock, nmr)

        # DO OTHER STUFF HERE WHILE THREAD RUNS IN BACKGROUND...
        #for msgid in NMEA_MSGIDS:
        while rclpy.ok:
            msgid = 'GPGGA'
            print(f"\n\nSending a GNQ message to poll for an {msgid} response...\n\n")
            msg = NMEAMessage("GPGGA", "GGA", GET, msgId=msgid)
            
            send_message(serial, serial_lock, msg)
            time.sleep(1)

        print("\nPolling complete. Pausing for any final responses...\n")
        sleep(1)
        print("\nStopping reader thread...\n")
        reading = False
        read_thread.join()
        print("\nProcessing Complete")



