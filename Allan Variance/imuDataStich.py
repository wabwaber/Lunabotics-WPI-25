
#the point of this script is to stich the acceleration and velocity measurements into one.

#Obselete now. check bob.py for actual implementation.

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

typestore = get_typestore(Stores.LATEST)
imuMsg = typestore.types["sensor_msgs/Imu"]
bagpath = Path("/home/cmdwiz/kalibr/data/imuCalibration/fullImu") #raw path to bag on system

def getNextTwoVelocity():
    currTotal = 0
    i = 0
    while i < 2:
        currTotal += 0

        i += 1

    return currTotal/4


#the accelerometer runs at 100hz while the angular velocity runs at 200hz. essentially I need to take 2 angular velocity measurements for 1 accelerometer measurement
#so I will just skip the middle measurement
def main():
    print("Stitching Script Starting...")
    with AnyReader([bagpath]) as reader, Writer("/home/cmdwiz/stitched.bag") as writer:
        connections = [x for x in reader]
        accelTopic = "/accel/sample" #100hz
        veloTopic = "/gyro/sample" #200hz
        outTopic = "imu" #100hz (hopefully)
        msgtype = imuMsg.__msgtype__
        writerConn = writer.add_connection(outTopic, msgtype, typestore=typestore)
        currVeloMeasurement = None #generic for now
        prevVeloMeasurement = None

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == accelTopic:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            elif connection.topic == veloTopic:
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            