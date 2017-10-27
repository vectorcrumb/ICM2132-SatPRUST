import threading, time, math
import mpu6050
import paho.mqtt.client as mqtt
import stream_cam


## MQTT Connection information
# broker = 'satprust.local'
broker = 'localhost'
port = 1883
topics = {'gyro': 'cubesat/gyro',
          'gyro_yaw': 'cubesat/gyro/yaw',
          'gyro_pitch': 'cubesat/gyro/pitch',
          'gyro_roll': 'cubesat/gyro/roll',
          'image': 'cubesat/image',
          'heartbeat': 'cubesat/heartbeat'}
# MQTT callbacks for connections and messages
def on_connect(client, userdata, flags, rc):
    # Inform connection
    print("Connected with result code {}".format(str(rc)))
    # Resubscribe on every reconnection to cubesat/ topic
    client.subscribe("cubesat/#")
def on_message(client, userdata, msg):
    # Skip gyro debug messages
    if topics['gyro'] in msg.topic or topics['image'] in msg.topic:
        return
    # Display message
    print(msg.topic + " " + str(msg.payload))
# Configure MQTT client
mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message
mqttc.connect(broker, port=int(port))
mqttc.loop_start()

## MPU6050 Configuration variables
# Sensor initialization
mpu = mpu6050.MPU6050()
mpu.dmpInitialize()
mpu.setDMPEnabled(True)
# Get expected DMP packet size for later comparison
packetSize = mpu.dmpGetFIFOPacketSize()
# Gyro values in degrees with DMP filtering
def getGyro():
    # Get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus()
    # check for DMP data ready interrupt (this should happen frequently)
    if mpuIntStatus >= 2:
        # get current FIFO count
        fifoCount = mpu.getFIFOCount()
        # check for overflow (this should never happen unless our code is too inefficient)
        if fifoCount == 1024:
            # reset so we can continue cleanly
            mpu.resetFIFO()
            print('FIFO overflow!')
        # wait for correct available data length, should be a VERY short wait
        fifoCount = mpu.getFIFOCount()
        while fifoCount < packetSize:
            fifoCount = mpu.getFIFOCount()
        result = mpu.getFIFOBytes(packetSize)
        q = mpu.dmpGetQuaternion(result)
        g = mpu.dmpGetGravity(q)
        pre_ypr = mpu.dmpGetYawPitchRoll(q, g)
        ypr = {}
        ypr['y'] = math.degrees(pre_ypr['yaw'])
        ypr['p'] = math.degrees(pre_ypr['pitch'])
        ypr['r'] = math.degrees(pre_ypr['roll'])
        # track FIFO count here in case there is > 1 packet available
        # (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize
        # Return final processed values in degrees
        return ypr
# Thread gyro update to avoid FIFO overflows
gyro_dict = {}
lock = threading.Lock()
def updateGyro():
    global gyro_dict
    while True:
        with lock:
            gyro_dict = getGyro()
        time.sleep(0.001)
gyroStateThread = threading.Thread(target=updateGyro)
gyroStateThread.setDaemon(True)
gyroStateThread.start()

def updateDashGyro():
    # TODO: Add previous values vector to avoid jittering in dash
    global gyro_dict
    while True:
        # Update gyro values and publish to corresponding topic
        with lock:
            gyro_ypr = gyro_dict
        if gyro_ypr == None:
            continue
        mqttc.publish(topics['gyro_yaw'], round(gyro_ypr['y'],3))
        mqttc.publish(topics['gyro_pitch'], round(gyro_ypr['p'],3))
        mqttc.publish(topics['gyro_roll'], round(gyro_ypr['r'],3))
        time.sleep(0.25)
gyroDashThread = threading.Thread(target=updateDashGyro)
gyroDashThread.setDaemon(True)
gyroDashThread.start()

## Camera configurations
res = (480, 360)
fps = 24
# Run server thread to server MJPG stream of camera
serverThread = threading.Thread(target=stream_cam.runServer, args=(res, fps,))
serverThread.setDaemon(True)
serverThread.start()


## Main loop
while True:
    # Heartbeat blip
    mqttc.publish(topics['heartbeat'], time.time())
    time.sleep(1)

# Stop comms loop and exit
# mqttc.loop_stop()
# exit(0)