#send_msg
# The CANlib library is initialized when the canlib module is imported. To be
# able to send a message, Frame also needs to be installed.
from canlib import canlib, Frame, kvadblib
import time



def printframe(db, frame):
    try:
        bmsg = db.interpret(frame)
    except kvadblib.KvdNoMessage:
        print("<<< No message found for frame with id %s >>>" % frame.id)
        print(frame.id)
        return

    if not bmsg._message.dlc == bmsg._frame.dlc:
        print(
            "<<< Could not interpret message because DLC does not match for frame with id %s >>>"
            % frame.id
        )
        print("\t- DLC (database): %s" % bmsg._message.dlc)
        print("\t- DLC (received frame): %s" % bmsg._frame.dlc)
        return

    msg = bmsg._message

    print('┏', msg.name)

    if msg.comment:
        print('┃', '"%s"' % msg.comment)
    # print(bmsg)
    for bsig in bmsg:
        print('┃', bsig.name + ':', bsig.value, bsig.unit)

    print('┗')



# Firstly, open two CAN channels, one to send the message and one to receive.
# Note that there needs to be a channel to receive, as otherwise the message
# can not be sent. In this example the channels are named ch_a and ch_b. To
# open the channels call on the openChannel method inside of canlib and, as an
# input put in channel=0 and channel=1. Where 0 and 1 represents the two
# CANlib channels 0 and 1.
db = kvadblib.Dbc(filename='/home/iac_user/data_collection_scripts/dadss_breath_sensor.dbc')
ch_a = canlib.openChannel(channel=1)
# ch_b = canlib.openChannel(channel=0)

# After opening the channel, we need to set the bus parameters. Some
# interfaces keep their params from previous programs. This can cause problems
# if the params are different between the interfaces/channels. For now we will
# use setBusParams() to set the canBitrate to 250K.
ch_a.setBusParams(canlib.canBITRATE_500K)
# ch_b.setBusParams(canlib.canBITRATE_500K)

# The next step is to Activate the CAN chip for each channel (ch_a and ch_b in
# this example) use .busOn() to make them ready to receive and send messages.
# ch_a.busOn()


# To transmit a message with (11-bit) CAN id = 123 and contents (decimal) 72,
# 69, 76, 76, 79, 33, first create the CANFrame (CANmessage) and name it. In
# this example, the CANFrame is named frame. Then send the message by calling on
# the channel that will act as the sender and use .write() with the CANFrame
# # as input. In this example ch_a will act as sender.
ch_a.busOn()
frame = Frame(id_=800, data=[1], flags=canlib.MessageFlag.STD)
ch_a.write(frame)
# To make sure the message was sent we will attempt to read the message. Using
# timeout, only 500 ms will be spent waiting to receive the CANFrame. If it takes
# longer the program will encounter a timeout error. read the CANFrame by calling
# .read() on the channel that receives the message, ch_b in this example. To
# then read the message we will use print() and send msg as the input.
while True:
    try:
        msg = ch_a.read(timeout=500)
        if msg.id == 783:
            print("Status:")
            print(msg)
            # printframe(db, msg)
            # break
        if msg.id == 799:
            print("Results:")
            print(msg)
            # printframe(db, msg)
            # break
    except KeyboardInterrupt:
        print("Stopping")
        break
        #Saving all_frame_data to a csv file using pandas dataframe
        # df = pd.DataFrame(all_frame_data)
        # df.to_csv(filename)
ch_a.busOff()


# After the message has been sent, received and read it is time to inactivate
# the CAN chip. To do this call .busOff() on both channels that went .busOn()
# ch_a.busOff()


# Lastly, close all channels with close() to finish up.
ch_a.close()
# ch_b.close()

# Depending on the situation it is not always necessary or preferable to go of
# the bus with the channels and, instead only use close(). But this will be
# talked more about later.


