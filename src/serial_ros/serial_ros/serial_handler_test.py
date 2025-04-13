from serial_handler import SerialHandler

ser = SerialHandler()

from sshkeyboard import listen_keyboard

def press(key):
    if(key=='r'):
        print("Reading Message")
        data = ser.readMsg()
        print(data)
    elif(key=='s'):
        print("Sending Message")
        data = [ord('A') for i in range(6)]
        ser.send(0,data)
    elif(key=='q'):
        exit(0)

def release(key):
    if(key !='r'): print("press q to exit, r to read message, s to send dummy message")

listen_keyboard(
    on_press=press,
    on_release=release,
)


    