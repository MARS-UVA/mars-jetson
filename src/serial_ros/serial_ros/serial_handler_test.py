# from serial_handler import SerialHandler

# ser = SerialHandler()

from sshkeyboard import listen_keyboard

def press(key):
    if(key=='r'):
        print('Reading Message')
        # data = ser.readMsg()
        # print(data)
    if(key=='q'):
        exit(0)

def release(key):
    if(key !='r'): print("press q to exit, r to read message")

listen_keyboard(
    on_press=press,
    on_release=release,
)

# ser.send('A', [0,0,0,0,0,0])

    