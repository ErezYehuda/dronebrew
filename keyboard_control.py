import serial, time, msvcrt

com = 'COM5'
baud = 9600

max_in = 2000
min_in = 710

throttle=min_in
aileron=min_in
elevator=min_in
rudder=min_in # yaw, rotates the drone

tg=10
ag=50
eg=50
rg=50

def write_channels(verbose=False):
    command = "%i,%i,%i,%i"% (throttle, aileron, elevator, rudder)
    if verbose:
        print("[PC]: "+command)
    arduino.write((command+"\n").encode())

try:
    print("Connecting to Serial on %s with baud %d"% (com,baud))
    arduino=serial.Serial(com, baud, timeout=.01)
    time.sleep(1) #give the connection a second to settle

    data = arduino.readline().decode('utf8')
    if data:
        print("[AU]: "+data)

    print("Config mode? (y/N)")
    if(input().upper() == 'Y'):
        print("Configuration mode activated")
        print("Throttle will move up; Press enter to begin")
        input()
        print("Throttle up; press enter to throttle down")
        throttle = max_in # Throttle stick up
        st_time = time.clock()
        while True:
            if msvcrt.kbhit():
                key = ord(msvcrt.getch())
                if key == 13:
                    up_duration = time.clock() - st_time
                    break
            write_channels()
            # data = arduino.readline().decode('utf8')
            # if data:
            #     print("[AU]: "+data)

        print("Up duration: %f" % (up_duration))
        print("Throttle down; press enter to complete configuration mode")

        throttle = min_in # Throttle stick down
        st_time = time.clock()
        while True:
            if msvcrt.kbhit():
                key = ord(msvcrt.getch())
                if key == 13:
                    down_duration = time.clock() - st_time
                    break
            write_channels()
            # data = arduino.readline().decode('utf8')
            # if data:
            #     print("[AU]: "+data)

        print("Down duration: %f" % (down_duration))

        print("Init complete")
    else:
        print("Init mode skipped")


    while True:
        data = arduino.readline().decode('utf8')
        if data:
            #String responses from Arduino Uno are prefaced with [AU]
            print("[AU]: "+data)
            # pass

        # Space:32, esc:27, enter:13
            
        if msvcrt.kbhit():
            key = ord(msvcrt.getch())
            if key == 27: #ESC
                throttle = min_in
                # print "[PC]: ESC exiting"
                print("Throttle min")
                # break
            elif key == 13: #Enter
            #     #select()
            #     print "[PC]: Enter"
                print("%i,%i,%i,%i"% (throttle, aileron, elevator, rudder))
            elif key == 119: #w
                throttle+=tg
                if throttle > max_in:
                    throttle = max_in
                print("%i,%i,%i,%i"% (throttle, aileron, elevator, rudder))
            # elif key == 97: #a
            #     rudder-=rg         
            elif key == 115: #s
                throttle-=tg
                if throttle < min_in:
                    throttle = min_in
                print("%i,%i,%i,%i"% (throttle, aileron, elevator, rudder))
            # elif key == 100: #d
            #     rudder+=rg
            # elif key == 224: #Special keys (arrows, f keys, ins, del, etc.)
            #     key = ord(msvcrt.getch())
            #     if key == 80: #Down arrow
            #         elevator-=eg
            #     elif key == 72: #Up arrow
            #         elevator+=eg
            #     elif key == 77: #right arroww
            #         aileron+=ag
            #     elif key == 75: #left arrow
            #         aileron-=ag               


            # string commands to the Arduino are prefaced with  [PC]           
            write_channels()

finally:
    print("Closing and rebooting Arduino")
    # close the connection
    arduino.close()
    # re-open the serial port which will also reset the Arduino Uno and
    # this forces the quadcopter to power off when the radio loses conection. 
    arduino=serial.Serial(com, baud, timeout=.01)
    arduino.close()
    # close it again so it can be reopened the next time it is run.  