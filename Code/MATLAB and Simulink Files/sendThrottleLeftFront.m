function sendThrottleLeftFront(x)

    global myserialdevice;
    global myconnection;
    
    if (myconnection > 0)
        write(myserialdevice, 13);

        x = int16(x);
        write(myserialdevice, x, 'int16');
    end
end 