function sendThrottleRightFront(x)

    global myserialdevice;
    global myconnection;
    
    if (myconnection > 0)
        write(myserialdevice, 12);

        x = int16(x);
        write(myserialdevice, x, 'int16');
    end
    
end 