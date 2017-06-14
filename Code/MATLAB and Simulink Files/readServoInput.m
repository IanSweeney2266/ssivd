function y = readServoInput(~)

    global myserialdevice;
    global myconnection;

    if (myconnection > 0)
        write(myserialdevice, 21);
        servo_in = typecast(read(myserialdevice,2), 'int16');

        y = double(servo_in);
    else 
        y = 0;
    end
    
end