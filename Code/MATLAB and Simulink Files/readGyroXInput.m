function y = readGyroXInput(~)

    global myserialdevice;
    global myconnection;
    
    if (myconnection > 0)
        write(myserialdevice, 23);
        x = typecast(read(myserialdevice,2), 'int16');

        y = double(x);
    else
        y = 0;
    end   
end