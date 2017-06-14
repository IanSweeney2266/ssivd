function y = setupPi(~)
    
    clear all;
    global rpi;
    global myserialdevice;
    global myconnection;
    
    rpi = raspi('169.254.0.2','pi','raspi');
    myserialdevice = serialdev(rpi, '/dev/ttyAMA0');
    myconnection = 0;
    
    y = 0;
end