function y = createMyConnection(~)

    global rpi;
    global myconnection;
    
    x = readDigitalPin(rpi, 4);

    if (x > 0)
        myconnection = 1;
    else 
        myconnection = 0;
    end
    y = 0;        
end 