clc
clear
disp('-----------------------------------------------------------')
disp('|Beware, this code is for GNU Octave ONLY !!!             |')
disp('-----------------------------------------------------------')

try
    pkg load instrument-control
end

global Arduino_baudrate;
Arduino_baudrate = 115200;
%///////////////////////////////////////////////////////////////////////////////////
margin = 0x03;     %high nibble, upper margin, low nibble, lower margin, that simple
palette = 0x00;    %0x00 is treated as default (= 0xE4)
intensity = 0x40;  %default intensity is 0x40, min is 0x00, max is 0x7F, values between 0x80 and 0xFF are treated as default
%///////////////////////////////////////////////////////////////////////////////////

% === Setup Serial Port ===
arduinoPort = detectArduino();
arduinoObj = serialport(arduinoPort,'baudrate',Arduino_baudrate, 'Parity', 'none', 'Timeout', 2);
configureTerminator(arduinoObj, "CR");  % Sets terminator to CR (carriage return)
pause(2);  % Give Arduino time to initialize

% === Check if printer is connected ===
ack=false;
while ack==false;
    if arduinoObj.NumBytesAvailable > 0 %to avoid loosing time with garbage
        discard = readline(arduinoObj);  % Clear all startup messages, remove semicolon to display
        if not(isempty(strfind(discard,"Printer connected")))
            disp("✅ Printer online")
            ack=true;
            read(arduinoObj, arduinoObj.NumBytesAvailable, "uint8");%get rid of a last lost character but this is just an issue with GNU Octave
        end
    end
end

for i=1:9
    % === Send Data Packets ===
    dataPayload=uint8(256*repelem(rand(1,40)<0.5,16));%just random black and white tiles for testing
    dataPacket = [uint8('D'), dataPayload, uint8(13)];
    disp(['=== Sending packet# ',num2str(i),' ==='])
    sendPacketAndConfirm(arduinoObj, dataPacket);
end

% === Send Print command ===
printPayload = uint8([margin, palette, intensity]);
printPacket = [uint8('P'), printPayload, uint8(13)];  % CR = 13
sendPacketAndConfirm(arduinoObj, printPacket);
arduinoObj=[];
