clc
clear

disp('-----------------------------------------------------------')
disp('|Beware, this code is for GNU Octave ONLY !!!             |')
disp('-----------------------------------------------------------')

try
  pkg load instrument-control
  pkg load image
end

[DATA_packets_to_print]=image_slicer("Public-Pixel.png");
[number_packets,~]=size(DATA_packets_to_print)

global Arduino_baudrate;
Arduino_baudrate = 250000;
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

% === Flush any previous welcome message ===
while arduinoObj.NumBytesAvailable > 0
  discard = readline(arduinoObj);  % Clear all startup messages
  if not(isempty(strfind(discard,"Printer connected")))
    disp("✅ Printer connected")
    read(arduinoObj, 1, "uint8");%get rid of a last lost character
  else
    disp("❌ Printer not yet connected");
  end
end

for i=1:1:number_packets
  % === Send Data Packets ===
  dataPayload=uint8(DATA_packets_to_print(i,:));
  dataPacket = [uint8('D'), dataPayload, uint8(13)];
  disp(['Sending packet# ',num2str(i)])
  sendPacketAndConfirm(arduinoObj, dataPacket);

  if rem(i,9)==0
    % === Send Print command without margin ===
    printPayload = uint8([0, palette, intensity]);
    printPacket = [uint8('P'), printPayload, uint8(13)];  % CR = 13
    sendPacketAndConfirm(arduinoObj, printPacket);
  end
end

% === Send Print command with margin===
printPayload = uint8([margin, palette, intensity]);
printPacket = [uint8('P'), printPayload, uint8(13)];  % CR = 13
sendPacketAndConfirm(arduinoObj, printPacket);
arduinoObj=[];



