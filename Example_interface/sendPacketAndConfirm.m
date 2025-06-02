% === Function to send and confirm echo ===
function sendPacketAndConfirm(arduinoObj, packet)
write(arduinoObj, packet, "uint8");  % Send packet
pause(0.02);  % Give Arduino time to echo, increase if packets are skipped

%%%%%%%%%%%%%%%%%%this step is not mandatory, just for debug%%%%%%%%%%%%%%%%%%%%
expectedLength = length(packet);
echoed = read(arduinoObj, expectedLength, "uint8");
if isequal(echoed, packet)
    disp("✅ Echo confirmed");
else
    disp("❌ Echo mismatch");
    fprintf("Sent:   %s\n", mat2str(packet));
    fprintf("Echoed: %s\n", mat2str(echoed));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

while arduinoObj.NumBytesAvailable > 0 %to avoid loosing time with garbage
    discard = readline(arduinoObj);  % Clear all startup messages, remove semicolon to display
    if not(isempty(strfind(discard,"Printer ready")))
        disp("✅ Printer Ready")
        read(arduinoObj, arduinoObj.NumBytesAvailable, "uint8");%get rid of a last lost character but this is just an issue with GNU Octave
    else
        disp("❌ Printer not yet ready");
    end
end
