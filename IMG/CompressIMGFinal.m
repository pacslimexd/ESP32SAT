%% Function to connect to Serial-ESP32 and wait for command to take 
% a webcam picture. The image is encoded in base64 to be send to serial

close all; clear all; clc

% Set up serial port
serialPort = "COM6"; % Change to your actual serial port, use seriallist
baudRate = 115200;
codeword = "GND:MSG: GETIMG";

s = serialport(serialPort, baudRate);
configureTerminator(s, "LF");  % Assuming LF (\n) line endings
flush(s);  % Clear buffer
chunkSize = 100;  % Change chunk size, the buffer limits and stability
%^^^^^^^^^^^^^^^

% Start webcam
cam = webcam;  % Default webcam
preview(cam);  % live preview

disp("Waiting for codeword...");

while true
    if s.NumBytesAvailable > 0
        line = readline(s);
        disp("Received: " + line);

        if strtrim(line) == codeword
            disp("Codeword matched. Taking a picture...");
            writeline(s,'TEL:SENDPIC');
            pause(0.2);
            img = snapshot(cam);
            YCBCR = rgb2ycbcr(img);
            picBW= YCBCR(:,:,1);
            filename = ['photo_' datestr(now, 'yyyymmdd_HHMMSS') '.jpg'];
            imwrite(img, filename);
            disp(['Saved to: ' filename]);
            %sending to GS
            %rs2=imresize(picBW,[128 NaN]);    %resize to send faster
            img = compressImage(8, 64, filename);
            rs2 = imresize(img,[128 NaN]);
            %^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            sz = size(rs2)
            encodedBase64 = matlab.net.base64encode(rs2(:));

            numChunks = ceil(length(encodedBase64) / chunkSize);
            for k = 1:numChunks
                startIdx = (k-1)*chunkSize + 1;
                endIdx = min(k*chunkSize, length(encodedBase64));
                chunk = encodedBase64(startIdx:endIdx)

                writeline(s, chunk);

                % pause for buffer
                pause(0.1);
            end

            writeline(s,'END');
            clear('cam')
            clear s
            break
        end
    end
    pause(0.1);  % delay
end


%% Decode received data and sho image, need to know image scale. 
% To solve this, size can be send with image info or known in advance, for
% this code is known

outpict = matlab.net.base64decode(encodedBase64);
outpict = reshape(outpict,size(rs2));
imshow(outpict)