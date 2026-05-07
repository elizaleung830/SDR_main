% 1. Open the binary file for reading
fid = fopen("C:\Users\eliza\SDR_main\SDR_main\temp\cap.bin", 'rb');

% 2. Read the data as 16-bit integers
% Using '*int16' keeps the data type as int16 in MATLAB (saves memory)
raw = fread(fid, inf, 'uint16'); 
% Now the standard header removal logic will work
headerIndices = find(raw >= 49152);
raw(headerIndices) = raw(headerIndices) - 49152;

% 3. Close the file
fclose(fid);

% 4. Separate I (odd indices) and Q (even indices) and form the complex array
% Note: MATLAB uses 1-based indexing
I = double(raw(1:2:end)); 
Q = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq_data = I(1:min_len) + 1i * Q(1:min_len);
iq_data = iq_data(1:873*1024);

numSweeps = floor(length(iq_data)/1024);
iq_data = I(1:numSweeps*2048) + 1i * Q(1:numSweeps*2048);
fs = 2048000;           % Sampling frequency (2.048 MHz)
fftSize = 1024;           % Samples per sweep
T_sweep=0.001;
BW=250000000;
targetSweep = 100;
[rangeAxis, rangePower] = plotRangeProfile(iq_data,  numSweeps, fs, T_sweep, BW,fftSize, 800);