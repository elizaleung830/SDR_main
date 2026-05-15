% 1. Open the binary file for reading
fid = fopen("C:\Users\eliza\SDR_main\SDR_main\temp\cap_background.bin", 'rb');
fid2 = fopen("C:\Users\eliza\SDR_main\SDR_main\temp\cap_wavinghand2.bin", 'rb');

% 2. Read the data as 16-bit integers
% Using '*int16' keeps the data type as int16 in MATLAB (saves memory)
raw = fread(fid, inf, 'uint16');
raw2 = fread(fid2, inf, 'uint16');

% Now the standard header removal logic will work
headerIndices = find(raw >= 49152);
raw(headerIndices) = raw(headerIndices) - 49152;

headerIndices2 = find(raw2 >= 49152);
raw2(headerIndices2) = raw2(headerIndices2) - 49152;
sampSize = 256;

% 3. Close the file
fclose(fid);
fclose(fid2);

% 4. Separate I (odd indices) and Q (even indices) and form the complex array
% Note: MATLAB uses 1-based indexing
I = double(raw(1:2:end)); 
Q = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq_data = I(1:min_len) + 1i * Q(1:min_len);

I2 = double(raw2(1:2:end)); 
Q2 = double(raw2(2:2:end));
min_len2 = min(length(I2), length(Q2));
iq_data2 = I2(1:min_len2) + 1i * Q2(1:min_len2);

% select the minimum length out of 2 data
numSweeps = floor(min(length(iq_data), length(iq_data2))/sampSize);
iq_data = I(1:numSweeps*sampSize) + 1i * Q(1:numSweeps*sampSize);
iq_data2 = I2(1:numSweeps*sampSize) + 1i * Q2(1:numSweeps*sampSize);

fs = 256000;           % Sampling frequency (2.048 MHz)
          % Samples per sweep
T_sweep=0.001;
BW=250000000;
targetSweep = 500;
[rangeAxis, rangePower] = plotRangeProfile(iq_data,  numSweeps, fs, T_sweep, BW,sampSize, targetSweep);
[rangeAxis_b, rangePower_b] = plotRangeProfile(iq_data2,  numSweeps, fs, T_sweep, BW,sampSize, targetSweep);


figure('Color','w');

% First range profile
plot(rangeAxis, rangePower, ...
    'g', 'LineWidth', 1.5);
hold on;

% Second range profile
plot(rangeAxis_b, rangePower_b, ...
    'r', 'LineWidth', 1.5);

% Graph formatting
xlabel( 'Range (m)');
ylabel('Power (dB)');
title('Comparison of Range Profiles');

legend('Background Capture', 'Waving hand  Capture');

grid on;

% Optional dark theme
set(gca, 'Color', 'k', ...
         'XColor', 'w', ...
         'YColor', 'w', ...
         'GridColor', [0.23 0.44 0.34], ...
         'GridLineStyle', '--');

hold off;