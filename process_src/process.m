isPUP = true;
filename1 = "C:\Users\eliza\OneDrive\Desktop\SDR\SDR_main\temp\1907_160to10cm_PUP_128ns_05ms_01.txt";
filename2 = "C:\Users\eliza\OneDrive\Desktop\SDR\SDR_main\temp\1907_100cm_PUP_128ns_05ms_01.txt";
if isPUP == false
    % 1. Open the binary file for reading
    fid = fopen(filename1, 'rb');
    fid2 = fopen(filename2, 'rb');
    
    % 2. Read the data as 16-bit integers
    % Using '*int16' keeps the data type as int16 in MATLAB (saves memory)
    raw = fread(fid, inf, 'uint16');
    raw2 = fread(fid2, inf, 'uint16');

    % Now the standard header removal logic will work
    headerIndices = find(raw >= 49152);
    raw(headerIndices) = raw(headerIndices) - 49152;
    
    headerIndices2 = find(raw2 >= 49152);
    raw2(headerIndices2) = raw2(headerIndices2) - 49152;
else
    raw = readmatrix(filename1); 
    raw2 = readmatrix(filename2); 
end

sampSize = 128;
T_sweep=0.5 * 1E-3;
fs = sampSize/ T_sweep;          % Sampling frequency
BW=250000000;
targetSweep = 1500;

% 3. Close the file
if isPUP == false
    fclose(fid);
    fclose(fid2);
end
% 4. Separate I (even indices) and Q (odd indices) and form the complex array
% Note: MATLAB uses 1-based indexing
Q = double(raw(1:2:end)); 
I = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq_data = I(1:min_len) + 1i * Q(1:min_len);

Q2 = double(raw2(1:2:end)); 
I2 = double(raw2(2:2:end));
min_len2 = min(length(I2), length(Q2));
iq_data2 = I2(1:min_len2) + 1i * Q2(1:min_len2);

% select the minimum length out of 2 data
numSweeps = floor(min(length(iq_data), length(iq_data2))/sampSize);
iq_data = I(1:numSweeps*sampSize) + 1i * Q(1:numSweeps*sampSize);
iq_data2 = I2(1:numSweeps*sampSize) + 1i * Q2(1:numSweeps*sampSize);

[rangeAxis, rangePower] = plotRangeProfile(iq_data,  numSweeps, fs, T_sweep, BW,256, targetSweep);
[rangeAxis_b, rangePower_b] = plotRangeProfile(iq_data2,  numSweeps, fs, T_sweep, BW,256, targetSweep);


figure('Color','k');

% First range profile
plot(rangeAxis, rangePower, ...
    'g', 'LineWidth', 1.5);
hold on;

%Second range profile
plot(rangeAxis_b, rangePower_b, ...
    'r', 'LineWidth', 1.5);

% Graph formatting
xlabel( 'Range (m)');
ylabel('Power (dB)');
title('Comparison of Range Profiles');

%legend('Background Capture', 'Waving hand  Capture');

grid on;

% Optional dark theme
set(gca, 'Color', 'k', ...
         'XColor', 'w', ...
         'YColor', 'w', ...
         'GridColor', [0.23 0.44 0.34], ...
         'GridLineStyle', '--');

hold off;