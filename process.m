% 1. Open the binary file for reading
fid = fopen("C:\Users\eliza\SDR_main\SDR_main\temp\cap.bin", 'r');

% 2. Read the data as 16-bit integers
% Using '*int16' keeps the data type as int16 in MATLAB (saves memory)
raw = fread(fid, '*int16'); 

% 3. Close the file
fclose(fid);

% 4. Separate I (odd indices) and Q (even indices) and form the complex array
% Note: MATLAB uses 1-based indexing
I = double(raw(1:2:end)); 
Q = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq_data = I(1:min_len) + 1i * Q(1:min_len);

Fs = 2048000;           % Sampling frequency (2.048 MHz)
N_fft = 2048;           % Samples per sweep


first_sweep = iq_data(1:N_fft);
window = hann(N_fft);
%first_sweep = first_sweep .* window;
spectrum = fftshift(fft(first_sweep));

% Convert to Magnitude in decibels (dB)
mag_dB = 20*log10(abs(spectrum) + 1e-10);

% --- 4. Calculate Frequency Axis ---
% Creates an axis from -1.024 MHz to +1.024 MHz
freq_axis_hz = (-N_fft/2 : N_fft/2-1) * (Fs / N_fft);
freq_axis_khz = freq_axis_hz / 1000;

% --- 5. Plot the Spectrum ---
figure;
plot(freq_axis_khz, mag_dB, 'LineWidth', 1.2, 'Color', '#0072BD');
grid on;
title('IF Beat Signal Frequency Spectrum');
xlabel('Frequency (kHz)');
ylabel('Magnitude (dB)');

% Set X-axis limits to match the Nyquist bandwidth (-Fs/2 to Fs/2)
xlim([-Fs/2000 Fs/2000]);