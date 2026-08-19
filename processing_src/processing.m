clear;
filename1 = "C:\Users\eliza\SDR_main\SDR_main\temp\1907_160cm_PUP_01.txt";
filename1 = "C:\Users\eliza\SDR_main\SDR_main\temp\1907_160cm_PUP_128ns_05ms_01.txt";

raw = readmatrix(filename1); 
raw = raw(41:end); % remove first 40 lines of meta data

sampSize = 128;
T_sweep=5e-4;
fs = sampSize/ T_sweep;          % Sampling frequency
fftSize = 128;
BW=1e9;
c = 3e8;

% Separate I (even indices) and Q (odd indices) and form the complex array
Q = double(raw(1:2:end)); 
I = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq_data = I(1:min_len) + 1i * Q(1:min_len);

% select the minimum length out of 2 data
numSweeps = floor(min(length(iq_data))/sampSize);
iq_data = I(1:numSweeps*sampSize) + 1i * Q(1:numSweeps*sampSize);

% Reshape into [sampSize x numSweeps] so each COLUMN of iq_sweeps is an
% array of data from single sweep (128 iq samples)
Iq_sweeps = reshape(iq_data, fftSize, numSweeps);

% Plot raw IQ
figure;
subplot(3,1,1);
plot(imag(Iq_sweeps(:,1)));
xlabel('Sample Index');
ylabel('Amplitude');
title('Raw Q Data');
grid on;

subplot(3,1,2);
plot(real(Iq_sweeps(:,1)));
xlabel('Sample Index');
ylabel('Amplitude');
title('Raw I Data');
grid on;

% Remove the DC offset
Iq_sweeps = Iq_sweeps - mean(Iq_sweeps, 'all');

% FFT each column (each sweep) independently
rangefft_all = fft(Iq_sweeps, fftSize, 1);

% Magnitude spectrum per sweep, then average across sweeps
magSpectra = abs(rangefft_all);              % [fftSize x numSweeps]
avgSpectrum = mean(magSpectra, 2);            % average across sweeps

% Frequency / range axis 
frequencyBinWidth = fs / fftSize;
freqAxis = (0:fftSize/2 - 1) * frequencyBinWidth;
rangeAxis = freqAxis * T_sweep / (2 * BW) * c * 1.4;

% Plot averaged spectrum (positive frequencies only)
subplot(3,1,3);
plot(rangeAxis, avgSpectrum(1:fftSize/2));
xlabel('Range (m)');
ylabel('Averaged Magnitude');
title('Range Profile Averaged Across Sweeps');
grid on;
