clear;
% Parameters
c = 3e8;                % Speed of light (m/s)
f_start = 24e9;         % Start frequency (Hz)
f_end = 26e9;         % Start frequency (Hz)
f_samp = 256000;       % sampling frequency of the beat signal (Hz)
B = f_end-f_start;              % Sweep bandwidth (Hz)
T_sweep = 1e-3;         % Sweep time (s)
R = 10;                 % Target range (m)
Num_sweeps =5;
fs = 2 * B;
D = fs / f_samp; 

% Derived parameters
S = B / T_sweep;        % Frequency slope (Hz/s)
tau = 2 * R / c;        % Time delay for round trip

% Time vector 
t_full = 0:1/fs:(Num_sweeps*T_sweep - 1/fs);   % one continuous time axis

n_tx = floor(t_full / T_sweep);
t_slow_tx = n_tx * T_sweep;
tx = exp(1i * (2*pi*f_start*(t_full - t_slow_tx) + pi*S*(t_full - t_slow_tx).^2));

k = round(tau * fs);   % delay in samples

rx = zeros(1, length(tx));
rx(k+1:end) = tx(1:end-k);

iq_signal = tx .* conj(rx);

t_full = (0:length(tx)-1) / fs;

% Extract I and Q components
I_channel = real(iq_signal);
Q_channel = imag(iq_signal);

% Down sample to the output 
I_beat = downsample(I_channel, D);
Q_beat = downsample(Q_channel, D);
IQ_beat = I_beat + Q_beat*1j;
t_beat = downsample(t_full, D);


% Plot
figure;
subplot(3,1,1);
phase_tx = unwrap(angle(tx));
inst_freq_tx = [0, diff(phase_tx)] .* fs / (2*pi); 
plot(t_full*  1e3, inst_freq_tx/1e9);
hold on
% phase_rx = unwrap(angle(rx));
% inst_freq_rx = [0, diff(phase_rx)] .* fs / (2*pi); 
% plot(t_full, inst_freq_rx/1e9);

xlabel('Time (ms)');
ylabel('Frequency (GHz)');
title("Freqeuncy transmitted by Tx channel of the radar");
grid on;



subplot(3,1,2);
% Nfft = length(iq_signal);
% f_axis = (0:Nfft-1) * (fs/Nfft);
% X = abs(fft(iq_signal));
% plot(f_axis, X);
subplot(3,1,2);
plot(t_beat*  1e3, I_beat, 'LineWidth', 1.2);
hold on;
plot(t_beat*  1e3, Q_beat, '--', 'LineWidth', 1.2);
hold off;
xlim([0, T_sweep/2 *  1e3]);
xlabel('Time (ms)');
ylabel('Amplitude');
legend('I (in-phase)', 'Q (quadrature)', 'Location', 'northeast');
title(sprintf('I and Q signals collected by the radar (sampled at %d Hz)', f_samp));
grid on;

% FFT Processing
grid on;
N = round(T_sweep * f_samp);      
[rangeAxis, rangePower] = plotRangeProfileRaw(IQ_beat, Num_sweeps, f_samp, T_sweep, B, N, 3);
R_max = f_samp * c / (2*S);

subplot(3,1,3);
plot(rangeAxis, rangePower);
xlabel('Range (m)');
ylabel('Amplitude');
xlim([0, R_max]);
ylim([0, max(rangePower)+10])
title_str = sprintf('FMCW Range Detection of a Perfect Reflector %d m away', R);
title(title_str);
grid on;
