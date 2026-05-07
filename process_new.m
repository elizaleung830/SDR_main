% process_new.m — Range-Doppler processing for PUP EN24C T2R4 .bin captures.
%
% Differences from process.m:
%   - Reshapes all chirps into a matrix instead of looking at just one sweep.
%   - Removes static clutter (ceiling, DC spike, Tx leakage) by subtracting
%     the mean range profile across all chirps.
%   - Plots a Range-vs-Time map so moving targets (hand) are visible.
%   - Plots a Range-Doppler map to show target velocity.
%   - X-axis in metres (not kHz) so physical interpretation is direct.

%% ── Parameters ───────────────────────────────────────────────────────────────
% Edit these to match the capture you want to load.
BIN_FILE = "C:\Users\eliza\SDR_main\SDR_main\temp\cap.bin";

c    = 3e8;         % speed of light (m/s)
fc   = 24.125e9;    % centre frequency (Hz) — midpoint of 24.0..24.25 GHz
BW   = 250e6;       % sweep bandwidth (Hz)  — 24.25 - 24.0 GHz
T_sw = 1e-3;        % sweep time (s)        — sweep_time_idx=2 → 1 ms
Fs   = 2048000;     % sampling rate (Hz)    — samples_per_sweep / T_sw
N    = 2048;        % samples per chirp
PRF  = 1 / T_sw;    % chirp repetition frequency (Hz) = 1000 Hz

MAX_RANGE_M = 10;   % clip display to this range (m) — indoor use

%% ── Load raw IQ ──────────────────────────────────────────────────────────────
fid = fopen(BIN_FILE, 'r');
if fid < 0, error('Cannot open %s', BIN_FILE); end
raw = fread(fid, '*int16');
fclose(fid);

I = double(raw(1:2:end));
Q = double(raw(2:2:end));
min_len = min(length(I), length(Q));
iq = I(1:min_len) + 1i * Q(1:min_len);

%% ── Reshape into chirp matrix [num_chirps × N] ───────────────────────────────
num_chirps = floor(length(iq) / N);
iq = iq(1 : num_chirps * N);
% Each row = one chirp (fast-time along columns, slow-time along rows).
chirp_matrix = reshape(iq, N, num_chirps).';    % [num_chirps × N]

fprintf('Loaded %d chirps  (%.2f s at PRF = %.0f Hz)\n', ...
        num_chirps, num_chirps / PRF, PRF);

%% ── Range FFT (fast-time) ────────────────────────────────────────────────────
win_range  = hann(N).';                             % 1×N — applied to each chirp
chirp_win  = chirp_matrix .* win_range;
range_fft  = fft(chirp_win, N, 2);                 % [num_chirps × N]
range_fft  = range_fft(:, 1:N/2);                  % keep positive-freq bins only

%% ── Static clutter removal ───────────────────────────────────────────────────
% Why: the DC spike and ceiling are static — they appear at the same range bin
% in every chirp.  Subtracting the mean across chirps removes anything that
% did not change over time (DC offset, Tx leakage, ceiling, walls).
% What remains: targets that moved — i.e. your hand.
clutter           = mean(range_fft, 1);             % 1×(N/2) — static mean
range_fft_clean   = range_fft - clutter;            % [num_chirps × (N/2)]

%% ── Axes ─────────────────────────────────────────────────────────────────────
% Range axis: f_beat = 2*R*BW / (c*T_sw)  →  R = f_beat * c*T_sw / (2*BW)
f_beat_axis = (0 : N/2-1) * (Fs / N);              % Hz per FFT bin
range_axis  = f_beat_axis * c * T_sw / (2 * BW);   % metres

% Time axis (slow-time)
time_axis   = (0 : num_chirps-1) * T_sw;            % seconds

% Doppler / velocity axis
win_dop     = hann(num_chirps);
rdm         = range_fft_clean .* win_dop;
rdm         = fftshift(fft(rdm, num_chirps, 1), 1); % FFT across chirps
doppler_hz  = (-num_chirps/2 : num_chirps/2-1) * (PRF / num_chirps);
velocity_ms = doppler_hz * c / (2 * fc);            % m/s

% Range bin index limit for display
[~, r_idx]  = min(abs(range_axis - MAX_RANGE_M));

%% ── Plot 1: Range profile — raw vs clutter-removed ──────────────────────────
% Shows the ceiling peak and what is left after removal.
figure('Name', 'Range Profile');

subplot(2,1,1);
rp_raw = 20*log10(abs(mean(range_fft, 1)) + 1e-10);
plot(range_axis(1:r_idx), rp_raw(1:r_idx), 'Color', '#0072BD', 'LineWidth', 1.2);
grid on;
title('Range Profile — raw (averaged over all chirps)');
xlabel('Range (m)');  ylabel('Magnitude (dB)');
% Annotate: ceiling should appear as the tallest peak beyond ~0.5 m.

subplot(2,1,2);
rp_clean = 20*log10(abs(mean(range_fft_clean, 1)) + 1e-10);
plot(range_axis(1:r_idx), rp_clean(1:r_idx), 'Color', '#D95319', 'LineWidth', 1.2);
grid on;
title('Range Profile — after clutter removal (residual moving energy)');
xlabel('Range (m)');  ylabel('Magnitude (dB)');

%% ── Plot 2: Range vs Time — hand movement ────────────────────────────────────
% Each row = one chirp.  Bright streaks show the hand at a changing range.
% The ceiling is gone (removed by clutter subtraction above).
figure('Name', 'Range vs Time');
rt_dB = 20*log10(abs(range_fft_clean(:, 1:r_idx)) + 1e-10);
imagesc(range_axis(1:r_idx), time_axis, rt_dB);
axis xy;
colormap jet;  colorbar;
title('Range vs Time  (clutter removed — hand movement visible as diagonal streak)');
xlabel('Range (m)');
ylabel('Time (s)');
clim([prctile(rt_dB(:), 50)  max(rt_dB(:))]);   % auto-scale to top 50% of dB range

%% ── Plot 3: Range-Doppler map ────────────────────────────────────────────────
% X = range, Y = radial velocity.  Moving hand appears away from 0 m/s line.
% Static clutter already removed, so 0 m/s line should be quiet.
figure('Name', 'Range-Doppler Map');
rdm_dB = 20*log10(abs(rdm(:, 1:r_idx)) + 1e-10);
imagesc(range_axis(1:r_idx), velocity_ms, rdm_dB);
axis xy;
colormap jet;  colorbar;
title('Range-Doppler Map  (clutter removed)');
xlabel('Range (m)');
ylabel('Radial velocity (m/s)');
clim([prctile(rdm_dB(:), 70)  max(rdm_dB(:))]);

fprintf('\nRange resolution : %.3f m\n', c / (2 * BW));
fprintf('Velocity resolution: %.4f m/s\n', c / (2 * fc * num_chirps * T_sw));
fprintf('Max unambiguous velocity: ±%.2f m/s\n', c * PRF / (4 * fc));
