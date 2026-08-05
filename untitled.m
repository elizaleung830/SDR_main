% Set illustrative parameters to match the visual style of the provided image
% For true RF signals, parameters would be much higher, but would blur 
% together in a plot, losing the illustrative power.
% We choose audio frequencies to create a clear visual of a chirp with 
% visible cycles. A ratio of only ~4-8 cycles per chirp is chosen for clarity.

f_start = 100;      % Starting frequency of chirp [Hz]
f_end = 1000;        % Ending frequency of chirp [Hz]
T_chirp = 0.01;     % Duration of one chirp (sweep time) [s] (10 ms)
num_chirps = 2;     % Number of consecutive chirps to show
fs = 10000;          % Sample rate [Hz] (above Nyquist, high enough for smooth plot)

% derived parameters
bandwidth_B = f_end - f_start;
f0_center = (f_start + f_end) / 2;
slope_s = bandwidth_B / T_chirp;

% create the total time vector for all chirps
total_time = T_chirp * num_chirps;
t = 0 : 1/fs : total_time - 1/fs;

% 1. Create Frequency Vector (Sawtooth Modulation Signal)
% The mathematical definition is freq(t) = f_start + slope * (t mod T_chirp)
% Constructing precision vectors is more robust than relying on mod function
N_samples_per_chirp = round(T_chirp * fs);
freq_one_chirp = linspace(f_start, f_end, N_samples_per_chirp);
freq_vec = repmat(freq_one_chirp, [1, num_chirps]);

% ensuring vectors match length, handling potential rounding mismatch
if length(t) ~= length(freq_vec)
    N_total = min(length(t), length(freq_vec));
    t = t(1:N_total);
    freq_vec = freq_vec(1:N_total);
end

% 2. Create the RF Modulated Signal
% The instantaneous frequency f(t) is defined by freq_vec.
% The phase phi(t) is the integral of 2*pi*f(t).
% We use cumsum to approximate the integral: phi = 2*pi * cumsum(f) * dt
dt = 1 / fs;
phase_vec = 2 * pi * cumsum(freq_vec) * dt;

% Generate the signal (starting from phase 0 to match visual start)
rf_signal = sin(phase_vec);

% 3. Create Plots
figure('Position', [100, 100, 800, 700], 'Name', 'LFM Chirp Modulation Plots');

% --- Top Subplot: Frequency Modulation ---
subplot(2, 1, 1);
hold on; grid on;

% plot consecutive chirps
for i = 0 : num_chirps-1
    t_start = i * T_chirp;
    t_end = (i+1) * T_chirp - 1/fs;
    
    % select points for current chirp to draw line perfectly to edge
    N_s = i * N_samples_per_chirp + 1;
    N_e = (i+1) * N_samples_per_chirp;
    
    current_t = t(N_s : N_e);
    current_freq = freq_vec(N_s : N_e);
    
    % draw the ramp and the rapid reset
    plot(current_t, current_freq, 'b-', 'LineWidth', 2);
    
    % Draw vertical line to visualize reset at chirp transition
    if i < num_chirps-1
        plot([t_end t_end + 1/fs], [f_end f_start], 'b-', 'LineWidth', 2);
    end
end

% replicate image annotations
f0_pos = -total_time*0.03; % define an offset outside axis

% plot f0 center frequency horizontal line

% clean up axis
xlabel('time [s]', 'FontSize', 11, 'FontName', 'Times New Roman');
ylabel('frequency [Hz]', 'FontSize', 11, 'FontName', 'Times New Roman');
axis([xlim, [f_start*0.4 f_end*1.1]]); % add extra vertical margin to show all annotations clearly
xticks(0:T_chirp:total_time); % clear markings at chirp points
yticks([f_start f_end]); % clear start/end frequency markings
yticklabels({'f_{min}', 'f_{max}'}); % use general labels but define f0 position

% replicate specific text to the right side of the plot

hold off;


% --- Bottom Subplot: RF Modulated Signal ---
subplot(2, 1, 2);
hold on; grid on;

% plot the total RF signal
plot(t, rf_signal, 'b-', 'LineWidth', 1);

% clean up axis to match image, keep -1 to 1 amplitude visible
ylabel('Amplitude', 'FontSize', 11, 'FontName', 'Times New Roman');
xlabel('time [s]', 'FontSize', 11, 'FontName', 'Times New Roman');
axis([xlim, [-1.2 1.3]]); % slight extra space for top label
xticks(0:T_chirp:total_time); % align time axis markings with top plot
yticks([]); % clean the standard y-axis labels as in image


hold off;