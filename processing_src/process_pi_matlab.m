filename = "C:\Users\eliza\SDR_main\SDR_main\data\30082026_matlab\3008_1GHz_128_0-5ms_outdoor_2m";
filename_pi = "C:\Users\eliza\SDR_main\SDR_main\data\30082026_pi\3008_1GHz_256_0-5ms_outdoor_3m_pi.bin";
sampSize=128;
T_sweep=0.5e-3;
BW=1e9;
display_sweep=200;
[I, Q, freqAxis, rangeAxis, avgSpectrum] = process_matlab(filename, sampSize, T_sweep, BW);
[I_pi, Q_pi, freqAxis_pi, rangeAxis_pi, avgSpectrum_pi] = process_pi(filename_pi, sampSize, T_sweep, BW);

% Plot raw IQ
figure;
subplot(3,1,1);
plot(Q(:,display_sweep), 'b--', 'LineWidth', 1.5, 'DisplayName', ['Q — Matlab GUI']);
hold on;
plot(Q_pi(:,display_sweep), 'r--', 'LineWidth', 1.5, 'DisplayName', ['Q — Pi']);
hold off;
xlabel('Sample Index');
ylabel('Amplitude');
title('Raw Q Data');
legend('Location', 'best');
grid on;

subplot(3,1,2);
plot(I(:,display_sweep), 'b--', 'LineWidth', 1.5, 'DisplayName', ['I — Matlab GUI']);
hold on;
plot(I_pi(:,display_sweep), 'r--', 'LineWidth', 1.5, 'DisplayName', ['I — Pi']);
hold off;
xlabel('Sample Index');
ylabel('Amplitude');
title('Raw I Data');
legend('Location', 'best');
grid on;


% Plot averaged spectrum (positive frequencies only)
subplot(3,1,3);
plot(rangeAxis, avgSpectrum', 'b--', 'LineWidth', 1.5, 'DisplayName', ['Matlab GUI']);
hold on;
plot(rangeAxis_pi, avgSpectrum_pi', 'r--', 'LineWidth', 1.5, 'DisplayName', ['Pi']);
hold off;
xlabel('Range (m)');
ylabel('Averaged Magnitude');
title('Range Profile Averaged Across Sweeps');
legend('Location', 'best');
grid on;
