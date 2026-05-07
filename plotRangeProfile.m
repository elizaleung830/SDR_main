function [rangeAxis, rangePower] = plotRangeProfile(iqArray1D, numSweeps, fs, T_sweep, BW, fftSize, targetSweep)
    % Inputs:
    % iqArray1D: 1D complex array of IQ samples
    % numSweeps: Number of sweeps contained in the data
    % fs:        Sampling frequency (Hz)
    % T_sweep:   Active sweep time (s)
    % BW:        Bandwidth (Hz)
    % fftSize:   Size of the FFT (GUI defaults to 1024)

    %% 1. Reshape 1D Array to Matrix
    totalSamples = length(iqArray1D);
    samplesPerSweep = totalSamples / numSweeps;
    
    if mod(totalSamples, numSweeps) ~= 0
        error('Total samples must be divisible by the number of sweeps.');
    end
    
    % Reshape into [Samples per Sweep x Number of Sweeps]
    iqMatrix = reshape(iqArray1D, [samplesPerSweep, numSweeps]);

    %% 2. Signal Conditioning (Per GUI Logic)
    % Remove DC offset (mean of each sweep) [cite: 83]
    iqMatrix = iqMatrix - mean(iqMatrix, 1);
    
    % Apply Butterworth bandpass filter
    % GUI uses [0.04, 0.85] normalized frequency [cite: 76, 83]
    [b, a] = butter(4, [0.04, 0.85]);
    iqFiltered = filter(b, a, iqMatrix);

    %% 3. Range FFT & Static Clutter Removal
    % Perform FFT along the first dimension (Fast-Time) [cite: 83]
    rangeFFT = fft(iqFiltered, fftSize, 1);
    
    % Keep positive range bins [cite: 84]
    rangeFFT = rangeFFT(1:fftSize/2, :); 
    
    % Subtract mean across the second dimension (Slow-Time)
    % This is the "Moving Target Indicator" logic to remove ground/static clutter [cite: 84]
    rangeFFT = rangeFFT - mean(rangeFFT, 2);

    %% 4. Power Calculation & Normalization
    % Extract magnitude (GUI typically visualizes index 32) [cite: 84]
    rangePowerRaw = abs(rangeFFT(:, targetSweep));
    
    % GUI-specific normalization: scales signal to a 5-50 dB visual range [cite: 85]
    pMax = max(rangePowerRaw);
    pMin = min(rangePowerRaw);
    rangePower = (rangePowerRaw - pMin) / (pMax - pMin) * 45 + 5;

    %% 5. Range Axis Mapping
    c = 3e8; 
    LARR = 1.4; % Calibration Range Ratio from source [cite: 53]
    
    % The GUI maps the axis up to fs/4 [cite: 86]
    freqAxis = linspace(0, fs/4, fftSize/2);
    
    % Convert frequency to range [cite: 86]
    rangeAxis = freqAxis * T_sweep / (2 * BW) * LARR * c;

    %% 6. Visualization (Strictly matched to GUI Source)
    figure('Color', 'w', 'Name', 'SDR Range Profile');
    
    % The source plots a solid green line with LineWidth 1 
    plot(rangeAxis, rangePower, 'Color', [0 1 0], 'LineWidth', 1);
    
    % The source uses a black background with specific dark green dashed grid lines [cite: 88, 89]
    set(gca, 'Color', 'k', ...
             'XGrid', 'on', 'YGrid', 'on', ...
             'GridColor', [0.23 0.44 0.34], ...
             'GridLineStyle', '--', ...
             'GridAlpha', 0.8);
             
    xlabel('Range (meters)'); 
    ylabel('Power/frequency (dB/Hz)'); 
    title(['Range Profile - Sweep ', num2str(targetSweep)]);
    
    % Lock axis limits to fit the normalized power bounds cleanly
    axis([0 max(rangeAxis) 0 55]);
end