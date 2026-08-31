function [I_out, Q_out, freqAxis, rangeAxis, avgSpectrum] = process_pi(filename, sampSize, T_sweep, BW)
% processRangeProfile  Load SDR data and compute averaged range profile
%
% INPUTS:
%   filename   - full path to data file (no extension needed)
%   sampSize   - number of samples per sweep (e.g. 128)
%   T_sweep    - sweep duration in seconds (e.g. 5e-4)
%   BW         - bandwidth in Hz (e.g. 0.25e9)
%
% OUTPUTS:
%   I_out       - raw I data, reshaped [sampSize/2 x numSweeps]
%   Q_out       - raw Q data, reshaped [sampSize/2 x numSweeps]
%   freqAxis    - frequency axis in Hz [1 x fftSize]
%   rangeAxis   - range axis in metres  [1 x fftSize]
%   avgSpectrum - averaged magnitude range profile [fftSize x 1]

    %% Constants
    c      = 3e8;
    fs     = sampSize / T_sweep / 2;   % sampling frequency
    fftSize = sampSize;

    
     % 1. Open the binary file for reading
    fid = fopen(filename, 'rb');
    raw = fread(fid, inf, 'uint16');
    headerIndices = find(raw >= 49152);
    raw(headerIndices) = raw(headerIndices) - 49152;
    fclose(fid);

    %% Separate interleaved Q/I
    % File format: Q at odd indices (1,3,5,...), I at even indices (2,4,6,...)
    Q = double(raw(1:2:end));
    I = double(raw(2:2:end));

    %% Form complex IQ, trim to equal length
    min_len  = min(length(I), length(Q));
    I        = I(1:min_len);
    Q        = Q(1:min_len);

    %% Trim to integer number of sweeps
    numSweeps = floor(min_len / (sampSize / 2));
    nSamples  = numSweeps * sampSize / 2;
    I         = I(1:nSamples);
    Q         = Q(1:nSamples);
    iq_data   = I + 1i * Q;

    %% Reshape: each column = one sweep
    Iq_sweeps = reshape(iq_data, sampSize/2, numSweeps);  % [sampSize/2 x numSweeps]

    %% Expose raw I and Q as reshaped matrices for output
    I_out = reshape(I, sampSize/2, numSweeps);  % [sampSize/2 x numSweeps]
    Q_out = reshape(Q, sampSize/2, numSweeps);  % [sampSize/2 x numSweeps]

    %% DC offset removal
    Iq_sweeps = Iq_sweeps - mean(Iq_sweeps, 'all');

    %% Range FFT across each sweep (column-wise)
    rangefft_all = fft(Iq_sweeps, fftSize, 1);   % [fftSize x numSweeps]

    %% Average magnitude across all sweeps
    magSpectra  = abs(rangefft_all);              % [fftSize x numSweeps]
    avgSpectrum = mean(magSpectra, 2);            % [fftSize x 1]

    %% Build frequency and range axes
    freqBinWidth = fs / fftSize;
    freqAxis     = (0:fftSize-1) * freqBinWidth; % [1 x fftSize]
    rangeAxis    = freqAxis * T_sweep / (2 * BW) * c; % [1 x fftSize]

end