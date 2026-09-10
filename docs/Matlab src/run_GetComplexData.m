%RUN_GETCOMPLEXDATA  Example: acquire I/Q outside the PUP GUI.
%
% Two modes:
%   USE_HARDWARE = true   -> init, configure, and read live from the radar
%   USE_HARDWARE = false  -> replay a saved RawData vector from a .mat file
%
% Run this from the folder containing the .mexw64 files ("Matlab src"),
% or add that folder to the path first.
%
% ORDER MATTERS. The GUI always does init -> Send_Basic_Parameter ->
% Send_PLL_Sawtooth -> GetComplexData. Skipping Send_Basic_Parameter leaves
% the MCU streaming whatever Tx/Rx layout it was last set to, the reshape in
% GetComplexData fails against the layout these handles assume, and you get a
% silent matrix of zeros.

clear; clc;

USE_HARDWARE = true;
RAW_FILE     = 'RawData.mat';   % replay mode: must contain a variable RawData

%% ---- Radar configuration ----------------------------------------------
% Only these are yours to choose; samples-per-sweep is derived from the
% sweep time and receiver count (see makeRadarHandles).
handles = makeRadarHandles( ...
    'SweepTimeIndex', 1, ...        % 1..5 = 0.5 / 1 / 2 / 4 / 8 ms
    'Transmitter',    'Tx1', ...    % 'Tx1' | 'Tx2' | 'Tx1.Tx2'
    'Receiver',       'Rx4', ...    % 'Rx1'..'Rx4' | 'Rx1.Rx2' | 'Rx3.Rx4' | 'Rx1.Rx2.Rx3.Rx4'
    'SamplingIndex',  4, ...        % 1 = BASN, 2 = BASN/2, 3 = /4, 4 = /8
    'NumSweeps',      128, ...
    'FrequencyLow',   24.0e9, ...
    'FrequencyHigh',  26.0e9);

LASN = handles.SN_Selections(handles.ActiveSamplingNumberValue);
fprintf('Config: %d Tx / %d Rx, %g ms sweep, %d samples/sweep/Rx, %d sweeps\n', ...
    handles.ActiveNum_Tx, handles.ActiveNum_Rx, ...
    handles.ActiveSweepTime*1e3, LASN, handles.NumSweeps);

%% ---- Get the data -----------------------------------------------------
if USE_HARDWARE
    % 1. Bring up USB and download the firmware.
    handles = PUPradar_initiating(handles, 'Verbose', true);

    % 2. Tell the MCU the modulation / sweep / sampling / Tx / Rx selection.
    %    Without this the board keeps its previous layout and the decode
    %    below returns zeros.
    fprintf('Sending basic parameters...\n');
    Send_Basic_Parameter(handles, 'Verbose', true);

    % 3. Program the FMCW chirp.
    fprintf('Programming PLL...\n');
    Send_PLL_Sawtooth(handles);

    % 4. Acquire.
    [Tx1, Tx2, NumSweeps, RawData] = GetComplexData(handles);
else
    S = load(RAW_FILE);
    [Tx1, Tx2, NumSweeps, RawData] = GetComplexData(handles, S.RawData);
end

fprintf('Decoded %d sweeps.\n', NumSweeps);
fprintf('Tx1: %s   Tx2: %s\n', mat2str(size(Tx1)), mat2str(size(Tx2)));

%% ---- Sanity check -----------------------------------------------------
% A [LASN*4 x NumSweeps] all-zero result is the decode-failure signature.
if ~any(Tx1(:)) && ~any(Tx2(:))
    warning(['Both channels are all zeros - the decode failed. See the ' ...
             'GetComplexData:decodeFailed warning above for the reason.']);
    return
end

%% ---- Quick look -------------------------------------------------------
Active = Tx1;
if ~any(Active(:)); Active = Tx2; end

figure('Name', 'GetComplexData output');

subplot(2,1,1);
plot(real(Active(:,1)), 'DisplayName', 'I');
hold on;
plot(imag(Active(:,1)), 'DisplayName', 'Q');
grid on; legend show;
title('First sweep - raw I/Q');
xlabel('Sample'); ylabel('Amplitude');

subplot(2,1,2);
RangeProfile = 20*log10(abs(fft(Active(:,1))) + eps);
plot(RangeProfile);
grid on;
title('First sweep - range profile');
xlabel('Range bin'); ylabel('dB');
