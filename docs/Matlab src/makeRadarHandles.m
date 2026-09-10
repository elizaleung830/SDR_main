function handles = makeRadarHandles(varargin)
%MAKERADARHANDLES  Build the handles struct the extracted PUP functions need,
%   outside the GUI.
%
%   handles = makeRadarHandles('Name', value, ...)
%
%   You set the four things the GUI lets you choose; everything else is
%   DERIVED, exactly as SetActiveParameters does (PUPradarGUI.m:696-801,
%   1982-2075):
%
%     'SweepTimeIndex'   1..5  = 0.5 / 1 / 2 / 4 / 8 ms      (default 1)
%     'Transmitter'      'Tx1' | 'Tx2' | 'Tx1.Tx2'           (default 'Tx1')
%     'Receiver'         'Rx1'|'Rx2'|'Rx3'|'Rx4'|
%                        'Rx1.Rx2'|'Rx3.Rx4'|'Rx1.Rx2.Rx3.Rx4'  (default 'Rx1')
%     'SamplingIndex'    1..4, where 1 = BASN, 2 = BASN/2, ... (default 1)
%     'NumSweeps'        sweeps to acquire                    (default 128)
%     'Modulation'       'Sawtooth' | 'CW'                    (default 'Sawtooth')
%     'FrequencyLow'     chirp start in Hz                    (default 24.0e9)
%     'FrequencyHigh'    chirp stop in Hz                     (default 24.25e9)
%
%   Derived for you: ActiveSweepTime, BASN, SN_Selections, ActiveNum_Tx,
%   ActiveNum_Rx, ActiveTxValue, ActiveRxValue, ActiveModulationValue.
%
%   Why the sampling NUMBER is not settable directly: only the sampling
%   INDEX goes to the MCU (opcode E3). The samples-per-sweep that index
%   resolves to depends on both the sweep time and the receiver count, per
%   the table at PUPradarGUI.m:699-801. Letting you set the number
%   independently makes it trivial to tell the MCU one thing and decode
%   assuming another, which throws in GetComplexData's reshape and silently
%   yields a matrix of zeros. Read the resolved value back from
%   handles.SN_Selections(handles.ActiveSamplingNumberValue).
%
%   Example:
%     h = makeRadarHandles('SweepTimeIndex', 1, 'Receiver', 'Rx1');
%     h.SN_Selections(h.ActiveSamplingNumberValue)   % -> 1024
%
%   See also SEND_BASIC_PARAMETER, SEND_PLL_SAWTOOTH, GETCOMPLEXDATA,
%   PUPRADAR_INITIATING.

p = struct( ...
    'SweepTimeIndex', 1, ...
    'Transmitter',    'Tx1', ...
    'Receiver',       'Rx1', ...
    'SamplingIndex',  1, ...
    'NumSweeps',      128, ...
    'Modulation',     'Sawtooth', ...
    'FrequencyLow',   24.0e9, ...
    'FrequencyHigh',  24.25e9);

if mod(numel(varargin), 2) ~= 0
    error('makeRadarHandles:badArgs', 'Arguments must be Name/value pairs.');
end
for k = 1:2:numel(varargin)
    if ~isfield(p, varargin{k})
        error('makeRadarHandles:badOpt', ...
            ['Unknown option ''%s''. Valid: SweepTimeIndex, Transmitter, ', ...
             'Receiver, SamplingIndex, NumSweeps, Modulation, FrequencyLow, ', ...
             'FrequencyHigh.'], varargin{k});
    end
    p.(varargin{k}) = varargin{k+1};
end

%% ---- Transmitter: string -> value + count (GUI 2000-2021) --------------
switch p.Transmitter
    case 'Tx1',     TxValue = 1; NumTx = 1;
    case 'Tx2',     TxValue = 2; NumTx = 1;
    case 'Tx1.Tx2', TxValue = 3; NumTx = 2;
    otherwise
        error('makeRadarHandles:badTx', ...
            'Transmitter must be ''Tx1'', ''Tx2'' or ''Tx1.Tx2''.');
end

%% ---- Receiver: string -> value + count (GUI 2026-2075) ----------------
switch p.Receiver
    case 'Rx1',             RxValue =  1; NumRx = 1;
    case 'Rx2',             RxValue =  2; NumRx = 1;
    case 'Rx3',             RxValue =  4; NumRx = 1;
    case 'Rx4',             RxValue =  8; NumRx = 1;
    case 'Rx1.Rx2',         RxValue =  3; NumRx = 2;
    case 'Rx3.Rx4',         RxValue = 12; NumRx = 2;
    case 'Rx1.Rx2.Rx3.Rx4', RxValue = 15; NumRx = 4;
    otherwise
        error('makeRadarHandles:badRx', ...
            ['Receiver must be ''Rx1''..''Rx4'', ''Rx1.Rx2'', ''Rx3.Rx4'' ', ...
             'or ''Rx1.Rx2.Rx3.Rx4''.']);
end

%% ---- Sweep time and the sampling table (GUI 696-801) -------------------
STI = p.SweepTimeIndex;
if ~ismember(STI, 1:5)
    error('makeRadarHandles:badSweepIdx', ...
        'SweepTimeIndex must be 1..5 (0.5 / 1 / 2 / 4 / 8 ms).');
end
SweepTimes = [0.5e-3, 1e-3, 2e-3, 4e-3, 8e-3];

% BASN(NumRx, SweepTimeIndex) — the base sampling number. Halves as the
% receiver count doubles, since the samples are shared across channels.
switch NumRx
    case 1, BASN_row = [1024, 2048, 4096, 8192, 16384];
    case 2, BASN_row = [ 512, 1024, 2048, 4096,  8192];
    case 4, BASN_row = [ 256,  512, 1024, 2048,  4096];
end
BASN = BASN_row(STI);
SN_Selections = [BASN, BASN/2, BASN/4, BASN/8];

if ~ismember(p.SamplingIndex, 1:4)
    error('makeRadarHandles:badSampIdx', 'SamplingIndex must be 1..4.');
end

%% ---- Modulation (GUI 219, 224) -----------------------------------------
switch p.Modulation
    case 'Sawtooth', ModValue = 0;
    case 'CW',       ModValue = 3;
    otherwise
        error('makeRadarHandles:badMod', ...
            'Modulation must be ''Sawtooth'' or ''CW''.');
end

if p.FrequencyHigh <= p.FrequencyLow
    error('makeRadarHandles:badSweep', ...
        'FrequencyHigh (%g) must be above FrequencyLow (%g).', ...
        p.FrequencyHigh, p.FrequencyLow);
end

%% ---- Assemble ----------------------------------------------------------
handles = struct( ...
    ... % --- basic parameters (Send_Basic_Parameter) ---
    'ActiveModulationValue',      ModValue, ...
    'ActiveModulationString',     p.Modulation, ...
    'ActiveSweepTimeValue',       STI, ...
    'ActiveSamplingNumberValue',  p.SamplingIndex, ...
    'ActiveTxValue',              TxValue, ...
    'ActiveRxValue',              RxValue, ...
    ... % --- derived channel layout (GetComplexData) ---
    'ActiveSweepTime',            SweepTimes(STI), ...
    'BASN',                       BASN, ...
    'SN_Selections',              SN_Selections, ...
    'ActiveNum_Tx',               NumTx, ...
    'ActiveNum_Rx',               NumRx, ...
    'ActiveTransmitterString',    p.Transmitter, ...
    'ActiveReceiverString',       p.Receiver, ...
    'NumSweeps',                  p.NumSweeps, ...
    ... % --- FMCW chirp (Send_PLL_Sawtooth) ---
    'ActiveFrequencyLow',         p.FrequencyLow, ...
    'ActiveFrequencyHigh',        p.FrequencyHigh, ...
    ... % --- filled in by PUPradar_initiating ---
    'EndPoint2_Num',              [], ...
    'EndPoint6_Num',              [], ...
    'modelcode',                  '');
end
