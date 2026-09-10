function cmds = Send_Basic_Parameter(handles, varargin)
%SEND_BASIC_PARAMETER  Standalone version of PUPradarGUI's Send_Basic_Parameter.
%
%   Sends the five MD/ST/SN/Tx/Rx parameters to the MCU. Without this, the
%   board keeps streaming in whatever mode it was last left in, and
%   GetComplexData's reshape will fail against the assumed channel layout.
%   Call it BEFORE Send_PLL_Sawtooth, as the GUI does.
%
%   cmds = Send_Basic_Parameter(handles)
%   cmds = Send_Basic_Parameter(handles, 'DryRun', true)   % compute, send nothing
%   cmds = Send_Basic_Parameter(handles, 'Verbose', true)  % print each write
%
%   Required handles fields (all set for you by makeRadarHandles):
%     ActiveModulationValue     : 0 = Sawtooth, 3 = CW
%     ActiveSweepTimeValue      : 1..5 = 0.5 / 1 / 2 / 4 / 8 ms
%     ActiveSamplingNumberValue : 1..4, index into SN_Selections
%     ActiveTxValue             : 1 = Tx1, 2 = Tx2, 3 = both
%     ActiveRxValue             : 1/2/4/8 single, 3 = Rx1.Rx2, 12 = Rx3.Rx4, 15 = all
%     EndPoint2_Num             : USB endpoint handle (unused when DryRun)
%
%   Returns cmds, an n-by-2 [command_word, value] table of what was sent.
%
%   Opcodes (high byte) per PUPradarGUI.m:2289-2331:
%     E1 modulation, E2 sweep-time index, E3 sampling-number index,
%     E4 Tx value, E5 Rx value.
%
%   See also MAKERADARHANDLES, SEND_PLL_SAWTOOTH, GETCOMPLEXDATA.

opts = struct('DryRun', false, 'Verbose', false);
if mod(numel(varargin), 2) ~= 0
    error('Send_Basic_Parameter:badArgs', 'Options must be Name/value pairs.');
end
for k = 1:2:numel(varargin)
    if ~isfield(opts, varargin{k})
        error('Send_Basic_Parameter:badOpt', 'Unknown option ''%s''.', varargin{k});
    end
    opts.(varargin{k}) = varargin{k+1};
end

items = { ...
    'E100', handles.ActiveModulationValue,     'modulation'; ...
    'E200', handles.ActiveSweepTimeValue,      'sweep time idx'; ...
    'E300', handles.ActiveSamplingNumberValue, 'sampling num idx'; ...
    'E400', handles.ActiveTxValue,             'Tx value'; ...
    'E500', handles.ActiveRxValue,             'Rx value'};

cmds = zeros(size(items,1), 2);
for k = 1:size(items, 1)
    Word = hex2dec(items{k,1}) + items{k,2};
    cmds(k, :) = [Word, items{k,2}];
    if opts.Verbose
        fprintf('  %-18s %s -> 0x%s\n', items{k,3}, items{k,1}, dec2hex(Word, 4));
    end
    if ~opts.DryRun
        ForwardData = zeros(1024, 1) + Word;
        SendOutData = uint16(ForwardData);
        miniradarputdata(SendOutData, handles.EndPoint2_Num);
    end
end
end
