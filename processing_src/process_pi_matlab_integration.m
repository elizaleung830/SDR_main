clear;
filenames = { ...
    "C:\Users\eliza\PycharmProjects\y4-Honours\control_system_tests\ground_integration_tests_02_10\SDR-data\1_time_55_37989719999996.bin", ...
    "C:\Users\eliza\PycharmProjects\y4-Honours\control_system_tests\ground_integration_tests_02_10\SDR-data\2_time_80_41679616700003.bin", ...
    "C:\Users\eliza\PycharmProjects\y4-Honours\control_system_tests\ground_integration_tests_02_10\SDR-data\3_time_101_44866456499994.bin"};
labels = {'Run 1 (t = 55 s)', 'Run 2 (t = 80 s)', 'Run 3 (t = 101 s)'};

sampSize      = 1024;
T_sweep       = 0.5e-3;
BW            = 2e9;
display_sweep = 100;

nFiles = numel(filenames);
I_all = cell(1, nFiles);  Q_all = cell(1, nFiles);
range_all = cell(1, nFiles);  spec_all = cell(1, nFiles);

for k = 1:nFiles
    [~, I_all{k}, Q_all{k}, ~, range_all{k}, spec_all{k}] = ...
        process_pi(filenames{k}, sampSize, T_sweep, BW, "butterworth", true , "iq_status","filtered", trim=true);
end

%% ---------------- Plot with interactive run selection ----------------
colors = lines(nFiles);
styles = {'-', '--', ':'};

fig = figure('Position', [100 100 900 750]);

% Leave room at the bottom for the controls
axPos = @(row) [0.10, 0.78 - (row-1)*0.27, 0.86, 0.19];

hQ = gobjects(1, nFiles);
hI = gobjects(1, nFiles);
hR = gobjects(1, nFiles);

% --- Q data ---
ax1 = axes('Position', axPos(1)); hold(ax1, 'on');
for k = 1:nFiles
    hQ(k) = plot(ax1, Q_all{k}(:, display_sweep), styles{k}, ...
        'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', labels{k});
end
hold(ax1, 'off'); grid(ax1, 'on');
xlabel(ax1, 'Sample Index'); ylabel(ax1, 'Amplitude');
title(ax1, sprintf('Q Data (sweep %d)', display_sweep));
legend(ax1, 'Location', 'best');

% --- I data ---
ax2 = axes('Position', axPos(2)); hold(ax2, 'on');
for k = 1:nFiles
    hI(k) = plot(ax2, I_all{k}(:, display_sweep), styles{k}, ...
        'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', labels{k});
end
hold(ax2, 'off'); grid(ax2, 'on');
xlabel(ax2, 'Sample Index'); ylabel(ax2, 'Amplitude');
title(ax2, sprintf('I Data (sweep %d)', display_sweep));
legend(ax2, 'Location', 'best');
linkaxes([ax1 ax2], 'x');

% --- Averaged range profile ---
ax3 = axes('Position', axPos(3)); hold(ax3, 'on');
for k = 1:nFiles
    hR(k) = plot(ax3, range_all{k}, spec_all{k}', styles{k}, ...
        'Color', colors(k,:), 'LineWidth', 1.5, ...
        'DisplayName', labels{k});
end
hold(ax3, 'off'); grid(ax3, 'on');
set(ax3, 'YScale', 'log');
xlabel(ax3, 'Range (m)'); ylabel(ax3, 'Averaged Magnitude (log)');
title(ax3, 'Range Profile Averaged Across Sweeps');
legend(ax3, 'Location', 'best');

%% ---------------- Checkbox controls ----------------
uicontrol(fig, 'Style', 'text', 'String', 'Show runs:', ...
    'Units', 'normalized', 'Position', [0.10 0.015 0.09 0.03], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', ...
    'BackgroundColor', get(fig, 'Color'));

for k = 1:nFiles
    uicontrol(fig, 'Style', 'checkbox', 'String', labels{k}, ...
        'Units', 'normalized', ...
        'Position', [0.20 + (k-1)*0.19, 0.015, 0.18, 0.03], ...
        'Value', 1, ...
        'ForegroundColor', colors(k,:), ...
        'BackgroundColor', get(fig, 'Color'), ...
        'Callback', @(src,~) toggleRun(src, k, hQ, hI, hR, ...
        [ax1 ax2 ax3]));
end

function toggleRun(src, k, hQ, hI, hR, axArr)
% Show or hide run k across all three subplots, and refresh the legends
% so hidden runs drop out rather than appearing as dead entries.
vis = matlab.lang.OnOffSwitchState(logical(src.Value));
hQ(k).Visible = vis;
hI(k).Visible = vis;
hR(k).Visible = vis;

for ax = axArr
    lines = findobj(ax, 'Type', 'line', 'Visible', 'on');
    if isempty(lines)
        legend(ax, 'off');
    else
        legend(ax, flipud(lines), 'Location', 'best');
    end
end
end