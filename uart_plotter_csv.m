function uart_plotter_csv_()
% Real-time single-channel UART CSV plotter
% - Assumes each line ends with CR/LF ("\r\n")
% - Uses only the first numeric value on each line (comma-separated)
% Close the figure to stop.

%% ---- CONFIG ----
PORT            = "COM5";
BAUD            = 115200;
TIMEOUT_S       = 0.1;
PLOT_WINDOW_SEC = 10;      % seconds shown on the x-axis
DECIMATE_PLOT   = 1;       % plot every Nth sample
%% -----------------

% Open port with fixed terminator
sp = serialport(PORT, BAUD, "Timeout", TIMEOUT_S);
configureTerminator(sp, "CR/LF");
flush(sp);

% Simple UI
f = figure('Name', sprintf('UART Plot [%s, CR/LF]', PORT), 'Color', 'w');
ax = axes(f); 
grid(ax, 'on'); 
hold(ax, 'on');
xlabel(ax, 'time (s)'); ylabel(ax, 'ch1');
h = animatedline('LineWidth', 1.1);

% State
t0 = []; % Record of current stopwatch time
hostTimer = tic; % Stopwatch timer for running timebase
n  = 0; % Number of samples processed

% Clean-up on exit
cleanup = onCleanup(@() local_close(sp)); %#ok<NASGU>
% Terminal printout
fprintf('Listening on %s @ %d baud (CR/LF). Close figure to stop.\n', PORT, BAUD);

% While the figure is still open and serial port connected
while ishghandle(f) && isvalid(sp)
    if sp.NumBytesAvailable == 0 % If UART input buffer empty
        pause(0.002);
        continue;
    end

    % Read a line, trim whitespace
    try
        line = strtrim(readline(sp));
    catch
        continue;
    end
    if line == "", continue; end
    
    tok  = strtok(line, ',');  % Split the line at the first comma, return first part
    val  = str2double(tok); % Convert to double
    % Check if the result is valid
    if ~isfinite(val), continue; end

    t_now = toc(hostTimer); % Get curret elapsed time
    if isempty(t0), t0 = t_now; end
    t_rel = t_now - t0;

    % Plot 
    n = n + 1; % Increment total sample counter
    if mod(n, DECIMATE_PLOT) == 0 % Plot every Nth sample
        addpoints(h, t_rel, val); % Append points to animated line
        xmin = max(0, t_rel - PLOT_WINDOW_SEC); % Get left edge of window
        xlim(ax, [xmin, xmin + PLOT_WINDOW_SEC]); % Get the sliding window
        drawnow limitrate; % refresh the plt
    end

end
end
% Safely close serial port 
function local_close(sp)
    try, if ~isempty(sp) && isvalid(sp), flush(sp); clear sp; end, catch, end
end
