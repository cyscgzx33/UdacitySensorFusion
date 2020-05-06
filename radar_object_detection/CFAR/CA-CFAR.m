% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
% Note: in the solution, the random generator is set to obtain abs(.), however it is not proper to use abs(.)
%       thus, in the scale part, I also added offset to balance the negative influence
s = randn(Ns,1);

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
plot(s);

% TODO done: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
T = 14;
G = 8;

% Scale: introducing room above noise threshold for desired SNR 
scale = 5;
offset = 5;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1 : (Ns - (2 * (G + T) + 1))

    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    curr_noise = 0;
    % 2. Start sliding the window
    for j = i : i + 2 *(G + T) + 1
        % Only process information in T cells
        if j < 2 * G + T + 1 + i && j > T + i
            continue;
        end
        % 3. Sum the signal (noise) within all the leading or lagging training cells
        curr_noise = curr_noise + s(j);
    end

    % 4. Averaging the sum to determine the noise threshold
    avg_noise = curr_noise / (2 * T); 
    threshold = avg_noise * scale + offset;

    % 5. Using an appropriate offset value scale the threshold
    threshold_cfar = [threshold_cfar, {threshold}];

    % 6. Measuring the signal within the CUT (Cell Under Test)
    pos_CUT = i + T + G;
    CUT = s(pos_CUT);

    % 8. Filter the signal above the threshold
    signal = CUT;
    if CUT < threshold
        signal = 0;
    end
    % Notification at detection
    if signal > 0
        disp(signal);
        disp(threshold);
    end

    signal_cfar = [signal_cfar, {signal}];
end



% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')