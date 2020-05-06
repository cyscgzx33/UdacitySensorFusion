% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
s=randn(Ns,1);

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
plot(s);

% TODO done: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
T = 3;
G = 1;

% Offset : Adding room above noise threshold for desired SNR 
offset=3;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T))     

    curr_noise = 0;
    % 2. - 5. Determine the noise threshold by measuring it within the training cells
    % 2. Start sliding the window
    for j = i : i + 2 *(G + T) + 1
        % Only process information in T cells
        if j < 2 * G + T + 1 + i and j > T + i
            continue;
        end

        % 3. Sum the signal (noise) within all the leading or lagging training cells
        curr_noise = curr_noise + s[j];
    end

    % 4. Averaging the sum to determine the noise threshold
    avg_noise = curr_noise / (2 * T); 

    % 5. Using an appropriate offset value scale the threshold
    threshold_cfar = avg_noise + offset;

    % 6. Measuring the signal within the CUT (Cell Under Test)
    pos_CUT = i + T + G + 1;
    CUT = s[pos_CUT];

    % 8. Filter the signal above the threshold

    signal_cfar = [signal_cfar, {signal}];
end



% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')