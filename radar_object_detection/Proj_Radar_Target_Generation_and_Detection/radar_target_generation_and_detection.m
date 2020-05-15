clear all
clc;

%% If using Octave %%
% pkg load signal

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
freq_op     = 77e9;
max_range   = 200.0;
range_resol = 1.0;
max_vel     = 100.0;

% speed of light = 3e8 [m/s]
c = 3e8;

%% User Defined Range and Velocity of target
% *%TODO* (done):
% define the target's initial position and velocity. 
% Note : Velocity remains contant
init_position_target = 110; % target initial position x  = 110 [m]
init_velocity_target = -20; % target initial velocity vx = -20 [m/s]

%% FMCW Waveform Generation

% *%TODO* (done):
% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (T_chirp) and Slope (slope) of the FMCW
% chirp using the requirements above.
B       = c / (2 * range_resol);
T_chirp = 5.5 * 2 * max_range / c;
Slope   = B / T_chirp;

% Operating carrier frequency of Radar 
fc = 77e9;  % carrier freq
                                            
% The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
% for Doppler Estimation. 
Nd = 128;   % #of doppler cells OR #of sent periods % number of chirps

% The number of samples on each chirp. 
Nr = 1024;  % for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each chirp
t  = linspace(0, Nd*T_chirp, Nr*Nd); % total time for samples

% Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx  = zeros(1,length(t)); % transmitted signal
Rx  = zeros(1,length(t)); % received signal
Mix = zeros(1,length(t)); % beat signal

% Similar vectors for range_covered and time delay.
r_t = zeros(1,length(t)); % - Question: what is "range_covered"? 
                          % - Ans: maybe it's meaning the range detected of object versus time
td  = zeros(1,length(t)); % - Question: why time delay are always 0? 
                          % - Ans: maybe waiting for updating later

%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         

    % Define some intermediate parameters
    fc    = freq_op;
    alpha = Slope;

    % *%TODO* (done):
    % For each time stamp update the Range of the Target for constant velocity.
    if i == 1
        r_t(i) = init_position_target;
    else
        r_t(i) = r_t(i-1) + init_velocity_target * (t(i) - t(i-1)); % const vel
    end 
    td(i)  = r_t(i) * 2 / c; % td(i) is the radar signal trip time

    % *%TODO* (done):
    % For each time sample we need update the transmitted and received signal. 
    Tx(i)  = cos(2 * pi * (fc * t(i) + alpha * t(i)^2 / 2));
    Rx(i)  = cos(2 * pi * (fc * (t(i) - td(i)) + alpha * (t(i) - td(i))^2 / 2));

    % *%TODO* (done):
    % Now by mixing the Transmit and Receive generate the beat signal
    % This is done by element wise matrix multiplication of Transmit and Receiver Signal
    Mix(i) = Tx(i) .* Rx(i);

end


%% RANGE MEASUREMENT
% *%TODO* (done):
% Reshape the vector into [Nr, Nd] array. 
% Nr and Nd here would also define the size of Range and Doppler FFT respectively.
Mix_reshaped = reshape(Mix, [Nr, Nd]);

% *%TODO* (done):
% Run the FFT on the beat signal along the range bins dimension (Nr) and normalize.
sig_fft = fft(Mix_reshaped, Nr);
sig_fft = sig_fft./Nr;

% *%TODO* (done):
% Take the absolute value of FFT output
sig_fft = abs(sig_fft);

% *%TODO* (done):
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum. 
% Hence we throw out half of the samples.
L = length(t);
sig_fft = sig_fft(1 : L/2 + 1);

%plotting the range
figure ('Name','Range from First FFT')
% subplot(2,1,1)

% *%TODO* (done):
% plot FFT output 
plot(sig_fft)
axis ([0 200 0 1]);


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix = reshape(Mix, [Nr, Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix, Nr, Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2, 1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10 * log10(RDM) ;

% use the surf function to plot the output of 2DFFT and to show axis in both
% dimensions
doppler_axis = linspace(-100, 100, Nd);
range_axis = linspace(-200,200,Nr/2) * ((Nr/2)/400);
figure ('Name','Range and Velocity from 2D FFT')
surf(doppler_axis, range_axis, RDM);
%% CFAR implementation

% Slide Window through the complete Range Doppler Map

% *%TODO* (done):
% Select the number of Training Cells in both the dimensions.
Tr = 12;
Td = 14;

% *%TODO* (done):
% Select the number of Guard Cells in both dimensions around the Cell under 
% test (CUT) for accurate estimation
Gr = 6;
Gd = 8;

% *%TODO* (done):
% Offset the threshold by SNR value in dB
SNR = 5; % Is it correct to do so???

% *%TODO* (done):
% Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% *%TODO* (done):
% Design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.
% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.
% Note: db2pow() and pow2db() are both Matlab built-in functions
Nr = Nr / 2;
sig_cfar2d = zeros(Nr, Nd);
for i = 1 : (Nr - (2 * (Gr + Tr) + 1))

    for j = 1 : (Nd - (2 * (Gd + Td) + 1))
        % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
        % CFAR
        noise_db_outer  = RDM(i:i+2*(Gr+Tr)+1, j:j+2*(Gd+Td)+1);
        noise_pow_outer = db2pow(noise_db_outer);
        noise_db_inner  = RDM(i+Tr:i+Gr+2*Tr+1, j+Td:j+Gd+2*Td+1);
        noise_pow_inner = db2pow(noise_db_inner);

        % Calculate the sum
        % Note: in matlab, one can use sum(x, 'all'), while in Octave one cannot
        noise_level_pow = sum(sum(noise_pow_outer)) - sum(sum(noise_pow_inner));

        % Number of training cells
        n_cells_t = (2*(Gr + Tr) + 1) * (2*(Gd + Td) + 1) - (Gr + Tr + 1) * (Gd + Td + 1);

        % Average noise level
        avg_noise_pow = noise_level_pow / n_cells_t;
        ave_noise_db  = pow2db(avg_noise_pow);

        % Offset the threshold
        threshold = ave_noise_db + SNR;

        % *%TODO* (done):
        % The process above will generate a thresholded block, which is smaller 
        % than the Range Doppler Map as the CUT cannot be located at the edges of
        % matrix. Hence,few cells will not be thresholded. To keep the map size same
        % set those values to 0. 

        % Assign the valid signal to CUT
        if RDM(i + Tr + Gr, j + Td + Gd) > threshold
            sig_cfar2d(i + Tr + Gr, j + Td + Gd) = 1;
        end
    end
end

% *%TODO* (done):
% Display the CFAR output using the Surf function like we did for Range
% Doppler Response output.
figure ('Name','Filtered Range and Velocity from 2D CFAR')
surf(doppler_axis, range_axis, sig_cfar2d);
colorbar;
