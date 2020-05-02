% Define light speed [m/s]
c = 3e8 
% Define range resolution [m]
S_r = 1

% TODO (done): Find the Bsweep of chirp for 1 m resolution
% Refer to lesson 3.2, or:
% Refer to: https://www.radartutorial.eu/01.basics/Range%20Resolution.en.html
% Note in the website above, 1/tau is the sweep bandwidth, B_sweep
B_sweep = c / (2 * S_r);

% TODO (done): Calculate the chirp time based on the Radar's Max Range
R_max = 300;                   % max range is 300 [m]
T_chirp = 5.5 * 2 * R_max / c; % 5.5 is a factor here, such that the sweep time is at least 5 to 6 times the round trip time
                               % Chirp time can also be denoted as T_s, in addition to T_chirp
% TODO (done): define the frequency shifts
% Note: it's given by the question
f_b = [0, 1.1e6, 13e6, 24e6];

% Calculate range
calculated_range = c * T_chirp * f_b / (2 * B_sweep);

% Display the calculated range
disp(calculated_range);