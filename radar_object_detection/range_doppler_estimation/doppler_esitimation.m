% Doppler Velocity Calculation
c = 3*10^8;         %speed of light
frequency = 77e9;   %frequency in Hz

% TODO (done): Calculate the wavelength
lambda = c / frequency;

% TODO (done): Define the doppler shifts in Hz using the information from above 
freq_shifts = [3e3, 4.5e3, 11e3, -3e3];

% TODO (done): Calculate the velocity of the targets  fd = 2*vr/lambda
vr = freq_shifts * lambda / 2;

% TODO (done): Display results
disp(vr);