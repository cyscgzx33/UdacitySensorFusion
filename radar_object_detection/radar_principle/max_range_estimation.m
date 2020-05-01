%Operating frequency (Hz)
fc = 77.0e9;

%Transmitted power (W)
P_S = 3e-3;

%Antenna Gain (linear)
G =  10000;

%Minimum Detectable Power
P_E_min = 1e-10;

%RCS of a car
RCS = 100;

%Speed of light
c = 3*10^8;

%TODO (done): Calculate the wavelength
lambda = c / fc;

%TODO (done): Measure the Maximum Range a Radar can see. 
R_max = P_S * G^2 * lambda^2 * RCS / (P_E_min * (4*pi)^3);

%display results
disp(lambda) % 0.0038961
disp(R_max)  % 2294841640.91854