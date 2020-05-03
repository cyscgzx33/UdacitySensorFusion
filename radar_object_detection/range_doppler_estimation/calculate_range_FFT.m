Fs = 1000;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 1500;             % Length of signal
t = (0:L-1)*T;        % Time vector

% TODO (done): Form a signal containing a 77 Hz sinusoid of amplitude 0.7 and a 43Hz sinusoid of amplitude 2.
freq_1 = 77;
freq_2 = 43;
amplitude_1 = 0.7;
amplitude_2 = 2.0;
S = amplitude_1 * cos(2 * pi * freq_1 * t) + amplitude_2 * cos(2 * pi * freq_2 * t);

% Corrupt the signal with noise 
X = S + 2*randn(size(t));

% Plot the noisy signal in the time domain. It is difficult to identify the frequency components by looking at the signal X(t). 
plot(1000*t(1:50) ,X(1:50))
title('Signal Corrupted with Zero-Mean Random Noise')
xlabel('t (milliseconds)')
ylabel('X(t)')

% TODO (done): Compute the Fourier transform of the signal. 
S_FFT = fft(X); % Note: don't have to provide sample number N here
S_FFT = abs(S_FFT); % The output is a complex number a + ib, since we only care about the magnitue we take abs(signal)

% TODO (done): Compute the two-sided spectrum P2. Then compute the single-sided spectrum P1 based on P2 and the even-valued signal length L.
S_FFT = S_FFT(1:L/2+1); % Be careful about the number index
P1 = S_FFT;

% Plotting
f = Fs*(0:(L/2))/L;
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')