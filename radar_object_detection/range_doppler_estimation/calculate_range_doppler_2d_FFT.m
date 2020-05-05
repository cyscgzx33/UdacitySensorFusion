% 2-D Transform
% The 2-D Fourier transform is useful for processing 2-D signals and other 2-D data such as images.
% Create and plot 2-D data with repeated blocks.

P = peaks(20);
X = repmat(P,[5 10]);
imagesc(X)

% TODO (done): Compute the 2-D Fourier transform of the data.  
% Shift the zero-frequency component to the center of the output, and 
% plot the resulting 100-by-200 matrix, which is the same size as X.
signal_fft = fft2(X, 100, 200);
signal_fft = fftshift(signal_fft);
signal_fft = abs(signal_fft);

% Here since it is a 2D output, it can be plotted as an image. 
% Hence, we use the imagesc function
imagesc(signal_fft);