% Mexican hat wavelet is chosen as the mother wavelet as it is a good
% representation of the footstep-induced vibration signals due to its similarity in shape.

wname = 'mexh'; % See https://www.mathworks.com/help/wavelet/ref/waveinfo.html
itr = 10;

[psi,xval] = wavefun(wname,itr);
plot(xval,psi)
grid on
title(['Approximation of ',wname,' Wavelet'])

% For more information, open openExample('wavelet/WaveletApproximationsExample')