function omegaSim = helperAllanVarianceExample(L, Fs, gyro, iters)
%HELPERALLANVARIANCEEXAMPLE Helper function used by AllanVarianceExample
%   OMEGASIM = HELPERALLANVARIANCEEXAMPLE(L, FS, GYRO) returns an L-by-3
%   matrix OMEGASIM containing 3 different simulations of a stationary
%   gyroscope with parameters GYRO and sampling rate FS.
%     
%   OMEGASIM = HELPERALLANVARIANCEEXAMPLE(L, FS, GYRO, ITERS) returns an 
%   L-by-ITERS matrix OMEGASIM.
%
%   
%   This function is for use with AllanVarianceExample only. It may be
%   removed in the future.

%   Copyright 2018-2019 The MathWorks, Inc.

if nargin < 4
    iters = 3;
end

% Create stationary input.
acc = zeros(L, 3);
angvel = zeros(L, 3);

omegaSim = zeros(L,iters);
for i = 1:iters
    imu = imuSensor('SampleRate', Fs, 'Gyroscope', gyro, ...
    'RandomStream', 'mt19937ar with seed', 'Seed', i);
    fprintf('Running simulation %d of %d ...\n', i, iters);
    [~, gyroDataSim] = imu(acc, angvel);
    omegaSim(:,i) = gyroDataSim(:,1);
end
