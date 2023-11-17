function likelihood = vdpMeasurementLikelihoodFcn(predictedParticles,measurement)
% vdpMeasurementLikelihoodFcn Example measurement likelihood function for 
%                             particle filter
%
% The measurement is the first state.
%
% likelihood = vdpMeasurementLikelihoodFcn(predictedParticles, measurement)
%
% Inputs:
%    predictedParticles - NumberOfStates-by-NumberOfParticles matrix that
%                         holds the predicted particles
%
% Outputs:
%    likelihood - A vector with NumberOfParticles elements whose n-th
%                 element is the likelihood of the n-th particle
%
% See also extendedKalmanFilter, unscentedKalmanFilter

%   Copyright 2017 The MathWorks, Inc.

%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

numberOfMeasurements = 1; % Expected number of measurements

% Validate the measurement
validateattributes(measurement, {'double'}, {'vector', 'numel', numberOfMeasurements}, ...
    'vdpMeasurementLikelihoodFcn', 'measurement');

% Assume that measurements are subject to Gaussian distributed noise with
% variance 0.016
% Specify noise as covariance matrix
measurementNoise = 0.016 * eye(numberOfMeasurements);
  
% The measurement contains the first state variable. Get the first state of
% all particles
predictedMeasurement = predictedParticles(1,:);

% Calculate error between predicted and actual measurement
measurementError = bsxfun(@minus, predictedMeasurement, measurement(:)');

% Use measurement noise and take inner product
measurementErrorProd = dot(measurementError, measurementNoise \ measurementError, 1);

% Convert error norms into likelihood measure. 
% Evaluate the PDF of the multivariate normal distribution. A measurement
% error of 0 results in the highest possible likelihood.
likelihood = 1/sqrt((2*pi).^numberOfMeasurements * det(measurementNoise)) * exp(-0.5 * measurementErrorProd);
end