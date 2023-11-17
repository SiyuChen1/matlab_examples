function exampleHelperUpdateScans(state, maxRange, observedLandmarks, ...
                                  sensorHandle, observationHandle)
	% Plots the lidar rays
    ranges = observedLandmarks(:, 1);
    angles = observedLandmarks(:, 2);
    x = repmat(state(1),3*length(ranges),1);
    y = repmat(state(2),3*length(ranges),1);
    x(3:3:end) = nan;
    y(3:3:end) = nan;
    x(2:3:end) = state(1) + ranges.*cos(angles + state(3));
    y(2:3:end) = state(2) + ranges.*sin(angles + state(3));
    clearpoints(sensorHandle);
    addpoints(sensorHandle, x, y);
    
    % Plot the semicircular region in front of robot from which the
    % observed landmarks are extracted
    th = linspace((state(3) + pi/2), (state(3) - pi/2), 100);
    x = [state(1), maxRange*cos(th) + state(1), state(1)];
    y = [state(2), maxRange*sin(th) + state(2), state(2)];
    addpoints(sensorHandle, x, y);

    % Plot the observed landmarks
    lmCart = nav.algs.rangeBearingInverseMeasurement(state(1:3), observedLandmarks);
    clearpoints(observationHandle);
	for i = 1:size(lmCart, 1)
        addpoints(observationHandle, lmCart(i,1), lmCart(i,2));
    	addpoints(observationHandle, NaN, NaN);
	end
end