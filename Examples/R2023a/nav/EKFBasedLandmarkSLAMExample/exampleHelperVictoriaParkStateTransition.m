function [updatedPose, Fx, Fv] = exampleHelperVictoriaParkStateTransition( ...
                                             curPose, speedSteer, timeStep)
    % extract the velocity from the control input
    speed = speedSteer(1);
    % create output variable for pose
    updatedPose = curPose;
    % compute trigonometric identities
    cosTheta = cos(curPose(3));
    sinTheta = sin(curPose(3));
    tanSteer = tan(speedSteer(2));

    % vehicle parameters
    a = 3.78;
    b = 0.5;
    H = 0.76;
    wheelBase = 2.83;

    % transform the velocity
    speed = speed / (1 - tanSteer * H / wheelBase);

    % get the updated pose
    updatedPose(1) = curPose(1) + timeStep * (speed * cosTheta - ...
        (speed * tanSteer / wheelBase) * (a * sinTheta + b * cosTheta));
    updatedPose(2) = curPose(2) + timeStep * (speed * sinTheta + ...
        (speed * tanSteer / wheelBase) * (a * cosTheta - b * sinTheta));
    updatedPose(3) = curPose(3) + timeStep * (speed * tanSteer / wheelBase);

    % compute the Jacobian w.r.t current pose
    Fx = [1 0 (-timeStep * (speed * sinTheta + (speed * tanSteer / wheelBase) * (a * cosTheta - b * sinTheta)));
          0 1 ( timeStep * (speed * cosTheta - (speed * tanSteer / wheelBase) * (a * sinTheta + b * cosTheta)));
          0 0                1               ];
    % compute the Jacobian w.r.t control input
    Fv = [[timeStep*(cosTheta - (tanSteer*(b*cosTheta + a*sinTheta))/wheelBase), -(timeStep*speed*(b*cosTheta + a*sinTheta)*(tanSteer^2 + 1))/wheelBase];
          [timeStep*(sinTheta + (tanSteer*(a*cosTheta - b*sinTheta))/wheelBase),  (timeStep*speed*(a*cosTheta - b*sinTheta)*(tanSteer^2 + 1))/wheelBase];
          [                                       (timeStep*tanSteer)/wheelBase,                           (timeStep*speed*(tanSteer^2 + 1))/wheelBase]];
end