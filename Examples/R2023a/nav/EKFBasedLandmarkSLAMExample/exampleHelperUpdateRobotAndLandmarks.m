function exampleHelperUpdateRobotAndLandmarks(state, covar, robotHandle, ...
                                       covarHandle, landmarkHandle)

    % Update the robot position
    scale = 0.5;
    t = state(1:2);
    a = state(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];
    
    % Update robot vertex locations
    RobotBodyTriangleVertices = G*[[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
    
    % Update patch
    robotHandle.XData = RobotBodyTriangleVertices(1,:);
    robotHandle.YData = RobotBodyTriangleVertices(2,:);
    
    numFeat = (length(state) - 3)/2;
    clearpoints(covarHandle);
    clearpoints(landmarkHandle);

    [x, y] = exampleHelperCovEllipsePoints(state(1:2), covar(1:2, 1:2));

    x = [x, NaN];
    y = [y, NaN];
    xF = [state(1), NaN];
    yF = [state(2), NaN];
    for i = 1:numFeat
        xF = [xF, state(2*i+2), NaN];
        yF = [yF, state(2*i+3), NaN];

        [xL, yL] = exampleHelperCovEllipsePoints(state(2*i+2:2*i+3), ...
            covar(2*i+2:2*i+3, 2*i+2:2*i+3));

        x = [x, xL, NaN];
        y = [y, yL, NaN];
    end
    addpoints(covarHandle, x, y);
    addpoints(landmarkHandle, xF, yF);
end