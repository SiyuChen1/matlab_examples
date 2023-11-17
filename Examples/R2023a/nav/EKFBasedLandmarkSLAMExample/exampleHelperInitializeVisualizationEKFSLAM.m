function [robotHandle, covarianceHandle, sensorHandle, observationHandle, ...
          landmarkHandle, deadRecHandle, estTrajHandle] = ...
                        exampleHelperInitializeVisualizationEKFSLAM(initialPose, gpsLatLong)

    fig = figure;
    figAx = axes(fig);

    set(fig, 'name', 'EKFSLAM')
    xlim([-200 160])
    ylim([-100 185])
    hold all

    % Get the color order
    co = colororder;

    % Ground Truth
    gtHandle = plot(figAx, gpsLatLong(:,2),gpsLatLong(:,1), ...
                'Color', co(1,:), 'Marker', '.', 'LineStyle','none');

    % Update initial robot position
    scale = 0.5;
    robotPos = initialPose(:);
    t = robotPos(1:2);
    a = robotPos(3);
    R = [cos(a) -sin(a); sin(a) cos(a)];
    G = [R t(:); 0 0 scale];

    RobotBodyTriangleVertices = G*[[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
    RobotBodyFaceColor = [0.866 0.918 0.753];

    % To plot robot at each updated step
    robotHandle = patch(figAx, ...
                        RobotBodyTriangleVertices(1,:), ...
                        RobotBodyTriangleVertices(2,:), ...
                        RobotBodyFaceColor);

    % To plot known landmarks at any time
    landmarkHandle = animatedline(figAx, 'Color', co(3,:), 'Marker', '.');

    % To plot covariance ellipses
    covarianceHandle = animatedline(figAx, 'Color', co(3,:));

    % To plot observed landmarks at any time
	observationHandle = animatedline(figAx, 'Color', 'black', 'Marker', '*');

    % To plot rays to observed landmarks
    sensorHandle = animatedline(figAx, 'Color', 'black');

    % Dead reckoning
    deadRecHandle = animatedline(figAx, 'Color', co(5,:), 'Marker', '.');

    % Estimated Trajectory
    estTrajHandle = animatedline(figAx, 'Color', co(7,:), 'Marker', '.');

    % Legends
    legend(figAx, [gtHandle, robotHandle, estTrajHandle, ...
               landmarkHandle, deadRecHandle, ...
               observationHandle, sensorHandle], ...
              {'Ground truth', 'Robot', 'Estimated trajectory', ...
               'Known landmarks', 'Dead reckoning', ...
               'Observed landmarks', 'Rays to observed LMs'}, ...
               'Position',[0.59394,0.14343,0.30536,0.28333]);
end