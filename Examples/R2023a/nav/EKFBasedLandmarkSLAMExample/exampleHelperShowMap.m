function exampleHelperShowMap(state, stateCovariance, ...
                              gpsLatLong, corrPoses, predPoses)
    reference = [-33.887742, 151.192081, 0];

    landmarks = reshape(state(4:end), 2, [])';
    numLM = size(landmarks,1);

    [xCov, yCov] = exampleHelperCovEllipsePoints(state(1:2), ...
                                                 stateCovariance(1:2,1:2));
    xCov = [xCov, NaN];
    yCov = [yCov, NaN];
    for i = 1:numLM
        [xL, yL] = exampleHelperCovEllipsePoints(state(2*i+2:2*i+3), ...
            stateCovariance(2*i+2:2*i+3, 2*i+2:2*i+3));

        xCov = [xCov, xL, NaN];
        yCov = [yCov, yL, NaN];
    end

    % replace NaNs in corr with pred values
    corrPoses(all(isnan(corrPoses),2),:) = predPoses(all(isnan(corrPoses),2),:);
    corrPoses(end+1:size(predPoses,1),:) = predPoses(size(corrPoses,1)+1:end,:);

    % Append zeros for third dimension
    gpsLatLong3 = [gpsLatLong - gpsLatLong(1,:), zeros(size(gpsLatLong,1),1)];
    estimatedTraj3 = [corrPoses(:,1:2)-corrPoses(1,1:2), zeros(size(corrPoses,1),1)];
    landmarks3 = [landmarks, zeros(size(landmarks,1),1)]- [corrPoses(1,1:2),0];
    cov3 = [xCov', yCov', zeros(size(xCov,2),1)] - [corrPoses(1,1:2),0];
    
    % Convert to Lat Long for geoplot
    gpsLLA = ned2lla(gpsLatLong3, reference, 'flat');
    estTrajLLA = enu2lla(estimatedTraj3, reference, 'flat');
    landmarksLLA = enu2lla(landmarks3, reference, 'flat');
    covLLA = enu2lla(cov3, reference, 'flat');

    fig2 = figure; 
    gx = geoaxes(fig2);
    co = colororder;
    %color order: GPS, estimated traj, LMs, Covariance
    gx.ColorOrder = [co(1,:); co(7,:); co(3,:); co(3,:)];
    hold on;
    geobasemap topographic
    
    geoplot(gx, gpsLLA(:,1), gpsLLA(:,2), '.');
    geoplot(gx, estTrajLLA(:,1), estTrajLLA(:,2), '.');
    geoplot(gx, landmarksLLA(:,1), landmarksLLA(:,2), '.');
    geoplot(gx, covLLA(:,1), covLLA(:,2), 'Marker', '.', 'MarkerSize', 4);
    legend('Ground truth', 'Estimated trajectory', 'Known landmarks');
end