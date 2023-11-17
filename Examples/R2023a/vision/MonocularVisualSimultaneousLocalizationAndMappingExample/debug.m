clc
clear
% Generate 3-D world points
worldPoints = rand(100, 3);

% Create a worldpointset object
wpSet = worldpointset;

% Add world points
wpSet = addWorldPoints(wpSet, worldPoints);

% Add 3-D to 2-D correspondences for view 1
viewId1 = 1;
pointIndices1   = 1:10;
featureIndices1 = 1:10;
wpSet  = addCorrespondences(wpSet, viewId1, pointIndices1, ...
  featureIndices1);

% Add 3-D to 2-D correspondences for view 2
viewId2 = 2;
pointIndices2   = 6:10;
featureIndices2 = 1:5;
wpSet  = addCorrespondences(wpSet, viewId2, pointIndices2, ...
  featureIndices2);

% Find world points in view 2
[pointIndices, featureIndices] = findWorldPointsInView(...
  wpSet, viewId2)