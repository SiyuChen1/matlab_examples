function [imagePoints, boardSize, imagesUsed] = helperDetectAprilTagCorners(imdsCalib, tagArrangement, tagFamily)
%helperDetectAprilTagCorners detects AprilTag calibration pattern in images.
%   [imagePoints, boardSize, imagesUsed] = helperDetectAprilTagCorners(imdsCalib,
%   tagArrangement, tagFamily) returns a M-by-2 matrix of [x,y] coordinates
%   of keypoints of the AprilTag based calibration pattern. boardSize
%   specifies the board dimensions as [rows, cols] measured in squares.
%   imagesUsed is a logical vector of the same size as the number of images
%   in imdsCalib. A value of true indicates that the pattern was detected
%   in the corresponding image.
%
%   Inputs
%   ------
%   imdsCalib         - Image datastore containing the captured images of
%                       the pattern
%   tagArrangement    - Arrangement of the tags in the pattern specified as
%                       a  2-element vector, [rows, cols].
%   tagFamily         - Tag family of the tags in the pattern
%
%   Outputs
%   -------
%   imagePoints       - [x,y] coordinates of keypoints of the calibration
%                       pattern
%   boardSize         - Corresponding checkerboard size of the pattern 
%   imagesUsed        - Logical vector indicating if the pattern was
%                       detected in the images

%   Copyright 2020 The MathWorks, Inc.

% Get the pattern size from tagArrangement.
boardSize = tagArrangement*2 + 1;

% Initialize number of images and tags.
numImages = length(imdsCalib.Files);
numTags = tagArrangement(1)*tagArrangement(2);

% Initialize number of corners in AprilTag pattern.
imagePoints = zeros(numTags*4,2,numImages);
imagesUsed = zeros(1, numImages);

% Get checkerboard corner indices from AprilTag corners.
checkerIdx = helperAprilTagToCheckerLocations(tagArrangement);

% Initialize invalid image indices
invalidIdx = [];

for idx = 1:numImages

    % Read and detect AprilTags in image.
    I = readimage(imdsCalib, idx);
    [tagIds, tagLocs] = readAprilTag(I, tagFamily);

    % Accept images if all tags are detected.
    if numel(tagIds) == numTags
        % Sort detected tags using ID values.
        [~, sortIdx] = sort(tagIds);
        tagLocs = tagLocs(:,:,sortIdx);
        
        % Reshape tag corner locations into a M-by-2 array.
        tagLocs = reshape(permute(tagLocs,[1,3,2]), [], 2);
        
        % Populate imagePoints using checkerboard corner indices.
        imagePoints(:,:,idx) = tagLocs(checkerIdx(:),:);
        imagesUsed(idx) = true; 
    else
        invalidIdx = [invalidIdx, idx]; %#ok<AGROW> 
    end
    
end

imagePoints(:,:,invalidIdx) = [];

end