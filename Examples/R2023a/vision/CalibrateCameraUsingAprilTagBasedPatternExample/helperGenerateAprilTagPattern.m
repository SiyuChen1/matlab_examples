function calibPattern = helperGenerateAprilTagPattern(imdsTags, tagArragement, tagFamily)
%helperGenerateAprilTagPattern generates an AprilTag based calibration pattern.
%   calibPattern = helperGenerateAprilTagPattern(imdsTags, tagArrangement,
%   tagFamily) returns a grayscale image, calibPattern, containing the
%   AprilTag based calibration pattern. 
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.
%
%   Inputs
%   ------
%   imdsTags          - Image datastore containing the images of the tags
%                       used to construct the calibration pattern
%   tagArrangement    - Arrangement of the tags in the pattern specified as
%                       a  2-element vector, [rows, cols].
%   tagFamily         - Tag family of the tags in the pattern
%
%   Output
%   ------
%   calibPattern      - Grayscale image containing the calibration pattern

%   Copyright 2020 The MathWorks, Inc.

numTags = tagArragement(1)*tagArragement(2);
tagIds = zeros(1,numTags);

% Read the first image.
I = readimage(imdsTags, 3);
Igray = im2gray(I);

% Scale up the thumbnail tag image.
Ires = imresize(Igray, 15, 'nearest');

% Detect the tag ID and location (in image coordinates).
[tagIds(1), tagLoc] = readAprilTag(Ires, tagFamily);

% Pad image with white boundaries (ensures the tags replace the black
% portions of the checkerboard).
tagSize = round(max(tagLoc(:,2)) - min(tagLoc(:,2)));
padSize = round(tagSize/2 - (size(Ires,2) - tagSize)/2);
Ires = padarray(Ires, [padSize,padSize], 255);

% Initialize tagImages array to hold the scaled tags.
tagImages = zeros(size(Ires,1), size(Ires,2), numTags);
tagImages(:,:,1) = Ires;

for idx = 2:numTags
   
    I = readimage(imdsTags, idx + 2);
    Igray = im2gray(I);
    Ires = imresize(Igray, 15, 'nearest');
    Ires = padarray(Ires, [padSize,padSize], 255);
    
    tagIds(idx) = readAprilTag(Ires, tagFamily);
    
    % Store the tag images.
    tagImages(:,:,idx) = Ires;
     
end

% Sort the tag images based on their IDs.
[~, sortIdx] = sort(tagIds);
tagImages = tagImages(:,:,sortIdx);

% Reshape the tag images to ensure that they appear in column-major order
% (montage function places image in row-major order).
columnMajIdx = reshape(1:numTags, tagArragement)';
tagImages = tagImages(:,:,columnMajIdx(:));

% Create the pattern using 'montage'.
imgData = montage(tagImages, 'Size', tagArragement);
calibPattern = imgData.CData;

end