clear; close all; clc;

%% Suit Magic project 
% Made for 2020 summer Hackathon at Mathworks
% Sam Maltz, Daniel Riehm, Rahul Guin, David Zhang
% Last updated o 07/10/2020

% Primary Use: For fun

% Secondary Use: Using Computer Vision toolbox's face tracking workflow (KLT algorithm),
% track user's face in front of the camera,  and then overlay the
% user(except for the face and neck) with preloaded costume pictures. 
% That way, you can look sharp in a meeting while staying in the comfort of
% your pajama.

%%
cam = webcam();

% Capture one frame to get its size.
videoFrame = snapshot(cam);

videoFrameGray = rgb2gray(videoFrame);

frameSize = size(videoFrame);

alphablend = vision.AlphaBlender('Operation', 'Blend', 'LocationSource', 'Input port', 'OpacitySource', 'Input port');
[suit, ~, suitAlpha] = imread('suit.png');
suitSize = size(suit);
suit = imresize(suit,frameSize(1:2));
suitAlpha = imresize(suitAlpha,frameSize(1:2));
suit = single(suit) / 255.0;
suitAlpha = single(suitAlpha) / 255.0;

suitLeftCorner = 618*(frameSize(2)/suitSize(2));

%% Setup
% Create objects for detecting faces, tracking points, acquiring and
% displaying video frames.

% Create the face detector object.
faceDetector = vision.CascadeObjectDetector();

% Create the point tracker object.
pointTracker = vision.PointTracker('MaxBidirectionalError', 2);


% Create the video player object. 
videoPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);

%% Detection and Tracking
% Capture and process video frames from the webcam in a loop to detect and
% track a face. The loop will run for 400 frames or until the video sdfnsdiofnsdoifnsd player
% window is closed.

runLoop = true;
numPts = 0;
frameCount = 0;

while runLoop && frameCount < 400
    
    % Get the next frame.
    videoFrame = snapshot(cam);
    videoFrameGray = rgb2gray(videoFrame);
    frameCount = frameCount + 1;
    
    bbox = faceDetector.step(videoFrameGray);

    if ~isempty(bbox)            
        % Convert the rectangle represented as [x, y, w, h] into an
        % M-by-2 matrix of [x,y] coordinates of the four corners. This
        % is needed to be able to transform the bounding box to display
        % the orientation of the face.
        bboxPoints = bbox2points(bbox(1, :));  

        bboxPolygon = reshape(bboxPoints', 1, []);
    end
    
    leftCorner = bboxPoints(4,:);
    
    videoFrame = single(videoFrame) / 255.0;
    out = alphablend(videoFrame, suit, suitAlpha,leftCorner + [-suitLeftCorner 0]);
        
    % Display the annotated video frame using the video player object.
    step(videoPlayer, out);

    % Check whether the video player window has been closed.
    runLoop = isOpen(videoPlayer);
end

% Clean up.
clear cam;
release(videoPlayer);
release(pointTracker);
release(faceDetector);

%% References
% Viola, Paul A. and Jones, Michael J. "Rapid Object Detection using a
% Boosted Cascade of Simple Features", IEEE CVPR, 2001.
%
% Bruce D. Lucas and Takeo Kanade. An Iterative Image Registration 
% Technique with an Application to Stereo Vision. 
% International Joint Conference on Artificial Intelligence, 1981.
%
% Carlo Tomasi and Takeo Kanade. Detection and Tracking of Point Features. 
% Carnegie Mellon University Technical Report CMU-CS-91-132, 1991.
%
% Jianbo Shi and Carlo Tomasi. Good Features to Track. 
% IEEE Conference on Computer Vision and Pattern Recognition, 1994.
%
% Zdenek Kalal, Krystian Mikolajczyk and Jiri Matas. Forward-Backward
% Error: Automatic Detection of Tracking Failures.
% International Conference on Pattern Recognition, 2010