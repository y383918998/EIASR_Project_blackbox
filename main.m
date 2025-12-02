% Ïà»úÄÚ²Î %
focalLength    = [309.4362, 344.2161]; % [fx, fy] ÒÔÏñËØÎªµ¥Î»
principalPoint = [318.9034, 257.5352]; % [cx, cy] ÏñËØÖÐÐÄµÄ¹âÑ§ÖÐÐÄ?
imageSize      = [360, 640];           % [nrows, mcols]Í¼Ïñ´óÐ¡
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
% Target size for all frames that enter the detector. Resize any input that
% does not match the calibrated intrinsics to avoid runtime errors.
targetImageSize = camIntrinsics.ImageSize;
% Ïà»úÍâ²¿Î»ÖÃÐÅÏ¢ %
height = 2.1798;    % ¾àµØÃæµÄ°²×°¸ß¶È£¨ÒÔÃ×Îªµ¥Î»£©?
pitch  = 14;        % ÉãÏñ»úµÄ¸©Ñö½Ç¶È£¨ÒÔ¶ÈÎªµ¥Î»£©?
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);% ¹¹ÔìÒ»¸öµ¥Ä¿Ïà»ú?
[file, path] = uigetfile({'*.mp4;*.avi', 'Video Files (*.mp4, *.avi)'}, 'Select a Video File');
if isequal(file, 0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path, file)]);
    % È»ºóÊ¹ÓÃVideoReader¶ÁÈ¡ÊÓÆµ¢
    videoReader = VideoReader(fullfile(path, file));
end
% ¶Á¸ÐÐËÈ¤µÄ²¿·Ö£¬ÆäÖÐ°üº¬³µµÀ±êÖ¾ºÍ³µÁ¾??
timeStamp = 0;                   % ´ÓÊÓÆµ¿ªÊ¼µÄÊ±¼ä 
videoReader.CurrentTime = timeStamp;   % Ö¸ÏòËùÑ¡Ö¡
frame = readFrame(videoReader);        % ÔÚtimeStampÃë¶ÁÈ¡Ö¡
if ~isequal(size(frame, 1), targetImageSize(1)) || ~isequal(size(frame, 2), targetImageSize(2))
    frame = imresize(frame, targetImageSize); % Ensure consistency with camera intrinsics
end
% ÔÚ³µÁ¾×ø±êÏµÏÂ×ª»»ÎªÄñî«Í¼£¬ÇøÓòÎªÇ°·½3-13Ã×£¬×óÓÒ6Ã× %
distAheadOfSensor = 13; 
spaceToOneSide    = 6;  
bottomOffset      = 3;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250];
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);


detector = vehicleDetectorACF();
vehicleWidth = [1.5, 2.5];            % ÆÕÍ¨³µÁ¾µÄ¿í¶ÈÔÚ1.5ÖÁ2.5Ã×Ö®¼ä?
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);    % ÅäÖÃAFCÌ½²âÆ÷µÄÉãÏñÍ·?
[bboxes, scores] = detect(monoDetector, frame);
locations = computeVehicleLocations(bboxes, sensor);
%  imgOut = insertVehicleDetections(frame, locations, bboxes); % ½«¼ì²â½á¹ûµþ¼ÓÔÚÊÓÆµÖ¡ÉÏ

videoReader.CurrentTime = 0;
isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
 
    % ×¥È¡ÊÓÆµÖ¡?
    frame = readFrame(videoReader);
    if ~isequal(size(frame, 1), targetImageSize(1)) || ~isequal(size(frame, 2), targetImageSize(2))
        frame = imresize(frame, targetImageSize); % Normalize frame size for detector
    end
    % Äñî«Í¼?
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);
    % ¼ì²â³µµÀ±ß½çÌØÕ÷?
    approxLaneMarkerWidthVehicle = 0.25;    % ³µµÀ¿í¶È25cm
    vehicleROI = outView - [-1, 2, -3, 3];  % [4,11,-3,3]
    laneSensitivity = 0.25;                 % ÁéÃô¶È?
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig,approxLaneMarkerWidthVehicle, 'ROI', vehicleROI,'Sensitivity', laneSensitivity);         %è¾å¥ç°åº¦å¾è½¬åä¸ºäºå?¼å¾
    % ½«ÏñËØ×ø±êÏµÏÂ³µµÀÏßµã×ª»¯µ½³µÁ¾×ø±êÏµÏÂ
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
    % Ñ°ÕÒ³µµÀ±ß½çºòÑ¡Õß?
    maxLanes      = 2;                      % Ñ°ÕÒ×î¶àÁ½¸ö³µµÀ?
    boundaryWidth = 3*approxLaneMarkerWidthVehicle; % À©Õ¹±ß½ç¿í¶ÈÒÔËÑË÷Á½¸ö³µµÀ?
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);   % ²éÕÒÅ×ÎïÏß³µµÀ±ß½ç?
    % æ ¹æ®é¿åº¦ ç­é??
    maxPossibleXLength = diff(vehicleROI(1:2));      % Ñ¡È¡¸ÐÐËÈ¤ÇøÓòROIÖÐ£¬x·½ÏòµÄ×î´ó³¤¶È?
    minXLength         = maxPossibleXLength * 0.6;   % ½¨Á¢×îÐ¡³¤¶ÈÃÅ¼÷?
    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    % ¸ù¾ÝÇ¿¶È£¬É¸Ñ¡?%
    birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    [laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
    vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);% ½«Í¼Ïñµã×ª»»Îª³µÁ¾µã
    maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));%unique£¨vehiclePointsÖÐµÚÒ»ÁÐÔªËØ£© ²éÕÒÈÎºÎ³µµÀ¿ÉÄÜµÄ×î´óÎ¨Ò»XÖáÎ»ÖÃÊý
    maxLaneLength = diff(vehicleROI(1:2));                 % ½«³µµÀ±ß½çµÄ×î´ó³¤¶ÈÉèÖÃÎªROI³¤¶È£¬28-4=24
    maxStrength   = maxPointsInOneLane/maxLaneLength;      % ¼ÆËã¸ÃÖ¡×î³¤³µµÀ³ß´ç/ ROI³ß´ç=×î´ó³µµÀÇ¿¶È ?  
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);
    boundaries = classifyLaneTypes(boundaries, boundaryPoints);% ·ÖÀà³µµÀ±ê¼ÇÀàÐÍ
    % Ñ°ÕÒ×ÔÎÒÍ¨µÀ
    xOffset    = 0;   % ¾àÀë´«¸ÐÆ÷0Ã×?
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
    % Ñ°ÕÒºòÑ¡×ÔÎÒ±ß½ç?
    leftEgoBoundaryIndex  = [];
    rightEgoBoundaryIndex = [];
    minLDistance = min(distanceToBoundaries(distanceToBoundaries>0));
    minRDistance = max(distanceToBoundaries(distanceToBoundaries<=0));
    if ~isempty(minLDistance)
        leftEgoBoundaryIndex  = distanceToBoundaries == minLDistance;
    end
    if ~isempty(minRDistance)
        rightEgoBoundaryIndex = distanceToBoundaries == minRDistance;
    end
    leftEgoBoundary       = boundaries(leftEgoBoundaryIndex);
    rightEgoBoundary      = boundaries(rightEgoBoundaryIndex);
    % ¼ì²â³µÁ¾?
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations(bboxes, sensor);
    % ´«¸ÐÆ÷Êä³ö?
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    % ´ò°üÆäËû¿ÉÊÓ»¯Êý¾Ý£¬°üÀ¨ÖÐ¼ä½á¹û
    intOut.birdsEyeImage   = birdsEyeImage;
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    timeStamp = 2; % ÔÚ2s¿ªÊ¼ÏÔÊ¾?
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
end
