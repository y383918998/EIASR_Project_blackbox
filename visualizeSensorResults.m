function isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut,...
    intOut, closePlayers)
 
    % 打开主要输入
    leftEgoBoundary  = sensorOut.leftEgoBoundary;
    rightEgoBoundary = sensorOut.rightEgoBoundary;
    locations        = sensorOut.vehicleLocations;
 
    xVehiclePoints   = sensorOut.xVehiclePoints;
    bboxes           = sensorOut.vehicleBoxes;
 
    % 打开其他数据
    birdsEyeViewImage = intOut.birdsEyeImage;
    birdsEyeConfig          = intOut.birdsEyeConfig;
    vehicleROI        = intOut.vehicleROI;
    birdsEyeViewBW    = intOut.birdsEyeBW;
 
    % 鸟瞰图中，可视化左右边界
    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeViewImage, leftEgoBoundary , birdsEyeConfig, xVehiclePoints, 'Color','Red');
    birdsEyeWithOverlays = insertLaneBoundary(birdsEyeWithOverlays, rightEgoBoundary, birdsEyeConfig, xVehiclePoints, 'Color','Green');
 
    % 常规视图中可视化左右边界
    frameWithOverlays = insertLaneBoundary(frame, leftEgoBoundary, sensor, xVehiclePoints, 'Color','Red');
    frameWithOverlays = insertLaneBoundary(frameWithOverlays, rightEgoBoundary, sensor, xVehiclePoints, 'Color','Green');
 
    frameWithOverlays = insertVehicleDetections(frameWithOverlays, locations, bboxes);
 
    imageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    ROI = [imageROI(1) imageROI(3) imageROI(2)-imageROI(1) imageROI(4)-imageROI(3)];
 
    % 突出显示包含异常值的候选车道点
    birdsEyeViewImage = insertShape(birdsEyeViewImage, 'rectangle', ROI); % show detection ROI
    birdsEyeViewImage = imoverlay(birdsEyeViewImage, birdsEyeViewBW, 'blue');
 
    % Display the results
    frames = {frameWithOverlays, birdsEyeViewImage, birdsEyeWithOverlays};
 
    persistent players;
    if isempty(players)
        frameNames = {'Lane marker and vehicle detections', 'Raw segmentation', 'Lane marker detections'};
        players = helperVideoPlayerSet(frames, frameNames);
    end
    update(players, frames);
 
    % Terminate the loop when the first player is closed
    isPlayerOpen = isOpen(players, 1);
 
    if (~isPlayerOpen || closePlayers) % close down the other players
        clear players;
    end
end