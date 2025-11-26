% 相机内参 %
focalLength    = [309.4362, 344.2161]; % [fx, fy] 以像素为单位
principalPoint = [318.9034, 257.5352]; % [cx, cy] 像素中心的光学中心?
imageSize      = [360, 640];           % [nrows, mcols]图像大小
camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);
% 相机外部位置信息 %
height = 2.1798;    % 距地面的安装高度（以米为单位）?
pitch  = 14;        % 摄像机的俯仰角度（以度为单位）?
sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);% 构造一个单目相机?
[file, path] = uigetfile({'*.mp4;*.avi', 'Video Files (*.mp4, *.avi)'}, 'Select a Video File');
if isequal(file, 0)
    disp('User selected Cancel');
else
    disp(['User selected ', fullfile(path, file)]);
    % 然后使用VideoReader读取视频
    videoReader = VideoReader(fullfile(path, file));
end
% 读感兴趣的部分，其中包含车道标志和车辆??
timeStamp = 0;                   % 从视频开始的时间 
videoReader.CurrentTime = timeStamp;   % 指向所选帧
frame = readFrame(videoReader);        % 在timeStamp秒读取帧
% 在车辆坐标系下转换为鸟瞰图，区域为前方3-13米，左右6米 %
distAheadOfSensor = 13; 
spaceToOneSide    = 6;  
bottomOffset      = 3;
outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250];
birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);


detector = vehicleDetectorACF();
vehicleWidth = [1.5, 2.5];            % 普通车辆的宽度在1.5至2.5米之间?
monoDetector = configureDetectorMonoCamera(detector, sensor, vehicleWidth);    % 配置AFC探测器的摄像头?
[bboxes, scores] = detect(monoDetector, frame);
locations = computeVehicleLocations(bboxes, sensor);
%  imgOut = insertVehicleDetections(frame, locations, bboxes); % 将检测结果叠加在视频帧上

videoReader.CurrentTime = 0;
isPlayerOpen = true;
snapshot     = [];
while hasFrame(videoReader) && isPlayerOpen
 
    % 抓取视频帧?
    frame = readFrame(videoReader);
    % 鸟瞰图?
    birdsEyeImage = transformImage(birdsEyeConfig, frame);
    birdsEyeImage = rgb2gray(birdsEyeImage);
    % 检测车道边界特征?
    approxLaneMarkerWidthVehicle = 0.25;    % 车道宽度25cm
    vehicleROI = outView - [-1, 2, -3, 3];  % [4,11,-3,3]
    laneSensitivity = 0.25;                 % 灵敏度?
    birdsEyeViewBW = segmentLaneMarkerRidge(birdsEyeImage, birdsEyeConfig,approxLaneMarkerWidthVehicle, 'ROI', vehicleROI,'Sensitivity', laneSensitivity);         %杈撳叆鐏板害鍥捐浆鍖栦负浜屽?煎浘
    % 将像素坐标系下车道线点转化到车辆坐标系下
    [imageX, imageY] = find(birdsEyeViewBW);
    xyBoundaryPoints = imageToVehicle(birdsEyeConfig, [imageY, imageX]);
    % 寻找车道边界候选者?
    maxLanes      = 2;                      % 寻找最多两个车道?
    boundaryWidth = 3*approxLaneMarkerWidthVehicle; % 扩展边界宽度以搜索两个车道?
    [boundaries, boundaryPoints] = findParabolicLaneBoundaries(xyBoundaryPoints,boundaryWidth, ...
        'MaxNumBoundaries', maxLanes, 'validateBoundaryFcn', @validateBoundaryFcn);   % 查找抛物线车道边界?
    % 鏍规嵁闀垮害 绛涢??
    maxPossibleXLength = diff(vehicleROI(1:2));      % 选取感兴趣区域ROI中，x方向的最大长度?
    minXLength         = maxPossibleXLength * 0.6;   % 建立最小长度门槛?
    isOfMinLength = arrayfun(@(b)diff(b.XExtent) > minXLength, boundaries);
    boundaries    = boundaries(isOfMinLength);
    % 根据强度，筛选?%
    birdsImageROI = vehicleToImageROI(birdsEyeConfig, vehicleROI);
    [laneImageX,laneImageY] = meshgrid(birdsImageROI(1):birdsImageROI(2),birdsImageROI(3):birdsImageROI(4));
    vehiclePoints = imageToVehicle(birdsEyeConfig,[laneImageX(:),laneImageY(:)]);% 将图像点转换为车辆点
    maxPointsInOneLane = numel(unique(vehiclePoints(:,1)));%unique（vehiclePoints中第一列元素） 查找任何车道可能的最大唯一X轴位置数
    maxLaneLength = diff(vehicleROI(1:2));                 % 将车道边界的最大长度设置为ROI长度，28-4=24
    maxStrength   = maxPointsInOneLane/maxLaneLength;      % 计算该帧最长车道尺寸/ ROI尺寸=最大车道强度 ?  
    isStrong      = [boundaries.Strength] > 0.2*maxStrength;
    boundaries    = boundaries(isStrong);
    boundaries = classifyLaneTypes(boundaries, boundaryPoints);% 分类车道标记类型
    % 寻找自我通道
    xOffset    = 0;   % 距离传感器0米?
    distanceToBoundaries  = boundaries.computeBoundaryModel(xOffset);
    % 寻找候选自我边界?
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
    % 检测车辆?
    [bboxes, scores] = detect(monoDetector, frame);
    locations = computeVehicleLocations(bboxes, sensor);
    % 传感器输出?
    sensorOut.leftEgoBoundary  = leftEgoBoundary;
    sensorOut.rightEgoBoundary = rightEgoBoundary;
    sensorOut.vehicleLocations = locations;
    sensorOut.xVehiclePoints   = bottomOffset:distAheadOfSensor;
    sensorOut.vehicleBoxes     = bboxes;
    % 打包其他可视化数据，包括中间结果
    intOut.birdsEyeImage   = birdsEyeImage;
    intOut.birdsEyeConfig  = birdsEyeConfig;
    intOut.vehicleScores   = scores;
    intOut.vehicleROI      = vehicleROI;
    intOut.birdsEyeBW      = birdsEyeViewBW;
    closePlayers = ~hasFrame(videoReader);
    isPlayerOpen = visualizeSensorResults(frame, sensor, sensorOut, ...
        intOut, closePlayers);
    timeStamp = 2; % 在2s开始显示?
    if abs(videoReader.CurrentTime - timeStamp) < 0.01
        snapshot = takeSnapshot(frame, sensor, sensorOut);
    end
end
