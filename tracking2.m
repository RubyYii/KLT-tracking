% 在处理之前记录开始时间
tic;

% 选择是否使用网络摄像头
useWebcam = false;

% 选择绘图类型，可以是 'distance' 或者 'position'

plotType = 'distance'; % 'distance'或者 'position'
% 在程序开始时初始化两条线的数据

%% processing 
if useWebcam
    % 获取摄像头视频
    cam = webcam;

    % Initialize a structure to hold the video frames
    videoFrames = struct('cdata', zeros(cam.Resolution, 3, 'uint8'), 'colormap', []);
    

    % Read the frames into the structure
    idx = 1;
    while idx <= 500  % assume we want to capture 500 frames
        videoFrames(idx).cdata = snapshot(cam);
        idx = idx + 1;
    end

    % Release the camera
    clear cam;
else
    % 读取视频文件
    %videoFile = 'E:\project\powerpack_tracking\moving2.mp4';
    
    videoFile = 'D:\Qiufeng\powerpack_tracking\moving2.mp4';
    videoReader = VideoReader(videoFile);

    % 初始化一个结构来存储视频帧
    videoFrames = struct('cdata', zeros(videoReader.Height, videoReader.Width, 3, 'uint8'), 'colormap', []);

    % 将帧读入结构体
    idx = 1;
    while hasFrame(videoReader)
        videoFrames(idx).cdata = readFrame(videoReader);
        idx = idx + 1;
    end
end

numFrames = numel(videoFrames);
grayFrames = cell(1, numFrames);

%
for idx = 1:numFrames
    grayFrames{idx} = rgb2gray(videoFrames(idx).cdata);
end

% Display the first frame
imshow(grayFrames{1});
%% start tracking

%% 手动输入
h = imrect;
rect = wait(h);
rect = round(rect);
rect(3:4) = abs(rect(3:4));
[centers, ~] = imfindcircles(grayFrames{1}, [round(rect(3)/2)-10 round(rect(3)/2)+10], 'ObjectPolarity', 'bright');

if isempty(centers)
    % 如果找不到圆，则让用户手动输入坐标点
    fprintf('Cannot find circle. Please input coordinate manually.\n');
    centers = ginput(1);
end

tracker = vision.PointTracker('MaxBidirectionalError', 1);
points = centers;
initialize(tracker, points, grayFrames{1});
fixedPoint = points(1,:);

% Initialize the KLT feature tracker
tracker = vision.PointTracker('MaxBidirectionalError', 1);

% Detect features (points) in the first frame within the selected rectangle
points = detectMinEigenFeatures(grayFrames{1}, 'ROI', rect);
points = points.Location;

% Check if points are detected
if isempty(points)
    error('No points detected in the initial frame. Please try selecting a different region.');
end

% Initialize the tracker with the detected points
initialize(tracker, points, grayFrames{1});

% Track the sperm throughout the video
trackedPositions = zeros(numFrames, 2);
distanceToFixedPoint = zeros(numFrames, 1);
trackedPositions(1,:) = mean(points, 1);
distanceToFixedPoint(1) = norm(trackedPositions(1,:) - fixedPoint);

%% Create a new figure for real-time plotting
% 创建一个新的 figure，以实时显示视频帧和距离曲线
figure;
distanceData = nan(1, numFrames);
positionData = nan(1, numFrames);

% 创建一个 subplot 来显示距离曲线
hPlot = subplot(2, 1, 2);
% 在初始化图形时，创建两条线
hLine1 = line(hPlot, nan, nan, 'Color', 'b');  % distance
hold on; % Allows multiple plots on the same axes
hLine2 = line(hPlot, nan, nan, 'Color', 'r');  % position
% Add a second y axis to the plot
yyaxis left;
ylabel(hPlot, 'Distance (pixels)');
% yyaxis right;
% ylabel(hPlot, 'Position (pixels)');

xlabel(hPlot, 'Time (s)');



% 创建另一个 subplot 来显示视频帧
hVideo = subplot(2, 1, 1);

if strcmp(plotType, 'distance')
    title(hPlot, 'Distance to the centre of the circle in real time');
    ylabel(hPlot, 'Distance (pixels)');
    set(hLine2, 'Visible', 'off'); % Hide the second line for position plotting
else
    title(hPlot, 'Real-time location coordinates');
    ylabel(hPlot, 'location');
end
xlabel(hPlot, 'time(s)');


for idx = 2:numFrames
    % 获取当前帧
    frame = videoFrames(idx).cdata;

    % Track the sperm in the current frame
    [points, isTracked] = step(tracker, grayFrames{idx});
    validPoints = points(isTracked,:);
    
    if ~isempty(validPoints)
        trackedPositions(idx,:) = mean(validPoints, 1);
        distanceToFixedPoint(idx) = norm(trackedPositions(idx,:) - fixedPoint);
        
        % Update the tracker with the current frame
        setPoints(tracker, validPoints);
        
        % 描绘跟踪位置的标记
                position = trackedPositions(idx,:);
        frame = insertMarker(frame, position, '*', 'Color', 'blue', 'Size', 15);
        %frame = insertMarker(frame, position, 'o', 'Color', 'blue', 'Size', 20);

        %frame = insertMarker(frame,'Circle', position,  'Color', 'blue', 'LineWidth', 20);
        % 为了更明显地标记出固定点，我们可以在固定点处描绘一个大一点的红色圆
        %frame = insertShape(frame, 'Circle', [fixedPoint, radii(1)], 'Color', 'red', 'LineWidth', 15);
        frame = insertMarker(frame, fixedPoint, 'o', 'Color', 'yellow', 'Size', 20);
        
        % 在 hVideo subplot 中显示帧
            % 在 hVideo subplot 中显示帧
        imshow(frame, 'Parent', hVideo);
        title(hVideo, sprintf('Frame %d', idx));


    else
        % Reinitialize the tracker with new points from the current frame
        points = detectMinEigenFeatures(grayFrames{idx}, 'ROI', rect);
        points = points.Location;
        
        if ~isempty(points)
            trackedPositions(idx,:) = mean(points, 1);
            distanceToFixedPoint(idx) = norm(trackedPositions(idx,:) - fixedPoint);
            
            % Release and reinitialize the tracker
            release(tracker);
            initialize(tracker, points, grayFrames{idx});
        else
            trackedPositions(idx,:) = trackedPositions(idx-1,:);
            distanceToFixedPoint(idx) = distanceToFixedPoint(idx-1);
            fprintf('Warning: No points detected in frame %d.\n', idx);
        end
    end

    
    % Update real-time plot
    if strcmp(plotType, 'distance')
        distanceData(idx) = distanceToFixedPoint(idx);
    else
        positionData(idx) = trackedPositions(idx, 1);
    end
    set(hLine1, 'XData', (1:idx) / videoReader.FrameRate, 'YData', distanceData(1:idx));  % Update the distance line
    yyaxis right;  % Switch to the right y axis
    set(hLine2, 'XData', (1:idx) / videoReader.FrameRate, 'YData', positionData(1:idx));  % Update the position line

    drawnow;  % Force MATLAB to update the plot
    pause(1/videoReader.FrameRate); % 根据视频帧速率调整 pause 的持续时间
    legend(hPlot, {'Distance', 'Position'});

end



% 在轨迹跟踪完成后绘制轨迹
figure;
plot(trackedPositions(:,1), trackedPositions(:,2));
title('Trajectory of tracked points');
xlabel('X Position');
ylabel('Y Position');
grid on;
% 设置坐标轴等比例
axis equal;
% 处理完成后记录结束时间
toc;

% 计算帧率
frameRate = numFrames / toc;
fprintf('Average frame rate: %.2f frames per second\n', frameRate);

