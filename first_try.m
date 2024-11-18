% 加载 radar_point_cloud_formatted_with_angle.mat 文件
dataFile = 'radar_point_cloud_formatted_with_angle_line.mat';
load(dataFile, 'timestamps', 'detectionLog'); % 加载时间戳和检测日志

% 显示范围界定
display = HelperTIRadarTrackingDisplayWithoutCamera('XLimits', [-3 3], ... % 限定X轴范围
                                                     'YLimits', [-1 1], ... % 限定Y轴范围
                                                     'MaxRange', 3, ...     % 最大检测范围3米
                                                     'RadarReferenceLines', zeros(2, 0)); % 占位，无雷达参考线

% 初始化检测参数
minRangeRate = 0.05; % 最小速度阈值，适配慢速步行
epsilon = 0.3;       % DBSCAN 聚类半径，适应检测点密度
minNumPts = 1;       % 允许单点作为目标

% 初始化跟踪器
tracker = trackerJPDA(TrackLogic="Integrated");
tracker.FilterInitializationFcn = @initPeopleTrackingFilter;

% 定义测量空间
azSpan = 60;  % 水平视角（度）
rSpan = 3;    % 距离范围（米）
dopplerSpan = 2; % 速度范围（-1 ~ 1 m/s）
V = azSpan * rSpan * dopplerSpan;

% JPDA 参数调整
tracker.ClutterDensity = 2 / V;       % 误报密度
tracker.NewTargetDensity = 0.05 / V; % 新目标密度
tracker.DetectionProbability = 0.95; % 目标检测概率
tracker.ConfirmationThreshold = 0.8; % 轨迹确认阈值
tracker.DeletionThreshold = 1e-2;    % 轨迹删除阈值

% 初始化轨迹
tracks = objectTrack.empty(0, 1);

% 动态目标检测与跟踪
for k = 1:numel(detectionLog)
    % 时间戳与检测数据
    time = timestamps(k);
    detections = detectionLog{k};

    % 过滤静止目标
    isDynamic = false(1, numel(detections));
    for d = 1:numel(detections)
        isDynamic(d) = abs(detections{d}.Measurement(3)) > minRangeRate; % 判断目标是否动态
    end
    detectionsDynamic = detections(isDynamic);

    % 聚类动态目标
    if isempty(detectionsDynamic)
        clusters = zeros(0, 1, 'uint32');
    else
        clusters = partitionDetections(detectionsDynamic, epsilon, minNumPts, 'Algorithm', 'DBSCAN');
    end
    clusteredDets = mergeDetections(detectionsDynamic, clusters);

    % 更新轨迹
    if isLocked(tracker) || ~isempty(clusteredDets)
        tracks = tracker(clusteredDets, time);
    end

    % 更新显示
    step(display, detectionsDynamic, clusteredDets, tracks);
    pause(0.05); % 实时更新显示
end

disp('所有帧处理完成！');

% 轨迹初始化函数
function filter = initPeopleTrackingFilter(detection)
    % 初始化目标跟踪滤波器
    state = [detection.Measurement(2:3); 0; 0]; % 初始化位置与速度
    stateCov = diag([0.5, 0.5, 0.1, 0.1]);    % 初始化协方差矩阵
    processNoise = diag([0.01, 0.01, 0.005, 0.005]); % 过程噪声
    measurementNoise = detection.MeasurementNoise;

    filter = trackingEKF(StateTransitionFcn = @constvel, ...
                         StateTransitionJacobianFcn = @constveljac, ...
                         MeasurementFcn = @cvmeas, ...
                         MeasurementJacobianFcn = @cvmeasjac, ...
                         State = state, ...
                         StateCovariance = stateCov, ...
                         ProcessNoise = processNoise, ...
                         MeasurementNoise = measurementNoise);
end

