classdef HelperTIRadarTrackingDisplayWithoutCamera < matlab.System
    properties (Nontunable)
        XLimits = [-3 3];
        YLimits = [-3 3];
        ZLimits = [-0.25 0.25];
        FieldOfView = [120 0];
        MaxRange = 25;
        MotionModel = 'constvel';
        RadarReferenceLines = [-3.5, -3.5,  3.5,  3.5, -3.5;
                       -3.5,  3.5,  3.5, -3.5, -3.5]; % 方形雷达参考线

    end

    properties (Access = protected)
        Axes
        RawDetectionPlotter
        ClusteredDetectionPlotter
        TrackPlotter
    end

    methods
        function obj = HelperTIRadarTrackingDisplayWithoutCamera(varargin)
            setProperties(obj, nargin, varargin{:});
        end
    end

    methods (Access = protected)
        function setupImpl(obj, detections, clusteredDets, tracks)
            % 创建显示雷达数据的单个轴
            f = figure('Visible','on','Units','normalized','Position',[0.1 0.1 0.8 0.8]);
            ax = axes(f);

            % 创建 theater plot
            tp = theaterPlot('Parent',ax,"XLimits",obj.XLimits,'YLimits',obj.YLimits,'ZLimits',obj.ZLimits);

            % 颜色设置
            clrs = lines(7);

            % 创建检测点 plotter
            dp = detectionPlotter(tp,'DisplayName','Point Cloud','MarkerFaceColor',clrs(3,:),'MarkerEdgeColor',clrs(3,:));

            % 创建聚类质心 plotter
            dcp = detectionPlotter(tp,'DisplayName','Centroid Estimate','MarkerFaceColor',clrs(2,:),'MarkerEdgeColor',clrs(2,:));

            % 创建轨迹 plotter
            trkP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor',clrs(1,:),'MarkerEdgeColor',clrs(1,:),'ColorizeHistory','off','ConnectHistory','off','HistoryDepth',30,'FontSize',20);

            % 绘制雷达覆盖范围
            cp = coveragePlotter(tp,'DisplayName','','Color',[0 0 0],'Alpha',[0.1 0.1]);
            fov = obj.FieldOfView;
            maxR = obj.MaxRange;
            scanLimits = [-1 1;-1 1].*([fov(1)/2;fov(2)/2]);
            cvg = struct('Index',1,'LookAngle',0,'FieldOfView',[120;0],'ScanLimits',scanLimits,'Range',maxR,'Position',[0 0 0],'Orientation',eye(3));
            cp.plotCoverage(cvg);

            % 顶视图
            view(ax,-90,90);

            obj.RawDetectionPlotter = dp;
            obj.ClusteredDetectionPlotter = dcp;
            obj.TrackPlotter = trkP;
        end

        function stepImpl(obj, detections, clusteredDets, tracks)
            % 绘制原始检测点
            pos = zeros(3,numel(detections));
            vel = zeros(3,numel(detections));
            posCov = zeros(3,3,numel(detections));
            for i = 1:numel(detections)
                [pos(:,i),vel(:,i),posCov(:,:,i)] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(detections{i},'radar','double');
            end
            obj.RawDetectionPlotter.plotDetection(pos',vel');
            setEdgeAlpha(obj.RawDetectionPlotter);

            % 绘制聚类质心
            pos = zeros(3,numel(clusteredDets));
            vel = zeros(3,numel(clusteredDets));
            posCov = zeros(3,3,numel(clusteredDets));
            for i = 1:numel(clusteredDets)
                [pos(:,i),vel(:,i),posCov(:,:,i)] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(clusteredDets{i},'radar','double');
            end
            obj.ClusteredDetectionPlotter.plotDetection(pos',vel',posCov);
            setEdgeAlpha(obj.ClusteredDetectionPlotter);

            % 绘制轨迹
            [pos, posCov] = getTrackPositions(tracks,obj.MotionModel);
            vel = getTrackVelocities(tracks,obj.MotionModel);
            if size(pos,2) == 2
                pos = [pos zeros(numel(tracks),1)];
                vel = [vel zeros(numel(tracks),1)];
                posCov3 = zeros(3,3,numel(tracks));
                for i = 1:numel(tracks)
                    posCov3(:,:,i) = blkdiag(posCov(:,:,i),1);
                end
                posCov = posCov3;
            end

            labels = "T" + string([tracks.TrackID]);
            obj.TrackPlotter.plotTrack(pos,vel,posCov,labels);
            setEdgeAlpha(obj.TrackPlotter);
        end
    end
end

function setEdgeAlpha(trkPlotter)
w = warning('off');
s = struct(trkPlotter);
warning(w);
for i = 1:numel(s.CovariancesPatches)
    set(s.CovariancesPatches(i),'EdgeAlpha',1);
    set(s.CovariancesPatches(i),'FaceAlpha',0);
end
end
