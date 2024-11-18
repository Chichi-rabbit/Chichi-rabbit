%adc to mat
% 包含时间戳（列向量） 径向距离，速度，角度。与matlab给出的数据结构保持高度一致，方便后续处理

clc; clear; close all;

% 从 JSON 文件提取的 DCA1000 配置信息
num_samples = 256; % 每帧采样点数 (可能来自 ADC 配置)
num_chirps = 128;  % 每帧 chirps 数
num_rx = 4;        % 接收天线数量
num_bits = 16;     % ADC 每样本位深度
frame_time = 0.1;  % 每帧时间间隔 (秒) (从 JSON 的 `durationToCapture_ms` 提取)
sampling_rate = 1e7; % 采样率 10 MHz (Hz)
freq_slope = 29.9817e6; % 频率斜率 (Hz/us)
c = 3e8;           % 光速 (m/s)

% 读取 ADC 数据的路径（根据 JSON 提供的信息）
filePath ="C:\Users\苏婉莹\Desktop\Line\adc_data (2).bin" ;

% 打开并加载 adc_data.bin 文件
fileID = fopen(filePath, 'r');
adc_data = fread(fileID, 'int16'); % 假设 ADC 数据是 16 位整数
fclose(fileID);

% 数据格式检查与重组
num_channels = 4; % 根据 `dataPortConfig` 中的通道数（4 个通道）
num_frames = length(adc_data) / (num_channels * num_chirps * num_samples * 2); % 每通道包含 I/Q 数据（2倍）

% 数据重组为帧格式
adc_data = reshape(adc_data, [2, num_samples, num_chirps, num_channels, num_frames]);
adc_data_complex = adc_data(1, :, :, :, :) + 1j * adc_data(2, :, :, :, :); % 重建复数数据
adc_data_complex = squeeze(adc_data_complex); % 移除多余的维度

% 初始化结果
timestamps = (0:num_frames-1) * frame_time; % 时间戳
detectionLog = cell(num_frames, 1); % 初始化 detectionLog 数组

% 计算距离分辨率和距离刻度
range_resolution = c / (2 * sampling_rate * freq_slope); % 距离分辨率 (m)
range_bins = (0:num_samples-1) * range_resolution; % 距离刻度

% 计算速度分辨率和速度刻度
velocity_resolution = c / (2 * freq_slope * num_chirps); % 速度分辨率 (m/s)
velocity_bins = (-num_chirps/2:num_chirps/2-1) * velocity_resolution; % 速度刻度

% 遍历每帧数据
for frame_idx = 1:num_frames
    % 提取当前帧数据
    frame_data = adc_data_complex(:, :, :, frame_idx); % [samples, chirps, rx]

    % 距离维 FFT
    range_fft = fft(frame_data, [], 1); % 对每个通道进行距离维 FFT
    range_fft = range_fft(1:num_samples/2, :, :); % 只取正频部分

    % 速度维 FFT
    velocity_fft = fftshift(fft(range_fft, [], 2), 2); % 对每个通道进行速度维 FFT
    velocity_magnitude = abs(velocity_fft); % 幅度

    % 角度维 FFT（波束形成）
    angle_fft = fftshift(fft(velocity_fft, num_rx, 3), 3); % 角度维 FFT
    angle_magnitude = abs(angle_fft); % 幅度

    % 找到检测点（以 SNR 或强度为判断依据）
    threshold = max(angle_magnitude(:)) * 0.5; % 设置阈值为最大值的 50%
    [range_idx, velocity_idx, angle_idx] = ind2sub(size(angle_magnitude), find(angle_magnitude > threshold));

    % 初始化当前帧的检测结果
    detections = cell(length(range_idx), 1); % 当前帧的目标数组

    for k = 1:length(range_idx)
        % 计算检测点的距离、速度、角度
        detected_range = range_bins(range_idx(k));
        detected_velocity = velocity_bins(velocity_idx(k));
        detected_angle = asind((angle_idx(k) - (num_rx / 2 + 0.5)) * (c / (60.25e9) / num_rx)); % 角度估计

        % 构造 objectDetection 对象
        meas = [detected_angle, detected_range, detected_velocity]; % [角度, 距离, 速度]
        noise = diag([1, range_resolution, velocity_resolution]); % 测量噪声协方差
        detections{k} = objectDetection(timestamps(frame_idx), meas, ...
                                        'MeasurementNoise', noise);
    end

    % 将当前帧的检测结果存入 detectionLog
    detectionLog{frame_idx} = detections;
end

% 保存为 .mat 文件
output_file = 'radar_point_cloud_formatted_with_angle_line.mat';
save(output_file, 'timestamps', 'detectionLog');
disp(['数据已成功保存到 ', output_file]);
