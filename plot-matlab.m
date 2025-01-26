% Initialize ROS 2 connection in MATLAB
ros2init('http://192.168.1.107:11311');  % Raspberry Pi IP

% Create a subscriber to the 'sensor_data' topic (assuming it's of type String)
sensor_sub = ros2subscriber('/sensor_data', 'std_msgs/msg/String');

% Set up the figure for plotting
figure;
hold on;
xlabel('Time (s)');
ylabel('Values');
title('Live Data from R60ABD1 Sensor');
heart_rate_plot = plot(NaN, NaN, 'r-', 'DisplayName', 'Heart Rate');
breath_rate_plot = plot(NaN, NaN, 'b-', 'DisplayName', 'Breath Rate');
legend;

% Create arrays to store the data
time_data = [];
heart_rate_data = [];
breath_rate_data = [];

% Set a time limit for the plot to update
start_time = tic;

while true
    % Get the latest message from the ROS2 subscriber
    msg = receive(sensor_sub, 10);  % Timeout of 10 seconds

    % Extract the data from the message
    sensor_report = msg.Data;

    % Parse the sensor report (this depends on how you format your data)
    % Extract heart rate and breath rate from the message string (simple example)
    heart_rate = extract_heart_rate(sensor_report);  % Function to extract heart rate
    breath_rate = extract_breath_rate(sensor_report);  % Function to extract breath rate

    % Append the new data to the arrays
    elapsed_time = toc(start_time);
    time_data = [time_data, elapsed_time];
    heart_rate_data = [heart_rate_data, heart_rate];
    breath_rate_data = [breath_rate_data, breath_rate];

    % Update the plots
    set(heart_rate_plot, 'XData', time_data, 'YData', heart_rate_data);
    set(breath_rate_plot, 'XData', time_data, 'YData', breath_rate_data);
    drawnow;

    % Optional: Set a rate for how often the data should update
    pause(0.1);  % Adjust this for faster or slower updates
end

% Function to extract heart rate from the string message
function heart_rate = extract_heart_rate(report)
    % Assuming the format: 'Heart Rate: 75 bpm' in the report string
    tokens = regexp(report, 'Heart Rate: (\d+) bpm', 'tokens');
    if ~isempty(tokens)
        heart_rate = str2double(tokens{1}{1});
    else
        heart_rate = NaN;  % In case heart rate data is not available
    end
end

% Function to extract breath rate from the string message
function breath_rate = extract_breath_rate(report)
    % Assuming the format: 'Breath Rate: 12 bpm' in the report string
    tokens = regexp(report, 'Breath Rate: (\d+) bpm', 'tokens');
    if ~isempty(tokens)
        breath_rate = str2double(tokens{1}{1});
    else
        breath_rate = NaN;  % In case breath rate data is not available
    end
end
