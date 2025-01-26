% Create a ROS2 subscriber and set up real-time plots for Heart Rate, Breath Rate, and Motion Level

% Initialize ROS2 connection
ros2init('http://192.168.1.107:11311');  % Raspberry Pi IP

% Create subscribers for the three topics
hr_sub = rossubscriber('/heart_rate', 'std_msgs/Float32');
br_sub = rossubscriber('/breath_rate', 'std_msgs/Float32');
motion_sub = rossubscriber('/motion_level', 'std_msgs/Float32');

% Create a figure for plotting
figure;
subplot(3,1,1);
heart_rate_plot = plot(NaN, NaN, 'r-', 'DisplayName', 'Heart Rate (bpm)');
title('Heart Rate');
xlabel('Time (seconds)');
ylabel('Heart Rate (bpm)');
legend;

subplot(3,1,2);
breath_rate_plot = plot(NaN, NaN, 'b-', 'DisplayName', 'Breath Rate (bpm)');
title('Breath Rate');
xlabel('Time (seconds)');
ylabel('Breath Rate (bpm)');
legend;

subplot(3,1,3);
motion_level_plot = plot(NaN, NaN, 'g-', 'DisplayName', 'Motion Level');
title('Motion Level');
xlabel('Time (seconds)');
ylabel('Motion Level');
legend;

% Data buffers to store the values
time_data = [];
heart_rate_data = [];
breath_rate_data = [];
motion_level_data = [];

% Time counter for x-axis
time_counter = 0;

% Loop to update plots
while true
    % Get the latest messages
    hr_msg = receive(hr_sub, 1);
    br_msg = receive(br_sub, 1);
    motion_msg = receive(motion_sub, 1);
    
    % Get the data from the messages
    heart_rate = hr_msg.Data;
    breath_rate = br_msg.Data;
    motion_level = motion_msg.Data;

    % Update time and store the new data
    time_counter = time_counter + 1;
    time_data = [time_data, time_counter];
    heart_rate_data = [heart_rate_data, heart_rate];
    breath_rate_data = [breath_rate_data, breath_rate];
    motion_level_data = [motion_level_data, motion_level];

    % Update the plots
    set(heart_rate_plot, 'XData', time_data, 'YData', heart_rate_data);
    set(breath_rate_plot, 'XData', time_data, 'YData', breath_rate_data);
    set(motion_level_plot, 'XData', time_data, 'YData', motion_level_data);

    % Pause to update plots every 0.1 seconds (adjust as needed)
    pause(0.1);
end
