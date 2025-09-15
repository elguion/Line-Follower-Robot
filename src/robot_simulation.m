%% LINE FOLLOWING ROBOT - MATLAB IMPLEMENTATION
% Complete simulation and control system design
% Author: Advanced Robotics Control System
% Date: September 2025

clear all; close all; clc;

%% SYSTEM PARAMETERS AND SPECIFICATIONS
fprintf('=== LINE FOLLOWING ROBOT SIMULATION ===\n\n');

% Robot Physical Parameters
robot_params = struct();
robot_params.wheelbase = 0.15;        % Distance between wheels (m)
robot_params.wheel_radius = 0.0325;   % Wheel radius (m)
robot_params.max_speed = 0.5;         % Maximum speed (m/s)
robot_params.max_angular_vel = 2.0;   % Maximum angular velocity (rad/s)
robot_params.mass = 0.8;              % Robot mass (kg)
robot_params.inertia = 0.02;          % Moment of inertia (kg⋅m²)

% Sensor Parameters
sensor_params = struct();
sensor_params.num_sensors = 5;
sensor_params.sensor_spacing = 0.02;   % 2cm between sensors
sensor_params.detection_width = 0.08;  % Total sensor array width
sensor_params.noise_std = 0.01;       % Sensor noise standard deviation
sensor_params.update_rate = 1000;     % Hz

% Control System Specifications
control_specs = struct();
control_specs.rise_time = 0.5;        % seconds
control_specs.settling_time = 1.0;    % seconds  
control_specs.overshoot = 0.10;       % 10%
control_specs.steady_state_error = 0.02; % 2%
control_specs.sampling_freq = 100;    % Hz

% PID Controller Parameters 
pid_params = struct();
pid_params.Kp = 1.2;    % Proportional gain
pid_params.Ki = 0.05;   % Integral gain  
pid_params.Kd = 0.3;    % Derivative gain
pid_params.max_output = 100; % Maximum control output

%% LINE PATH GENERATION
% Create a complex test track with straight lines, curves, and sharp turns
t = linspace(0, 4*pi, 1000);
track_x = cumsum([0; cos(t(1:end-1))' * 0.1]);
track_y = sin(t)' * 0.5 + 0.3*sin(3*t)';

% Add some sharp turns and complexity
sharp_turn_x = linspace(max(track_x), max(track_x) + 1, 100);
sharp_turn_y = max(track_y) * ones(size(sharp_turn_x));
sharp_turn_y(50:end) = max(track_y) - (0:50)*0.02;

track_x = [track_x; sharp_turn_x'];
track_y = [track_y; sharp_turn_y'];

% Create reference path structure
reference_path = struct();
reference_path.x = track_x;
reference_path.y = track_y;
reference_path.length = length(track_x);

%% PLANT MODEL (ROBOT DYNAMICS)
% Transfer function for the robot system
% G(s) = K/(τs + 1) where K is DC gain, τ is time constant
K_plant = 2.5;      % DC gain
tau_plant = 0.2;    % Time constant (mechanical inertia)


s = tf('s');
G_plant = K_plant / (tau_plant * s + 1);

% Display plant characteristics
fprintf('Plant Transfer Function: G(s) = %.1f/(%.1fs + 1)\n', K_plant, tau_plant);

%% PID CONTROLLER DESIGN
% Create PID controller
pid_controller = pid(pid_params.Kp, pid_params.Ki, pid_params.Kd);
pid_controller.Name = 'Line Following PID Controller';

% Display PID parameters
fprintf('\nPID Controller Parameters:\n');
fprintf('Kp = %.2f (Proportional Gain)\n', pid_params.Kp);
fprintf('Ki = %.3f (Integral Gain)\n', pid_params.Ki); 
fprintf('Kd = %.2f (Derivative Gain)\n', pid_params.Kd);

%% CLOSED-LOOP SYSTEM ANALYSIS
% Create closed-loop system
T_closedloop = feedback(pid_controller * G_plant, 1);

% Analyze system performance
step_info = stepinfo(T_closedloop);
fprintf('\nClosed-Loop System Performance:\n');
fprintf('Rise Time: %.3f seconds (Spec: ≤%.1f s) %s\n', ...
    step_info.RiseTime, control_specs.rise_time, ...
    char(9989 + (step_info.RiseTime > control_specs.rise_time)));
fprintf('Settling Time: %.3f seconds (Spec: ≤%.1f s) %s\n', ...
    step_info.SettlingTime, control_specs.settling_time, ...
    char(9989 + (step_info.SettlingTime > control_specs.settling_time)));
fprintf('Overshoot: %.1f%% (Spec: ≤%.0f%%) %s\n', ...
    step_info.Overshoot, control_specs.overshoot*100, ...
    char(9989 + (step_info.Overshoot/100 > control_specs.overshoot)));
fprintf('Peak: %.3f at %.3f seconds\n', step_info.Peak, step_info.PeakTime);

%% SENSOR SIMULATION FUNCTIONS
function sensor_readings = simulate_ir_sensors(robot_pos, line_pos, sensor_params)
    % Simulate 5 IR sensors detecting line position
    sensor_positions = linspace(-sensor_params.detection_width/2, ...
                               sensor_params.detection_width/2, ...
                               sensor_params.num_sensors);
    
    % Calculate distance from each sensor to line
    distances = abs(sensor_positions - (line_pos - robot_pos));
    
    % Convert to sensor readings (closer = higher reading)
    max_detection_range = 0.05; % 5cm maximum detection range
    sensor_readings = max(0, 1 - distances/max_detection_range);
    
    % Add noise
    noise = sensor_params.noise_std * randn(size(sensor_readings));
    sensor_readings = max(0, min(1, sensor_readings + noise));
end

function error = calculate_line_error(sensor_readings)
    % Calculate weighted average position
    num_sensors = length(sensor_readings);
    sensor_weights = 0:num_sensors-1;
    
    if sum(sensor_readings) > 0
        weighted_position = sum(sensor_weights .* sensor_readings) / sum(sensor_readings);
        center_position = (num_sensors - 1) / 2;
        error = weighted_position - center_position;
    else
        error = 0; % No line detected
    end
end

%% ROBOT SIMULATION (WITHOUT CONTROL)
fprintf('\n=== UNCONTROLLED ROBOT SIMULATION ===\n');

% Simulation parameters
dt = 0.01; % Time step
sim_time = 20; % Total simulation time
t_sim = 0:dt:sim_time;
num_steps = length(t_sim);

% Initialize uncontrolled robot state
robot_uncontrolled = struct();
robot_uncontrolled.x = zeros(num_steps, 1);
robot_uncontrolled.y = zeros(num_steps, 1);
robot_uncontrolled.theta = zeros(num_steps, 1);
robot_uncontrolled.speed = 0.3; % Constant speed
robot_uncontrolled.error = zeros(num_steps, 1);

% Set initial position
robot_uncontrolled.x(1) = track_x(1);
robot_uncontrolled.y(1) = track_y(1);
robot_uncontrolled.theta(1) = atan2(track_y(2) - track_y(1), track_x(2) - track_x(1));

% Simulate uncontrolled robot motion
for k = 2:num_steps
    % Find closest point on track
    distances = sqrt((track_x - robot_uncontrolled.x(k-1)).^2 + ...
                    (track_y - robot_uncontrolled.y(k-1)).^2);
    [~, closest_idx] = min(distances);
    
    % Calculate sensor readings
    line_y = track_y(closest_idx);
    sensor_readings = simulate_ir_sensors(robot_uncontrolled.y(k-1), line_y, sensor_params);
    
    % Calculate error (but don't use for control)
    robot_uncontrolled.error(k-1) = calculate_line_error(sensor_readings);
    
    % Move robot with no steering correction (straight motion)
    robot_uncontrolled.x(k) = robot_uncontrolled.x(k-1) + ...
        robot_uncontrolled.speed * cos(robot_uncontrolled.theta(k-1)) * dt;
    robot_uncontrolled.y(k) = robot_uncontrolled.y(k-1) + ...
        robot_uncontrolled.speed * sin(robot_uncontrolled.theta(k-1)) * dt;
    robot_uncontrolled.theta(k) = robot_uncontrolled.theta(k-1); % No steering
end

%% CONTROLLED ROBOT SIMULATION (WITH PID)
fprintf('\n=== PID CONTROLLED ROBOT SIMULATION ===\n');

% Initialize controlled robot state
robot_controlled = struct();
robot_controlled.x = zeros(num_steps, 1);
robot_controlled.y = zeros(num_steps, 1);  
robot_controlled.theta = zeros(num_steps, 1);
robot_controlled.speed = 0.3;
robot_controlled.error = zeros(num_steps, 1);
robot_controlled.control_output = zeros(num_steps, 1);
robot_controlled.integral_error = 0;
robot_controlled.prev_error = 0;

% Set initial position
robot_controlled.x(1) = track_x(1);
robot_controlled.y(1) = track_y(1);
robot_controlled.theta(1) = atan2(track_y(2) - track_y(1), track_x(2) - track_x(1));

% Simulate PID controlled robot motion
for k = 2:num_steps
    % Find closest point on track
    distances = sqrt((track_x - robot_controlled.x(k-1)).^2 + ...
                    (track_y - robot_controlled.y(k-1)).^2);
    [~, closest_idx] = min(distances);
    
    % Calculate sensor readings
    line_y = track_y(closest_idx);
    sensor_readings = simulate_ir_sensors(robot_controlled.y(k-1), line_y, sensor_params);
    
    % Calculate error
    current_error = calculate_line_error(sensor_readings);
    robot_controlled.error(k-1) = current_error;
    
    % PID Control calculation
    proportional = pid_params.Kp * current_error;
    robot_controlled.integral_error = robot_controlled.integral_error + current_error * dt;
    integral = pid_params.Ki * robot_controlled.integral_error;
    derivative = pid_params.Kd * (current_error - robot_controlled.prev_error) / dt;
    
    % Total control output
    control_output = proportional + integral + derivative;
    control_output = max(-pid_params.max_output, min(pid_params.max_output, control_output));
    robot_controlled.control_output(k-1) = control_output;
    
    % Convert control output to steering angle
    steering_angle = control_output * 0.01; % Scale factor
    
    % Update robot position with steering
    robot_controlled.x(k) = robot_controlled.x(k-1) + ...
        robot_controlled.speed * cos(robot_controlled.theta(k-1)) * dt;
    robot_controlled.y(k) = robot_controlled.y(k-1) + ...
        robot_controlled.speed * sin(robot_controlled.theta(k-1)) * dt;
    robot_controlled.theta(k) = robot_controlled.theta(k-1) + ...
        (robot_controlled.speed / robot_params.wheelbase) * tan(steering_angle) * dt;
    
    % Update previous error
    robot_controlled.prev_error = current_error;
end

%% PERFORMANCE ANALYSIS
% Calculate performance metrics
uncontrolled_deviation = mean(abs(robot_uncontrolled.error));
controlled_deviation = mean(abs(robot_controlled.error));
improvement_factor = uncontrolled_deviation / controlled_deviation;

fprintf('\nPerformance Analysis:\n');
fprintf('Average deviation (Uncontrolled): %.4f units\n', uncontrolled_deviation);
fprintf('Average deviation (PID Controlled): %.4f units\n', controlled_deviation);
fprintf('Improvement Factor: %.1fx better\n', improvement_factor);

% Calculate RMS error
rms_uncontrolled = sqrt(mean(robot_uncontrolled.error.^2));
rms_controlled = sqrt(mean(robot_controlled.error.^2));
fprintf('RMS Error (Uncontrolled): %.4f\n', rms_uncontrolled);
fprintf('RMS Error (PID Controlled): %.4f\n', rms_controlled);

%% VISUALIZATION
figure('Position', [100, 100, 1200, 800]);

% Track and robot paths
subplot(2,2,1);
plot(track_x, track_y, 'k-', 'LineWidth', 3, 'DisplayName', 'Reference Track');
hold on;
plot(robot_uncontrolled.x, robot_uncontrolled.y, 'r--', 'LineWidth', 2, ...
     'DisplayName', 'Uncontrolled Robot');
plot(robot_controlled.x, robot_controlled.y, 'b-', 'LineWidth', 2, ...
     'DisplayName', 'PID Controlled Robot');
plot(robot_controlled.x(1), robot_controlled.y(1), 'go', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(robot_controlled.x(end), robot_controlled.y(end), 'ro', 'MarkerSize', 10, ...
     'MarkerFaceColor', 'r', 'DisplayName', 'End');
grid on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Robot Path Comparison');
legend('Location', 'best');
axis equal;

% Error comparison
subplot(2,2,2);
plot(t_sim(1:end-1), robot_uncontrolled.error(1:end-1), 'r-', 'LineWidth', 1.5, ...
     'DisplayName', 'Uncontrolled Error');
hold on;
plot(t_sim(1:end-1), robot_controlled.error(1:end-1), 'b-', 'LineWidth', 1.5, ...
     'DisplayName', 'PID Controlled Error');
grid on;
xlabel('Time (s)');
ylabel('Line Following Error');
title('Error Comparison Over Time');
legend('Location', 'best');

% Control signal
subplot(2,2,3);
plot(t_sim(1:end-1), robot_controlled.control_output(1:end-1), 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Output');
title('PID Controller Output');

% Step response analysis
subplot(2,2,4);
step(T_closedloop);
grid on;
title('Closed-Loop Step Response');

% Save results to workspace
fprintf('\n=== SIMULATION COMPLETED ===\n');
fprintf('Results saved to workspace variables:\n');
fprintf('- robot_controlled: PID controlled robot data\n'); 
fprintf('- robot_uncontrolled: Uncontrolled robot data\n');
fprintf('- reference_path: Track waypoints\n');
fprintf('- pid_controller: PID controller object\n');

%% ADDITIONAL ANALYSIS FUNCTIONS
function analyze_frequency_response()
    % Frequency response analysis
    figure('Name', 'Frequency Response Analysis');
    
    subplot(2,1,1);
    bode(G_plant);
    title('Plant Frequency Response');
    grid on;
    
    subplot(2,1,2);
    bode(T_closedloop);
    title('Closed-Loop Frequency Response');
    grid on;
end

function analyze_stability_margins()
    % Stability margin analysis
    [Gm, Pm, Wcg, Wcp] = margin(pid_controller * G_plant);
    
    fprintf('\nStability Analysis:\n');
    fprintf('Gain Margin: %.1f dB at %.2f rad/s\n', 20*log10(Gm), Wcg);
    fprintf('Phase Margin: %.1f° at %.2f rad/s\n', Pm, Wcp);
    
    if Pm >= 45
        fprintf('✓ System is stable with good phase margin\n');
    else
        fprintf('⚠ Warning: Low phase margin, system may be unstable\n');
    end
end

% Call additional analysis
analyze_frequency_response();
analyze_stability_margins();

fprintf('\n✓ MATLAB Implementation Complete!\n');
fprintf('Ready for Simulink integration and web app development.\n');
