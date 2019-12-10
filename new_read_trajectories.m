% This function reads vehicle trajectories from a pre-formatted data file
% (Dr. Zhou's data format in NEXTA), and outputs formatted tables with
% trajectory data for each vehicle.

% input: file_name = CSV or Excel file from which to read trajectory data

% temporary variables: temp_data = raw dataset read directly from file
%                      vehicle_counter = counter which identifies the
%                                        number of vehicles in the dataset
%                      data_points = counter which identifies the number of
%                                    data points available for a single vehicle 

% output:   traj = structure containing the raw vehicle trajectory data for
%                  each vehicle (1 = time stamp, 2 = local y, 3 = calculated_speed,
%                  4 = speed, 5 = acceleration, 6 = preceding_vehicle_id,
%                  7 = preced_y, 8 = preced_calculated_speed, 9 = preced_speed,
%                  10 = preced_accel, 11 = follow_id, 12 = follow_y,
%                  13 = follow_calculated_speed, 14 = follow_speed, 15 = follow_accel
%           vehicle_char = structure containing the vehicle characteristics
%                          for each vehicle (1 = vehicle ID, 2 = vehicle
%                          type, 3 = starting lane, 4 = number of data
%                          points in dataset)

% Potential improvement: use segments to divide data (check raw_filter_mod
% for example)

function [traj, vehicle_char] = new_read_trajectories(file_name)

temp_data = xlsread(file_name); % read Excel or CSV file into temporary variable
vehicle_counter = 0; % initialize vehicle counter

data_points = 0; % initialize data point counter
    
for i = 1:size(temp_data,1) % for each data point in the tabular file
    % first row has vehicle ID in it, and is only present when data for a
    % new vehicle starts
    if temp_data(i,1) > 0 % if data for new vehicle starts
        vehicle_counter = vehicle_counter+1; % move the vehicle counter forward
        
        % store initial vehicle characteristics for the current vehicle
        vehicle_char(vehicle_counter,1:3) = temp_data(i,1:3); % vehicle_id
        
        
        if i>1 % if it is not the first vehicle in the list
            % identify the number of data points for the previous vehicle
            vehicle_char(vehicle_counter-1,4) = data_points-1;
            % store the trajectory data for each vehicle in a new array for output
            traj(vehicle_counter-1,1).data(1:vehicle_char(vehicle_counter-1,4),1:15) = temp_data(i-vehicle_char(vehicle_counter-1,4)-1:i-2,4:18); 

            data_points = 0; % reset the counter which identifies the number of data points
        end
    end
    % if we don't start looking at a new vehicle, continue counting the
    % number of data points in the data for this vehicle
    data_points = data_points+1;
end
% catch the last vehicle in the dataset
vehicle_char(vehicle_counter,4) = data_points;
traj(vehicle_counter,1).data(1:vehicle_char(vehicle_counter,4),1:15) = temp_data(i-vehicle_char(vehicle_counter,4)+1:i,4:18);