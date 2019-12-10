% Function for reading NGSIM data in native format

% Native Data Format:
% Column 1: Vehicle ID 
    % Vehicle identification number (ascending by time of entry into
    % section)
% Column 2: Frame ID (1/10 of a second)
    % Frame Identification number (ascending by start time)
% Column 3: Total Frames (1/10 of a second)
    % Total number of frames in which the vehicle appears in this data set
% Column 4: Global Time (milleseconds)
    % Elapsed time since Jan 1, 1970 in milliseconds
% Column 5: Local X (feet)
    % Lateral (X) coordinate of the front center of the vehicle with
    % respect to the left-most edge of the section in the direction of
    % travel
% Column 6: Local Y (feet)
    % Longitudinal (Y) coordinate of the front center of the vehicle with
    % respect to the entry edge of the section in the direction of travel
% Column 7: Global X (feet)
    % X Coordinate of the front center of the vehicle
% Column 8: Global Y (feet)
    % Y Coordinate of the front center of the vehicle based
% Column 9: Vehicle Length (feet)
% Column 10: Vehicle Width (feet)
% Column 11: Vehicle Class
    % Vehicle type: 1 - motorcycle, 2 - auto, 3 - truck
% Column 12: Vehicle Velocity (feet/second)
    % Instantaneous velocity of vehicle
% Column 13: Vehicle Acceleration (feet/second^2)
    % Instantaneous acceleration of vehicle
% Column 14: Lane Identification
    % Current lane position of vehicle. Lane 1 is farthest left lane; lane 6
    % is farthest right lane. Lane 7 is the on-ramp at Powell Street, and
    % Lane 9 is the shoulder on the right-side.
% Column 15: Preceding Vehicle
    % Vehicle Id of the lead vehicle in the same lane. A value of '0'
    % represents no preceding vehicle - occurs at the end of the study
    % section and off-ramp due to the fact that only complete trajectories
    % were recorded by this data collection effort (vehicles already in the
    % section at the start of the study period were not recorded).
% Column 16: Following Vehicle
    % Vehicle Id of the vehicle following the subject vehicle in the same
    % lane. A value of '0' represents no following vehicle - occurs at the
    % beginning of the study section and on-ramp due to the fact that only
    % complete trajectories were recorded by this data collection effort
    % (vehicle that did not traverse the downstream boundaries of the
    % section by the end of the study period were not recorded).
% Column 17: Spacing or Space Headway (feet)
    % The distance between the front-center of a vehicle to the
    % front-center of the preceding vehicle.
% Column 18: Headway or Time Headway (seconds)
    % The time to travel from the front-center of a vehicle (at the speed
    % of the vehicle) to the front-center of the preceding vehicle. A
    % headway value of 9999.99 means that the vehicle is traveling at zero
    % speed (congested conditions).

% Output files: 
    % output: an array containing 2 structures - one for trajectory data,
    % and another for vehicle characteristic data, for each vehicle
    % vehicle_char: an array of vehicle characteristic data (each vehicle
    % on a separate row)
    % runtime: total time in seconds required for calculations
    
% Output file formats:

% output(vehicle_id).data: 
    % Header: 1 = timestamp, 2 = local x, 3 = local y, 4 = velocity, 5 =
    % acceleration, 6 = # preceding vehicle ID, 7 = space headway, 8 = time
    % headway, 9 = lane id
% vehicle_char: 
    % Header: 1 = vehicle id, 2 = class, 3 = starting lane, 
    % 4 = # data points, 5 = vehicle length, 6 = vehicle width
    
function [output,vehicle_char,runtime] = read_ngsim(filename)

% read the text file in its native format
t_start1 = tic;
C = dlmread(filename, '');
t_elapsed1 = toc(t_start1);

% Only columns 1,2,3,5,6,9,10,11,14,15,18 should be kept

temp_C = [C(:,1:3),C(:,5:6),C(:,9:15),C(:,17:18)];
% There are now 12 columns:
% 1 = Vehicle ID, 2 = Timestamp, 3 = # of Timestamps, 4 = Local X, 5 =
% Local Y, 6 = Vehicle Length, 7 = Vehicle Width, 8 = Vehicle Class, 
% 9 = Velocity, 10 = Acceleration, 11 = Lane ID, 12 = Preceding Vehicle, 
% 13 = Space Headway, 14 = Time Headway

% lane_list = unique(temp_C(:,9)); % list of unique lane IDs

vehicle_list = unique(temp_C(:,1)); % list of unique vehicle IDs
vehicle_char = zeros(size(vehicle_list,1),6); % preallocate for vehicle characteristic data

% clear the original variable to free up memory
clear C

i = 1;

for k = 1:size(vehicle_list,1) % for each vehicle in the dataset
    
    % while the current vehicle is in the vehicle list
    vehicle_data = [temp_C(i:i+temp_C(i,3)-1,2) temp_C(i:i+temp_C(i,3)-1,4:5) temp_C(i:i+temp_C(i,3)-1,9:10) temp_C(i:i+temp_C(i,3)-1,12:14) temp_C(i:i+temp_C(i,3)-1,11)];
    % Header: 1 = timestamp, 2 = local x, 3 = local y, 4 = velocity, 5 =
    % acceleration, 6 = # preceding vehicle ID, 7 = space headway, 8 = time
    % headway, 9 = lane id
        
    vehicle_char(k,:) = [temp_C(i,1),temp_C(i,8),temp_C(i,11),temp_C(i,3),temp_C(i,6),temp_C(i,7)]; 
    % Header: 1 = vehicle id, 2 = class, 3 = starting lane, 
    % 4 = # data points, 5 = vehicle length, 6 = vehicle width
    
    i = i+temp_C(i,3); % move to the next row in the dataset
    
    output_vehicle_data(k,1).data(:,:) = vehicle_data;
    output_vehicle_data(k,1).vehicle_char(:,:) = vehicle_char(k,:);
    clear vehicle_data
end

output = output_vehicle_data;

runtime = t_elapsed1;

