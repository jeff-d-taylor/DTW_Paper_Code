% This function divides trajectory data based on when the leader changes or
% when the change lanes.

% input: traj = all trajectory data for the dataset
%        vehicle_ID = vehicle number (referring to index number)
%        vehicle_char = trajectory data characteristics for different
%                       combinations of leader and follower
%        min_length = minimum number of time stamps (in 1/10 second)
%                     required to keep trajectory data and label it as usable

% temporary variables: 

% output: follower_data
%         leader_data

function  [follower_data, leader_data, unused_follow_data] = build_trajectory_segments(trajectory_data, vehicle_ID, vehicle_char, min_length)

segment_counter = 0;
unused_data_counter = 0;

for i = 1:size(trajectory_data(vehicle_ID).trajectory_attribute_data,1)
    % if there is a leader, and if the trajectory is longer than 1 time stamp
    if trajectory_data(vehicle_ID).trajectory_attribute_data(i,4) ~= 0 && ((trajectory_data(vehicle_ID).trajectory_attribute_data(i,3) - trajectory_data(vehicle_ID).trajectory_attribute_data(i,2)) > min_length)
        segment_counter = segment_counter + 1;
        % add that segment of the follower's trajectory to the dataset
        if segment_counter == 1 % if it is the first dataset, create the matrix variable
            follower_data(segment_counter,1).data = trajectory_data(vehicle_ID).data(trajectory_data(vehicle_ID).trajectory_attribute_data(i,2):trajectory_data(vehicle_ID).trajectory_attribute_data(i,3),:);
            % keep track of the start time and end time of the follower's trajectory segment
            % start_time = follower_data(segment_counter,1).data(1,1);
            end_time = follower_data(segment_counter,1).data(size(follower_data(segment_counter,1).data,1),1);
            start_position = follower_data(segment_counter,1).data(1,3);
            % end_position = follower_data(segment_counter,1).data(size(follower_data(segment_counter,1).data,1),3);
            
            % find the vehicle ID number for the leader in the attribute data
            [leader_id,~] = find(vehicle_char(:,1) == trajectory_data(vehicle_ID,1).trajectory_attribute_data(i,4));
            % find the starting row where the trajectory positions are nearly the same
            [row.start,~] = find(trajectory_data(leader_id,1).data(:,3) < start_position);
            % also check for lane change in the leader's trajectory in that section
            [row.lane,~] = find(trajectory_data(leader_id,1).data(:,9) == follower_data.data(1,9));
            row.start = max(row.start); % identifies the time at which the position is closest
            row.lane = min(row.lane); % identifies the time at which the lanes are the same
            if isempty(row.start)
                row.start = 1;
            end
            row.start = max(row.lane,row.start); % use the best time period to start to new trajectory
            % find the last row where the trajectory time is nearly the same
            [row.end,~] = find(trajectory_data(leader_id,1).data(:,1) >= end_time);
            row.end = min(row.end);
            
            % construct the trajectory segment for the leader
            leader_data(segment_counter,1).data = trajectory_data(leader_id,1).data(row.start:row.end,:);
            
        else % if it isn't the first dataset, add the data to the matrix
            % start_index = size(follower_data(segment_counter-1,1).data,1) + 1;
            follower_data(segment_counter,1).data = trajectory_data(vehicle_ID,1).data(trajectory_data(vehicle_ID).trajectory_attribute_data(i,2):trajectory_data(vehicle_ID).trajectory_attribute_data(i,3),:);
            % start_time = follower_data(segment_counter,1).data(start_index,1);
            start_position = follower_data(segment_counter,1).data(1,3);
            end_time = follower_data(segment_counter,1).data(size(follower_data(segment_counter,1).data,1),1);
            % end_position = follower_data(segment_counter,1).data(size(follower_data(segment_counter,1).data,1),3);
            
            % find the vehicle ID number for the leader in the attribute data
            [leader_id,~] = find(vehicle_char(:,1) == trajectory_data(vehicle_ID).trajectory_attribute_data(i,4));
            % find the starting row, where the trajectory positions are nearly the same
            [row.start,~] = find(trajectory_data(leader_id,1).data(:,3) < start_position);
            row.start = max(row.start);
            if isempty(row.start)
                row.start = 1;
            end
            % find the last row where the trajectory time is nearly the same
            [row.end,~] = find(trajectory_data(leader_id,1).data(:,1) >= end_time);
            row.end = min(row.end);
            
            % construct the trajectory segment for the leader
            leader_data(segment_counter,1).data = trajectory_data(leader_id,1).data(row.start:row.end,:);
            
        end
    else
        unused_data_counter = unused_data_counter + 1;
        unused_follow_data(unused_data_counter,1).data = trajectory_data(vehicle_ID).data(trajectory_data(vehicle_ID).trajectory_attribute_data(i,2):trajectory_data(vehicle_ID).trajectory_attribute_data(i,3),:);
    end
end

if segment_counter == 0
    follower_data = [];
    leader_data = [];
end
if unused_data_counter == 0
    unused_follow_data = [];
end