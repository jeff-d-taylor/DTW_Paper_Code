% Analysis script (with optional reduced data introduced)

load_trajectory_data = input('Load trajectory data? [Y=1 (default), N=0] ');

if isempty(load_trajectory_data) % Use if you've already loaded data
    load_trajectory_data = 1; % default is to load trajectory data
elseif load_trajectory_data ~= 1 && load_trajectory_data ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end
if load_trajectory_data == 1 % Use the GUI to grab the input file information
    [input_file_name,input_path_name,~] = uigetfile({'*.txt;*.csv;*.mat','Text, CSV, or MAT (*.txt, *.csv, *.mat)'},'Select NGSIM Text File to be Read');
    if isequal(input_file_name,0) % error checking on GUI
        error('App:no_input', 'No Input File Identified')
    end
    file_ID = fullfile(input_path_name,input_file_name); % rebuild the full file name for reading
    % store the file extension to determine how to handle the input file
    [~, ~, ext] = fileparts(file_ID); % use ~ to ignore outputs from function
    
    % Check the input file type to decide which operation to perform
    fprintf('Reading input data...\n');
    if strcmp(ext, '.csv') % if CSV file is chosen (Dr. Zhou's format)
        tic
        [traj,vehicle_char] = new_read_trajectories(file_ID); % read trajectories from file
        toc
    elseif (strcmp(ext, '.mat')) % if MAT file, just load the trajectories
        load(file_ID);
    elseif (strcmp(ext, '.txt')) % text file assumes data is in NGSIM format
        tic
        [traj,vehicle_char,~] = read_ngsim(file_ID); % Using new raw NGSIM reading functionality
        toc
    else % error checking
        error('App:extension_error', 'Unsupported file extension')
    end
    
    % if reading a new dataset, save the formatted data so we don't have to read it again
    if (strcmp(ext, '.txt') || strcmp(ext, '.csv'))
        % Use the GUI to specify the output file
        [output_file_name,output_path_name,~] = uiputfile('*.mat','Save MAT output file');
        if isequal(output_file_name,0) % Perform some minor error checking on user inputs
            error('App:output_error', 'No output file to be created')
        end
        output_fileID = fullfile(output_path_name,output_file_name); % build output file name
        save(output_fileID, 'traj', 'vehicle_char'); % and save to MAT file
        fprintf('%s saved for future analysis\n', output_file_name); % provide message to user
    end
    clear ext;
end

% Ask about how many trajectories to run
long_run = input('Calculate for more than 1 trajectory? [Y=1, N=0 (default)] ');
if isempty(long_run)
    long_run = 0; % default is to run only one trajectory pair
elseif long_run ~= 1 && long_run ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

% Ask whether to print all of the trajectories in a figure
print_opt = input('Print all trajectories? [Y=1, N=0 (default)] ');
if isempty(print_opt)
    print_opt = 0; % default is to not print all trajectories
elseif print_opt ~= 1 && print_opt ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

% Ask whether to print the trajectory match for the trajectories to be printed
second_plot = input('Plot trajectory match? [Y=1, N=0 (default)] ');
if isempty(second_plot)
    second_plot = 0; % default is to not plot the matching points
elseif second_plot ~= 1 && second_plot ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

% to do: replace data reduction algorithm
reduction = input('Use data reduction algorithm? (Not recommended) [Y=1, N=0 (default)] ');
if isempty(reduction)
    reduction = 0; % default is to not use the data reduction algorithm
elseif reduction ~= 1 && reduction ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

one_to_one_match = input('Use one-to-one DTW path algorithm? (Not recommended) [Y=1, N=0 (default)] ');
if isempty(one_to_one_match)
    one_to_one_match = 0; % default is to not use one-to-one matching
elseif one_to_one_match ~= 1 && one_to_one_match ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

prior_info = input('Use prior information algorithm? (Recommended) [Y=1 (default), N=0] ');
if isempty(prior_info)
    prior_info = 1; % default is to use the prior information algorithm
elseif prior_info ~= 1 && prior_info ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

match_term = input('Use speed or acceleration data? (Speed recommended) [Acceleration=1, Speed=0 (default)] ');
if isempty(match_term)
    match_term = 0; % default is to use the speed data becuase acceleration data is too noisy
elseif match_term ~= 1 && match_term ~= 0
    error('App:input_error', 'Value must be 1 or 0')
end

if long_run == 1
    start_point = 1; end_point = size(traj,1);
    if size(vehicle_char,1) > 800
        fprintf('There are %i vehicles in the dataset\n',size(vehicle_char,1));
        fprintf('Memory limitations could be an issue. Recommend using smaller dataset.\n');
        start_point = input('Starting vehicle trajectory to analyze? (By index number in the dataset) ');
        if isempty(start_point) % error handling
            error('App:emptyinput', 'Did not identify starting vehicle number to analyze')
        end
        end_point = start_point + input('How many vehicles to analyze? ') - 1;
    end
else % if only one vehicle trajectory pair is to be run
    % Ask which vehicle to use for trajectory analysis
    fprintf('There are %i vehicles in the dataset\n',size(vehicle_char,1));
    vehicle_ID = input('Which vehicle trajectory to analyze? (By index number in the dataset) ');
    start_point = vehicle_ID; end_point = vehicle_ID;
    if isempty(vehicle_ID) % error handling
        error('App:emptyinput', 'Did not identify vehicle number to analyze')
    elseif vehicle_ID > size(vehicle_char,1)
        error('App:input_error', 'Value must refer to a vehicle in the dataset')
    end
end

unusable_list = 0;
tic % start keeping track of how long it takes to run

for vehicle_ID = start_point:end_point
    
    if mod(vehicle_ID,100) == 0
        fprintf('%d vehicles processed\n',vehicle_ID);
    end
    % if vehicle_ID == 557
    %    test = 0;
    % end
    
    % detect when leader changes lanes or the leader id changes
    traj(vehicle_ID,1).trajectory_attribute_data = detect_trajectory_change(traj(vehicle_ID,1).data);
    % assemble trajectory segments according to lane change and leader
    % change, building both follower and leader trajectory data
    [traj(vehicle_ID,1).follower_data, traj(vehicle_ID,1).leader_data, traj(vehicle_ID,1).unused_follower_data] = build_trajectory_segments(traj, vehicle_ID, vehicle_char, 20);
    
    if isempty(traj(vehicle_ID,1).follower_data)
        fprintf('No data available for vehicle %d\n', vehicle_ID);
        if unusable_list == 0
            unusable_list = vehicle_ID;
        else
            unusable_list = [unusable_list, vehicle_ID];
        end
        continue
    end
    
    for segment_id = 1:size(traj(vehicle_ID,1).follower_data,1)% for each segment within the vehicle trajectory dataset
        
        % Create temporary trajectory data variables with which to perform analysis
        
        % To Do: new piecewise algorithm based on stepwise regression?
        % ridge regression?
        
        % if data reduction method is requested by user (experimental, needs fixing)
        if reduction == 1 % replace trajectory data with linear-interpolated data
            traj(vehicle_ID,1).follower_data(segment_id,1).data = piecewise(traj(vehicle_ID,1).follower_data(segment_id,1).data);
            traj(vehicle_ID,1).leader_data(segment_id,1).data = piecewise(traj(vehicle_ID,1).leader_data(segment_id,1).data);
        end
        
        % calculate the number of timesteps in the leader and follower's
        % trajectory, to be used in the cost matrix and cumulative cost
        % matrix calculations
        data_points(1) = size(traj(vehicle_ID,1).leader_data,1);
        data_points(2) = size(traj(vehicle_ID,1).follower_data,1);
        
        % parameter_estimates(1:8) = [0,1.6,27,12,1,0.2,0.2,0.2]; % original estimates
        
        % assuming kjam = 112 vehicles/km/lane
        % prior_w = -19; % km/hr
        % prior_t = 1.7; % seconds
        % prior_d = -9; % meters
        
        % old weights: a = 0.8; b = 0.2; c = 0.105; e = 0.095;
        % parameter_estimates(1) = prior speed difference = prior estimate for the speed difference, in feet/second
        % parameter_estimates(2) = prior_t = prior estimate for the time lag or time headway, in seconds
        % parameter_estimates(3) = prior_d = prior estimate for the distance headway, in feet
        % parameter_estimates(4) = prior_w = prior estimate for the wave speed, in MPH
        % parameter_estimates(5) = a = coefficient for weight on comparison value
        % parameter_estimates(6) = b = coefficient for weight on time lag
        % parameter_estimates(7) = c = coefficient for weight on distance headway
        % parameter_estimates(8) = e = coefficient for weight on wave speed
        
        if vehicle_ID == 962
            if segment_id == 2
                test = 1;
            end
        end
        
        % estimate parameters (prior estimates) for Newell's model
%        traj(vehicle_ID,1).follower_data(segment_id,1).estimated_parameters = newell_calibration(traj(vehicle_ID,1).leader_data(segment_id,1).data,traj(vehicle_ID,1).follower_data(segment_id,1).data,5);
        traj(vehicle_ID,1).follower_data(segment_id,1).estimated_parameters = [0, 0, 0, 0];
        parameter_estimates(1,1) = 0; % speed difference estimate
        parameter_estimates(1,2) = traj(vehicle_ID,1).follower_data(segment_id,1).estimated_parameters(1,2); % time lag estimate
        parameter_estimates(1,3) = traj(vehicle_ID,1).follower_data(segment_id,1).estimated_parameters(1,3); % jam spacing estimate
        parameter_estimates(1,4) = traj(vehicle_ID,1).follower_data(segment_id,1).estimated_parameters(1,4); % wave speed estimate
        parameter_estimates(1,5:8) = 1; % initial weight estimate
        
        % start counting iterations
        iteration_number = 1;
        % initiate difference calculations before check
        difference = [1,1,1,1];
        % [time_lag, distance_headway, wave_speed, speed_difference]
        
        
        speed_difference = 0;
        
        while (difference(iteration_number,1) > threshold || difference(iteration_number,2) > threshold || difference(iteration_number,3) > threshold || difference(iteration_number,4) > threshold)
            
            [model_parameters,path,~,~] = modular_DTW(traj(vehicle_ID,1).leader_data(segment_id,1).data,traj(vehicle_ID,1).follower_data(segment_id,1).data,parameter_estimates(iteration_number,:),prior_info,one_to_one_match,match_term);
            iteration_number = iteration_number + 1;
            
            last_mean_speed_difference = mean(speed_difference);
            % Replace infinite wave speed values with the value in the last time step
            speed_difference = zeros(size(model_parameters,1),1);
            for i = 1:size(model_parameters,1)
                if (model_parameters(i,3) == -Inf)
                    if i == 1
                        model_parameters(i,3) = model_parameters(i+1,3);
                    else
                        model_parameters(i,3) = model_parameters(i-1,3);
                    end
                end
                % Calculate speed difference
                speed_difference(i,1) = traj(vehicle_ID,1).leader_data(segment_id,1).data(path.x(i,1),4)-traj(vehicle_ID,1).follower_data(segment_id,1).data(path.y(i,1),4);
            end
            
            % Calculate new average values for parameters
            parameter_estimates(iteration_number,1:4)= [0,mean(model_parameters(:,1)),mean(model_parameters(:,2)),mean(model_parameters(:,3))];
            mean_speed_difference(iteration_number,1) = mean(speed_difference);
            % Calculate new weights based on the inverse of the standard deviation
            parameter_estimates(iteration_number,5:8) = [1/std(speed_difference),1/std(model_parameters(:,1)),1/std(model_parameters(:,2)),1/std(model_parameters(:,3))];
            
            % Calculate change in estimates
            difference(iteration_number,1) = abs((mean(speed_difference)-last_mean_speed_difference)/last_mean_speed_difference);
            difference(iteration_number,2) = abs((parameter_estimates(iteration_number,2)-parameter_estimates(iteration_number-1,2))/parameter_estimates(iteration_number-1,2));
            difference(iteration_number,3) = abs((parameter_estimates(iteration_number,3)-parameter_estimates(iteration_number-1,3))/parameter_estimates(iteration_number-1,3));
            difference(iteration_number,4) = abs((parameter_estimates(iteration_number,4)-parameter_estimates(iteration_number-1,4))/parameter_estimates(iteration_number-1,4));
            
            % if the code gets stuck in a loop, take the average difference
            % between the stuck values, and use these estimates as the final estimates
            if (iteration_number > 5  && sum(parameter_estimates(iteration_number,:)-parameter_estimates(iteration_number-2,:)) == 0)
                parameter_estimates(iteration_number,1:4) = 0.5*(parameter_estimates(iteration_number,1:4) + parameter_estimates(iteration_number-1,1:4));
                break
            elseif (iteration_number > 5  && sum(parameter_estimates(iteration_number,:)-parameter_estimates(iteration_number-3,:)) == 0)
                parameter_estimates(iteration_number,1:4) = (1/3)*(parameter_estimates(iteration_number,1:4) + parameter_estimates(iteration_number-1,1:4)+parameter_estimates(iteration_number-2,1:4));
                break
            end
            
        end
        if iteration_number > 50
            fprintf('Vehicle %d more than 50 iterations\n',vehicle_ID);
        end
        traj(vehicle_ID,1).follower_data(segment_id,1).model_inputs = parameter_estimates;
        traj(vehicle_ID,1).follower_data(segment_id,1).mean_speed_difference = mean_speed_difference;
        traj(vehicle_ID,1).follower_data(segment_id,1).adjustments = difference;
        
        [model_parameters,path,cost,cumulative_cost] = modular_DTW(traj(vehicle_ID,1).leader_data(segment_id,1).data,traj(vehicle_ID,1).follower_data(segment_id,1).data,parameter_estimates(iteration_number,:),prior_info,one_to_one_match,match_term);
        
        clear parameter_estimates mean_speed_difference; 
        
        % plot the matching trajectories
        if second_plot == 1
            plot(traj(vehicle_ID,1).leader_data(segment_id,1).data(:,1),traj(vehicle_ID,1).leader_data(segment_id,1).data(:,3),'k'); hold on
            % function inputs: (leader time stamps, follower time stamps, leader
            % position, follower position, warp path, time lag, critical
            % spacing, and cost matrix)
            match(segment_id) = match_plot(traj(vehicle_ID,1).follower_data(segment_id,1).data,traj(vehicle_ID,1).leader_data(segment_id,1).data,path,model_parameters(:,1),model_parameters(:,2),cost);
            hold on
        end
        
        % Aggregate and store the results, divided into each segment
        %         if start_point == end_point
        %             vehicle_ID = 1;
        %         end
        traj(vehicle_ID,1).follower_data(segment_id,1).path_x = path.x;
        traj(vehicle_ID,1).follower_data(segment_id,1).path_y = path.y;
        traj(vehicle_ID,1).follower_data(segment_id,1).tao = model_parameters(:,1);
        traj(vehicle_ID,1).follower_data(segment_id,1).dist = model_parameters(:,2);
        traj(vehicle_ID,1).follower_data(segment_id,1).wave = model_parameters(:,3);
        %traj(vehicle_ID,1).follower_data(segment_id,1).cost = cost;
        %traj(vehicle_ID,1).follower_data(segment_id,1).cumulative_cost = cumulative_cost;
        
    end
end

% Print all trajectories
if print_opt == 1
    if long_run < 1
        figure('Name', 'All Trajectories');
    end
    for j = 1:size(traj,2)
        if vehicle_char(j,2) == 3
            plot_color = 'r'; % plot trucks in red
        else
            plot_color = 'g'; % plot other vehicles in green
        end
        plot(traj(j,1).data(:,1),traj(j,1).data(:,2),plot_color);
        hold on
        
    end
end

% Convert the dtw results back out into a more useable format
% for m = 1:size(data,2)
%     [new_data(1,m).follow, new_data(1,m).leader] = trajectory_data_reformat(data(1,m).follow,data(1,m).leader);
%     new_data(1,m).aggregate = data_reformat(data(1,m).aggregate);
%     if size(new_data(1,m).aggregate,1) == 1
%         continue
%     else
%         for n = 1:size(new_data(1,m).aggregate,1)
%             new_data(1,m).aggregate(n,6:9) = new_data(1,m).follow(new_data(1,m).aggregate(n,1),:);
%             new_data(1,m).aggregate(n,10:13) = new_data(1,m).leader(new_data(1,m).aggregate(n,2),:);
%         end
%         count = 0;
%         for p = 1:size(new_data(1,m).aggregate,1)-1
%             if new_data(1,m).aggregate(p,6) ~= new_data(1,m).aggregate(p+1,6)
%                 count = count + 1;
%                 final_data(1,m).aggregate(count,:) = new_data(1,m).aggregate(p,:);
%             end
%             if p == size(new_data(1,m).aggregate,1)-1
%                 count = count+1;
%                 final_data(1,m).aggregate(count,:) = new_data(1,m).aggregate(p+1,:);
%             end
%         end
%     end
% end

toc
