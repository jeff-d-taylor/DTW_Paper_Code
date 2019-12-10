% Simplified, modular DTW function

% inputs: raw time-series data for leader and follower, prior estimates for
% cost function, command line arguments (type of cost function)

% prior estimates: prior speed difference, prior t, prior d, prior w, weight coefficients in cost function

% Step 1: Calculate cost matrix
% Step 2: Calculate cumulative cost matrix
% Step 3: Produce the warp path
% Step 4: Calculate time-dependent model parameters

% outputs: 
%   model parameters:  time-dependent model parameter estimates
%          tao = calculated time headway between vehicles at a potential
%          match point in seconds
%          dist = calculated distance between vehicles at a potential match
%          point in feet
%          wave = calculated wave speed at a potential match point in MPH
%   path: warp path, which provides the estimated match points between
%          leader and follower
%   cost and cumulative cost matrices


function [model_parameters,path,cost,cumulative] = modular_DTW(leader_data,follower_data,prior_estimates,prior_info,one_to_one_match,match_term)
% calculate the number of timesteps in the leader and follower's trajectory,
% to be used in the cost matrix and cumulative cost matrix calculations
data_points(1) = size(leader_data,1);
data_points(2) = size(follower_data,1);

if match_term == 0 % use speed data
    match_data = 4;
else % use acceleration data
    match_data = 5;
end
    
% Step 1: Calculate cost matrix (using time and distance constraints)
if prior_info == 0
    % cost_matrix_constain function inputs: (leader time, follower
    % time, leader position, follower position, leader comparison
    % value, follower comparison value, matrix size)
    cost = cost_matrix_constrain(leader_data(:,1),follower_data(:,1),leader_data(:,3),follower_data(:,3),leader_data(:,match_data),follower_data(:,match_data),data_points);
    % output is the cost matrix
else
    % cost_matrix_multi_prior function inputs: (leader time,
    % follower time, leader position, follower position, leader
    % comparison value, follower comparison value, matrix size,
    % prior speed difference, prior w, prior t, prior d, prior w, weight coefficients in cost function)
    cost = quick_cost_matrix_multi_prior(leader_data(:,1),follower_data(:,1),leader_data(:,3),follower_data(:,3),leader_data(:,match_data),follower_data(:,match_data),data_points,prior_estimates);
    % output is the cost matrix, the individual terms of the cost
    % matrix, the time lag (in seconds), the distance headway (in
    % feet), and the wave speed (in MPH) (cost1,t,d,w are optional, using ~
    % to remove them from outputs speeds up implementation)
end

% calculate the cumulative cost matrix
cumulative = quick_cumulative_cost_function(cost,data_points);

% find the shortest path
if one_to_one_match == 0
    % shortest path function inputs: (cumulative cost matrix, size
    % of leader and follower matrices)
    path = short_path_function(cumulative,data_points);
else
    % one-to-one matching function inputs: (cumulative cost matrix,
    % size of leader and follower matrices)
    path = one_match(cumulative,data_points);
    % doesn't allow singularities for the follower (many leader
    % positions can be mapped to one follower position, not vice versa)
end

% calculate the time lag (tao), critical spacing (distance), and
% wave speed (wave) for the match solution
% function inputs: (leader time stamps, follower time stamps, leader
% position, follower position, warp path)
[tao, dist, wave] = follow_parameters(leader_data(:,1),follower_data(:,1),leader_data(:,3),follower_data(:,3),path);
model_parameters = [tao,dist,wave];