% This version of the cost function calculates the cost matrix for matching
% all points in one time series to all points in another time series. It
% also employs an artificial constraint to make sure that impossible
% matches (where the time and spacing headways are not positive) are
% discouraged, where possible.

% input: leader's time series (timestamps, position, comparison value)
%        follower's time series (timestamps, position, comparison value)
%        lead_matrix = the column of data for which the leader is compared
%        follow_matrix = the column of data for which the follower is compared
%        matrix_size = number of time stamps in each vehicle's time series
%           matrix_size(1) is for leader, matrix_size(2) is for follower

% temporary variables: match = temporary calculations for time and distance
%   t = calculated time headway between vehicles at a potential match point
%       in seconds
%   d = calculated distance between vehicles at a potential match point in
%       feet

% output: cost matrix, of size [matrix_size(1) X matrix_size(2)]

function [cost] = cost_matrix_constrain(lead_time,follow_time,lead_pos,follow_pos,lead_matrix,follow_matrix,matrix_size)

% preallocation to improve speed
cost = zeros(matrix_size(1),matrix_size(2));

for i = 1:matrix_size(1) % for each leading vehicle
    for j = 1:matrix_size(2) % for each following vehicle
        % Store the leader's time and follower's time at each time step.
        match.time(1) = lead_time(i,1); match.time(2) = follow_time(j,1);
        % calculate the time lag between vehicles
        t = (match.time(2) - match.time(1))/10; % in sec
        % Store the leader's position and follower's position at each time step.
        match.dist(1) = lead_pos(i,1); match.dist(2) = follow_pos(j,1);
        % calculate the spacing between vehicles
        d = match.dist(2) - match.dist(1); % in feet, (should be negative)
        % enforce some constraints on time lag and distance:
        if t > 0 || d < 0 % time lag must be positive, spacing must be negative
            cost(i,j) = abs(lead_matrix(i,1) - follow_matrix(j,1));
        else
            % use an artificially high cost to encourage the algorithm to
            % avoid these solutions
            cost(i,j) = 100;
        end
    end
end