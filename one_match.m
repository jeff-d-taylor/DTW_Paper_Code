% This function finds the shortest path through the cumulative cost matrix,
% using one-to-many matching for the follower. In other words, many leader
% positions can be mapped to one follower location, but not vice versa.

% input: cumulative_cost = cumulative cost matrix
%        matrix size = number of time stamps in each vehicle's time series
%           matrix_size(1) is for leader, matrix_size(2) is for follower

% temporary variables: i = leader position index, j = follower position
%                      index, k = path index
%                      path = structure of leader and follower positions,
%                      from end to beginning

% output: path_sort = structure containing the leader(x) and follower(y)
%                positions for optimal matching, from beginning to end

function [path_sort] = one_match(cumulative_cost,matrix_size)

i = matrix_size(1); % number of timeseries points in leader's trajectory
% i is also used to keep track of the current location in the matrix
j = matrix_size(2);% number of timeseries points in follower's trajectory
% j is also used to keep track of the current location in the matrix
k = 1;% initialize the first step in the shortest path
% start building the shortest path from the end of the matrix, starting
% with the last points in the matrix
path.x(k) = matrix_size(1); % i and x = leader
path.y(k) = matrix_size(2); % j and y = follower

% In the cumulative cost matrix(i,j), the leader location stays the same
% within the row, and the follower location stays the same within the column

% Find shortest path
% one-to-one match requires always moving along the follower's path
% (changing the j value for each step along the path)

while i>1 && j>1 % as long as we don't reach the beginning of the matrix
    k = k+1; % move forward a step in the path
    % calculate the next step cost - the minimum cost to reach the next
    % point in the matrix, moving vertically, horizontally, or diagonally
    next_step_cost = min(min(cumulative_cost(i-1,j),cumulative_cost(i,j-1)),cumulative_cost(i-1,j-1));
    
    % check which step to take next, store the next step in the
    % shortest path, and increment to the next step location in i & j
    % (always move in the j direction to maintain one-to-many mapping)
    if next_step_cost == cumulative_cost(i-1,j-1)
         % if all steps cost the same, default step is diagonally
        path.x(k) = i-1; path.y(k) = j-1; i = i-1; j = j-1;
    elseif next_step_cost == cumulative_cost(i,j-1)
        path.x(k) = i; path.y(k) = j-1; j = j-1;
    else
        path.x(k) = i-1; path.y(k) = j; i = i-1; j = j-1;
    end
end
% resort the datasets to set the path from beginning to end
 path_sort.x = sort(path.x);  path_sort.y = sort(path.y);