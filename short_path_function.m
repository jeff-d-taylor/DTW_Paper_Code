% This function finds the shortest path through the cumulative cost matrix.

% input: cumulative_cost = cumulative cost matrix
%        matrix size = number of time stamps in each vehicle's time series
%           matrix_size(1) is for leader, matrix_size(2) is for follower

% temporary variables: i = leader position index, j = follower position
%                      index, k = path index
%                      path = structure of leader and follower positions,
%                      from end to beginning

% output: path_sort = structure containing the leader(x) and follower(y)
%                positions for optimal matching, from beginning to end

function [path_sort] = short_path_function(cumulative_cost,matrix_size)

i = matrix_size(1); % number of timeseries points in leader's trajectory
% i is also used to keep track of the current location in the matrix
j = matrix_size(2); % number of timeseries points in follower's trajectory
% j is also used to keep track of the current location in the matrix
k = 1; % initialize the first step in the shortest path
% start building the shortest path from the end of the matrix, starting
% with the last points in the matrix
path.x(k,1) = matrix_size(1); % i and x = leader
path.y(k,1) = matrix_size(2); % j and y = follower

% In the cumulative cost matrix(i,j), the leader location stays the same
% within the row, and the follower location stays the same within the column

% Find shortest path

while i>=1 && j>=1 % as long as we don't reach the beginning of the matrix
    k = k+1; % move forward a step in the path
    
    if i == 1 && j == 1 % if we reach the beginning of the matrix
        break % stop moving through the matrix
    elseif i == 1 % if we reach the first row, stay in the same row
        path.x(k,1) = i; path.y(k,1) = j-1; j = j-1; % only increment to next column
        continue
    elseif j == 1 % if we reach the first column, stay in the same column
        path.x(k,1) = i-1; path.y(k,1) = j; i = i-1; % only increment to next row
        continue
    end
    
    if i>1 && j>1 % if we haven't reached the end of the matrix yet
        % calculate the next step cost - the minimum cost to reach the next
        % point in the matrix, moving vertically, horizontally, or diagonally
        next_step_cost = min(min(cumulative_cost(i-1,j),cumulative_cost(i,j-1)),cumulative_cost(i-1,j-1));
        
        % check which step to take next, store the next step in the
        % shortest path, and increment to the next step location in i & j
        if next_step_cost == cumulative_cost(i-1,j-1)
            % if all steps cost the same, default step is diagonally
            path.x(k,1) = i-1; path.y(k,1) = j-1; i = i-1; j = j-1;
        elseif next_step_cost == cumulative_cost(i,j-1)
            path.x(k,1) = i; path.y(k,1) = j-1; j = j-1;
        else
            path.x(k,1) = i-1; path.y(k,1) = j; i = i-1;
        end
    end
end
% resort the datasets to set the path from beginning to end
 path_sort.x = sort(path.x);  path_sort.y = sort(path.y);