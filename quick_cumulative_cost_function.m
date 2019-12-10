% This function calculates the cumulative cost matrix. It uses a specific
% search pattern to do so.

% input: cost_matrix = cost of matching two timeseries points in their
%                      respective datasets
%        matrix_size = number of time stamps in each vehicle's time series
%                      matrix_size(1) is for leader, matrix_size(2) is for
%                      follower 

% temporary variables: 
%        minimum_step = the least-cost required to arrive at the previous
%                       step before the current location in the matrix

% output: cumulative_cost = matrix containing the cumulative cost of
%                           matching all points in one timeseries dataset
%                           to all of the other points in the other
%                           timeseries dataset


function [cumulative_cost] = quick_cumulative_cost_function(cost_matrix,matrix_size)

% pre-allocate to improve speed
cumulative_cost = zeros(matrix_size(1),matrix_size(2));

% at the very beginning (1,1), the initial starting cost is zero

for j = 2:matrix_size(2) % working with the first row of the matrix
    cumulative_cost(1,j) = cumulative_cost(1,j-1) + cost_matrix(1,j);
end
for i = 2:matrix_size(1) % working with the first column of the matrix
    cumulative_cost(i,1) = cumulative_cost(i-1,1)+cost_matrix(i,1);
end
    
for i = 2:matrix_size(1) % for each leader vehicle timestamp
    for j = 2:matrix_size(2) % for each following vehicle timestamp
        % calculate the minimum cost amongst three options
        % for movement in the matrix (back within this column, back
        % within this row, and back diagonally
        % use the least cost location to calculate the least cost for
        % arriving at the current location in the matrix
        cumulative_cost(i,j) = cost_matrix(i,j) + min(min(cumulative_cost(i-1,j),cumulative_cost(i,j-1)),cumulative_cost(i-1,j-1));
    end
end