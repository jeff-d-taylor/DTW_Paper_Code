% This version of the cost function follows Dr. Zhou's recommendation for
% calculating the cost matrix - it uses prior information to try to
% encourage more reasonable results. It calculates the cost matrix for
% matching all points in one time series to all points in another time
% series, considering prior information about parameters. It also employs
% an artificial constraint to make sure that impossible matches (where the
% time and spacing headways are not positive) are discouraged where
% possible.

% input: leader's time series (timestamps, position, comparison value)
%        follower's time series (timestamps, position, comparison value)
%        lead_matrix = the column of data for which the leader is compared
%        follow_matrix = the column of data for which the follower is compared
%        matrix_size = number of time stamps in each vehicle's time series
%             matrix_size(1) is for leader, matrix_size(2) is for follower
%        prior_estimates(1) = comparison difference = estimate for comparison difference (typically 0)
%        prior_estimates(2) = prior_t = prior estimate for the time lag or time headway, in seconds
%        prior_estimates(3) = prior_d = prior estimate for the distance headway, in feet
%        prior_estimates(4) = prior_w = prior estimate for the wave speed, in MPH
%        prior_estimates(5) = a = coefficient for weight on comparison value
%        prior_estimates(6) = b = coefficient for weight on time lag
%        prior_estimates(7) = c = coefficient for weight on distance headway
%        prior_estimates(8) = e = coefficient for weight on wave speed

% temporary variables: match = temporary calculations for time and distance headway

% output: cost matrix, of size [matrix_size(1) X matrix_size(2)]
%         t = calculated time headway between vehicles at a potential match
%         point in seconds
%         d = calculated distance between vehicles at a potential match
%         point in feet
%         w = calculated wave speed at a potential match point in MPH

function cost = quick_cost_matrix_multi_prior(lead_time,follow_time,lead_pos,follow_pos,lead_matrix,follow_matrix,matrix_size,prior_estimates)

% preallocation to improve speed
cost = zeros(matrix_size(1),matrix_size(2));

for i = 1:matrix_size(1) % for each leading vehicle timestamp
    for j = 1:matrix_size(2) % for each following vehicle timestamp
        % cost calculation with all terms
        cost(i,j) = prior_estimates(5)*(abs(lead_matrix(i,1) - follow_matrix(j,1)-prior_estimates(1))) + prior_estimates(6)*(abs(((follow_time(j,1) - lead_time(i,1))/10) - prior_estimates(2))) + prior_estimates(7)*(abs((follow_pos(j,1) - lead_pos(i,1)) - prior_estimates(3))) + prior_estimates(8)*(abs(((follow_pos(j,1) - lead_pos(i,1))/((follow_time(j,1) - lead_time(i,1))/10)*(3600/5280)) - prior_estimates(4)));
        
        % enforce some constraints on time lag and distance:
        if ((follow_time(j,1) - lead_time(i,1))/10) < 0 || (follow_pos(j,1) - lead_pos(i,1)) > 0
            % use an artificially high cost to encourage the algorithm to
            % avoid impossible solutions
            cost(i,j) = 100*cost(i,j);
        end
        
    end
end