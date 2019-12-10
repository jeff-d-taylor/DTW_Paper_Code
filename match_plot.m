% This function plots the match solution from DTW

% input: leader time = column of timestamp data for leader vehicle
%        follower time = column of timestamp data for follower vehicle
%        leader position = column of position data for leading vehicle
%        follower position = column of position data for following vehicle
%        warp path = structure (x and y) which identifies the match points
%                    between the two vehicle trajectories
%        t = calculated time headway between vehicles at a potential match
%            point in seconds
%        d = calculated distance between vehicles at a potential match
%            point in feet
%        cost = cost of matching two timeseries points in their respective
%               datasets

% temporary variables:

% output: match_solution =

function match_solution = match_plot(follower_data,leader_data,path,tao,dist,cost)

% plot following vehicle trajectory (black line is default)
plot(follower_data(:,1),follower_data(:,3),'k'); hold on

% plot best matching points
% x = leading vehicle
% y = following vehicle
for i = 1:size(path.x,1) % for each match point in the match solution
    % assemble the match point line to be plotted as a structure
    match.time(1) = leader_data(path.x(i,1),1); match.time(2) = follower_data(path.y(i,1),1);
    match.dist(1) = leader_data(path.x(i,1),3); match.dist(2) = follower_data(path.y(i,1),3);
    line_color = 'k'; % default line color is black
    if tao(i,1) < 0.1 || dist(i,1) > 0
        line_color = 'r'; % use red line color with impossible solutions
    end
    if cost(path.x(i,1),path.y(i,1)) < 100 % when cost is artificial, we are not sure of the values
        % only plot for values of which we are relatively sure of their value
        plot(match.time,match.dist,line_color);
    else
        line_color = 'm';
        plot(match.time,match.dist,line_color);
    end
    hold on
    match_solution.time(i,1) = match.time(2); % Why produce these outputs?
    match_solution.dist(i,1) = match.dist(2);
end
