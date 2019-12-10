% This function calculates the time lag, distance spacing, and wave speed
% for the match solution from the DTW algorithm (warp path).

% input: leader time stamps, follower time stamps, leader position,
%        follower position
%        warp path = structure (x and y) which identifies the match points
%                    between the two vehicle trajectories

% temporary variables: match = temporary calculations for time and distance

% output: t = calculated time headway between vehicles at a potential match point
%             in seconds
%         d = calculated distance between vehicles at a potential match point in
%             feet
%         w = calculated wave speed at a potential match point in MPH

function [t,d,w] = follow_parameters(lead_time, follow_time,lead_pos,follow_pos,path)
% pre-allocate variables for improved speed
t = zeros(size(path.x,1),1); % time headway
d = zeros(size(path.x,1),1); % distance spacing
w = zeros(size(path.x,1),1); % wave speed
% x = leading vehicle
% y = following vehicle
for i = 1:size(path.x,1)
    % Store the leader's time and follower's time at each time step
    match.time(1) = lead_time(path.x(i),1); match.time(2) = follow_time(path.y(i),1);
    % calculate the time lag between vehicles
    t(i,1) = (match.time(2) - match.time(1))/10; % in sec
    % Store the leader's position and follower's position at each time step
    match.dist(1) = lead_pos(path.x(i),1); match.dist(2) = follow_pos(path.y(i),1);
    % calculate the spacing between vehicles
    d(i,1) = match.dist(2) - match.dist(1); % in feet, should be negative
    % feet to kilometers: multiply by 0.3048/1000
    % seconds to hours: divide by 3600
    w(i,1) = d(i,1)/t(i,1)*(3600/5280); % in miles per hour
end