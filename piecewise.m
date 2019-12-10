% Piecewise linear interpolation

% recently re-written for taking in single trajectory data
function [output_data] = piecewise(trajectory_data)
output_data = trajectory_data;

n = 1;
output_data(n,1) = trajectory_data(1,1);
output_data(n,3) = trajectory_data(1,3);
output_data(n,4) = trajectory_data(1,4);
output_data(n,5) = trajectory_data(1,5);

for i = 2:size(trajectory_data,1)-1
    if(trajectory_data(i,5)==0)
        if (trajectory_data(i+1,5) ~= 0 && trajectory_data(i-1,5) == 0)
            n = n+1;
            output_data(n,1) = trajectory_data(i,1);
            output_data(n,3) = trajectory_data(i,3);
            output_data(n,4) = trajectory_data(i,4);
            output_data(n,5) = trajectory_data(i,5);
        end
    end
end

output_data(n+1,1) = trajectory_data(size(trajectory_data,1),1);
output_data(n+1,3) = trajectory_data(size(trajectory_data,1),3);
output_data(n+1,4) = trajectory_data(size(trajectory_data,1),4);
output_data(n+1,5) = trajectory_data(size(trajectory_data,1),5);
