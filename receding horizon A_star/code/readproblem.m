function [N,C,R,T,M] = readproblem(filename)
%READPROBLEM Read problem definition text file.
%   Params:
%         filename    - name of problem text file
%         N           - dimensions of map (1x2)
%         C           - collision threshold for map (1x1)
%         R           - robot start location (1x2)
%         T           - target trajectory (Mx2)
%         M           - map cell costs (PxQ where P = N(1), Q = N(2))

FID = fopen(filename, 'r');

if(fgetl(FID) ~= 'N')
    fprintf('Error parsing problem file.')
    return;
end
N = fscanf(FID, '%d,%d')';

if(fgetl(FID) ~= 'C')
    fprintf('Error parsing problem file.')
    return;
end
C = fscanf(FID, '%d');

if(fgetl(FID) ~= 'R')
    fprintf('Error parsing problem file.')
    return;
end
R = fscanf(FID, '%f,%f')';

if(fgetl(FID) ~= 'T')
    fprintf('Error parsing problem file.')
    return;
end
T = textscan(FID, '%f%f', 'CollectOutput', true, 'Delimiter', ',');
T = T{1};

if(fgetl(FID) ~= 'M')
    fprintf('Error parsing problem file.')
    return;
end
formatSpec = repmat(replace(cat(2,repmat('%f,', 1, N(2)), '\n'), ',\n', '\n'), 1, N(1));
M = reshape(fscanf(FID, formatSpec), N(2), N(1))';

end