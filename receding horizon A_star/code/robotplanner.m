function[action] = robotplanner(envmap, robotpos, targettraj, targetpos, time, C)

MEX = 1;

numofdirs = 8;
dX = [-1 -1 -1  0  0  1 1 1];
dY = [-1  0  1 -1  1 -1 0 1];

if (MEX == 1)
    % if using MEX, you would call the planner here
    action = planner(envmap, robotpos, targettraj, targetpos, time, C);
    
else
    % otherwise do planning right here
    
    % for now greedily move towards the final target position,
    % but this is where you can put your planner
    goalpos = targettraj(end,:);
    
    olddisttotarget = sqrt((robotpos(1)-goalpos(1))^2 + (robotpos(2)-goalpos(2))^2);
    bestX = 0; bestY = 0;
    for dir = 1:numofdirs
        newx = robotpos(1) + dX(dir);
        newy = robotpos(2) + dY(dir);

        if (newx >= 1 && newx <= size(envmap, 1) && newy >= 1 && newy <= size(envmap, 2))
            if (envmap(newx, newy) >= 0 && envmap(newx, newy) < C)  % if valid
                disttotarget = sqrt((newx-goalpos(1))^2 + (newy-goalpos(2))^2);
                if(disttotarget < olddisttotarget)
                    olddisttotarget = disttotarget;
                    bestX = dX(dir);
                    bestY = dY(dir);
                end
            end
        end
    end

    robotpos = [robotpos(1) + bestX, robotpos(2) + bestY];
    action = robotpos;
end
