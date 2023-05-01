% Initializing the agents to random positions with barrier certificates. 
% This script shows how to initialize robots to a particular point.
% Sean Wilson
% 07/2019

%initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);

N = 4;
initial_positions = [ -1.0450, 0, 1, 0.5; -0.5, 0 , 0, 0; 0, 0, 0, 0];
final_goal_points = [ 0, 0, 0, 0; 0, 0, 0, 0; 0, 0, 0, 0];
is_human = [0, 0, 1, 1];


numHumans = 2;
numRobots = 2;

rateHuman = [0.16, 0.20];
rateRobot = 0.3;

%N = 2;
%initial_positions = [ -1.0450, 0; -0.5, 0 ; 0, 0];
%final_goal_points = [ 0, 0;  0, 0; 0, 0];
%numHumans = 1;
%numRobots = 1;

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);
obstacles = cell(10, 1);
% Create a barrier certificate so that the robots don't collide
si_barrier_certificate = create_si_barrier_certificate();
si_to_uni_dynamics = create_si_to_uni_dynamics();

%% PATH PLANNING PART 1


%DEFINE THE 2-D MAP ARRAY
MAX_X=30;
MAX_Y=20;
MAX_VAL=1;
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=(zeros(MAX_X,MAX_Y));

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-2,Target = 0,Robot=1,Space=2, Human =-1
%% Reason why its not following well its cause we just head in direction every mod 8, lets decrement every 8 of the chosen path

%% OR WE COULD connect it with the TEB We can just check if above and below are also free, remember when going upso we check all four side 3 sides that are not in direction of where we go.  
j=0;
x_val = 1;
y_val = 1;
axis([-1.5 1.5 -1 1])
grid on;
hold on;
n=0;%Number of Obstacles

% BEGIN Interactive Obstacle, Target, Start Location selection

but=0;
targetAmount = numRobots;
counter = 1;
xTarget(targetAmount) = 0;
yTarget(targetAmount) = 0;

while (counter < (targetAmount + 1)) %Repeat until the Left button is not clicked
    [xvalOG,yvalOG,but]=ginput(1);
    xTarget(counter) = ((xvalOG + 1.5) / 3) * 30 + 1;
    yTarget(counter) = ((yvalOG + 1) / 2) * 20 + 1;
    
    xTarget(counter)=floor(xTarget(counter));
    yTarget(counter)=floor(yTarget(counter));
    counter = counter + 1;
    plot(xvalOG,yvalOG,'gd');
    text(xvalOG,yvalOG,'Target');
end
%% Adjusting to robotarium map

%xval = xTarget;
%yval = yTarget;

%MAP(xval,yval)=0;%Initialize MAP with location of the target



obstacleNum = 0;
while but == 1
    obstacleNum = obstacleNum+1;
    [xvalOG,yvalOG,but] = ginput(1);
    %% Adjusting to robotarium map
    xval = ((xvalOG + 1.5) / 3) * 30 + 1;
    yval = ((yvalOG + 1) / 2) * 20 + 1;
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval) = -2;%Put on the closed list as well

    obstacle_size = 4;
    obstacle_size_Perm = 3;
    unknown_size = 6;
    % Iterate over a range of indices around the obstacle center
    for io = xval - floor(obstacle_size_Perm/2) : xval + floor(obstacle_size_Perm/2)
        for j = yval - floor(obstacle_size_Perm/2) : yval + floor(obstacle_size_Perm/2)
            % Set the current index to be an obstacle
            MAP(io,j) = -2;
        end
    end
    plot(xvalOG, yvalOG, 'ro', 'MarkerSize', 20);
    obstacles{obstacleNum} = [xvalOG, yvalOG]; 
end

x=r.get_poses();
r.step();

%% Initialize robots initial position
xvalOG = x(1, 1);
yvalOG = x(2, 1);

xval = ((xvalOG + 1.5) / 3) * 30 + 1;
yval = ((yvalOG + 1) / 2) * 20 + 1;
xval=floor(xval);
yval=floor(yval);
xStart=xval;%Starting Position
yStart=yval;%Starting Position
%MAP(xval,yval) = 1;
plot(xvalOG,yvalOG,'bo');
xvalSTART = xval;
yvalSTART = yval;
probColl = 0;
%End of obstacle-Target pickup

%% ==============================================


% We'll make the rotation error huge so that the initialization checker
% doesn't care about it
args = {'PositionError', 0.02, 'RotationError', 50};
init_checker = create_is_initialized(args{:});
controller = create_si_position_controller();

%% Plotting Setup

% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';

CM = ['k','b','r','g'];

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 5;

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
marker_size_robot = determine_robot_marker_size(r);
font_size = determine_font_size(r, 0.05);
line_width = 5;

start_time = tic; %The start time to compute time elapsed.

%% plot part 2 robot details
arrowDir = [0.3; 0.4];

for i = 1:N
    human = 0;
    % Initialize additional information plot here. Note the order of
    % plotting matters, objects that are plotted later will appear over
    % object plotted previously.
    % Text for robot identification
    if is_human(i)
        robot_caption = sprintf('Human %d', i);
    else
        robot_caption = sprintf('Robot %d', i);
    end
    % Text with robot position information
    robot_details = sprintf('X-Pos: %d \nY-Pos: %d', x(1,i), x(2,i));
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    g(i) = plot(x(1,i),x(2,i),'o','MarkerSize', marker_size_robot,'LineWidth',5,'Color',CM(3));
    a(i) =  quiver(x(1,i),x(2,i), arrowDir(1), arrowDir(2), 'MaxHeadSize', 0.5, 'LineWidth', 2, 'Color', 'blue');
    robot_labels{i} = text(500, 500, robot_caption, 'FontSize', font_size, 'FontWeight', 'bold');
    robot_details_text{i} = text(500, 500, robot_details, 'FontSize', font_size, 'FontWeight', 'bold'); 
 end


% Get initial location data for while loop condition.

iterations = 1000;
test1 = 0;
test2 = 0;
prev_action = 0;
historicActions = zeros(1000, numHumans);
prevXHUMAN(numHumans) = -1;
prevYHUMAN(numHumans) = -1;
prevPredictedCorrectly(numHumans) = -1;
presentPredictedCorrectly(numHumans) = -1;

u = -1;
xvalH = -1;
yvalH = -1;
xvalR(numRobots) = -1;
yvalR(numRobots) = -1;
Optimal_path=[];
humans = 1:(N/2); 
humanIterations(numHumans) = 0;
for n= 1 : numHumans
    humanIterations(n) = 0;
    prevXHUMAN(n) = -1;
    prevYHUMAN(n) = -1;
    prevPredictedCorrectly(n) = -1;
    presentPredictedCorrectly(n) = -1;
    xvalR(n) = -1;
    yvalR(n) = -1;
end
currentRobot =0;
for i = 1:iterations
    %% Mimicking Human Movement
    %% test
    for q = 1:N
        x = r.get_poses();

        robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15];
        tempx = ((x(1,q) + 1.5) / 3) * 30 +1;
        tempy = ((x(2,q) + 1) / 2) * 20 +1;
        tempx = floor(tempx);
        tempy = floor(tempy);
        robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f',tempx, tempy);
        robot_details_text{q}.String = robot_details;
        robot_details_text{q}.Position = x(1:2, q) - [0.2;0.25];
        
        if is_human(q) == 0
                % Robot Tags
                 currentRobot = q;
                set(a(q), 'Visible', 'off');
                set(g(q), 'Visible', 'off');
                % Make Robot Coords
                xvalR(currentRobot) = ((x(1,q) + 1.5) / 3) * 30 +1;
                yvalR(currentRobot) = ((x(2,q) + 1) / 2) * 20 +1;
                xvalR(currentRobot) = floor(xvalR(currentRobot));
                yvalR(currentRobot) = floor(yvalR(currentRobot));
        end

        %% AKA IF CURRENT ROBOT IS HUMAN SPLIT INTO MORE
        %% STAY HERE
        if is_human(q) 
            currentHuman = q - numRobots;
            humanIterations(currentHuman) = humanIterations(currentHuman) + 1;
            % GET Next Direction 
            isEqualPrediction = 0;
            % Assume a straigter path than taken
            % Ensures that if headed out of bounds, our direction is changed and corrected
            if(humanIterations(currentHuman) <= 4)
                u = actionTaken(x, -1, rateHuman(currentHuman));   
            else 
                u = actionTaken(x, historicActions((humanIterations(currentHuman) - 1), currentHuman), rateHuman(currentHuman));
            end
            outofbounds = 0;
            if x(1, q) > 1.4 && (u == 1 || u == 7 || u == 8)
                outofbounds = 1;
            end
            if x(1, q) < -1.4 && (u == 3 || u == 4 || u == 5)
                outofbounds = 1;
            end
            if x(2, q) > 0.8 && (u == 1 || u == 2 || u == 3)
                outofbounds = 1;
            end
            if x(2, q) < -0.8 && (u == 5 || u == 6 || u == 7)
                outofbounds = 1;
            end
            while outofbounds == 1
                u = actionTaken(x, 0, 0);
                outofbounds = 0;
                if x(1, q) > 1.4 && (u == 1 || u == 7 || u == 8)
                    disp("HERE 1");
                    outofbounds = 1;
                end
                if x(1, q) < -1.4 && (u == 3 || u == 4 || u == 5)
                    disp("HERE 2");
                    outofbounds = 1;
                end
                if x(2, q) > 0.8 && (u == 1 || u == 2 || u == 3)
                    disp("HERE 3");
                    outofbounds = 1;
                end
                if x(2, q) < -0.8 && (u == 5 || u == 6 || u == 7)
                    disp("HERE 4");
                    outofbounds = 1;
                end
            end
            if (i <= numHumans)
                upredicted = actionTaken(x, 0, rateRobot);
            else 
                upredicted = actionTaken(x, historicActions(humanIterations(currentHuman) - 1, currentHuman), rateRobot);
            end
            % THis now has more recent, from now on treat -1 as prev action

            % Update robot pose
            % 1: 45%(NE)
            % 2: 90%(N)
            % 3: 135%(NW)
            % 4: 180%(W)
            % 5: 225%(SW)
            % 6: 270%(S)
            % 7: 315%(SE)
            % 8: 0%(E)
            coordinatesTaken = [0, 0];
            if u == 1
            coordinatesTaken = [1.5,1];
            elseif u == 2
                coordinatesTaken = [0,1];
            elseif u == 3
                coordinatesTaken = [-1.5,1];
            elseif u == 4
                coordinatesTaken = [-1.5,0];
            elseif u == 5
                coordinatesTaken = [-1.50,-1];
            elseif u == 6
                coordinatesTaken = [0,-1];
            elseif u == 7
                coordinatesTaken = [1.5,-1];
            elseif u == 8
                coordinatesTaken = [1.5,0];
            end

            %% Notes for future eddie, It somewhat works but we probably 
            % need more than just a single prev action history to properly
            % connotate a predcited path, two might be enough but more
            % might be neccessary

            if (i > (numHumans* 3))
                if (u == upredicted) && (u ==  historicActions(humanIterations(currentHuman) - 1, currentHuman) && (u == historicActions((humanIterations(currentHuman) - 2), currentHuman))  && (u == historicActions((humanIterations(currentHuman) - 3), currentHuman)))
                    presentPredictedCorrectly(currentHuman) = 1;
                    set(a(q), 'Visible', 'on');                   
                    set(g(q), 'Visible', 'off');
                    set(a(q), 'Color', 'b');

                    g(q).XData = x(1,q);
                    g(q).YData = x(2,q);
                    set(a(q), 'XData', x(1,q), 'YData',x(2,q), 'UData',  coordinatesTaken(1), 'VData', coordinatesTaken(2));
                    isEqualPrediction = 1;
                    test1 = test1 + 1;

                elseif (abs(u - upredicted) == 1  ) ||(abs(u - upredicted) ==   7) 
                    presentPredictedCorrectly(currentHuman) = 1;
                    set(a(q), 'Visible', 'on');                   
                    set(g(q), 'Visible', 'off');
                    set(a(q), 'Color', 'y');

                    g(q).XData = x(1,q);
                    g(q).YData = x(2,q);
                    set(a(2), 'XData', x(1,q), 'YData',x(2,q), 'UData',  coordinatesTaken(1), 'VData', coordinatesTaken(2));
                    isEqualPrediction = 1;
                    test1 = test1 + 1;

                else
                    set(a(q), 'Visible', 'off');
                    set(g(q), 'Visible', 'on');
                    presentPredictedCorrectly(currentHuman) = 0;
                    isEqualPrediction = 0;
                    test2 = test2 + 1;                
                end
            end

            %% Make Human obstacles
            xvalH = ((x(1,q) + 1.5) / 3) * 30 +1;
            yvalH = ((x(2,q) + 1) / 2) * 20 +1;
            xvalH = floor(xvalH);
            yvalH = floor(yvalH);

            % Remove previous human obstacles
            if prevXHUMAN(currentHuman) ~= -1 && prevYHUMAN(currentHuman) ~= -1 %% shape specify
                if prevPredictedCorrectly(currentHuman) == 0    
                   for s = xvalH - floor(4/2) : xvalH + floor(4/2)
                        for j = yvalH - floor(4/2) : yvalH + floor(4/2)
                            % Set the current index to be an obstacle
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = 0;
                            end
                        end
                    end
                    % Put on the closed list as well
                    % Iterate over a range of indices around the obstacle center
                    for s = prevXHUMAN(currentHuman) - floor(unknown_size/2) : prevXHUMAN(currentHuman) + floor(unknown_size/2)
                        for j = prevYHUMAN(currentHuman) - floor(unknown_size/2) : prevYHUMAN(currentHuman) + floor(unknown_size/2)
                            % Set the current index to be an obstacle
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                MAP(s,j) = 0;
                            end
                        end
                    end
                elseif prevPredictedCorrectly(currentHuman) == 1
                    % Put on the closed list as well
                    for s = prevXHUMAN(currentHuman) - floor(2/2) : prevXHUMAN(currentHuman) + floor(2/2)
                        for j = prevYHUMAN(currentHuman) - floor(2/2) : prevYHUMAN(currentHuman) + floor(2/2)
                            % Set the current index to be an obstacle
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = 0;
                            end
                        end
                    end
                    %% You can ask about this edges are not well made
                    if historicActions(humanIterations(currentHuman) - 1, currentHuman) == 1 %NE
                        for p = 0: obstacle_size- 1
                            for y = 0 : obstacle_size- 1
                                s = prevXHUMAN(currentHuman) + p;
                                j = prevYHUMAN(currentHuman) + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                        MAP(s, j) = 0;
                                end                            
                            end 
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 2 %N
                        for s = prevXHUMAN(currentHuman) - floor(obstacle_size/2) : prevXHUMAN(currentHuman) + floor(obstacle_size/2)
                            for j = prevYHUMAN(currentHuman) : prevYHUMAN(currentHuman) + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 3 %NW
                        for p = 0: obstacle_size- 1
                            for y = 0 : obstacle_size- 1
                                s = prevXHUMAN(currentHuman) - p;
                                j = prevYHUMAN(currentHuman) + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                        MAP(s, j) = 0;
                                end                            
                            end 
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 4 %W
                         for s = prevXHUMAN - floor(obstacle_size/2) : prevXHUMAN
                            for j = prevYHUMAN(currentHuman) - floor(obstacle_size/2) : prevYHUMAN(currentHuman) + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 5 %SW
                        for p = 0: obstacle_size- 1
                            for y = 0 : obstacle_size- 1
                                s = prevXHUMAN(currentHuman) - p;
                                j = prevYHUMAN(currentHuman) - y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                        MAP(s, j) = 0;
                                end                            
                            end 
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 6 %S
                        for s = prevXHUMAN(currentHuman) - floor(obstacle_size/2) : prevXHUMAN(currentHuman) + floor(obstacle_size/2)
                            for j = prevYHUMAN(currentHuman) - floor(obstacle_size/2) : prevYHUMAN(currentHuman)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 7 %SE
                        for p = 0: obstacle_size - 1
                            for y = 0 : obstacle_size - 1
                                s = prevXHUMAN(currentHuman) + p;
                                j = prevYHUMAN(currentHuman) - y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                        MAP(s, j) = 0;
                                end                            
                            end 
                        end
                    elseif historicActions(humanIterations(currentHuman) - 1, currentHuman) == 8 %E
                        for s = prevXHUMAN(currentHuman) : prevXHUMAN(currentHuman) + floor(obstacle_size/2)
                            for j = prevYHUMAN(currentHuman) - floor(obstacle_size/2) : prevYHUMAN(currentHuman) + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && MAP(s,j) ~= 0)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    end
                end
            end


            %%should not be prev
            if presentPredictedCorrectly(currentHuman) == 0    
                % Put on the closed list as well
                % Iterate over a range of indices around the obstacle center
                for s = xvalH - floor(unknown_size/2) : xvalH + floor(unknown_size/2)
                    for j = yvalH - floor(unknown_size/2) : yvalH + floor(unknown_size/2)
                        % Set the current index to be an obstacle
                        if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                            MAP(s,j) = -1;
                        end
                    end
                end
                for s = xvalH - floor(4/2) : xvalH + floor(4/2)
                    for j = yvalH - floor(4/2) : yvalH + floor(4/2)
                        % Set the current index to be an obstacle
                        if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                            MAP(s,j) = -3;
                        end
                    end
                end
                if (xvalH >= 1 && xvalH <= 30 && yvalH >= 1 && yvalH <=20 && MAP(xvalH,yvalH) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                  MAP(xvalH,yvalH) = -4;
                end
            elseif presentPredictedCorrectly(currentHuman) == 1
                % Put on the closed list as well
                if (u == 1) %NE
                    for p = 0: obstacle_size- 1
                        for y = 0 : obstacle_size- 1
                            s = xvalH + p;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                    MAP(s, j) = -1;
                            end                            
                        end 
                    end
                elseif u == 2 %N
                    for s = xvalH - floor(obstacle_size/2) : xvalH + floor(obstacle_size/2)
                        for j = yvalH : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 3 %NW
                    for p = 0: obstacle_size- 1
                        for y = 0 : obstacle_size- 1
                            s = xvalH - p;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                    MAP(s, j) = -1;
                            end                            
                        end 
                    end
                elseif u == 4 %W
                     for s = xvalH - floor(obstacle_size/2) : xvalH
                        for j = yvalH - floor(obstacle_size/2) : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 5 %SW
                    for p = 0: obstacle_size- 1
                        for y = 0 : obstacle_size- 1
                            s = xvalH - p;
                            j = yvalH - y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                    MAP(s, j) = -1;                                     
                            end                            
                        end 
                    end
                elseif u == 6 %S
                    for s = xvalH - floor(obstacle_size/2) : xvalH + floor(obstacle_size/2)
                        for j = yvalH - floor(obstacle_size/2) : yvalH 
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 7 %SE
                    for p = 0: obstacle_size - 1
                        for y = 0 : obstacle_size - 1
                            s = xvalH + p;
                            j = yvalH - y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                    MAP(s, j) = -1;
                            end                            
                        end 
                    end
                elseif u == 8 %E
                    for s = xvalH : xvalH+ floor(obstacle_size/2)
                        for j = yvalH - floor(obstacle_size/2) : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                                MAP(s,j) = -1;
                            end
                        end
                    end
                end
                if (xvalH >= 1 && xvalH <= 30 && yvalH >= 1 && yvalH <=20 && MAP(xvalH,yvalH) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                  MAP(xvalH,yvalH) = -4;
                end
                %% You can ask about this edges are not well made
                for s = xvalH - floor(2/2) : xvalH + floor(2/2)
                    for j = yvalH - floor(2/2) : yvalH + floor(2/2)
                        % Set the current index to be an obstacle
                        if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2 && ~ismember([s, j], [xvalR', yvalR'], 'rows'))
                            MAP(s,j) = -3;
                        end
                    end
                end
            end
            %% Only update prev action until after everuthing
            prev_action = u;
            historicActions(humanIterations(currentHuman), currentHuman) = u;

            prevPredictedCorrectly(currentHuman) = presentPredictedCorrectly(currentHuman);
            prevXHUMAN(currentHuman) = xvalH;
            prevYHUMAN(currentHuman) = yvalH;
            dx = [cos(u*pi/4), sin(u*pi/4)];
            %% Reset goal coordinates to location then to direction where we will move 

            final_goal_points(1:2, q) = x(1:2, q);

            final_goal_points(1, q) = final_goal_points(1, q) + dx(:, 1);
            final_goal_points(2, q) = final_goal_points(2, q) + dx(:, 2);

            % try to slow down human
            final_goal_points(1:2, q) = final_goal_points(1:2, q);
   

            %% End of Human =======================================================
        end

    %% Path PLanning
    % First mod just to slow down calculate/TEB

    if(mod(i, 4) == 0 )
        %Current Robot position
        for irt = 1:length(xvalR)
            xval1 = xvalR(irt);
            yval1 = yvalR(irt);
            MAP(xval1, yval1) = -10; % Set the current point to -10
            obstacle_size = 3; % Define the size of the surrounding area to be set to -10
            for s = xval1 - floor(2/2) : xval1 + floor(2/2)
                for j = yval1 - floor(2/2) : yval1 + floor(2/2)
                    if  s >= 1 && s <= 30 && j >= 1 && j <= 20 && MAP(s, j) ~= -2
                        MAP(s,j) = -7;
                        if (s == xval1 && yval1 == j)
                            MAP(s,j) = -10;
                        end
                    end
                end
            end 
        end


    if(is_human(q) == 0)
        xvalOG = x(1, q);
        yvalOG = x(2, q);
        %% DOnt know about the +1
        xval = ((xvalOG + 1.5) / 3) * 30 + 1;
        yval = ((yvalOG + 1) / 2) * 20 + 1;
        xval=floor(xval);
        yval=floor(yval);
        xStart=xval;%Starting Position
        yStart=yval;%Starting Position



        plot(xvalOG,yvalOG,'bo');
        xvalSTART = xval;
        yvalSTART = yval;
    
        OPEN=[];
        CLOSED=[];
        k=1;%Dummy counter
        for in=1:MAX_X
            for j=1:MAX_Y
                if(MAP(in,j) == -2)
                    CLOSED(k,1)=in; 
                    CLOSED(k,2)=j; 
                    k=k+1;
                % We accept -1 as passable (so robots dont get stuck in a humans 
                % expansive territory
                % but in doing -1 in calculations
                % surrounding area is higher priority to avoid
                elseif (MAP(in,j) < 0 && MAP(in,j) ~= -1 && MAP(in,j) ~= -7)
                    %% EDDIE's RISK MANAGMENT
                    probColl = calc_prob_collision(xvalR(currentRobot), yvalR(currentRobot), in,j, MAP);
                    if probColl > 0.15
                        disp("RISKY");
                     %   disp(probColl);
                        CLOSED(k,1)=in; 
                        CLOSED(k,2)=j; 
                        k=k+1;
                    end
                end
            end
        end
        CLOSED_COUNT=size(CLOSED,1);
        %set the starting node as the first node
        xNode=xvalSTART;
        yNode=yvalSTART;
        
        OPEN_COUNT=1;
        path_cost=0;
        goal_distance=distance(xNode,yNode,xTarget(currentRobot),yTarget(currentRobot));
        OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
        OPEN(OPEN_COUNT,1)=0;
        CLOSED_COUNT=CLOSED_COUNT+1;
        CLOSED(CLOSED_COUNT,1)=xNode;
        CLOSED(CLOSED_COUNT,2)=yNode;
        NoPath=1;
        % START ALGORITHM
        while((xNode ~= xTarget(currentRobot) || yNode ~= yTarget(currentRobot)) && NoPath == 1)
        %  plot(xNode+.5,yNode+.5,'go');
        exp_array=expand_array(xNode,yNode,path_cost,xTarget(currentRobot),yTarget(currentRobot),CLOSED,MAX_X,MAX_Y);
        exp_count=size(exp_array,1);
        for ie=1:exp_count
            flag=0;
            for j=1:OPEN_COUNT
                if(exp_array(ie,1) == OPEN(j,2) && exp_array(ie,2) == OPEN(j,3) )
                    OPEN(j,8)=min(OPEN(j,8),exp_array(ie,5)); %#ok<*SAGROW>
                    if OPEN(j,8)== exp_array(ie,5)
                        %UPDATE PARENTS,gn,hn
                        OPEN(j,4)=xNode;
                        OPEN(j,5)=yNode;
                        OPEN(j,6)=exp_array(ie,3);
                        OPEN(j,7)=exp_array(ie,4);
                    end%End of minimum fn check
                    flag=1;
                end%End of node check
        %         if flag == 1
        %             break;
            end%End of j for
            if flag == 0
                OPEN_COUNT = OPEN_COUNT+1;
                OPEN(OPEN_COUNT,:)=insert_open(exp_array(ie,1),exp_array(ie,2),xNode,yNode,exp_array(ie,3),exp_array(ie,4),exp_array(ie,5));
            end%End of insert new element into the OPEN list
        end
        index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget(currentRobot),yTarget(currentRobot));
        if (index_min_node ~= -1 && index_min_node ~= -2)    
           %Set xNode and yNode to the node with minimum fn
            xNode=OPEN(index_min_node,2);
            yNode=OPEN(index_min_node,3);
            path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
          %Move the Node to list CLOSED
          CLOSED_COUNT=CLOSED_COUNT+1;
          CLOSED(CLOSED_COUNT,1)=xNode;
          CLOSED(CLOSED_COUNT,2)=yNode;
          OPEN(index_min_node,1)=0;
          else
              %No path exists to the Target!!
              NoPath=0;%Exits the loop!
          end%End of index_min_node check
        end%End of While Loop
        
        ie=size(CLOSED,1);
        Optimal_path=[];
        xval=CLOSED(ie,1);
        yval=CLOSED(ie,2);
        ie=1;
        Optimal_path(ie,1)=xval;
        Optimal_path(ie,2)=yval;
        ie=ie+1;
        
        if ( (xval == xTarget(currentRobot)) && (yval == yTarget(currentRobot)))
            inode=0;
           %Traverse OPEN and determine the parent nodes
           parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
           parent_y=OPEN(node_index(OPEN,xval,yval),5);
           
        while( parent_x ~= xStart || parent_y ~= yStart)
               Optimal_path(ie,1) = parent_x;
               Optimal_path(ie,2) = parent_y;
               %Get the grandparents:-)
               inode=node_index(OPEN,parent_x,parent_y);
               parent_x=OPEN(inode,4);%node_index returns the index of the node
               parent_y=OPEN(inode,5);
               ie=ie+1;
        end
        
        j=size(Optimal_path,1);
        %Plot the Optimal Path!
        Optimal_path(j,1) = (Optimal_path(j,1) / 10) - 1.5; 
        Optimal_path(j,2) = (Optimal_path(j,2) / 10) - 1;
        
        p=plot(Optimal_path(j,1),Optimal_path(j,2),'bo');
        j=j-1;
        
        for ih=j:-1:1
          %pause(.05);
          Optimal_path(ih,1) = (Optimal_path(ih,1) / 10) - 1.5;
          Optimal_path(ih,2) = (Optimal_path(ih,2) / 10) - 1;
          set(p,'XData',Optimal_path(ih,1),'YData',Optimal_path(ih,2));
        end
        plot(Optimal_path(:,1),Optimal_path(:,2));
        else
            disp("No path yet)");
            final_goal_points(1:2, q) = x(1:2, q);
        end
        [max_row, max_col] = size(Optimal_path);
        if (max_row>1)
            final_goal_points(1, q) = Optimal_path((max_row - 1),1);
            final_goal_points(2, q) = Optimal_path((max_row - 1),2);
        end
    end
        for irt = 1:length(xvalR)
            xval1 = xvalR(irt);
            yval1 = yvalR(irt);
            MAP(xval1, yval1) = -10; % Set the current point to -10
            obstacle_size = 3; % Define the size of the surrounding area to be set to -10
            for s = xval1 - floor(2/2) : xval1 + floor(2/2)
                for j = yval1 - floor(2/2) : yval1 + floor(2/2)
                    if  s >= 1 && s <= 30 && j >= 1 && j <= 20 && MAP(s, j) ~= -2
                        MAP(s,j) = 0;
                    end
                end
            end 
        end
    end

% ===========================================================================
        dxi = controller(x(1:2, :), final_goal_points(1:2, :)) ;

        dxi = si_barrier_certificate(dxi, x(1:2, :));    
        dxu = si_to_uni_dynamics(dxi, x);
        
        r.set_velocities(1:N, dxu);
        r.step();   
    end

    
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

function dX = actionTaken(x, prev_action, rate)
            beta_i = 0.5;           % inverse temperature parameter
            theta_i = randn(8,1);   % parameter vector for reward function
           % xt = [x(2,1); x(2,2)]; % current robot position pretty sure
           % its useless
              xt = 0;
            % Define reward function
            if prev_action == 0 || prev_action == -1
                riz = @(xt, utx, theta) theta(utx); % example reward function
                % Define Boltzmann distribution function
                Qi = @(xt, utx) riz(xt, utx, theta_i);      
                P = @(xt, utx) exp(beta_i*Qi(xt,utx))/sum(exp(beta_i*Qi(xt,1:8)));
                
                % Choose next action using Boltzmann distribution
                probs = zeros(1,8);
                for j = 1:8
                    utx = j; % next action
                    probs(j) = P(xt,utx);
                end
            else
                %  riz = @(xt, utx, theta, prev_action) theta(utx)*exp(-0.1*(abs(xt(1)-xt(2))));
                riz = @(xt, utx, theta) theta(utx);
                Qi = @(xt, utx) riz(xt, utx, theta_i);      
                %  P = @(xt, utx, prev_action) exp(beta_i*Qi(xt,utx, prev_action))/sum(exp(beta_i*Qi(xt,1:8, prev_action)));
                P = @(xt, utx) exp(beta_i*Qi(xt,utx))/sum(exp(beta_i*Qi(xt,1:8)));

                % Choose next action using Boltzmann distribution
                probs = zeros(1,8);
                for j = 1:8
                    utx = j; % next action
                    probs(j) = P(xt,utx);
                    probs(j) = probs(j)  - (rate*abs(utx - prev_action));
                end
            end
            [maxProb, actionIndex] = max(probs);
            dX = actionIndex;
end

%% Path planning ======================================================================

function prob_coll = calc_prob_collision(robotX, robotY, humanX, humanY, MAP)
    % robot_state: planned state of the robot at time tau
    % human_states: cell array containing the states of all humans at time tau
    % We define TEB as 1 unit around robot
    OOB = -3;
    % if 1 then too high, if 2 then too low
    outOfBoundsX = 0;
    outOfBoundsY = 0;
    overlap = 0;
    
    if humanX == 30 
        outOfBoundsX = 1;
    elseif humanX == 1
        outOfBoundsX = 2;
    end
    
    if humanY == 20 
        outOfBoundsY = 1;
    elseif humanY == 1
        outOfBoundsY = 2;
    end

    if    (outOfBoundsX == 0 && outOfBoundsY == 0)
            overlap = MAP(humanX + 1, humanY + 1) + MAP(humanX - 1, humanY - 1) + MAP(humanX - 1, humanY + 1) + MAP(humanX + 1, humanY - 1) + MAP(humanX, humanY + 1) + MAP(humanX, humanY - 1) + MAP(humanX + 1, humanY) + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 0 && outOfBoundsY == 1)
            overlap = OOB + MAP(humanX - 1, humanY - 1) + MAP(humanX + 1, humanY - 1) + MAP(humanX, humanY - 1) + MAP(humanX + 1, humanY) + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 0 && outOfBoundsY == 2)
            overlap = OOB + MAP(humanX + 1, humanY + 1) + MAP(humanX - 1, humanY + 1)  + MAP(humanX, humanY + 1) + MAP(humanX + 1, humanY) + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 1 && outOfBoundsY == 0)
            overlap = OOB + MAP(humanX - 1, humanY - 1) + MAP(humanX - 1, humanY + 1)  + MAP(humanX, humanY + 1) + MAP(humanX, humanY - 1) + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 1 && outOfBoundsY == 1)
            overlap =  OOB - 2 + MAP(humanX - 1, humanY - 1)  + MAP(humanX, humanY - 1) + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 1 && outOfBoundsY == 2)
            overlap = OOB -2 +  MAP(humanX - 1, humanY + 1)  + MAP(humanX, humanY + 1)  + MAP(humanX - 1, humanY);
    elseif(outOfBoundsX == 2 && outOfBoundsY == 0)
            overlap = OOB + MAP(humanX + 1, humanY + 1)  + MAP(humanX + 1, humanY - 1) + MAP(humanX, humanY + 1) + MAP(humanX, humanY - 1) + MAP(humanX + 1, humanY);
    elseif(outOfBoundsX == 2 && outOfBoundsY == 1)
             overlap = OOB -2 +  MAP(humanX + 1, humanY - 1) + MAP(humanX, humanY - 1) + MAP(humanX + 1, humanY);
    elseif(outOfBoundsX == 2 && outOfBoundsY == 2)
             overlap = OOB - 2 + MAP(humanX + 1, humanY + 1)   + MAP(humanX, humanY + 1) + MAP(humanX + 1, humanY) ;
    end
    overlap = overlap / 5;
    overlap = -overlap;
    
    distanceFrom = distance(robotX,robotY,humanX, humanY);
    prob_coll = overlap * (1/distanceFrom);
end















































%% Helper Functions

function marker_size = determine_robot_marker_size(robotarium_instance)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
robot_ratio = (robotarium_instance.robot_diameter + 0.03)/...
    (robotarium_instance.boundaries(2) - robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * robot_ratio;
marker_size = 40;

end

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Pixels');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end



