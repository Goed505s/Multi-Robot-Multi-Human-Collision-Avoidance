% Initializing the agents to random positions with barrier certificates. 
% This script shows how to initialize robots to a particular point.
% Sean Wilson
% 07/2019

N = 2;
%initial_positions = generate_initial_conditions(N, 'Spacing', 0.5);
initial_positions = [ -1.0450, 0; -0.4450, 0 ;-0.4696, 0];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

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
MAP=(ones(MAX_X,MAX_Y));

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
while (but ~= 1) %Repeat until the Left button is not clicked
    [xvalOG,yvalOG,but]=ginput(1);
end
%% Adjusting to robotarium map
xTarget = ((xvalOG + 1.5) / 3) * 30 + 1;
yTarget = ((yvalOG + 1) / 2) * 20 + 1;

xTarget=floor(xTarget);
yTarget=floor(yTarget);
xval = xTarget;
yval = yTarget;

MAP(xval,yval)=0;%Initialize MAP with location of the target

plot(xvalOG,yvalOG,'gd');
text(xvalOG,yvalOG,'Target');

while but == 1
    [xvalOG,yvalOG,but] = ginput(1);
    %% Adjusting to robotarium map
    xval = ((xvalOG + 1.5) / 3) * 30 + 1;
    yval = ((yvalOG + 1) / 2) * 20 + 1;
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval) = -2;%Put on the closed list as well

    obstacle_size = 4;
    obstacle_size_Perm = 3.5;
    uknown_size = 6;
    % Iterate over a range of indices around the obstacle center
    for i = xval - floor(obstacle_size_Perm/2) : xval + floor(obstacle_size_Perm/2)
        for j = yval - floor(obstacle_size_Perm/2) : yval + floor(obstacle_size_Perm/2)
            % Set the current index to be an obstacle
            MAP(i,j) = -2;
        end
    end

    plot(xvalOG, yvalOG, 'ro', 'MarkerSize', 20);
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
MAP(xval,yval) = 1;
plot(xvalOG,yvalOG,'bo');
xvalSTART = xval;
yvalSTART = yval;

%End of obstacle-Target pickup

%% ==============================================

final_goal_points = [ 1.0450, 0; 0.4450, -0.5; 0.4696, 1];

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
    if i == 1
        robot_caption = sprintf('Robot %d', i);
        human = 0;
    else
         robot_caption = sprintf('Human %d');
         human = 1;
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

rateHuman = 0.15;
rateRobot = 0.3;
iterations = 1000;
test1 = 0;
test2 = 0;
prev_action = 0;
Secondprev_action = 0;
historicActions = zeros(1000, 1);
prevXHUMAN = -1;
prevYHUMAN = -1;
prevPredictedCorrectly = -1;
presentPredictedCorrectly = -1;

u = -1;
xvalH = -1;
yvalH = -1;
xvalR = -1;
yvalR = -1;
Optimal_path=[];

for i = 1:iterations

    %% Mimicking Human Movement
    %% test


    for q = 1:N
        x = r.get_poses();
        % Robot Tags
        set(a(1), 'Visible', 'off');
        set(g(1), 'Visible', 'off');


        robot_labels{q}.Position = x(1:2, q) + [-0.15;0.15];
        robot_details = sprintf('X-Pos: %0.2f \nY-Pos: %0.2f', x(1,q), x(2,q));
        robot_details_text{q}.String = robot_details;
        robot_details_text{q}.Position = x(1:2, q) - [0.2;0.25];

        %% Make Robot Coords
        xvalR = ((x(1,1) + 1.5) / 3) * 30 +1;
        yvalR = ((x(2,1) + 1) / 2) * 20 +1;
        xvalR = floor(xvalR);
        yvalR = floor(yvalR);
            
        %% AKA IF CURRENT ROBOT IS HUMAN
        if q == 2

            %% GET Next Direction 
            isEqualPrediction = 0;
            % Assume a straigter path than taken
            %% Ensures that if headed out of bounds, our direction is changed and corrected
            u = actionTaken(x, prev_action, rateHuman);
            outofbounds = 0;
            if x(1, 2) > 1.4 && (u == 1 || u == 7 || u == 8)
                outofbounds = 1;
            end
            if x(1, 2) < -1.4 && (u == 3 || u == 4 || u == 5)
                outofbounds = 1;
            end
            if x(2, 2) > 0.8 && (u == 1 || u == 2 || u == 3)
                outofbounds = 1;
            end
            if x(2, 2) < -0.8 && (u == 5 || u == 6 || u == 7)
                outofbounds = 1;
            end
            while outofbounds == 1
                u = actionTaken(x, prev_action, 0);
                outofbounds = 0;
                if x(1, 2) > 1.4 && (u == 1 || u == 7 || u == 8)
                    disp("HERE 1");
                    outofbounds = 1;
                end
                if x(1, 2) < -1.4 && (u == 3 || u == 4 || u == 5)
                    disp("HERE 2");
                    outofbounds = 1;
                end
                if x(2, 2) > 0.8 && (u == 1 || u == 2 || u == 3)
                    disp("HERE 3");
                    outofbounds = 1;
                end
                if x(2, 2) < -0.8 && (u == 5 || u == 6 || u == 7)
                    disp("HERE 4");
                    outofbounds = 1;
                end
            end

            upredicted = actionTaken(x, prev_action, rateRobot);
            historicActions(i) = u;
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

        
            %plot(x(1,2), x(2,2), 'ro', 'MarkerSize', 20);


            %% Notes for future eddie, It somewhat works but we probably 
            % need more than just a single prev action history to properly
            % connotate a predcited path, two might be enough but more
            % might be neccessary

            if (i > 3)
                if ((u == upredicted) && (u == prev_action) && (u == historicActions(i- 2))  && (u == historicActions(i - 3)))
                    presentPredictedCorrectly = 1;
                    set(a(2), 'Visible', 'on');                   
                    set(g(2), 'Visible', 'off');
                    set(a(2), 'Color', 'b');

                    g(2).XData = x(1,2);
                    g(2).YData = x(2,2);
                    set(a(2), 'XData', x(1,2), 'YData',x(2,2), 'UData',  coordinatesTaken(1), 'VData', coordinatesTaken(2));
                    isEqualPrediction = 1;
                    test1 = test1 + 1;

                elseif (abs(u - upredicted) == 1  ) ||(abs(u - upredicted) ==   7) 
                    presentPredictedCorrectly = 1;
                    set(a(2), 'Visible', 'on');                   
                    set(g(2), 'Visible', 'off');
                    set(a(2), 'Color', 'y');

                    g(2).XData = x(1,2);
                    g(2).YData = x(2,2);
                    set(a(2), 'XData', x(1,2), 'YData',x(2,2), 'UData',  coordinatesTaken(1), 'VData', coordinatesTaken(2));
                    isEqualPrediction = 1;
                    test1 = test1 + 1;

                else
                    set(a(2), 'Visible', 'off');
                    set(g(2), 'Visible', 'on');
                    presentPredictedCorrectly = 0;
                    isEqualPrediction = 0;
                    test2 = test2 + 1;                
                end
    
                if ((u == upredicted) && u == prev_action)
                    Secondprev_action = prev_action;
                end
            end

            %% Make Human obstacles
            xvalH = ((x(1,2) + 1.5) / 3) * 30 +1;
            yvalH = ((x(2,2) + 1) / 2) * 20 +1;
            xvalH = floor(xvalH);
            yvalH = floor(yvalH);
            % Remove previous human obstacles
            

            if prevXHUMAN ~= -1 && prevYHUMAN ~= -1 %% shape specify
                if prevPredictedCorrectly == 0
                    MAP(prevXHUMAN,prevYHUMAN) = 0;
                    % Iterate over a range of indices around the obstacle center
                    for s = prevXHUMAN - floor(uknown_size/2) : prevXHUMAN + floor(uknown_size/2)
                        for j = prevYHUMAN - floor(uknown_size/2) : prevYHUMAN + floor(uknown_size/2)
                            % Set the current index to be an obstacle
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s,j) = 0;
                            end
                        end
                    end
                elseif prevPredictedCorrectly == 1
                    % Put on the closed list as well
                    MAP(prevXHUMAN,prevYHUMAN) = 0;

                    %% You can ask about this edges are not well made
                    if (prev_action == 1) %NE
                        temp = 0;
                        for (p = 1: 3)
                            xz= -obstacle_size;
                            for y = obstacle_size : -obstacle_size
                                s = prevXHUMAN + xz + temp;
                                j = prevYHUMAN + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s, j) = 0;
                                end
                                xz = xz + 1;
                            end 
                            temp = temp + 1;
                        end
                    elseif prev_action == 2 %N
                        for s = prevXHUMAN - floor(obstacle_size/2) : prevXHUMAN + floor(obstacle_size/2)
                            for j = prevYHUMAN : prevYHUMAN + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif prev_action == 3 %NW
                        temp = 0;
                        for (p = 1: 3)
                            xz= obstacle_size;
                            for y = obstacle_size : -obstacle_size
                                s = prevXHUMAN + xz - temp;
                                j = prevYHUMAN + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s, j) = 0;
                                end
                                xz = xz - 1;
                            end 
                            temp = temp + 1;
                        end
                    elseif prev_action == 4 %W
                         for s = prevXHUMAN - floor(obstacle_size/2) : prevXHUMAN
                            for j = prevYHUMAN - floor(obstacle_size/2) : prevYHUMAN + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif prev_action == 5 %SW
                        temp = 0;
                        for (p = 1: 3)
                            xz = obstacle_size;
                            for y = -obstacle_size : obstacle_size
                                s = prevXHUMAN + xz - temp;
                                j = prevYHUMAN + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s, j) = 0;
                                end                            
                                xz = xz - 1;
                            end 
                            temp = temp + 1;
                        end
                    elseif prev_action == 6 %S
                        for s = prevXHUMAN - floor(obstacle_size/2) : prevXHUMAN + floor(obstacle_size/2)
                            for j = prevYHUMAN - floor(obstacle_size/2) : prevYHUMAN 
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    elseif prev_action == 7 %SE
                        temp = 0;
                        for (p = 1: 3)
                            xz= -obstacle_size;
                            for y = -obstacle_size : obstacle_size
                                s = prevXHUMAN + xz + temp;
                                j = prevYHUMAN + y;
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s, j) = 0;
                                end
                                xz = xz + 1;
                            end 
                            temp = temp + 1;
                        end
                    elseif prev_action == 8 %E
                        for s = prevXHUMAN : prevXHUMAN+ floor(obstacle_size/2)
                            for j = prevYHUMAN - floor(obstacle_size/2) : prevYHUMAN + floor(obstacle_size/2)
                                if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s,j) = 0;
                                end
                            end
                        end
                    end
                end   
            end
%%should not be prev
            if presentPredictedCorrectly == 0    
                MAP(xvalH,yvalH) = -1;
                % Put on the closed list as well
                % Iterate over a range of indices around the obstacle center
                for s = xvalH - floor(uknown_size/2) : xvalH + floor(uknown_size/2)
                    for j = yvalH - floor(uknown_size/2) : yvalH + floor(uknown_size/2)
                        % Set the current index to be an obstacle
                        if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                            MAP(s,j) = -1;
                        end
                    end
                end
            elseif presentPredictedCorrectly == 1
                % Put on the closed list as well
                MAP(xvalH,yvalH) = -1;
                %% You can ask about this edges are not well made
                if (u == 1) %NE
                    temp = 0;
                    for (p = 1: 3)
                        xz= -obstacle_size;
                        for y = obstacle_size : -obstacle_size
                            s = xvalH + xz + temp;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s, j) = -1;
                            end
                            xz = xz + 1;
                        end 
                        temp = temp + 1;
                    end
                elseif u == 2 %N
                    for s = xvalH - floor(obstacle_size/2) : xvalH + floor(obstacle_size/2)
                        for j = yvalH : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 3 %NW
                    temp = 0;
                    for (p = 1: 3)
                        xz= obstacle_size;
                        for y = obstacle_size : -obstacle_size
                            s = xvalH + xz - temp;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s, j) = -1;
                            end
                            xz = xz - 1;
                        end 
                        temp = temp + 1;
                    end
                elseif u == 4 %W
                     for s = xvalH - floor(obstacle_size/2) : xvalH
                        for j = yvalH - floor(obstacle_size/2) : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 5 %SW
                    temp = 0;
                    for (p = 1: 3)
                        xz= obstacle_size;
                        for y = -obstacle_size : obstacle_size
                            s = xvalH + xz - temp;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                    MAP(s, j) = -1;
                            end                            
                            xz = xz - 1;
                        end 
                        temp = temp + 1;
                    end
                elseif u == 6 %S
                    for s = xvalH - floor(obstacle_size/2) : xvalH + floor(obstacle_size/2)
                        for j = yvalH - floor(obstacle_size/2) : yvalH 
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s,j) = -1;
                            end
                        end
                    end
                elseif u == 7 %SE
                    temp = 0;
                    for (p = 1: 3)
                        xz= -obstacle_size;
                        for y = -obstacle_size : obstacle_size
                            s = xvalH + xz + temp;
                            j = yvalH + y;
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s, j) = -1;
                            end
                            xz = xz + 1;
                        end 
                        temp = temp + 1;
                    end
                elseif u == 8 %E
                    for s = xvalH : xvalH+ floor(obstacle_size/2)
                        for j = yvalH - floor(obstacle_size/2) : yvalH + floor(obstacle_size/2)
                            if (s >= 1 && s <= 30 && j >= 1 && j <=20 && MAP(s,j) ~= -2)
                                MAP(s,j) = -1;
                            end
                        end
                    end
                end
            end
             
            end
        
            %% Only update prev action until after everuthing
            prev_action = u;
            prevPredictedCorrectly = presentPredictedCorrectly;
            prevXHUMAN = xvalH;
            prevYHUMAN = yvalH;
            dx = [cos(u*pi/4), sin(u*pi/4)];
          %  x(1, 2) = x(1, 2) + dx(:, 1);
          %  x(2, 2) = x(2, 2) + dx(:, 2);
            %% Reset goal coordinates to location then to direction where we will move 
           
            final_goal_points(1:2, 2) = x(1:2, 2);

            final_goal_points(1, 2) = final_goal_points(1, 2) + dx(:, 1);
            final_goal_points(2, 2) = final_goal_points(2, 2) + dx(:, 2);

            % try to slow down human
            final_goal_points(1:2, 2) = final_goal_points(1:2, 2)/1;
   

%% =======================================================
    % Path PLanning



    if(mod(i, 6) == 0 )
        xvalOG = x(1, 1);
    yvalOG = x(2, 1);
    %% DOnt know about the +1
    xval = ((xvalOG + 1.5) / 3) * 30 + 1;
    yval = ((yvalOG + 1) / 2) * 20 + 1;
    xval=floor(xval);
    yval=floor(yval);
    xStart=xval;%Starting Position
    yStart=yval;%Starting Position
    MAP(xval,yval) = 1;
    plot(xvalOG,yvalOG,'bo');
    xvalSTART = xval;
    yvalSTART = yval;

    OPEN=[];
    CLOSED=[];
    k=1;%Dummy counter
    for i=1:MAX_X
        for j=1:MAX_Y
            if(MAP(i,j) == -2)
                CLOSED(k,1)=i; 
                CLOSED(k,2)=j; 
                k=k+1;
            elseif (MAP(i,j) == -1)
                %% EDDIE COME BACK RISK MANAGMENT
                probColl = calc_prob_collision(xvalR, yvalR, i,j, MAP);
                if probColl > 0.20
                    disp("RISKY");
                    CLOSED(k,1)=i; 
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
    goal_distance=distance(xNode,yNode,xTarget,yTarget);
    OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
    OPEN(OPEN_COUNT,1)=0;
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=xNode;
    CLOSED(CLOSED_COUNT,2)=yNode;
    NoPath=1;
    % START ALGORITHM
    while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    %  plot(xNode+.5,yNode+.5,'go');
    exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
    exp_count=size(exp_array,1);
    for i=1:exp_count
        flag=0;
        for j=1:OPEN_COUNT
            if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
                OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
                if OPEN(j,8)== exp_array(i,5)
                    %UPDATE PARENTS,gn,hn
                    OPEN(j,4)=xNode;
                    OPEN(j,5)=yNode;
                    OPEN(j,6)=exp_array(i,3);
                    OPEN(j,7)=exp_array(i,4);
                end%End of minimum fn check
                flag=1;
            end%End of node check
    %         if flag == 1
    %             break;
        end%End of j for
        if flag == 0
            OPEN_COUNT = OPEN_COUNT+1;
            OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
        end%End of insert new element into the OPEN list
    end
    index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
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
    
    i=size(CLOSED,1);
    Optimal_path=[];
    xval=CLOSED(i,1);
    yval=CLOSED(i,2);
    i=1;
    Optimal_path(i,1)=xval;
    Optimal_path(i,2)=yval;
    i=i+1;
    
    if ( (xval == xTarget) && (yval == yTarget))
        inode=0;
       %Traverse OPEN and determine the parent nodes
       parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
       parent_y=OPEN(node_index(OPEN,xval,yval),5);
       
    while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end
    
    j=size(Optimal_path,1);
    %Plot the Optimal Path!
    Optimal_path(j,1) = (Optimal_path(j,1) / 10) - 1.5; 
    Optimal_path(j,2) = (Optimal_path(j,2) / 10) - 1;
    
    p=plot(Optimal_path(j,1),Optimal_path(j,2),'bo');
    j=j-1;
    
    for i=j:-1:1
      %pause(.05);
      Optimal_path(i,1) = (Optimal_path(i,1) / 10) - 1.5;
      Optimal_path(i,2) = (Optimal_path(i,2) / 10) - 1;
      set(p,'XData',Optimal_path(i,1),'YData',Optimal_path(i,2));

    end
    plot(Optimal_path(:,1),Optimal_path(:,2));
    else
     disp("No path yet)");
    end
end
[max_row, max_col] = size(Optimal_path);
if (max_row>1)
    final_goal_points(1, 1) = Optimal_path((max_row - 1),1);
    final_goal_points(2, 1) = Optimal_path((max_row - 1),2);
end
%% ===========================================================================
        
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
            xt = [x(2,1); x(2,2)]; % current robot position

            % Define reward function
            if prev_action == 0 || prev_action == -1
                disp(")0000000000000000000000000000000");
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



overlap = overlap / 8;
overlap = -overlap;

distanceFrom = distance(robotX,robotY,humanX, humanY);

prob_coll = overlap / distanceFrom;

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



