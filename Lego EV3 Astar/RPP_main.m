clear;
clc;

%% Lego EV3 Hardware Initialization
myLego = legoev3('usb');                    % Create Class object for legoev3

motorRight = motor(myLego,'B');             % Define Right motor as connected to port B
resetRotation(motorRight);                  % Reset the reading from Right Motor
motorRight.Speed = 0;                       % Initialize Right motor speed = 0;
start(motorRight);                          % Start the power to Right Motor
motorLeft = motor(myLego,'A');              % Define Left motor as connected to port A
resetRotation(motorLeft)                    % Reset the reading from Left Motor
motorLeft.Speed = 0;                        % Initialize Left motor speed = 0;
start(motorLeft);                           % Start the power to Left Motor

myGyroSensor = gyroSensor(myLego)           % Define Gyro Sensor connected to Lego EV3
resetRotationAngle(myGyroSensor)            % Reset the Rotation Angle of the gyro

%% Indicate completion of initialization by a beep
beep(myLego);
beep on;
pause(1);
beep off;

%% Read Image into the program
inputImage = imread('PRM.jpg');        % Read the input image from file system
inputImage = inputImage(:,:,1);             % Read only the red channel
inputImage = imresize(inputImage,0.05);     % Resize the image to a smaller size

% Do morphological operation to extract usable data
map = im2bw(inputImage, 0.4);
structDilate = strel('square', 3);          % Create a structure for Dilation operation
structErode = strel('square', 23);          % Create a structure for Erosion operation
map = imdilate(map,structDilate);           % Dilate the image to remove small black points in map
map = imerode(map,structErode);             % Erode the image to remove small white points in map
map = ~map;                                 % invert map

[nrows, ncols] = size(map);                 % Save the size of the map
workspaceMap = zeros(nrows,ncols);          % Create map to save the states of each grid cell
workspaceMap(~map) = 1;                     % Mark free cells on map
workspaceMap(map) = 2;                      % Mark obstacle cells on map

start_coords = [72, 15];                   % Save the location of start coordinate
dest_coords = [50, 130];                     % Save location of destination coordinate
start_node = sub2ind(size(map), start_coords(1), start_coords(2));      % Generate linear indices of start node
dest_node = sub2ind(size(map), dest_coords(1), dest_coords(2));         % Generate linear indices of dest node
workspaceMap(start_node) = 5;               % Mark start node on map
workspaceMap(dest_node) = 6;                % Mark destination node on map

% Create a color map
cmap = [1   1   1;                          % Mark Free Space as white
    0   0   0;                              % Mark Obstacles as black
    1   0   0;                              % Mark Processing points as Red
    0   0   1;                              % Mark Processed points as Blue
    0   1   0;                              % Mark Start point as Green
    1   1   0;                              % Mark Destination point as Yellow
    1   0   1;                              % Mark Trajectory points as Violet
    0.5 0.5 0.5];                           % Mark Path as Grey
colormap(cmap);                             % Sets the colormap for the current figure

distanceFromStart = Inf(nrows,ncols);       % Initialize distance from start array to inifinity
distanceFromEnd = Inf(nrows,ncols);         % Initialize distance from end array to inifinity

parent = zeros(nrows, ncols);               % Create a map for holding parent's index for each grid cell

distanceFromStart(start_node) = 0;          % distance of start node from start is zero
distanceFromEnd(dest_node) = 0;             % distance of end node from end is zero

% Update the values of all grid pixels for distance from end
[X, Y] = meshgrid(1:ncols, 1:nrows);
xd = dest_coords(1);
yd = dest_coords(2);
distanceFromEnd = abs(X - yd) + abs(Y - xd);

% Display the map
figure(1)
image(1,1,workspaceMap)                     % Display the image with scaled colors
grid on;                                    % Display grid lines
axis image;                                 % Set axis according to image
drawnow;                                    % Update figure

drawMapEveryTime = true;                    % To see how nodes expand on the grid

%% Process the map to update the parent information and distance from start
while true                                  % Create an infinite loop
    workspaceMap(start_node) = 5;           % Mark start node on map
    workspaceMap(dest_node) = 6;            % Mark destination node on map
    
    if (drawMapEveryTime)
        image(1, 1, workspaceMap);
        grid on;                            % Display grid lines
        axis image;                         % Set axis limits
        drawnow;                            % Update figure
    end
    
    % Find the node with the minimum heuristic distance
    heuristicDist = (distanceFromStart) + distanceFromEnd;
    [min_dist, current] = min(heuristicDist(:));
    
    % Compute row, column coordinates of current node from linear index
    [i, j] = ind2sub(size(heuristicDist), current);
    
    % Create an exit condition for the infinite loop to end
    if ((current == dest_node) || isinf(min_dist)) break
    end
    
    % Update distance value of element right of current element
    if (i+1 <= nrows && distanceFromStart(i+1, j) > distanceFromStart(i,j) + 1)
        if (parent(i+1, j) == 0 && map(i+1,j)~=1 && parent(current)~= sub2ind(size(map), i+1, j))
            distanceFromStart(i+1, j) = distanceFromStart(i,j) + 1;
            workspaceMap(sub2ind(size(map), i+1, j)) = 4;    % Mark the neighbour of current as processing
            parent(i+1, j)= current;
        end
    end
    
    % Update distance value of element left of current element
    if (i-1 >= 1 && distanceFromStart(i-1, j) > distanceFromStart(i,j) + 1)
        if (parent(i-1, j) == 0 && map(i-1,j)~=1 && parent(current)~= sub2ind(size(map), i-1, j))
            distanceFromStart(i-1, j) = distanceFromStart(i,j) + 1;
            workspaceMap(sub2ind(size(map), i-1, j)) = 4;    % Mark the neighbour of current as processing
            parent(i-1, j)= current;
        end
    end
    
    % Update distance value of element top of current element
    if (j-1 >= 1 && distanceFromStart(i, j-1) > distanceFromStart(i,j) + 1)
        if (parent(i, j-1) == 0 && map(i,j-1)~=1 && parent(current)~= sub2ind(size(map), i, j-1))
            distanceFromStart(i, j-1) = distanceFromStart(i,j) + 1;
            workspaceMap(sub2ind(size(map), i, j-1)) = 4;    % Mark the neighbour of current as processing
            parent(i, j-1)= current;
        end
    end
    
    % Update distance value of element bottom of current element
    if (j+1 <= ncols && distanceFromStart(i, j+1) > distanceFromStart(i,j) + 1)
        if (parent(i, j+1) == 0 && map(i,j+1)~=1 && parent(current)~= sub2ind(size(map), i, j+1))
            distanceFromStart(i, j+1) = distanceFromStart(i,j) + 1;
            workspaceMap(sub2ind(size(map), i, j+1)) = 4;    % Mark the neighbour of current as processing
            parent(i, j+1)= current;
        end
    end
    
    distanceFromStart(current) = -log(0);           % change the distance of current from start as infinity
    workspaceMap(current) = 3;                      % mark the current point as processed
end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node))) route = [];    % if distance to destination node is infinity
else route = [dest_node];                               % else backtrace the route from destination node
    while (parent(route(1)) ~= 0)                       % check front of route for start node
        route = [parent(route(1)), route];              % add parent of current node to front of route
    end
    
    for k = 2:length(route) - 1                         % To visualize the map and the path
        workspaceMap(route(k)) = 8;
        image(1, 1, workspaceMap);
        grid on;                                        % Display grid lines
        axis image;                                     % Set axis lengths
        drawnow;
    end
end

%% Reduce the trajectory to points connectable by straight lines
[trajX, trajY] = ind2sub(size(map), route);
trajectory = [trajX; trajY];
trajectory = transpose(trajectory);
[tRow, tCol] = size(trajectory);

i = 1;
while (i < tRow -1)
    if (trajectory(i, 1) == trajectory(i+1, 1) && trajectory(i+1, 1) == trajectory(i+2, 1))
        trajectory(i+1, :) = [];
        [tRow, tCol] = size(trajectory);
    elseif (trajectory(i, 2) == trajectory(i+1, 2) && trajectory(i+1, 2) == trajectory(i+2, 2))
        trajectory(i+1, :) = [];
        [tRow, tCol] = size(trajectory);
    else
        i = i+1;
    end
end

%% Mark the Trajectory Points on the Map
for k = 1:tRow                          % To visualize the map and the path
    workspaceMap(sub2ind(size(map), trajectory(k,1), trajectory(k,2))) = 7;
    image(1, 1, workspaceMap);
    grid on;                            % Display grid lines
    axis image;                         % Set axis lengths
    drawnow;
end

%% Indicate completion of initialization by a beep
beep(myLego);
beep on;
pause(0.5);
beep off;
pause(0.5);
beep on;
pause(0.5);
beep off;

%% Pass the trajectory points to Go to Goal
currentPos = trajectory(1,:)
for i = 1:tRow
    desiredPos = trajectory(i,:)
    currRotAngle = readRotationAngle(myGyroSensor)
    [currentPos] = go2Goal(motorRight, motorLeft, myGyroSensor, currentPos, desiredPos)
end
