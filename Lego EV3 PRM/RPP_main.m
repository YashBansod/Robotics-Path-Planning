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
inputImage = imread('PRM.jpg');             % Read the input image from file system
inputImage = imresize(inputImage,0.049);    % Resize the image to a smaller size
subplot(2,2,1);
imshow(inputImage);
title('Input Image')

inputImage = inputImage(:,:,1);             % Read only the red channel

% Do morphological operation to extract usable data
map = im2bw(inputImage, 0.4);
structDilate = strel('square', 2);          % Create a structure for Dilation operation
structErode = strel('square', 21);          % Create a structure for Erosion operation
map = imdilate(map,structDilate);           % Dilate the image to remove small black points in map
map = imerode(map,structErode);             % Erode the image to remove small white points in map
% map = ~map;                               % Invert the map

[nrows, ncols] = size(map);                 % Save the size of the map
workspaceMap = zeros(nrows,ncols);          % Create map to save the states of each grid cell
workspaceMap(~map) = 2;                     % Mark obstacle cells on map
workspaceMap(map) = 1;                      % Mark free cells on map

start_coords = [72, 15];                    % Save the location of start coordinate
dest_coords = [55, 130];                    % Save location of destination coordinate
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
    0.5 0.5 0.5];                           % Mark trajectory as Grey
colormap(cmap);                             % Sets the colormap for the current figure

% Display the workspaceMap
subplot(2,2,2);
image(1,1,workspaceMap)                     % Display the workspaceMap with scaled colors
title('Map after image processing')
grid on;                                    % Display grid lines
axis image;                                 % Set axis according to image
drawnow;

% Display the map
subplot(2,2,3);
image(1,1,~map)                             % Display the map with scaled colors
title('Map after PRM')
grid on;                                    % Display grid lines
axis image;                                 % Set axis according to image
drawnow;
hold on;

[samplePoints, trajectory] = prmMap(map, start_coords, dest_coords)
colormap(cmap);                             % Sets the colormap for the current figure

[nrows, ncols] = size(map);                 % Save the size of the map
finalMap = zeros(nrows,ncols);              % Create map to save the states of each grid cell
finalMap(~map) = 2;                         % Mark obstacle cells on map
finalMap(map) = 1;                          % Mark free cells on map

start_coords = [100, 150];                  % Save the location of start coordinate
dest_coords = [60, 20];                     % Save location of destination coordinate
start_node = sub2ind(size(map), start_coords(1), start_coords(2));      % Generate linear indices of start node
dest_node = sub2ind(size(map), dest_coords(1), dest_coords(2));         % Generate linear indices of dest node
finalMap(start_node) = 5;                   % Mark start node on map
finalMap(dest_node) = 6;                    % Mark destination node on map

%Display the workspaceMap
subplot(2,2,4);
image(1,1,finalMap)                         % Display the image with scaled colors
line(trajectory(:,2),trajectory(:,1),'color','r');
title('Map with Trajectory')
grid on;                                    % Display grid lines
axis image;                                 % Set axis according to image
drawnow;
colormap(cmap);                             % Sets the colormap for the current figure

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
for i = 1:length(trajectory)
    desiredPos = trajectory(i,:)
    currRotAngle = readRotationAngle(myGyroSensor)
    [currentPos] = go2Goal(motorRight, motorLeft, myGyroSensor, currentPos, desiredPos)
end
