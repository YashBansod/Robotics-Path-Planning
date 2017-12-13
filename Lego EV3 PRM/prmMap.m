function [samplePoints, trajectory] = prmMap(map ,start_coords, dest_coords)
%% Error Handling
if isColliding(start_coords, map)                   % Print Error Message if
    error('Start point is on an obstracle');        % Start point lies on an obstacle
end
if isColliding(dest_coords, map)                    % Print Error Message if
    error('Destination point is on an obstracle');  % Destination point lies on an obstacle
end

%% Sample Point Initialization
numSamplePoints = 50;                              % Number of points to sample
samplePoints = [start_coords;dest_coords];          % Add start and destination points as sample points

% Sample points on map till 'numSamplePoints' without collision with obstacles are sampled
while (length(samplePoints) < numSamplePoints + 2)
    
    sample = double(int32(rand(1,2) .*size(map)));  % Calculate a Random point
    
    if ~isColliding(sample, map)                    % If sample is not colliding with any obstacle
        samplePoints = [samplePoints;sample];       % Append the sample to the list of sampled Points
        plot(sample(2),sample(1),'r*', 'MarkerSize', 5);    % Mark the sample on the map
        hold on;
    end
end
drawnow;

% Connect the sampled points. The edge must not collide with an obstacle
edges = cell(numSamplePoints+2 ,1);                 % Create an empty matrix to store edges

for i = 1:length(samplePoints)                      % For every sample point
    for j = i+1:length(samplePoints)                % Check connectivity with all other sample points        
        if ~isPathColliding(samplePoints(i,:),samplePoints(j,:),map)    % If non-collision path is found
            edges{i} = [edges{i};j];                                    % Add the path as edge
            edges{j} = [edges{j};i];                                    % Add the path as edge
            line([samplePoints(i,2);samplePoints(j,2)],[samplePoints(i,1);samplePoints(j,1)]); % Mark the edges on the map
            hold on;
        end       
    end
    drawnow;
end


% vertex, historic cost, heuristic cost, total cost, parent index
Q = [1 0 Dist(samplePoints(1,:),dest_coords) 0+Dist(samplePoints(1,:),dest_coords) -1];
closed = [];
trajectoryFound = false;

% Compute the trajectory using sample points and edges
while size(Q,1)>0
    [A,  I] = min(Q,[],1);
    n = Q(I(4),:);
    Q = [Q(1:I(4)-1,:);Q(I(4)+1:end,:)];
    
    if n(1) == 2
        trajectoryFound = true;
        break;
    end
    
    for m = 1:1:length(edges{n(1),1})
        newV = edges{n(1),1}(m);
        if (length(closed)==0||length(find(closed(:,1)==newV))==0)
            histCost = n(2)+Dist(samplePoints(n(1),:),samplePoints(newV,:));
            heurist = Dist(samplePoints(newV,:),dest_coords);
            total = histCost+heurist;
            add = true;
            if length(find(Q(:,1)==newV))>=1
                I = find(Q(:,1) == newV);
                if(Q(I,4)<total)
                    add = false;
                else
                    Q=[Q(1:I-1,:);Q(I+1:end,:);];
                    add = true;
                end
            end
            if add
                Q = [Q;newV histCost heurist total size(closed,1)+1];
            end
        end
    end
    
    closed = [closed;n];
    
end

if ~trajectoryFound
    error('trajectory not found')
end

trajectory = [samplePoints(n(1),:)];
prev = n(5);

while prev > 0
    trajectory = [samplePoints(closed(prev,1),:);trajectory];
    prev = closed(prev,5);
end

end