function status = isPathColliding(currSamplePoint,destSamplePoint,map)
status = false;
dir = atan2(destSamplePoint(1)-currSamplePoint(1),destSamplePoint(2)-currSamplePoint(2));
for r = 0:1:sqrt((currSamplePoint(1)-destSamplePoint(1))^2+(currSamplePoint(2)-destSamplePoint(2))^2)
    poCheck = currSamplePoint+r.*[sin(dir) cos(dir)];
    if (isColliding(ceil(poCheck),map)&& isColliding(floor(poCheck),map)&&...
          isColliding([ceil(poCheck(1)) floor(poCheck(2))],map) && isColliding([floor(poCheck(1)) ceil(poCheck(2))],map))
        status = true;
        break;
    end
    if isColliding(destSamplePoint,map)
        status = true;
    end
end