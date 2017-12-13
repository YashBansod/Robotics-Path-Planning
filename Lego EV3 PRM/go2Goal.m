function [currentPos] = go2Goal(motorRight, motorLeft, myGyroSensor, currentPos, desiredPos)
%% Variable Initializations
currY = currentPos(1,1);
currX = currentPos(1,2);

destY = desiredPos(1,1);
destX = desiredPos(1,2);

distance = sqrt(double(power((destX - currX),2) + power((destY - currY), 2)));

steadyStateRight = false;
steadyTestRight = 0;
currRotRight = 0;
destRotRight = round((360 * distance) / (pi * 5.5));

steadyStateLeft = false;
steadyTestLeft = 0;
currRotLeft = 0;
destRotLeft = round((360 * distance) / (pi * 5.5));

steadyStateRot = false;
steadyTestRot = 0;
destRotAngle = atan2d(double(destY - currY), double(destX - currX));

%% Start the P Control Loop for rotation of robot
while (steadyStateRot == 0)
    currRotAngle = readRotationAngle(myGyroSensor);
    errorAngle = destRotAngle - currRotAngle;
    
    % Control of Angle of Robot
    if(abs(errorAngle) > 10)
        steadyTestRot = 0;
        controlAngle = 1 * errorAngle;
        if (controlAngle > 100)
            controlAngle = 100;
        elseif (controlAngle < -100)
            controlAngle = -100;
        end
        motorLeft.Speed = controlAngle;
        motorRight.Speed = -controlAngle;
    elseif (abs(errorAngle) > 2)
        steadyTestRot = 0;
        controlAngle = 5 * errorAngle;
        motorLeft.Speed = controlAngle;
        motorRight.Speed = -controlAngle;
    else
        motorLeft.Speed = 0;
        motorRight.Speed = 0;
        steadyTestRot = steadyTestRot + 1;
        if (steadyTestRot > 100)
            steadyStateRot = 1;
        end
    end
end

resetRotation(motorRight)                   % Reset the reading from Right Motor
resetRotation(motorLeft)                    % Reset the reading from Left Motor

%% Start the P Control Loop for translation of robot
while (steadyStateLeft == 0 || steadyStateRight == 0)
    currRotLeft = readRotation(motorLeft);
    errorL = destRotLeft  - currRotLeft;
    
    currRotRight = readRotation(motorRight);
    errorR = destRotRight - currRotRight;
    
    % Control of Left Wheel of Robot
    if(abs(errorL) > 10)
        steadyTestLeft = 0;
        controlL = 0.6 * errorL;
        if (controlL > 100)
            controlL = 100;
        elseif (controlL < -100)
            controlL = -100;
        end
        motorLeft.Speed = controlL;
    elseif (abs(errorL) > 1)
        steadyTestLeft = 0;
        controlL = 2 * errorL;
        motorLeft.Speed = controlL;
    else
        motorLeft.Speed = 0;
        steadyTestLeft = steadyTestLeft + 1;
        if (steadyTestLeft > 10)
            steadyStateLeft = 1;
        end
    end
    
    % Control of Right Wheel of Robot
    if(abs(errorR) > 10)
        steadyTestRight = 0;
        controlR =  0.6 * errorR;
        if (controlR > 100)
            controlR = 100;
        elseif (controlR < -100)
            controlR = -100;
        end
        motorRight.Speed = controlR;
    elseif (abs(errorR) > 1)
        steadyTestRight = 0;
        controlR = 2 * errorR;
        motorRight.Speed = controlR;
    else
        motorRight.Speed = 0;
        steadyTestRight = steadyTestRight + 1;
        if (steadyTestRight > 10)
            steadyStateRight = 1;
        end
    end
end

%% Print the Final Positon and Orientation of the Robot

currRotAngle = readRotationAngle(myGyroSensor)     % Read the rotation angle of the Robot
currX = currX + (((readRotation(motorLeft) + readRotation(motorRight)) * 0.5 * pi * 5.5 * cosd(double(currRotAngle))) / (360))
currY = currY + (((readRotation(motorLeft) + readRotation(motorRight)) * 0.5 * pi * 5.5 * sind(double(currRotAngle))) / (360))
currentPos = [currY, currX];

end
