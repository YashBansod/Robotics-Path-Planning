function status = isColliding(point,map)
if(~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1))
    status = true;
else
    status = false;
end