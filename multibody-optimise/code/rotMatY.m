function [r] = rotMatY(th)
r = [cosd(th) 0 sind(th) 0 1 0  -sind(th) 0 cosd(th)] ;
end