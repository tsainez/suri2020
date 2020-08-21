function [outputArg1] = euclideanDistance(inputArg1,inputArg2)
%EUCLIDEANDISTANCE Function returns Euclidean distance between two 2x1 row
%vectors.
%   Row vectors components are used to create a hypotenuse that can be
%   measured as the distance, and that single value is returned. 
outputArg1 = sqrt((inputArg2(1,1)-inputArg1(1,1))^2+(inputArg2(1,2)-inputArg1(1,2))^2);
end