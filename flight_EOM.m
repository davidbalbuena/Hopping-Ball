function [ ydot ] = flight_EOM( t,y )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here
g = 9.81; %m/s^2

ydot = [y(2); -g];

end

