function [ ydot ] = stance_EOM( t,y )
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
[m,b,k] = constants();

ydot = [y(2);0 - (b/m)*y(2) - (k/m)*y(1)];

end

