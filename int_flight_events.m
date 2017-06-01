function [ value,isterminal,direction ] = int_flight_events( t,y )
%This is the function for the events call in ode45 options that is used for
%the flight phase of the system
value = y(1);   %when position = 0;
isterminal = 1; %stop when this condition is found (will not terminate at IC)
direction = 0;  %direction doesnt matter
end

