function [ t,y,stance ] = damper_bounce( drop_height )
%This function calculates the dynamic response of a bouncing ball using
%hybrid dynamics with a ballistic part and a contact part. The contact
%dynamics are modeled as a mass spring damper with stiffness that controls
%the stance time of the ball and damping that controls the coefficient of
%restitution of the ball.
%Inputs: Initial drop height
%Outputs: time vector (t), state vector (y), stance vector that has the
%stance times for each contact with the ground

tol = .1;    %stop computing once height gets below this value
tspan = [0:0.01:50];   %overestimate time span

%set integrator options  - should i try just making a struct array?
int_options_flight = odeset('events',@int_flight_events);
int_options_stance = odeset('events', @int_stance_events);

t = [0];                %initialize time vector
y = [drop_height, 0];   %initialize solution vector
y_init = [drop_height 0];   %initial conditions
stance = [];        %will record the stance time for each stance phase

peak = [];
max_height = drop_height;


for ctr = 1:10      %run for 10 cycles
% while( max_height > tol)
%solve flight phase ode until it hits the ground
[t_flight, y_flight,~,~,~] = ...
    ode45(@flight_EOM,tspan,y_init,int_options_flight);

%calculate maximum height for this cycle to see if loop should stop
max_height = abs(max(y_flight(:,1)));
peak = [peak; max_height];  %records the peak for each cycle
%update total time and state vectors
t = [t; t_flight + t(end)];
y = [y; y_flight];

%now use a mass spring damper model of contact to find the dynamic response
%to the ball hitting the ground and find the velocity after it is done
%compressing.

%find initial conditions for stance phase
y_init = [0, -1*y(end,2)];  %negative sign on velocity because coordinate systems are backwards
[t_stance, y_stance,te,ye,~] = ...
    ode45(@stance_EOM,tspan,y_init,int_options_stance);

%display stance contact model response
% figure();
% hold on;
% plot(t_stance,y_stance(:,1),'b',te,ye(:,1),'ro')
% % plot(t_stance,y_stance(:,2),'g',te,ye,'ro')
% legend('position','event');


% xlabel('time')
% title('Contact model response');


%now find the response at the end and that will be the initial conditions
%for the next flight phase.
y_init = [0, abs(y_stance(end,2))];

%record duration of stance phase for tuning purposes
stance = [stance; t_stance(end)];

end

coefficient_of_resitution = sqrt(peak(2)/peak(1));
disp('done');
end

