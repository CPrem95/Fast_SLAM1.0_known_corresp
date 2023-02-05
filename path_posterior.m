function [x] = path_posterior(u, x_prev)

mul = 1;

tran = u(1) + mul*normrnd(0, 0.007) ;
rot1 = u(2) + mul*normrnd(0, 0.2);
% rot2 = u(3) + mul*normrnd(0, 0.1);


% odometry motion model
odo = [tran*cos(x_prev(3) + rot1); % mu, sigma
       tran*sin(x_prev(3) + rot1);
       u(2) + u(3) + mul*normrnd(0, 0.01)];

% next particle (sample pose)
x =  x_prev + odo;

end