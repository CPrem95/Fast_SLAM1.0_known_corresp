function xT = sample_motion_model_odometry(ut, xt, alp)
%Probabilistic robotics pg136
%   ut = input controls = [x_bar_t-1    x_bar_t]
%   xt = previous pose = [x0, y0, th0]
a1 = alp(1);
a2 = alp(2);
a3 = alp(3);
a4 = alp(4);

xbt = ut(1,:);
xbT = ut(2,:);

xb0 = xbt(1);
yb0 = xbt(2);
thb0 = xbt(3);

xb1 = xbT(1);
yb1 = xbT(2);
thb1 = xbT(3);

x0 = xt(1);
y0 = xt(2);
th0 = xt(3);

dr1 = atan2(yb1 - yb0, xb1 - xb0) - thb0;
dtr = sqrt((xb0 - xb1)^2 + (yb0 - yb1)^2);
dr2 = thb1 - thb0 - dr1;

dhr1 = dr1 - sample_norm_dist(a1*dr1^2 + a2*dtr^2);
dhtr = dtr - sample_norm_dist(a3*dtr^2 + a4*dr1^2 + a4*dr2^2);
dhr2 = dr2 - sample_norm_dist(a1*dr2^2 + a2*dtr^2);

x1 = x0 + dhtr*cos(th0 + dhr1);
y1 = y0 + dhtr*sin(th0 + dhr1);
th1 = th0 + dhr1 + dhr2;

xT = [x1, y1, th1]';
end

function sample = sample_norm_dist(b)
% b is the standard deviation
    sample = 0;
    for i = 1:12
        sample = sample + b*randn(1,1);
    end
    sample = 0.5*sample;
end