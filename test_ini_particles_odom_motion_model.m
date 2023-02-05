% This example uses the sampled odometry motion model to generate a set of
% particles. Change:
% n = number of particles
% alp = [distribution of particles]\

xt = [0,0,0];

alp = [100 1 0.01 0.01];
alp = [1.5 1.5 0.01 0.005];
% alp = [1 1 1 1];
n = 500;
xT = zeros(3, n);

for i = 1:n
    xT(:,i) = sample_motion_model_odometry(ut, xt, alp);
end

figure()
sc1 = scatter(xt(1), xt(2));
sc1.MarkerEdgeColor = 'red';
sc1.SizeData = 500;
hold on

r = 0.001;
px1 = [xt(1) xt(1) + r*cos(xt(3))];
px2 = [xt(2) xt(2) + r*sin(xt(3))];
line(px1, px2, 'Color', 'red');
hold on

sc1 = scatter(ut(2,1), ut(2,2));
sc1.MarkerEdgeColor = 'red';
sc1.SizeData = 500;
hold on

sc1 = scatter(path(2,1), path(2,2));
sc1.MarkerEdgeColor = 'green';
sc1.SizeData = 500;
hold on

sc2 = scatter(xT(1,:), xT(2,:), 5, 'filled', 'blue'); % Visualize new
sc2.AlphaData = 0.001*ones(n,1);
sc2.MarkerFaceAlpha = 'flat';

axis equal