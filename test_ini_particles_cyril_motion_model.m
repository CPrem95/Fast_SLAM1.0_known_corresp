% This example uses the sampled odometry motion model to generate a set of
% particles. Change:
% n = number of particles
% alp = [distribution of particles]\

xt = [0,0,0];

alp = [100 1 0.01 0.005];
alp = [1.5 1.5 0.01 0.005];
% alp = [1 1 1 1];
n = 500;
xT = zeros(3, n);

for i = 1:n
    %     xT(:,i) = sample_motion_model_odometry(ut, xt, alp);
    xT(:,i) = path_posterior(u ,xt');
end

figure()
% Initial pose
sc1 = scatter(xt(1), xt(2));
sc1.MarkerEdgeColor = 'red';
sc1.SizeData = 500;
hold on
r = 0.001;
px1 = [xt(1) xt(1) + r*cos(xt(3))];
px2 = [xt(2) xt(2) + r*sin(xt(3))];
line(px1, px2, 'Color', 'red');
hold on

% Next pose
sc1 = scatter(odom(2,1), odom(2,2));
sc1.MarkerEdgeColor = 'red';
sc1.SizeData = 500;
hold on

% True pose
sc1 = scatter(path(2,1), path(2,2));
sc1.MarkerEdgeColor = 'green';
sc1.SizeData = 500;
hold on
r = 0.0001;
px1 = [path(2, 1) path(2, 1) + r*cos(path(2, 3))];
px2 = [path(2, 2) path(2, 2) + r*sin(path(2, 3))];
line(px1, px2, 'Color', 'green');
hold on

sc2 = scatter(xT(1,:), xT(2,:), 5, 'filled', 'blue'); % Visualize new
sc2.AlphaData = 0.001*ones(n,1);
sc2.MarkerFaceAlpha = 'flat';
hold on

r = 0.0001
for i = 1:n
    px3 = [xT(1, i) xT(1, i) + r*cos(xT(3, i))];
    px4 = [xT(2, i) xT(2, i) + r*sin(xT(3, i))];
    line(px3, px4, 'Color', 'blue');
    hold on
end

axis equal