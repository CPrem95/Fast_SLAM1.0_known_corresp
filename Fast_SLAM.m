clc
close all
clear all

%% Setting up the Environment and the Robot
% Environment
env_lims = [12, 10]; % [xlimit, ylimit]
% env_lims = [1.2, 1.1]; % [xlimit, ylimit]
n_landm = 20;

% Robot path
path_start = [0, 0];
path_stop = [11, 10];
% path_stop = [1, 0.9];
exp_sample_n = 400; % number of expected samples in the range

% Robot's maximum radar range
rad_range = 10;

figure()

[env, plt1] = create_env(n_landm, env_lims, 'rand'); % 'predef' or 'rand'
hold on
% load('env')
% plt1 = scatter(env(:,1), env(:,2), '*');

[path, path_samples, plt2] = path_plan(path_start, path_stop, exp_sample_n, 'simple'); % 'simple' or 'comp'
hold on

plt4 = scatter(NaN, NaN, 'MarkerEdgeColor','#77AC30');
plt3 = scatter(NaN, NaN, 1, 'filled', 'blue');
plt5 = scatter(NaN, NaN, 1, 'filled', 'magenta');
% plt6 = line(NaN, NaN,'Color','#00FFFF');
legend([plt1, plt2, plt4, plt3, plt5],{'Landmarks', 'True path', 'Odometry', 'Estimated path particles', 'Estimated Landmarks', 'Observations'}, 'AutoUpdate','off');

xlim([-2 12])
ylim([-3 12])
% xlim([-0.1 1.2])
% ylim([-0.1 1.1])
[obs ,odom] = robot(env, path, rad_range, false);
hold on
scatter(odom(:,1), odom(:,2),'MarkerEdgeColor','#77AC30');
hold on
xlabel('X [m]')
ylabel('Y [m]')
% pause(1)
% load('obs.mat')
% load('odom.mat')

vis_pos = true;
vis_rt = false;

%% SLAM algo

% Observation uncertainty
% Qt = [0.25 0;
%     0 0.1];
Qt = [0.01 0;
    0 0.001];

initial_N = 500;
Y0 = zeros(3, initial_N);
Y1 = Y0;

[N_odom, ~] = size(odom);

mu = zeros(2*n_landm, initial_N);
w_prod = zeros(1, initial_N);
sig = zeros(2*n_landm, 2*initial_N);

% alp = [1.5 1.5 0.01 0.005];
w = zeros(n_landm, initial_N);

M = initial_N;
t = 1;
for k = 1:M % loop over all particles
    for j = 1:n_landm % loop over all observations in a odom position of a particle
        if  obs(t,j,3) ~= 0
            r = obs(t,j,1);
            psi = obs(t,j,2);
            z = [r, psi]';
            if (sig(2*j -1, 2*k -1) == 0 && sig(2*j, 2*k) == 0)
                rel_meas = [r*cos(psi + Y1(3, k)), r*sin(psi + Y1(3, k))]';
                mu(2*j -1:2*j, k) = Y1(1:2, k) + rel_meas;

                H = (1/r^2)*[rel_meas(1)*r, rel_meas(2)*r;
                    -rel_meas(2), rel_meas(1)];

                %                     sig(2*j -1:2*j, 2*k -1:2*k) = inv(H)\Qt/inv(H)';
                sig(2*j -1:2*j, 2*k -1:2*k) = inv(H)*Qt*inv(H)';
                w(j, k) = 0.3;
                disp('New lm')
            end
        end
    end
end

disp('origin initialization done')

% N_odom = 2;
for t = 2:N_odom % loop over all odom readings
    t
    w = zeros(n_landm, initial_N);
    %     ut = [odom(t -1, :); odom(t, :)];
    u = odom2u(odom(t -1, :), odom(t, :));
    for k = 1:M % loop over all particles
        %         Y1(:, k) = sample_motion_model_odometry(ut, Y0(:, k), alp);
        Y1(:, k) = path_posterior(u, Y0(:, k));
%         Y1(:, M) = path(2,:)';
        for j = 1:n_landm % loop over all observations in a odom position of a particle
            if  obs(t,j,3) ~= 0
                r = obs(t,j,1);
                psi = obs(t,j,2);
                z = [r, psi]';
                if (sig(2*j -1, 2*k -1) == 0 && sig(2*j, 2*k) == 0)
                    rel_meas = [r*cos(psi + Y1(3, k)), r*sin(psi + Y1(3, k))]';
                    mu(2*j -1:2*j, k) = Y1(1:2, k) + rel_meas;

                    H = (1/r^2)*[rel_meas(1)*r, rel_meas(2)*r;
                        -rel_meas(2), rel_meas(1)];

                    %                     sig(2*j -1:2*j, 2*k -1:2*k) = inv(H)\Qt/inv(H)';
                    sig(2*j -1:2*j, 2*k -1:2*k) = inv(H)*Qt*inv(H)';
                    w(j, k) = 0.3;
                    disp('New lm')

                else
                    del = [mu(2*j -1, k) - Y1(1, k), mu(2*j, k) - Y1(2, k)]';
                    q = del'*del;
                    sq = sqrt(q);
                    zn = [sq, atan2(del(2), del(1)) - Y1(3, k)]';

                    H = (1/q)*[del(1)*sq, del(2)*sq;
                        -del(2), del(1)];

                    Q = H*sig(2*j -1:2*j, 2*k -1:2*k)*H' + Qt;

                    K = sig(2*j -1:2*j, 2*k -1:2*k)*H'/Q;

                    mu(2*j -1:2*j, k) = mu(2*j -1:2*j, k) + K*(z - zn);

                    sig(2*j -1:2*j, 2*k -1:2*k) = (eye(2) - K*H)*sig(2*j -1:2*j, 2*k -1:2*k);

                    w(j, k) = det(2*pi*Q)^-0.5*exp(-0.5*(z - zn)'/Q*(z - zn));
                end
            end
        end
    end

    if vis_rt == true
        sc1 = scatter(Y1(1,:), Y1(2,:), 5, 'filled', 'blue'); % Visualize new
        sc1.AlphaData = 0.001*ones(length(Y1(1,:)),1);
        sc1.MarkerFaceAlpha = 'flat';
        hold on

        sc_lm = scatter(mu(1:2:2*n_landm,:), mu(2:2:2*n_landm,:), 4, 'filled',  'magenta');
        hold on

        pause(1);
        for v = 1:initial_N
            sc_lm(v).Visible = 'off';
        end
        %     sc_lm.Visible = 'off';
        sc1.Visible = 'off';
    end

    if vis_pos == true
        figure(1)
        sc1 = scatter(Y1(1,:), Y1(2,:), 5, 'filled', 'blue'); % Visualize new
        sc1.AlphaData = 0.001*ones(length(Y1(1,:)),1);
        sc1.MarkerFaceAlpha = 'flat';
        pause(0.01);
        hold on
%         pause(3)
        if t ~= N_odom
            delete(sc1);
        end
    end


    for p = 1:initial_N
        temp_w = w(:,p);
        idw = temp_w~=0;
        w_prod(p) = prod(temp_w(idw));
    end
%     figure(2);
%     plot(w_prod);
    
    max_resamp_n = 30;
    [~, max_w_prod_i] = maxk(w_prod, max_resamp_n);

    if vis_pos == true
        figure(1)
        sc2 = scatter(Y1(1,max_w_prod_i), Y1(2,max_w_prod_i), 5, 'filled', 'red'); % Visualize new
%         sc2.AlphaData = 0.001*ones(length(Y1(1,max_w_prod_i)),1);
%         sc2.MarkerFaceAlpha = 'flat';
%         pause(0.01);
%         pause(3)
        if t ~= N_odom
            delete(sc2);
        end
%         r = 0.0001;
%         for i = 1:initial_N
%             px3 = [Y1(1, i) Y1(1, i) + r*cos(Y1(3, i))];
%             px4 = [Y1(2, i) Y1(2, i) + r*sin(Y1(3, i))];
%             line(px3, px4, 'Color', 'blue');
%             hold on
%         end
    end

    for q = 1:initial_N
%         selected_part_ind = roulette_wheel(w_prod);
        Y0(:, q) = Y1(:, max_w_prod_i(randi(max_resamp_n,1)));
    end
end