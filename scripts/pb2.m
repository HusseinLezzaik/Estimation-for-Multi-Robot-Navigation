%ARS4 mini-project
clear
close all
clc
animation = false;
plots = true;
%% Calibration data
x_std_f = 1.0550132608651832;
y_std_f = 0.8851236902556164;
yaw_std_f = 0.0398325634250031;

yaw_rate_offset_f = 0.003264;
speed_scale_f = 1.0124;
lever_arm_f = 0.4847;

speed_std_f = 0.10344498814394902;
yaw_rate_std_f = 0.04471455284474848;

x_std_l = 1.0156331264014704;
y_std_l = 0.9009600934652383;
yaw_std_l = 0.03621906547404191;

yaw_rate_offset_l = 0.003325;
speed_scale_l = 1.0157;
lever_arm_l = 0.5284;

speed_std_l = 0.10585673297965029;
yaw_rate_std_l = 0.04346252687730287;

%% Datasets
data_lidar_f = readtable('../csv_sync/follower_plicp.csv');
data_gnss_f = readtable('../csv_sync/sync_follower_gnss.csv');
data_kinetics_f = readtable('../csv_sync/sync_follower_kinetics.csv');
data_gnss_ref_f = readtable('../csv_sync/sync_follower_gnss_ref.csv');

data_gnss_l = readtable('../csv_sync/sync_leader_gnss.csv');
data_kinetics_l = readtable('../csv_sync/sync_leader_kinetics.csv');
data_gnss_ref_l = readtable('../csv_sync/sync_leader_gnss_ref.csv');

%% Map
data_map = readtable('../csv_sync/map_seville.csv');
%% init
X_f = [-250 340 0 0 0 0.08]';
P_f = eye(6);
P_f(1,1) = 100^2;
P_f(2,2) = 100^2;
P_f(3,3) = pi^2;
P_f(4,4) = speed_std_f^2;
P_f(5,5) = yaw_rate_std_f^2;
P_f(6,6) = 0.002^2;

X_l = [-250 340 0 0 0 0.08]';
P_l = eye(6);
P_l(1,1) = 100^2;
P_l(2,2) = 100^2;
P_l(3,3) = pi^2;
P_f(4,4) = speed_std_l^2;
P_f(5,5) = yaw_rate_std_l^2;
P_f(6,6) = 0.002^2;

jacob = @(X)[1   0   -X(6)*X(4)*sin(X(3))   X(6)*cos(X(3))    0       X(4)*cos(X(3))
             0   1    X(6)*X(4)*cos(X(3))   X(6)*sin(X(3))    0       X(4)*sin(X(3))
             0   0            1                    0         X(6)         X(5)
             0   0            0                    1          0            0
             0   0            0                    0          1            0
             0   0            0                    0          0            1        ];

evolution = @(X) [X(1)+X(6)*X(4)*cos(X(3))
                  X(2)+X(6)*X(4)*sin(X(3))
                       X(3)+X(6)*X(5)
                             X(4)
                             X(5)
                             X(6)        ];
                             
                
%%
zoom = 20;
sleeping_time = 0.001;
nb_steps=size(data_lidar_f,1);
skip_steps=1;

covs_f_hist = zeros(3,nb_steps);
covs_l_hist = zeros(3,nb_steps);

error_f = zeros(3,nb_steps);
error_l = zeros(3,nb_steps);

mahalanobises = zeros(1,nb_steps);
outlier_thresh = 100;
for step = 1:skip_steps:nb_steps
    %% Pose LiDAR
    pos_lidar_f = [data_lidar_f.x(step); data_lidar_f.y(step); data_lidar_f.yaw(step)];
    cov_lidar_f = [data_lidar_f.cov_x(step)        data_lidar_f.cov_xy(step)     data_lidar_f.cov_xyaw(step); 
                          data_lidar_f.cov_xy(step)      data_lidar_f.cov_y(step)       data_lidar_f.cov_yyaw(step);
                          data_lidar_f.cov_xyaw(step)  data_lidar_f.cov_yyaw(step) data_lidar_f.cov_yaw(step)];
    
    %% Pose GNSS for the follower                
    pos_gnss_f = [data_gnss_f.x(step)+data_gnss_f.bias_x(step); 
                            data_gnss_f.y(step)+data_gnss_f.bias_y(step);
                            data_gnss_f.yaw(step)];
                        
    cov_gnss_f = [x_std_f^2  0.0  0.0; 
                            0.0  y_std_f^2  0.0;
                            0.0  0.0  yaw_std_f^2];
    
    %% Speed and yaw rate for the follower
    yaw_rate =  data_kinetics_f.yaw_rate(step) + yaw_rate_offset_f;
    speed = speed_scale_f*data_kinetics_f.lon_vel(step) - lever_arm_f*abs(yaw_rate);
    vel_f = [speed; yaw_rate];
    cov_vel_f = [speed_std_f^2  0.0;
                        0.0 yaw_rate_std_f^2];
                        
    %% Ground truth for the follower      
    pos_ref_f = [data_gnss_ref_f.x(step); data_gnss_ref_f.y(step); data_gnss_ref_f.yaw(step)];
    cov_ref_f = [data_gnss_ref_f.x_std(step)^2  0.0 0.0; 
                        0.0  data_gnss_ref_f.y_std(step)^2  0.0;
                        0.0  0.0  data_gnss_ref_f.yaw_std(step)^2];
    
    %% Pose GNSS for the leader                
    pos_gnss_l = [data_gnss_l.x(step)+data_gnss_l.bias_x(step); 
                           data_gnss_l.y(step)+data_gnss_l.bias_y(step); 
                           data_gnss_l.yaw(step)];
                       
    cov_gnss_l = [x_std_l^2  0.0  0.0; 
                            0.0  y_std_l^2  0.0;
                            0.0  0.0  yaw_std_l^2];
    
    %% Speed and yaw rate for the leader
    yaw_rate =  data_kinetics_l.yaw_rate(step) + yaw_rate_offset_l;
    speed = speed_scale_l*data_kinetics_l.lon_vel(step) - lever_arm_l*abs(yaw_rate);
    vel_l = [speed; yaw_rate];
    cov_vel_l = [speed_std_l^2  0.0;
                        0.0 yaw_rate_std_l^2];
                        
    %% Ground truth for the leader      
    pos_ref_l = [data_gnss_ref_l.x(step); data_gnss_ref_l.y(step); data_gnss_ref_l.yaw(step)];
    cov_ref_l = [data_gnss_ref_l.x_std(step)^2  0.0 0.0; 
                        0.0  data_gnss_ref_l.y_std(step)^2  0.0;
                        0.0  0.0  data_gnss_ref_l.yaw_std(step)^2];
    %% Period observation
    if step == 1
        obv_dt = 0.08;
    else
        obv_dt = data_lidar_f.time(step)-data_lidar_f.time(step-1);
    end
    %% Filter
    
    % measurement
    
      
    
    R_f = blkdiag(cov_gnss_f,cov_vel_f,0.0^2);
    C_f = eye(6);
    y_f = [pos_gnss_f;vel_f;obv_dt];
    K_f = P_f*C_f'/(C_f*P_f*C_f'+R_f);
    innov = (y_f - C_f*X_f);
    innov(3) = wrap_angle(innov(3));
    X_f = X_f + K_f*innov;
    P_f = (eye(6)-K_f*C_f)*P_f;
    
    X_f(3) = wrap_angle(X_f(3));
    
    
    if data_lidar_f.usable(step)
        [pos_f_in_l, cov_f_in_l] = ominus(pos_lidar_f, cov_lidar_f);
        pos_f_in_l(3) = wrap_angle(pos_f_in_l(3));
        
        [pos_f_from_lidar, cov_f_from_lidar] = oplus(X_l(1:3),pos_f_in_l,P_l(1:3,1:3),cov_f_in_l);
        pos_f_from_lidar(3) = wrap_angle(pos_f_from_lidar(3));
        
        R_f = cov_f_from_lidar;
        C_f = [eye(3) zeros(3)];
        y_f = pos_f_from_lidar;
        
        mahalanobises(step) = mahalanobis(y_f, X_f, C_f, P_f, R_f);
        
        if mahalanobises(step) < outlier_thresh
            K_f = P_f*C_f'/(C_f*P_f*C_f'+R_f);
            innov = (y_f - C_f*X_f);
            innov(3) = wrap_angle(innov(3));
            X_f = X_f + K_f*innov;
            P_f = (eye(6)-K_f*C_f)*P_f;

            X_f(3) = wrap_angle(X_f(3));
        end

    end

    
    
    R_l = blkdiag(cov_gnss_l,cov_vel_l,0.0^2);
    C_l = eye(6);
    y_l = [pos_gnss_l;vel_l;obv_dt];
    K_l = P_l*C_l'/(C_l*P_l*C_l'+R_l);
    innov = (y_l - C_l*X_l);
    innov(3) = wrap_angle(innov(3));
    X_l = X_l + K_l*innov;
    P_l = (eye(6)-K_l*C_l)*P_l;
    
    X_l(3) = wrap_angle(X_l(3));
    
    % saving
    
    poses_l = X_l(1:3);
    covs_l = P_l(1:3,1:3);
    ref_poses_l = pos_ref_l;
    ref_covs_l = cov_ref_l;
    
    poses_f = X_f(1:3);
    covs_f = P_f(1:3,1:3);
    ref_poses_f = pos_ref_f;
    ref_covs_f = cov_ref_f;
    
    % data for consistency
    error_f(:,step) = X_f(1:3) - pos_ref_f;
    error_l(:,step) = X_l(1:3) - pos_ref_l;
    error_f(3,step) = wrap_angle(error_f(3,step));
    error_l(3,step) = wrap_angle(error_l(3,step));

    covs_f_hist(:,step) = diag(P_f(1:3,1:3));
    covs_l_hist(:,step) = diag(P_l(1:3,1:3));

    % prediction
   
    Q_l = diag([.3 .3 .3 speed_std_l^2 yaw_rate_std_l^2 0.002^2]);
    A_l = jacob(X_l);
    X_l = evolution(X_l);
    P_l = A_l*P_l*A_l' + Q_l;
    
    X_l(3) = wrap_angle(X_l(3));
    
    Q_f = diag([.8 .8 .8 speed_std_f^2 yaw_rate_std_f^2 0.002^2]);
    A_f = jacob(X_f);
    X_f = evolution(X_f);
    P_f = A_f*P_f*A_f' + Q_f;
    
    X_f(3) = wrap_angle(X_f(3));
    
    % plots
    if animation
        figure(1); clf;
        % Map display
        plot(data_map.x, data_map.y, '-k'); hold on;
        % Display of the ground truth of the leader Vehicle
        displayPos(pos_ref_l, vel_l(1), 'k');
        displayCov(pos_ref_l, cov_ref_l, 0.95, 'k');
        % Display of the Ublox receiver computed position of the leader Vehicle
        displayPos(poses_l, 1, 'b');
        displayCov(poses_l, covs_l, 0.95, 'b');
        % Display of the ground truth of the follower Vehicle
        displayPos(pos_ref_f, vel_f(1), 'k');
        displayCov(pos_ref_f, cov_ref_f, 0.95, 'k');
        % Display of the Ublox receiver computed position of the follower Vehicle
        displayPos(poses_f, 1, 'b');
        displayCov(poses_f, covs_f, 0.95, 'b');
        if zoom ~= 0 %zoom in view
            xlim([pos_ref_f(1)-zoom pos_ref_f(1)+zoom]);
            ylim([pos_ref_f(2)-zoom pos_ref_f(2)+zoom]);
        end
        xlabel('m');ylabel('m');
        title(['The two cars on SEVILLE (',num2str(round(100*step/nb_steps)),'% of the test)']);

        pause(sleeping_time);
    end
end



if plots
    figure("name","Follower Pos errors within +/- 3-sigma bounds","position",[300,100,1000,500])
    subplot(3,1,1)
    plot(1:nb_steps, -3*sqrt(covs_f_hist(1,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_f_hist(1,:)))
    plot(1:nb_steps, error_f(1,:))

    ylabel("pos\_x error (m)")
    title("pos\_x errors within +/- 3-sigma bounds")

    subplot(3,1,2)
    plot(1:nb_steps, -3*sqrt(covs_f_hist(2,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_f_hist(2,:)))
    plot(1:nb_steps, error_f(2,:))

    ylabel("pos\_y error (m)")
    title("pos\_y errors within +/- 3-sigma bounds")

    subplot(3,1,3)
    plot(1:nb_steps, -3*sqrt(covs_f_hist(3,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_f_hist(3,:)))
    plot(1:nb_steps, error_f(3,:))
    xlabel("step")
    ylabel("pos\_yaw error (rad)")
    title("pos\_yaw errors within +/- 3-sigma bounds")


    figure("name","Leader Pos errors within +/- 3-sigma bounds","position",[300,100,1000,500])
    subplot(3,1,1)
    plot(1:nb_steps, -3*sqrt(covs_l_hist(1,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_l_hist(1,:)))
    plot(1:nb_steps, error_l(1,:))

    ylabel("pos\_x error (m)")
    title("pos\_x errors within +/- 3-sigma bounds")

    subplot(3,1,2)
    plot(1:nb_steps, -3*sqrt(covs_l_hist(2,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_l_hist(2,:)))
    plot(1:nb_steps, error_l(2,:))

    ylabel("pos\_y error (m)")
    title("pos\_y errors within +/- 3-sigma bounds")

    subplot(3,1,3)
    plot(1:nb_steps, -3*sqrt(covs_l_hist(3,:)))
    hold on
    plot(1:nb_steps, 3*sqrt(covs_l_hist(3,:)))
    plot(1:nb_steps, error_l(3,:))
    xlabel("step")
    ylabel("pos\_yaw error (rad)")
    title("pos\_yaw errors within +/- 3-sigma bounds")
end
disp("Percentage of follower pos_x errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_f(1,:)) < 3*sqrt(covs_f_hist(1,:)))))

disp("Percentage of follower pos_y errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_f(2,:)) < 3*sqrt(covs_f_hist(2,:)))))

disp("Percentage of follower pos_yaw errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_f(3,:)) < 3*sqrt(covs_f_hist(3,:)))))

disp("Percentage of leader pos_x errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_l(1,:)) < 3*sqrt(covs_l_hist(1,:)))))

disp("Percentage of leader pos_y errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_l(2,:)) < 3*sqrt(covs_l_hist(2,:)))))

disp("Percentage of leader pos_yaw errors inside +/- 3-sigma bounds:")
disp(100/nb_steps*(sum(abs(error_l(3,:)) < 3*sqrt(covs_l_hist(3,:)))))

disp("Mean of (x,y,yaw) follower errors:")
disp(mean(error_f,2))
disp("Max of (x,y,yaw) follower errors:")
disp(max(abs(error_f),[],2))

disp("Mean of (x,y,yaw) leader errors:")
disp(mean(error_l,2))
disp("Max of (x,y,yaw) leader errors:")
disp(max(abs(error_l),[],2))


figure
scatter(1:nb_steps, mahalanobises)
xlabel("step")
ylabel("Mahalanobis Distance Squared")
 function test = mahalanobis(y_meas,X,C,P,R)
        obv = C*X;
        test = ((obv-y_meas)'/(C*P*C'+R))*(obv-y_meas);
  end