%%
% Data 분석용 코드
% SOC 0.8에서 진행하는 것으로 수행 ( 내부저항 일정하다는 가정)
% 분석 방향
% 1. alpha
% 2. vehicle flow
% 결국 어떤 환경에서 에너지 소비 효율과 degradation이 달라지느냐??
% 15건의 에피소드 
clc
clear
file_path = 'C:\Users\user\Downloads\BSL_\1_연구\EREV에너지관리연구\Code\EV-optimization-DraftV2\Save';
cd(file_path)
list = dir('*.mat');
%%
figure(1000)
hold on
plot(history.base_Q_loss,'*')
plot(history.applied_Q_loss,'o')
hold off
legend('Base','Optimized')
%% 우선 각 조건별 driving efficiency & degradation loss evaluation
drv_dist_before = zeros(15,1);
drv_dist_applied = zeros(15,1);
drv_dist_diff = zeros(15,1);

drv_spd_before = zeros(15,1);
drv_spd_applied = zeros(15,1);

pwr_before = zeros(15,1);
pwr_applied = zeros(15,1);

cons_before = zeros(15,1);
cons_applied = zeros(15,1);
% SOC 사용량
soc_before = zeros(15,1);
soc_applied = zeros(15,1);

degrad_loss_before = zeros(15,1);
degrad_loss_applied = zeros(15,1);

flow_rec = zeros(15,1);
alpha_rec = zeros(15,1);

before_spd = []; %
applied_spd = []; %

before_grade = [];
applied_grade = [];

before_drv_current = [];
applied_drv_current = [];

before_v_minmax = [];
applied_v_minmax = [];

before_spd_var = [];
applied_spd_var = []; %temp_hist.history.applied_Q_loss;

before_grade_var = [];
applied_grade_var = [];

Obj_SOC = [];
Obj_LOSS = [];
% alpha

for i = 1:size(list,1)
    %%
    cd(file_path)
    temp_hist = load(list(i).name);

    %%
    temp_str = strsplit(list(i).name,'_');
    temp_aph = cell2mat(temp_str(4));

    if strcmp(temp_aph,'alpha100')
        temp_aph = 'alpha1';
    elseif strcmp(temp_aph,'alpha200')
        temp_aph = 'alpha3';
    end
    %%
    % driving speed
    before_avg_spd = mean(temp_hist.history.base_v);
    applied_avg_spd = mean(temp_hist.history.applied_spd);

    % driving efficiency
    before_drv_dist = temp_hist.history.base_pos/1000; % km
    before_drv_pwr = temp_hist.history.base_P_req/1000/3600; % [kWh]
    before_drv_cons = temp_hist.history.base_Q_Ah(1)-temp_hist.history.base_Q_Ah(end); % [Ah]

    applied_drv_dist = temp_hist.history.applied_pos/1000; % km
    applied_drv_pwr = temp_hist.history.applied_P_req/1000/3600; % [kWh]
    applied_drv_cons = temp_hist.history.applied_Q_Ah(1)-temp_hist.history.applied_Q_Ah(end); % [Ah]

    before_drv_eff = sum(before_drv_pwr)/(before_drv_dist(end)); %kWh/km
    applied_drv_eff = sum(applied_drv_pwr)/(applied_drv_dist(end)); %kWh/km

    before_degrad_loss = temp_hist.history.base_Q_loss(end);
    applied_degrad_loss = temp_hist.history.applied_Q_loss(end);
    %%
    % % close
    % figure(1000+i)
    % 
    % hold on
    % plot(temp_hist.history.base_v*3.6,'*')
    % plot(temp_hist.history.applied_spd*3.6,'o')
    % hold off
    % legend('Base','Optimized')

    %%
    % 평균속도
    drv_spd_before(i) = before_avg_spd*3.6;
    drv_spd_applied(i) = applied_avg_spd*3.6;
    % 총 power 합 [Wh]
    pwr_before = sum(before_drv_pwr);
    pwr_applied = sum(applied_drv_pwr);
    % 총 Ah 사용량
    cons_before(i) = temp_hist.history.base_SOC(1)-temp_hist.history.base_SOC(end);
    cons_applied(i) = temp_hist.history.applied_SOC(1)-temp_hist.history.applied_SOC(end);
    % SOC 사용량
    soc_before(i) = before_drv_cons;
    soc_applied(i) = applied_drv_cons;
    % 거리 차이 & 효율
    drv_dist_diff(i) = abs(before_drv_dist(end)-applied_drv_dist(end));
    drv_dist_before(i) = before_drv_dist(end);
    drv_dist_applied(i) = applied_drv_dist(end);
    % 용량 손실
    degrad_loss_before(i) = before_degrad_loss;
    degrad_loss_applied(i) = applied_degrad_loss;

    flow_rec(i) =  str2double(cell2mat(temp_str(3)));
    alpha_rec(i) = str2double(temp_aph(end));
    %% second-by-second analysis 
    % before_spd = []; %temp_hist.history.base_v;
    % applied_spd = []; %temp_hist.history.applied_spd;
    % 
    % % driving efficiency
    % % before_drv_current = temp_hist.history.base_I_batt; % [A]
    % % before_drv_cons = diff(temp_hist.history.base_Q_Ah); % [Ah]
    % 
    % % applied_drv_current = temp_hist.history.applied_I_batt; % [A]
    % % applied_drv_cons = temp_hist.history.applied_Q_Ah(1)-temp_hist.history.applied_Q_Ah(end); % [Ah]
    % 
    % % before_degrad_loss = []; %temp_hist.history.base_Q_loss;
    % % applied_degrad_loss = []; %temp_hist.history.applied_Q_loss;
    % 
    % before_grade = [];
    % applied_grade = [];
    % 
    % before_drv_current = [];
    % applied_drv_current = [];
    % 
    % before_v_minmax = [];
    % % before_v_min = [];
    % 
    % applied_v_minmax = [];
    % applied_v_min = [];

    for j = 1:size(temp_hist.history.pred,2)
        before_drv_current = [before_drv_current; ...
            [str2double(temp_aph(end)) sum(temp_hist.history.pred(j).I_batt)]]; % [A]
        applied_drv_current = [applied_drv_current; ...
            [str2double(temp_aph(end)) sum(temp_hist.history.opt(j).I_batt)]];

        before_grade = [before_grade; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.pred(j).grade)]];
        applied_grade = [applied_grade; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.opt(j).grade)]];

        before_grade_var = [before_grade_var; ...
            [str2double(temp_aph(end)) var(temp_hist.history.pred(j).grade)]];
        applied_grade_var = [applied_grade_var; ...
            [str2double(temp_aph(end)) var(temp_hist.history.opt(j).grade)]];

        before_v_minmax = [before_v_minmax; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.pred(j).bound.v_max - temp_hist.history.pred(j).bound.v_min)]];
        % before_v_min = [before_v_min; mean(temp_hist.history.pred(j).bound.v_min)];

        applied_v_minmax = [applied_v_minmax; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.opt(j).bound.v_max - temp_hist.history.opt(j).bound.v_min)]];
        % applied_v_min = [applied_v_min; mean(temp_hist.history.opt(j).bound.v_min)];

        before_spd = [before_spd; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.pred(j).v)]];
        applied_spd = [applied_spd; ...
            [str2double(temp_aph(end)) mean(temp_hist.history.opt(j).v_plot)]]; %temp_hist.history.applied_Q_loss;

        before_spd_var = [before_spd_var; ...
            [str2double(temp_aph(end)) var(temp_hist.history.pred(j).v)]];
        applied_spd_var = [applied_spd_var; ...
            [str2double(temp_aph(end)) var(temp_hist.history.opt(j).v_plot)]]; %temp_hist.history.applied_Q_loss;

        % J = J + alpha * EVout.P_req * dt(k) + (1-alpha) * EVout.Q_loss * V_nom * Q_nom;
        temp_loss = temp_hist.history.pred(j).Q_loss*360*120;
        % Q_loss가 누적되는 형태임
        Obj_SOC = [Obj_SOC;temp_hist.history.pred(j).P_req];
        Obj_LOSS = [Obj_LOSS;temp_loss];
    end
    % before_grade = temp_hist.history.pred(2).grade;

end
%% SOC VS LOSS
figure(1)
plot(Obj_SOC/3600,Obj_LOSS/100,'*')
xlabel('Power [Wh]')
ylabel('Degradation [Wh]')

%% second-analysis
% alpha 0.2: 2, 0.5: 5, 0.8: 8, 10: 1, 20: 3,
idx_aph1 = before_spd(:,1) == 2;
idx_aph2 = before_spd(:,1) == 5;
idx_aph3 = before_spd(:,1) == 8;
idx_aph4 = before_spd(:,1) == 1;
idx_aph5 = before_spd(:,1) == 3;

target_cur_before1 = before_drv_current(idx_aph1,2)/3600; % Ah
target_cur_applied1 = applied_drv_current(idx_aph1,2)/3600;  % Ah
target_cur_before2 = before_drv_current(idx_aph2,2)/3600; % Ah
target_cur_applied2 = applied_drv_current(idx_aph2,2)/3600;  % Ah
target_cur_before3 = before_drv_current(idx_aph3,2)/3600; % Ah
target_cur_applied3 = applied_drv_current(idx_aph3,2)/3600;  % Ah
target_cur_before4 = before_drv_current(idx_aph4,2)/3600; % Ah
target_cur_applied4 = applied_drv_current(idx_aph4,2)/3600;  % Ah

target_v_before1 = before_spd(idx_aph1,2)*3.6; % km/h
target_v_applied1 = applied_spd(idx_aph1,2)*3.6; % km/h
target_v_before2 = before_spd(idx_aph2,2)*3.6; % km/h
target_v_applied2 = applied_spd(idx_aph2,2)*3.6; % km/h
target_v_before3 = before_spd(idx_aph3,2)*3.6; % km/h
target_v_applied3 = applied_spd(idx_aph3,2)*3.6; % km/h
target_v_before4 = before_spd(idx_aph4,2)*3.6; % km/h
target_v_applied4 = applied_spd(idx_aph4,2)*3.6; % km/h

target_v_var_before1 = before_spd_var(idx_aph1,2)*3.6; % km/h
target_v_var_applied1 = applied_spd_var(idx_aph1,2)*3.6; % km/h
target_v_var_before2 = before_spd_var(idx_aph2,2)*3.6; % km/h
target_v_var_applied2 = applied_spd_var(idx_aph2,2)*3.6; % km/h
target_v_var_before3 = before_spd_var(idx_aph3,2)*3.6; % km/h
target_v_var_applied3 = applied_spd_var(idx_aph3,2)*3.6; % km/h
target_v_var_before4 = before_spd_var(idx_aph4,2)*3.6; % km/h
target_v_var_applied4 = applied_spd_var(idx_aph4,2)*3.6; % km/h

target_grad_before1 = before_grade(idx_aph1,2); % degree
target_grad_applied1 = applied_grade(idx_aph1,2); % degree
target_grad_before2 = before_grade(idx_aph2,2); % degree
target_grad_applied2 = applied_grade(idx_aph2,2); % degree
target_grad_before3 = before_grade(idx_aph3,2); % degree
target_grad_applied3 = applied_grade(idx_aph3,2); % degree
target_grad_before4 = before_grade(idx_aph4,2); % degree
target_grad_applied4 = applied_grade(idx_aph4,2); % degree

target_grad_var_before1 = before_grade_var(idx_aph1,2); % degree
target_grad_var_applied1 = applied_grade_var(idx_aph1,2); % degree
target_grad_var_before2 = before_grade_var(idx_aph2,2); % degree
target_grad_var_applied2 = applied_grade_var(idx_aph2,2); % degree
target_grad_var_before3 = before_grade_var(idx_aph3,2); % degree
target_grad_var_applied3 = applied_grade_var(idx_aph3,2); % degree
target_grad_var_before4 = before_grade_var(idx_aph4,2); % degree
target_grad_var_applied4 = applied_grade_var(idx_aph4,2); % degree

target_gap_before1 = before_v_minmax(idx_aph1,2);
target_gap_applied1 = applied_v_minmax(idx_aph1,2);
target_gap_before2 = before_v_minmax(idx_aph2,2);
target_gap_applied2 = applied_v_minmax(idx_aph2,2);
target_gap_before3 = before_v_minmax(idx_aph3,2);
target_gap_applied3 = applied_v_minmax(idx_aph3,2);
target_gap_before4 = before_v_minmax(idx_aph4,2);
target_gap_applied4 = applied_v_minmax(idx_aph4,2);

%%
close all
f = figure;
f.Position(3:4) = [1200 400];
subplot(2,2,1)
hold on
plot(target_v_before1, target_cur_before1, '*')
plot(target_v_applied1, target_cur_applied1, '*')
hold off
title('Alpha: 0.2')
ylabel('Sum of current [Ah]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

subplot(2,2,2)
hold on
plot(target_v_before2, target_cur_before2, '*')
plot(target_v_applied2, target_cur_applied2, '*')
hold off
title('Alpha: 0.5')
ylabel('Sum of current [Ah]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

subplot(2,2,3)
hold on
plot(target_v_before3, target_cur_before3, '*')
plot(target_v_applied3, target_cur_applied3, '*')
hold off
title('Alpha: 0.8')
ylabel('Sum of current [Ah]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

subplot(2,2,4)
hold on
plot(target_v_before4, target_cur_before4, '*')
plot(target_v_applied4, target_cur_applied4, '*')
hold off
title('Alpha: 10')
ylabel('Sum of current [Ah]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')
%% speed VS current
x1 = target_cur_before1-target_cur_applied1;
x2 = target_cur_before2-target_cur_applied2;
x3 = target_cur_before3-target_cur_applied3;
figure(11)
% boxplot([x1,x2,x3],'Labels',{'Alpha = 0.2','Alpha = 0.5','Alpha = 0.8'})
% plot(target_v_before1, target_cur_before1-target_cur_applied1, '*')
% plot(target_v_before1, target_cur_before1-target_cur_applied1, '*')
subplot(1,2,1)
hold on
plot(target_v_before2, target_cur_before2, '*')
plot(target_v_applied2, target_cur_applied2, '*')
hold off
title('Alpha: 0.5')
ylabel('Sum of current [Ah]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

subplot(1,2,2)
plot(target_v_before2, target_cur_before2-target_cur_applied2, 'ko')
% plot(target_v_applied3, target_cur_before3-target_cur_applied3, '*')
% title('Alpha: 0.8')
ylabel('Optimization Margin [Ah]')
xlabel('Avg. speed [km/h]')
ylim([-0.1 0.36])
% legend('Alpha: 0.2','Alpha: 0.5','Alpha: 0.8','Location','northwest')
% legend('Alpha: 0.2','Alpha: 0.8','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

%% grade VS current
% x2 = target_cur_before2-target_cur_applied2;
x1=target_v_var_before2(target_cur_before2 > 0.1);% & target_v_var_before2 > 20);
w1=target_grad_var_before2(target_cur_before2 > 0.1);% & target_v_var_before2 > 20);
g1=target_grad_before2(target_cur_before2 > 0.1);% & target_v_var_before2 > 20);

y1=target_v_before2(target_cur_before2 > 0.1);% & target_v_var_before2 > 20);
z1=target_cur_before2(target_cur_before2 > 0.1);% & target_v_var_before2 > 20);

x2=target_v_var_applied2(target_cur_applied2 > 0.1);% & target_v_var_applied2 > 20);
w2=target_grad_var_applied2(target_cur_applied2 > 0.1);% & target_v_var_applied2 > 20);
g2=target_grad_applied2(target_cur_applied2 > 0.1);% & target_v_var_applied2 > 20);

y2=target_v_applied2(target_cur_applied2 > 0.1);% & target_v_var_applied2 > 20);
z2=target_cur_applied2(target_cur_applied2 > 0.1);% & target_v_var_applied2 > 20);
close
f = figure(12);
f.Position(3:4) = [650 400];
% subplot(1,2,1)
% hold on
% cmp = linspace(min(z1),max(z1),numel(z1));
% scatter3(x1,y1,z1, [], z1,'filled'); colorbar;
% scatter3(x2,y2,z2, [], z2,'filled'); colorbar;


% colormap('hot')
% grid on;grid minor
% plot3(x1,y1, z1, '*')
% colorbar
% plot(w1,z1, '*')
% plot(w2,z2, '*')
% plot3(g1,w1,z1, '*')
% plot3(g2,w2,z2, 'o')
% plot3(target_v_var_applied2,target_grad_applied2, target_cur_applied2, '*')

% hold off
% ylabel('Var. grade [rad]')
% xlabel('Avg. grade [rad]')
% zlabel('Sum of current [Ah]')
% ylabel('Avg. speed [km/h]')
% legend('Before','Applied','Location','northwest')
% set(gca,'FontSize',14,'fontname','Times New Roman')

% subplot(1,2,2)
plot3(target_v_before2,target_grad_var_before2, target_cur_before2-target_cur_applied2, 'ko')
% plot3(x1,w1, z1-z2, 'ko')

zlabel('Optimization Margin [Ah]')
% % ylabel('Avg. speed [km/h]')
ylabel('Var. grade [rad]')
xlabel('Avg. speed [km/h]')
% ylim([-0.1 0.36])
set(gca,'FontSize',14,'fontname','Times New Roman')

%% gap VS current
% x2 = target_cur_before2-target_cur_applied2;
f = figure(13);
f.Position(3:4) = [1200 400];
subplot(1,2,1)
hold on
plot3(target_v_before2,target_gap_before2, target_cur_before2, '*')
plot3(target_v_applied2,target_gap_applied2, target_cur_applied2, '*')
hold off
zlabel('Sum of current [Ah]')
ylabel('Speed gap [km/h]')
xlabel('Avg. speed [km/h]')
legend('Before','Applied','Location','northwest')
set(gca,'FontSize',14,'fontname','Times New Roman')

subplot(1,2,2)
plot3(target_v_before2, target_gap_before2, target_cur_before2-target_cur_applied2, 'ko')
zlabel('Sum of current [Ah]')
ylabel('Speed gap [km/h]')
xlabel('Avg. speed [km/h]')
% ylim([-0.1 0.36])
set(gca,'FontSize',14,'fontname','Times New Roman')