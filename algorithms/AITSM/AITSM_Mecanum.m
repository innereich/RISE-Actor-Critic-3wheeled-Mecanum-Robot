clear;close all;clc;
    %% parameters
        m=10;
        l=0.5; r=0.05;
        I_theta=5; I_phi=0.1;
        muy=1;
        %ro=-0.1;
        m1=4*m*r^2/9+I_theta*r^2/(9*l^2)+I_phi;
        m2=I_theta*r^2/(9*l^2)-2*m*r^2/9;
        M=[m1 m2 m2; m2 m1 m2; m2 m2 m1];
    %% Time interval and simulation time
        Step=0.001;
        t=0:Step:150;
    %% Variables
        % States
        eta             = cell(1,size(t,2));
        %eta_dot         = cell(1,size(t,2));
        
        upsilon         = cell(1,size(t,2));
%        upsilon_dot     = cell(1,size(t,2));
        
        e1              = cell(1,size(t,2));
        
        
        % Reference cordination
        eta_d           = cell(1,size(t,2));
        eta_d_dot       = cell(1,size(t,2));
        eta_d_dot_dot   = cell(1,size(t,2));
        % Auxiliary functions
      
        ro               = zeros(1,size(t,2));
        % Control signal
        tau             = cell(1,size(t,2));
        tau_d             = cell(1,size(t,2));
        tau_n             = cell(1,size(t,2));
       
  
       
        nominal_E = 0;
        nominal_E_ref = 0;
    %% Important parameters
        
        % RISE
        lamda_1        = diag([10 10 10]);
        lamda_2        = diag([10 10 10]);
       
    %% Initial conditions
        
        tau{1} = [0;0;0];
        tau_d{1}=[0;0;0];
        % States
% %         Trajectory 1
%         eta{1}          = [1.2 1.5 1.5]';
%         upsilon{1}      = [0.1;0.2;0.3];
%        Trajectory 2
        eta{1}          = [0.5;1;0.5];
        upsilon{1}      = [0;0;0];

        
        
        e_I = zeros(3,1);
    for i = 1:size(t,2)
        %% Disturbance caused by slipping
        
       % ro(i)            = cos(0.5*t(i))*sin(0.5*t(i)) + cos(0.5*t(i))^2*sin(0.5*t(i));
            ro(i) = -0.1;
        %% Calculate H, H_dot, H_inv
        Rm=[cos(eta{i}(3)) -sin(eta{i}(3)) 0; sin(eta{i}(3)) cos(eta{i}(3)) 0;0 0 1];
        Am=[sin(pi/3) -cos(pi/3) -l; sin(pi) -cos(pi) -l; sin(-pi/3) -cos(-pi/3) -l];
        H=Rm*pinv(Am);
        Rm_dot=[-sin(eta{i}(3)) -cos(eta{i}(3)) 0; cos(eta{i}(3)) -sin(eta{i}(3)) 0; 0 0 0];
        H_dot=Rm_dot*pinv(Am);
        H_inv=[sin(eta{i}(3)+pi/3) -cos(eta{i}(3)+pi/3) -l; -sin(eta{i}(3)) cos(eta{i}(3)) ...
            -l; sin(eta{i}(3)-pi/3) -cos(eta{i}(3)-pi/3) -l];
        %% Generate reference trajectory 
%        % Trajectory 1
%         eta_d{i}=[0.5*cos(2*t(i)); cos(t(i)); cos(0.5*t(i))];
%         eta_d_dot{i}    = [-sin(2*t(i)); -sin(t(i)); -0.5*sin(0.5*t(i))];
%         eta_d_dot_dot{i} = [-2*cos(2*t(i)); -cos(t(i)); -0.25*cos(0.5*t(i))];   
        
 % %     Trajectory 2
 %        eta_d{i}        = [0.5*sin(0.2*t(i));-0.5*cos(0.2*t(i));0.5*sin(0.2*t(i))];
 %        eta_d_dot{i}    = [0.5*0.2*cos(0.2*t(i));0.5*0.2*sin(0.2*t(i));0.5*0.2*cos(0.2*t(i))]; 
 %        eta_d_dot_dot{i} = [-0.5*0.2*0.2*sin(0.2*t(i));0.5*0.2*0.2*cos(0.2*t(i));-0.5*0.2*0.2*sin(0.2*t(i))];   
 %      Trajectory 3
         eta_d{i}        = [sin(t(i)) sin(2*t(i)) cos(0.5*t(i))]';
        eta_d_dot{i}    = [cos(t(i)) 2*cos(2*t(i)) -0.5*sin(0.5*t(i))]'; 
        eta_d_dot_dot{i} = [-sin(t(i)) -4*sin(2*t(i)) -0.25*cos(0.5*t(i))]';  
        %% Errors
        e1{i}=eta{i} - eta_d{i};
        %e2{i}=eta_d_dot{i} - upsilon{i}+lamda_1*e1{i};
        %% Calculate F,G,F_d,d
        F=-H_dot*H_inv+muy*H*pinv(M)*H_inv;
        G=r*H*pinv(M);
        %% nominal u
        p_c = 5; q = 3;
        
        e_I_dot = sign(e1{i}) .* abs(e1{i}).^(p_c/q);
        %e_I_dot = e1{i}.^(p_c/q);
        s = (-eta_d_dot{i} + upsilon{i}) + lamda_1*e1{i} + lamda_2*e_I;
        u_eq = M*H_inv/r*(eta_d_dot_dot{i} - lamda_1*(-eta_d_dot{i} + upsilon{i}) - lamda_2*e_I_dot);

        k_1 = 5*diag([1 1 1]); k_2 = 22*diag([1 1 1]);
        u_re = - H_inv*(k_1*sign(s) + k_2*s);
        % tau
        tau{i} = u_eq + u_re + pinv(G)*F*upsilon{i};
        
        %% Update state
        ep_1 = [5;5;5];
        ep_2 = [10;10;10];
        e_I = e_I_dot*Step + e_I;
        lamda_1 = diag([-ep_1(1)*s(1)*e1{i}(1) -ep_1(2)*s(2)*e1{i}(2) -ep_1(3)*s(3)*e1{i}(3)])*Step + lamda_1;
        lamda_2 = diag([-ep_2(1)*s(1)*e_I(1) -ep_2(2)*s(2)*e_I(2) -ep_2(3)*s(3)*e_I(3)])*Step + lamda_2;

        F_d=-ro(i)*(G*tau{i});
        if i > 50000
            nominal_E = tau{i}'*tau{i}*Step + nominal_E;
            tau_ref = pinv(G)*(eta_d_dot_dot{i} + F*eta_d_dot{i}-F_d);
            nominal_E_ref = tau_ref'*tau_ref*Step + nominal_E_ref;
        end
        %d=inv(G)*F_d;
        %tau_d{i}=G*tau{i};% Wait
        tau_n{i}=F_d;%Wait hmmm
        upsilon{i+1}=(tau_n{i}+G*tau{i}-F*upsilon{i})*Step+upsilon{i};
        eta{i+1}=upsilon{i}*Step+eta{i};
        
    end
%% Convert cell into matrix
    eta = cell2mat(eta);
    eta_d = cell2mat(eta_d);
   
    e1=cell2mat(e1);
    tau = cell2mat(tau);
    

   
   
%% Plot X,Y,Theta
    figure(1);
    
    plot(t,eta(1,1:size(t,2)),'-b','LineWidth',1.2);
    hold on
    plot(t,eta_d(1,:),'--r','LineWidth',1.2);
    title('X position');
    legend({'\eta_1','\eta_{d1}'},'FontSize',6);
    ylabel('\eta_1(m)');
    grid on
    hold off
    
    figure(2);
    plot(t,eta(2,1:size(t,2)),'-b','LineWidth',1.2);
    hold on
    plot(t,eta_d(2,:),'--r','LineWidth',1.2);
    title('Y position');
    legend({'\eta_2','\eta_{d2}'},'FontSize',6);
    ylabel('\eta_2(m)');
    grid on
    hold off
    
    figure(3);
    plot(t,eta(3,1:size(t,2)),'-b','LineWidth',1.2);
    hold on
    plot(t,eta_d(3,:),'--r','LineWidth',1.2);
    title('Heading angle');
    legend({'\eta_3','\eta_{d3}'},'FontSize',6);
    ylabel('\eta_3(rad)');
    grid on
    hold off
    
    figure(4);
    plot(t,eta(1,1:size(t,2))-eta_d(1,:),'-b','LineWidth',1.2);
    hold on
    plot(t,eta(2,1:size(t,2))-eta_d(2,:),'--r','LineWidth',1.2);
    hold on
    plot(t,eta(3,1:size(t,2))-eta_d(3,:),'-.g','LineWidth',1.2);
    title('Error signal');
    legend({'\epsilon_1(1)','\epsilon_1(2)','\epsilon_1(3)'},'FontSize',8);
    xlabel('Time(sec)');
    ylabel('\eta-\eta_d(m or rad)');
    %ylim([-0.1 0.6])
    grid on
    hold off
% %% Plot Real Trajectory vs Reference Trajectory
%     figure(5)
%     plot(eta(1,1:size(t,2)),eta(2,1:size(t,2)));
%     hold on
%     plot(eta_d(1,:),eta_d(2,:));
%     title('Trajectory');
%     legend('Real Trajectory','Reference Trajectory');
%     hold off
% 
%% Plot control signal

    figure(8);
    p=plot(t,tau(1,1:length(t)),'LineWidth',1.17);    
    p.Color="r";
    %ylim([-25 20]);
    grid on
    ylabel('Torque generated by the motor 1','FontSize',14,'LineWidth',1.17);

    figure(9);
    p=plot(t,tau(2,1:length(t)),'LineWidth',1.17);
    p.Color="b";
    grid on
    ylabel('Torque generated by the motor 2','FontSize',14,'LineWidth',1.17);
    %ylim([-25 20]);

    figure(10);
    p=plot(t,tau(3,1:length(t)));
    p.Color="m";
    ylabel('Torque generated by the motor 3','FontSize',14,'LineWidth',1.17);
    grid on
    %ylim([-25 20]);
% 
% 
%     writematrix(tau, 'tau_RISE.csv');
% 
    
 writematrix(eta,'eta_AITSM.csv');
 writematrix(nominal_E,'consumption_AITSM.csv');
% eta_Robust=readmatrix('eta_Robust.csv');
% eta_RISE=readmatrix('eta_RISE.csv');
%  eta_d=readmatrix('eta_d.csv');
% start=50000;
% E1=abs(eta_RISE(1,start:size(t,2))-eta_d(1,start:size(t,2)))+abs(eta_RISE(2,start:size(t,2))-eta_d(2,start:size(t,2)))+abs(eta_RISE(3,start:size(t,2))-eta_d(3,start:size(t,2)));
% E2=abs(eta_Robust(1,start:size(t,2))-eta_d(1,start:size(t,2)))+abs(eta_Robust(2,start:size(t,2))-eta_d(2,start:size(t,2)))+abs(eta_Robust(3,start:size(t,2))-eta_d(3,start:size(t,2)));
% 
% e_Robust=trapz(Step,E2);
% e_RISE=trapz(Step,E1);
% 
% e_RISE=e_RISE/100
% e_Robust=e_Robust/100


% 
% start=50000;
% F1=abs(tau_Robust(1,start:size(t,2)))+abs(tau_Robust(2,start:size(t,2)))+abs(tau_Robust(3,start:size(t,2)));
% F2=abs(tau_RISE(1,start:size(t,2)))+abs(tau_RISE(2,start:size(t,2)))+abs(tau_RISE(3,start:size(t,2)));
% 
% Robust=trapz(Step,F1);
% RISE=trapz(Step,F2);
% 
% figure
% %x=['RISE-based controller' 'Robust-based RL controller'];
% x=categorical({'RISE-based controller', 'Robust-based RL controller'});
% x = reordercats(x,{'RISE-based controller', 'Robust-based RL controller'});
% y=[RISE Robust];
% 
% bar(x(1),y(1),'FaceColor','g')
% hold on
% bar(x(2),y(2),'FaceColor','r')
% ylabel('$$ \int_{50}^{150} (|\tau_1|+|\tau_2|+|\tau_3|) dt $$', 'Interpreter', 'LaTeX')

% writematrix(eta,'eta_RISE.csv');
% writematrix(eta_d,'eta_d.csv'); 
% writematrix(tau,'tau_RISE.csv');
 
% writematrix(V_z,'V_RISE.csv');
% V_Robust=readmatrix('V_Robust.csv');
% V_RISE=readmatrix('V_RISE.csv');
% 
% figure
% plot(t,V_RISE)
% hold on
% plot(t, V_Robust)
% legend('Robust-based RL controller','RISE-based RL controller')
% ylim([-1 100])
























































