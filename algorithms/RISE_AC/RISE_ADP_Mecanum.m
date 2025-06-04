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
        t=0:Step:70;
    %% Variables
        % States
        eta             = cell(1,size(t,2));
        %eta_dot         = cell(1,size(t,2));
        
        upsilon         = cell(1,size(t,2));
%        upsilon_dot     = cell(1,size(t,2));
        
        e1              = cell(1,size(t,2));
        e2              = cell(1,size(t,2));
        z             = cell(1,size(t,2));
        
        % Reference cordination
        eta_d           = cell(1,size(t,2));
        eta_d_dot       = cell(1,size(t,2));
        eta_d_dot_dot   = cell(1,size(t,2));
        % Auxiliary functions
        f               = cell(1,size(t,2));
        ro               = zeros(1,size(t,2));
        % Control signal
        tau             = cell(1,size(t,2));
        tau_d             = cell(1,size(t,2));
        tau_n             = cell(1,size(t,2));
        % Actor-Critic ADP
        W_c             = cell(1,size(t,2));
        W_a             = cell(1,size(t,2));
        cap_gamma       = cell(1,size(t,2));
        % Error estimation
        e_es              = cell(1,size(t,2));
        % RISE
        Mu                =cell(1,size(t,2));
        
        noise             =zeros(1,size(t,2));
        V_z = zeros(1,size(t,2));
        J = zeros(1,size(t,2));
        nominal_E = 0;
        nominal_E_ref = 0;
    %% Important parameters
        % Actor-Critic ADP

        R= 1*eye(3);
        Q=1*eye(6);
        % controller variables
        gamma=0.5;
        %gamma=0;
        eta_c=0.08;
        
        eta_a1=0.2;
        
        eta_a2=50;
        beta=0.001;
        v=0.001;
        % RISE
        lamda_1        = 1*eye(3); %4
        lamda_2        =10; %50;
        Beta_1         =6;
        ks             =100;
        N = 3; %3
    %% Initial conditions
        chi=0;
        cap_gamma{1}=10000*eye(N);
        tau{1} = [-2;1;0];
        tau_d{1}=[0;0;0];
        PE = zeros(N,N);
        % States
% %         Trajectory 1
%         eta{1}          = [1.2 1.5 1.5]';
%         upsilon{1}      = [0.1;0.2;0.3];
%        Trajectory 2
        eta{1}          = [1.8;1.6;1.4];
        upsilon{1}      = [0;0;0];

        
        % Actor-Critic ADP [0.1778, 0.3360, -1.4796]
        % W_c{1}          = [0.078, 0.2360, -1.6796]';
        % W_a{1}          = [0.378, 0.5360, -1.2796]';

        W_c{1}          = zeros(N,1);
        W_a{1}          = ones(N,1);

        Nf=250;
        % freqs = 50-100*rand(Nf,1);
        max_freq = 50;
        freqs = max_freq-2*max_freq*rand(Nf,1);
    for i = 1:size(t,2)
        %% Disturbance caused by slipping
        % Probing noise
        noise(i) = 0;
        if i < 50000
            % noise(i) =  1*rand-0.5 + 2*rand-1;
            
            % noise(i) =  30*sin(1.1*t(i))*cos(t(i))+ 25*sin(1.5*t(i))^2*cos(2*t(i)) + 50*cos(t(i))^3+ 10*sin(t(i))*cos(t(i))^2 + 50*sin(t(i))^4*cos(t(i))^3; 
             % noise(i) = 90*(sin(t(i))^2*cos(t(i)) + sin(2*t(i))^2*cos(0.1*t(i))...
             %    + sin(-1.2*t(i))^2*cos(0.5*t(i)) + sin(t(i))^5 + sin(1.2*t(i))^2 ...
             %    + cos(2.4*t(i))*sin(2.4*t(i))^3);
            for j = 1:Nf
                % random phase for diversity
                noise(i) = noise(i) + 50*sin(freqs(j)*t(i)+rand);
            end
             
        end
        %ro(i)            = cos(0.5*t(i))*sin(0.5*t(i)) + cos(0.5*t(i))^2*sin(0.5*t(i));
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

        Rm_ref = [cos(eta_d{i}(3)) -sin(eta_d{i}(3)) 0; sin(eta_d{i}(3)) cos(eta_d{i}(3)) 0;0 0 1];
        H_ref = Rm_ref*pinv(Am);
        %% Errors
        e1{i}=eta_d{i}-eta{i};
        e2{i}=eta_d_dot{i} - upsilon{i}+lamda_1*e1{i};
        %% Calculate F,G,F_d,d
        F=-H_dot*H_inv+muy*H*pinv(M)*H_inv;
        G=r*H*pinv(M);
        G_ref = r*H_ref*pinv(M);
        %% Calculate f
        f{i}=-(eta_d_dot_dot{i}+lamda_1*(eta_d_dot{i} - upsilon{i}))-F*(eta_d_dot{i}+lamda_1*e1{i});
        %% Z model
        z{i}=[e1{i}' e2{i}' eta_d{i}' eta_d_dot{i}']';
        A=[-lamda_1*e1{i}+e2{i};
            -F*e2{i}           ;
            eta_d_dot{i}           ;
            eta_d_dot_dot{i}      ];
        B=[zeros(3,3)  ;
            eye(3)  ;
            zeros(3,3) ;
            zeros(3,3)];    
        %%  Activation function
        % phi_z=[z{i}(1)*z{i}(1) z{i}(2)*z{i}(2) z{i}(3)*z{i}(3) z{i}(1)*z{i}(4)...
        %      z{i}(1)*z{i}(5) z{i}(1)*z{i}(6) z{i}(2)*z{i}(4) z{i}(2)*z{i}(5) z{i}(2)*z{i}(6)...
        %      z{i}(3)*z{i}(4) z{i}(3)*z{i}(5) z{i}(3)*z{i}(6) z{i}(1)^2*z{i}(2)^2 z{i}(1)^2*z{i}(3)^2 ...
        %      z{i}(2)^2*z{i}(3)^2 z{i}(1)^2*z{i}(7)^2 z{i}(1)^2*z{i}(8)^2 z{i}(1)^2*z{i}(9)^2 z{i}(1)^2*z{i}(10)^2 ...
        %      z{i}(1)^2*z{i}(11)^2 z{i}(1)^2*z{i}(12)^2 z{i}(2)^2*z{i}(7)^2 z{i}(2)^2*z{i}(8)^2 ...
        %      z{i}(2)^2*z{i}(9)^2 z{i}(2)^2*z{i}(10)^2 z{i}(2)^2*z{i}(11)^2 z{i}(2)^2*z{i}(12)^2 ...
        %      z{i}(3)^2*z{i}(7)^2 z{i}(3)^2*z{i}(8)^2 z{i}(3)^2*z{i}(9)^2 z{i}(3)^2*z{i}(10)^2 ...
        %      z{i}(3)^2*z{i}(11)^2 z{i}(3)^2*z{i}(12)^2 z{i}(4)^2*z{i}(7)^2 z{i}(4)^2*z{i}(8)^2 ...
        %      z{i}(4)^2*z{i}(9)^2 z{i}(4)^2*z{i}(10)^2 z{i}(4)^2*z{i}(11)^2 z{i}(4)^2*z{i}(12)^2 ...
        %      z{i}(5)^2*z{i}(7)^2 z{i}(5)^2*z{i}(8)^2 z{i}(5)^2*z{i}(9)^2 z{i}(5)^2*z{i}(10)^2 ...
        %      z{i}(5)^2*z{i}(11)^2 z{i}(5)^2*z{i}(12)^2 z{i}(6)^2*z{i}(7)^2 z{i}(6)^2*z{i}(8)^2 ...
        %      z{i}(6)^2*z{i}(9)^2 z{i}(6)^2*z{i}(10)^2 z{i}(6)^2*z{i}(11)^2 z{i}(6)^2*z{i}(12)^2]';
        %  grad_phi_z = [2*z{i}(1) 0 0 0 0 0 0 0 0 0 0 0; ...
        %     0 2*z{i}(2) 0 0 0 0 0 0 0 0 0 0; ...
        %     0 0 2*z{i}(3) 0 0 0 0 0 0 0 0 0; ...
        %     z{i}(4) 0 0 z{i}(1) 0 0 0 0 0 0 0 0; ...
        %     z{i}(5) 0 0 0 z{i}(1) 0 0 0 0 0 0 0; ...
        %     z{i}(6) 0 0 0 0 z{i}(1) 0 0 0 0 0 0; ...
        %     0 z{i}(4) 0 z{i}(2) 0 0 0 0 0 0 0 0; ...
        %     0 z{i}(5) 0 0 z{i}(2) 0 0 0 0 0 0 0; ...
        %     0 z{i}(6) 0 0 0 z{i}(2) 0 0 0 0 0 0; ...
        %     0 0 z{i}(4) z{i}(3) 0 0 0 0 0 0 0 0; ...
        %     0 0 z{i}(5) 0 z{i}(3) 0 0 0 0 0 0 0; ...
        %     0 0 z{i}(6) 0 0 z{i}(3) 0 0 0 0 0 0; ...
        %     2*z{i}(1)*(z{i}(2)^2) 2*(z{i}(1)^2)*z{i}(2) 0 0 0 0 0 0 0 0 0 0; ...
        %     2*z{i}(1)*(z{i}(3)^2) 0 2*(z{i}(1)^2)*z{i}(3) 0 0 0 0 0 0 0 0 0; ...
        %     0 2*z{i}(2)*(z{i}(3)^2) 2*(z{i}(2)^2)*z{i}(3) 0 0 0 0 0 0 0 0 0; ...
        %     2*z{i}(1)*(z{i}(7)^2) 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(7) 0 0 0 0 0; ...
        %     2*z{i}(1)*(z{i}(8)^2) 0 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(8) 0 0 0 0; ...
        %     2*z{i}(1)*(z{i}(9)^2) 0 0 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(9) 0 0 0; ...
        %     2*z{i}(1)*(z{i}(10)^2) 0 0 0 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(10) 0 0; ...
        %     2*z{i}(1)*(z{i}(11)^2) 0 0 0 0 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(11) 0; ...
        %     2*z{i}(1)*(z{i}(12)^2) 0 0 0 0 0 0 0 0 0 0 2*(z{i}(1)^2)*z{i}(12); ...
        %     0 2*z{i}(2)*(z{i}(7)^2) 0 0 0 0 2*(z{i}(2)^2)*z{i}(7) 0 0 0 0 0; ...
        %     0 2*z{i}(2)*(z{i}(8)^2) 0 0 0 0 0 2*(z{i}(2)^2)*z{i}(8) 0 0 0 0; ...
        %     0 2*z{i}(2)*(z{i}(9)^2) 0 0 0 0 0 0 2*(z{i}(2)^2)*z{i}(9) 0 0 0; ...
        %     0 2*z{i}(2)*(z{i}(10)^2) 0 0 0 0 0 0 0 2*(z{i}(2)^2)*z{i}(10) 0 0; ...
        %     0 2*z{i}(2)*(z{i}(11)^2) 0 0 0 0 0 0 0 0 2*(z{i}(2)^2)*z{i}(11) 0; ...
        %     0 2*z{i}(2)*(z{i}(12)^2) 0 0 0 0 0 0 0 0 0 2*(z{i}(2)^2)*z{i}(12); ...
        %     0 0 2*z{i}(3)*(z{i}(7)^2) 0 0 0 2*(z{i}(3)^2)*z{i}(7) 0 0 0 0 0; ...
        %     0 0 2*z{i}(3)*(z{i}(8)^2) 0 0 0 0 2*(z{i}(3)^2)*z{i}(8) 0 0 0 0; ...
        %     0 0 2*z{i}(3)*(z{i}(9)^2) 0 0 0 0 0 2*(z{i}(3)^2)*z{i}(9) 0 0 0; ...
        %     0 0 2*z{i}(3)*(z{i}(10)^2) 0 0 0 0 0 0 2*(z{i}(3)^2)*z{i}(10) 0 0; ...
        %     0 0 2*z{i}(3)*(z{i}(11)^2) 0 0 0 0 0 0 0 2*(z{i}(3)^2)*z{i}(11) 0; ...
        %     0 0 2*z{i}(3)*(z{i}(12)^2) 0 0 0 0 0 0 0 0 2*(z{i}(3)^2)*z{i}(12); ...
        %     0 0 0 2*z{i}(4)*(z{i}(7)^2) 0 0 2*(z{i}(4)^2)*z{i}(7) 0 0 0 0 0; ...
        %     0 0 0 2*z{i}(4)*(z{i}(8)^2) 0 0 0 2*(z{i}(4)^2)*z{i}(8) 0 0 0 0; ...
        %     0 0 0 2*z{i}(4)*(z{i}(9)^2) 0 0 0 0 2*(z{i}(4)^2)*z{i}(9) 0 0 0; ...
        %     0 0 0 2*z{i}(4)*(z{i}(10)^2) 0 0 0 0 0 2*(z{i}(4)^2)*z{i}(10) 0 0; ...
        %     0 0 0 2*z{i}(4)*(z{i}(11)^2) 0 0 0 0 0 0 2*(z{i}(4)^2)*z{i}(11) 0; ...
        %     0 0 0 2*z{i}(4)*(z{i}(12)^2) 0 0 0 0 0 0 0 2*(z{i}(4)^2)*z{i}(12); ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(7)^2) 0 2*(z{i}(5)^2)*z{i}(7) 0 0 0 0 0; ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(8)^2) 0 0 2*(z{i}(5)^2)*z{i}(8) 0 0 0 0; ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(9)^2) 0 0 0 2*(z{i}(5)^2)*z{i}(9) 0 0 0; ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(10)^2) 0 0 0 0 2*(z{i}(5)^2)*z{i}(10) 0 0; ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(11)^2) 0 0 0 0 0 2*(z{i}(5)^2)*z{i}(11) 0; ...
        %     0 0 0 0 2*z{i}(5)*(z{i}(12)^2) 0 0 0 0 0 0 2*(z{i}(5)^2)*z{i}(12); ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(7)^2) 2*(z{i}(6)^2)*z{i}(7) 0 0 0 0 0; ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(8)^2) 0 2*(z{i}(6)^2)*z{i}(8) 0 0 0 0; ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(9)^2) 0 0 2*(z{i}(6)^2)*z{i}(9) 0 0 0; ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(10)^2) 0 0 0 2*(z{i}(6)^2)*z{i}(10) 0 0; ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(11)^2) 0 0 0 0 2*(z{i}(6)^2)*z{i}(11) 0; ...
        %     0 0 0 0 0 2*z{i}(6)*(z{i}(12)^2) 0 0 0 0 0 2*(z{i}(6)^2)*z{i}(12)];  
        phi_z=[z{i}(1)*z{i}(1) z{i}(2)*z{i}(2) z{i}(3)*z{i}(3)...
         ]';
         grad_phi_z = [2*z{i}(1) 0 0 0 0 0 0 0 0 0 0 0; ...
            0 2*z{i}(2) 0 0 0 0 0 0 0 0 0 0;
             0 0 2*z{i}(3) 0 0 0 0 0 0 0 0 0];
        %% Cost function
        V_z(i)=W_c{i}'*phi_z;
        %% Controller
        u=-1/2*pinv(R)*B'*grad_phi_z'*W_a{i} + noise(i);
        J(i+1) = (z{i}'*[Q zeros(6); zeros(6) zeros(6)]*z{i} + u'*R*u)*Step+J(i);
        %% RISE update
        Mu{i}=(ks+1)*(e2{i}-e2{1})+chi; % @@
        chi=((ks+1)*lamda_2*e2{i}+Beta_1*sign(e2{i}))*Step+chi;
        %% tau input
        tau_d{i+1}=Mu{i}-u;% needed checkin      % @@
        tau{i+1}=pinv(G)*tau_d{i+1};          % @@
        %% Update NN weight
        sigma=grad_phi_z*(A+B*u)-gamma*phi_z;
        m_s=(1+v*sigma'*cap_gamma{i}*sigma)^(1/2);
        % Check PE condition
        sl=sigma/m_s;% not used
        PE = PE + sl*sl';
        disp(min(eig(PE)))

        D1=grad_phi_z*B*pinv(R)*B'*(grad_phi_z)';
        delta = 1/2*z{i}'*[Q zeros(6); zeros(6) zeros(6)]*z{i} + 1/8*W_a{i}'*D1*W_a{i} + W_c{i}'*(sigma);
        W_c{i+1}=(-eta_c*cap_gamma{i}*sigma*delta/(m_s^2))*Step+W_c{i};
        cap_gamma{i+1}=(-eta_c*(-beta*cap_gamma{i}+cap_gamma{i}*(sigma*sigma')*...
                        cap_gamma{i}/(m_s^2)))*Step+cap_gamma{i};
        
        W_a{i+1}=(-eta_a2*(W_a{i}-W_c{i})-eta_a1*(1/(1+sigma'*sigma)^(1/2))...
            *grad_phi_z*B*pinv(R)*B'*grad_phi_z'*(W_a{i}-W_c{i})*delta)*Step+W_a{i};
         %W_a{i+1} = W_c{i+1};
        %% Update state
        F_d=-ro(i)*(G*tau{i});
        % Calculate consumption
        if i > 50000
            nominal_E = tau{i}'*tau{i}*Step + nominal_E;
            tau_ref = pinv(G_ref)*(eta_d_dot_dot{i} + F*eta_d_dot{i} - F_d);
            %tau_ref = pinv(G_ref)*(eta_d_dot_dot{i} + F*eta_d_dot{i} );
            nominal_E_ref = tau_ref'*tau_ref*Step + nominal_E_ref;
        end
        %d=inv(G)*F_d;
        %tau_d{i}=G*tau{i};% Wait
        tau_n{i}=F_d;%Wait hmmm
        upsilon{i+1}=(tau_n{i}+tau_d{i}-F*upsilon{i})*Step+upsilon{i};
        eta{i+1}=upsilon{i}*Step+eta{i};
        %% Error between the RISE estimation and -(f+tau_n)
        e_es{i}=Mu{i}+f{i}+tau_n{i};
    end
%% Convert cell into matrix
    eta = cell2mat(eta);
    eta_d = cell2mat(eta_d);
    W_a = cell2mat(W_a);
    W_c = cell2mat(W_c);
    e_es=cell2mat(e_es);
    e1=cell2mat(e1);
    tau = cell2mat(tau);
    Mu=cell2mat(Mu);
    f=cell2mat(f);
    tau_n=cell2mat(tau_n);

    dumeno = cell2mat(e2);
    plot(t,dumeno(1,:));
%% Plot X,Y,Theta
    figure(1);
    subplot(3,1,1)
    plot(t,eta(1,1:size(t,2)),'-','LineWidth',1.2);
    hold on
    plot(t,eta_d(1,:),'--r  ','LineWidth',1.2);
    %title('X position');
    legend({'\eta_1','\eta_{d1}'},'NumColumns',2,'FontSize',12);
    ylabel('\eta_1(m)');
    xticks([0 25 50 75 100 125 150])
    grid on
    % axes('position',[.65 .175 .25 .25])
    % box on % put box around new pair of axes
    % indexOfInterest = (t < 11*pi/8) & (t > 9*pi/8); % range of t near perturbation
    % plot(t(indexOfInterest),signal(indexOfInterest)) % plot on new axes
    % axis tight

    hold off

    subplot(3,1,2)
    plot(t,eta(2,1:size(t,2)),'-','LineWidth',1.2);
    hold on
    plot(t,eta_d(2,:),'--r','LineWidth',1.2);
    %title('Y position');
    legend({'\eta_2','\eta_{d2}'},'NumColumns',2,'FontSize',12);
    ylabel('\eta_2(m)');
    xticks([0 25 50 75 100 125 150])
    grid on
    hold off

    subplot(3,1,3)
    plot(t,eta(3,1:size(t,2)),'-','LineWidth',1.2);
    hold on
    plot(t,eta_d(3,:),'--r','LineWidth',1.2);
    %title('Heading angle');
    legend({'\eta_3','\eta_{d3}'},'NumColumns',2,'FontSize',12);
    ylabel('\eta_3(rad)');
    xticks([0 25 50 75 100 125 150])
    grid on
    hold off
% 
%     figure(4);
%     plot(t,eta(1,1:size(t,2))-eta_d(1,:),'-b','LineWidth',1.2);
%     hold on
%     plot(t,eta(2,1:size(t,2))-eta_d(2,:),'--r','LineWidth',1.2);
%     hold on
%     plot(t,eta(3,1:size(t,2))-eta_d(3,:),'-.g','LineWidth',1.2);
%     title('Error signal');
%     legend({'\epsilon_1(1)','\epsilon_1(2)','\epsilon_1(3)'},'NumColumns',3,'FontSize',8);
%     xlabel('Time(sec)');
%     ylabel('\eta-\eta_d(m or rad)');
%     ylim([-0.5 1])
%     yticks([-0.5 -0.25 0 0.25 0.5 0.75 1])
%     xticks([0 25 50 75 100 125 150])
%     grid on
%     hold off
% %% Plot Real Trajectory vs Reference Trajectory
%     figure(5)
%     plot(eta(1,1:size(t,2)),eta(2,1:size(t,2)));
%     hold on
%     plot(eta_d(1,:),eta_d(2,:));
%     title('Trajectory');
%     legend('Real Trajectory','Reference Trajectory');
%     hold off
%% Plot Wa, Wc
    figure(6)

    plot(t,W_c(1,1:length(t)),'LineWidth',1.2);
    hold on;
    plot(t,W_c(2,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(3,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(4,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(5,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(6,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(7,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(8,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(9,1:length(t)),'LineWidth',1.2);
    plot(t,W_c(10,1:length(t)),'LineWidth',1.2);
    title('Parameters of the critic NN W_{c}');
   legend({'W_{c1}','W_{c2}','W_{c3}','W_{c4}','W_{c5}','W_{c6}',...
        'W_{c7}','W_{c8}','W_{c9}','W_{c10}'},'NumColumns',5,'FontSize',12);
    %ylim([-0.5 4])
    xticks([0 25 50 75 100 125 150])
    grid on
    ylabel('Parameters of the critic NN W_{c}');
    hold off

    figure(7)
    plot(t,W_a(1,1:length(t)),'LineWidth',1.2);
    hold on;
    plot(t,W_a(2,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(3,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(4,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(5,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(6,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(7,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(8,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(9,1:length(t)),'LineWidth',1.2);
    plot(t,W_a(10,1:length(t)),'LineWidth',1.2);
    title('Parameters of the actor NN W_{a}');
    legend({'W_{a1}','W_{a2}','W_{a3}','W_{a4}','W_{a5}','W_{a6}',...
        'W_{a7}','W_{a8}','W_{a9}','W_{a10}'},'NumColumns',5,'FontSize',8);
    xticks([0 25 50 75 100 125 150])
    ylabel('Parameters of the critic NN W_{a}')
    grid on
    hold off
 %% Plot control signal
% 
%     figure(8);
%     p=plot(t,tau(1,1:length(t)),'LineWidth',1.17);    
%     p.Color="r";
%     ylim([-25 20]);
%     hold on
%     grid on
%     ylabel('Torque generated by the motor 1','FontSize',14,'LineWidth',1.17);
% 
%     %figure(9);
%     p=plot(t,tau(2,1:length(t)),'LineWidth',1.17);
%     p.Color="b";
%     %grid on
%     ylabel('Torque generated by the motor 2','FontSize',14,'LineWidth',1.17);
%     ylim([-25 20]);
% 
%     %figure(10);
%     p=plot(t,tau(3,1:length(t)));
%     p.Color="m";
%     ylabel('Torque generated by the motor 3','FontSize',14,'LineWidth',1.17);
%     grid on
%     ylim([-25 20]);
%     legend({'\tau_1', '\tau_2', '\tau_3'}, 'FontSize', 12)
%     xticks([0 25 50 75 100 125 150])
% %% Plot error between the RISE estimation and -(f+tau_n)
% 
%     d=-(f+tau_n);
%     % text(0.5, 0.5, '$$\tilde{a}$$', 'Interpreter', 'LaTeX')
%     figure(9);
%     plot(t,Mu(1,:),'-b');  
%     %title('Graph of the RISE estimation in the x-axes')
%     ylabel('Graph of the RISE estimation in the x-axes')
%     hold on
%     plot(t,d(1,:),'--g');
%     plot(t,e_es(1,:),'r');
%     legend('RISE estimation','$-(\varrho+\tau_s)$','$\widetilde{\Delta}$', 'Interpreter', 'LaTeX');
%     ylim([-12 10])
%     xticks([0 25 50 75 100 125 150])
%     grid on
%     hold off
% 
%     figure(10);
%     plot(t,Mu(2,:),'-b');    
%     %title('Graph of the RISE estimation in the y-axes')
%     ylabel('Graph of the RISE estimation in the y-axes')
%     hold on
%     plot(t,d(2,:),'--g');
%     plot(t,e_es(2,:),'r');
%     legend('RISE estimation','$-(\varrho+\tau_s)$','$\widetilde{\Delta}$', 'Interpreter', 'LaTeX');
%     ylim([-10 10])
%     grid on
%     xticks([0 25 50 75 100 125 150])
%     hold off
% 
%     figure(11);
%     plot(t,Mu(3,:),'-b');
%     %title('Graph of the RISE estimation in the heading angle')
%     ylabel('Graph of the RISE estimation in the heading angle')
%     hold on    
%     plot(t,d(3,:),'--g');
%     plot(t,e_es(3,:),'r');
%     legend('RISE estimation','$-(\varrho+\tau_s)$','$\widetilde{\Delta}$', 'Interpreter', 'LaTeX');
%     ylim([-3 2.5])
%     grid on
%     xticks([0 25 50 75 100 125 150])
%     hold off
%     % %% in the last 10 seconds
%     % figure(12)
%     % plot(eta(1,size(t,2)-10000:size(t,2)-1),eta(2,size(t,2)-10000:size(t,2)-1));
%     % hold on
%     % plot(eta_d(1,end-10000:end),eta_d(2,end-10000:end));
%     % title('Trajectory');
%     % legend('Real Trajectory','Reference Trajectory');
%     % hold off
% 
% 
%     writematrix(tau, 'tau_RISE.csv');
writematrix(V_z,'V_RISE.csv');
    %% Cost function
    figure(13)
    plot(t(1:length(t)),V_z)
    ylim([0 10e-3])


    writematrix(tau, 'tau_RISE.csv');
    

% eta_Robust = readmatrix('eta_Robust.csv');
% eta_RISE = readmatrix('eta_RISE.csv');
% eta_DO = readmatrix('eta_DO.csv');
% eta_AITSM = readmatrix('eta_AITSM.csv');
% eta_d=readmatrix('eta_d.csv');
% 
% figure(10);
%     subplot(3,1,1)
%     plot(t,eta_Robust(1,1:size(t,2)) - eta_d(1,1:size(t,2)),'-','LineWidth',1.2);
%     hold on
%     plot(t,eta_RISE(1,1:size(t,2)) - eta_d(1,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_DO(1,1:size(t,2)) - eta_d(1,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_AITSM(1,1:size(t,2)) - eta_d(1,1:size(t,2)),'-','LineWidth',1.2);   
%     %plot(t,eta_d(1,:),'--r  ','LineWidth',1.2);
% 
%     %title('X position');
%     %legend({'\eta_1','\eta_{d1}'},'NumColumns',2,'FontSize',12);
%     ylabel('\eta_1(m)');
%     xticks([0 25 50 75 100 125 150])
%     grid on
%     % axes('position',[.65 .175 .25 .25])
%     % box on % put box around new pair of axes
%     % indexOfInterest = (t < 11*pi/8) & (t > 9*pi/8); % range of t near perturbation
%     % plot(t(indexOfInterest),signal(indexOfInterest)) % plot on new axes
%     % axis tight
% 
%     hold off
% 
%     subplot(3,1,2)
%     plot(t,eta_Robust(2,1:size(t,2)) - eta_d(2,1:size(t,2)),'-','LineWidth',1.2);
%     hold on
%     plot(t,eta_RISE(2,1:size(t,2)) - eta_d(2,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_DO(2,1:size(t,2)) - eta_d(2,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_AITSM(2,1:size(t,2)) - eta_d(2,1:size(t,2)),'-','LineWidth',1.2);  
%     %title('Y position');
%     %legend({'\eta_2','\eta_{d2}'},'NumColumns',2,'FontSize',12);
%     ylabel('\eta_2(m)');
%     xticks([0 25 50 75 100 125 150])
%     grid on
%     hold off
% 
%     subplot(3,1,3)
%     plot(t,eta_Robust(3,1:size(t,2)) - eta_d(3,1:size(t,2)),'-','LineWidth',1.2);
%     hold on
%     plot(t,eta_RISE(3,1:size(t,2)) - eta_d(3,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_DO(3,1:size(t,2)) - eta_d(3,1:size(t,2)),'-','LineWidth',1.2);
%     plot(t,eta_AITSM(3,1:size(t,2)) - eta_d(3,1:size(t,2)),'-','LineWidth',1.2);  
%     %title('Heading angle');
%     %legend({'\eta_3','\eta_{d3}'},'NumColumns',2,'FontSize',12);
%     ylabel('\eta_3(rad)');
%     xticks([0 25 50 75 100 125 150])
%     grid on
%     hold off

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
figure
%x=['RISE-based controller' 'Robust-based RL controller'];
Robust = readmatrix('consumption_Robust.csv');
RISE = readmatrix('consumption_RISE.csv');
DO = readmatrix('consumption_DO.csv');
AITSM = readmatrix('consumption_AITSM.csv');
x=categorical({'1','2', '3', '4' });
x = reordercats(x,{'1','2', '3', '4'});

y=[ RISE DO Robust AITSM];

bar(x(1),y(1),'FaceColor','g')
hold on
bar(x(2),y(2),'FaceColor','r')
bar(x(3),y(3),'FaceColor','b')
bar(x(4),y(4),'FaceColor','k')
ylabel('$$ \int_{50}^{150} (|\tau_1|+|\tau_2|+|\tau_3|) dt $$', 'Interpreter', 'LaTeX')
ylim([10e4 15e4])
 writematrix(eta,'eta_RISE.csv');
 writematrix(nominal_E,'consumption_RISE.csv');
% writematrix(eta_d,'eta_d.csv'); 
% writematrix(tau,'tau_RISE.csv');
 
writematrix(V_z,'V_RISE.csv');


V_Robust=readmatrix('V_Robust.csv');
V_RISE=readmatrix('V_RISE.csv');
V_DO = readmatrix('V_DO.csv');

figure
plot(t,V_RISE)
hold on
plot(t, V_Robust)
plot(t, V_DO)
legend('RISE-based RL controller','Robust-based RL controller', 'DO-based RL controller')
ylim([-1 100])

