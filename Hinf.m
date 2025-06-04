clc;clear;
%% Parameters
A=[0.906488 0.0816012 -0.0005;
   0.0741349 0.90121 -0.000708383;
   0         0        0.132655];
B=[-0.00150808;
    -0.0096;
    0.867345];
D=[0.00951892;
   0.00038373;
   0];
C=[0 0 1;
   0 0.6 0;
   -1.26 -0.9788 0.4852];
Q=[9.5256 7.3997 -3.6681;
   7.3997 7.9083 -2.8495;
   -3.6681 -2.8495 7.4125];
R=1;
gamma=1;
%% Optimal gain
H_star=[17.0727 -0.2606 -22.6994 0.3240 0.2674;
-0.2606 94.3007 0.3691 -0.7792 0.0566;
-22.6994 0.3691 47.0065 0.8347 0.5539;
0.3240 -0.7792 0.8347 5.7715 -0.0105;
0.2674 0.0566 -0.5539 -0.0105 -0.9929];
K_star=[-0.0383 0.1524 -0.1283];
Kw_star=[0.2699 0.0556 -0.5563];
%% Variables
iteration=250;step_number=5000;
K=cell(1,iteration);
Kw=cell(1,iteration);
H=cell(1,iteration);
L=cell(1,iteration);
y=cell(1,step_number);
u=cell(1,step_number);
wk=cell(1,step_number);
%% Initialization
L{1}=[0.1001    0.0998    0.1001];
H{1}= [17.0727 -0.2606 -22.6994 0.3240 0.2674;
-0.2606 94.3007 0.3691 -0.7792 0.0566;
-22.6994 0.3691 47.0065 0.8347 0.5539;
0.3240 -0.7792 0.8347 5.7715 -0.0105;
0.2674 0.0566 -0.5539 -0.0105 -0.9929];
y{1}=[-0.02000;0.03000;-0.18464];
K{1}=[-0.0383 0.1524 -0.1283];
Kw{1}=[0.2699 0.0556 -0.5563];
wk{1}=0.2*rand;
%% Loop
    for i=1:iteration
        r=[];
        X=[];
        for k=(i-1)*20+1:i*20
            %% Noise
            if k<=3000
                e=2*(sin(1.007*k) + (cos(0.62*k))^2 + sin(1.2*k) +cos(100*k))+0.2*rand;
                %e=0;
            else
                e=0;
            end
            u{k}=-K{i}*y{k}+e;
            %wk{k}=0.2*rand;
            %% Update stage
            %y{k+1}=C*A*pinv(C)*y{k}+C*B*u{k}+C*D*wk{k};
            y{k+1}=C*A*pinv(C)*y{k}+C*B*u{k};
            %% Policy evaluation
            r=[r;(y{k}'*(Q+K{i}'*R*K{i}-gamma^2*Kw{i}'*Kw{i})*y{k})];
            Zk=[eye(3);-K{i};-Kw{i}]*y{k};
            Zkp1=[eye(3);-K{i};-Kw{i}]*y{k+1};
            X=[X;(kron(Zk',Zk')-kron(Zkp1',Zkp1'))];  
        end
   % K_1=K;
   % Kw_1=Kw;
    m=H{i};
    s=pinv(X)*r;
    H{i+1}=reshape(s(1:25),5,5);
   % H=(H+H')/2;
   % index=index+1;
    %%
    Hyy=m(1:3,1:3);Hyu=m(1:3,4);Hyw=m(1:3,5);
    Huy=m(4,1:3);  Huu=m(4,4);  Huw=m(4,5);
    Hwy=m(5,1:3);  Hwu=m(5,4);  Hww=m(5,5);
    %% Policy improvement
    K{i+1}=pinv(Huu-Huw*pinv(Hww)*Hwu)*(Huw*pinv(Hww)*Hwy-Huy+L{i});
    L{i+1}=(Huu-Huw*pinv(Hww)*Hwu)*K{i+1}-(Huw*pinv(Hww)*Hwy-Huy);
    Kw{i+1}=pinv(Hww-Hwu*pinv(Huu)*Huw)*(Hwu*pinv(Huu)*Huy-Hwy);  
    end
    %% Plot
    K=cell2mat(K);
    Kw=cell2mat(Kw);
    H=cell2mat(H);
    L=cell2mat(L);
    y=cell2mat(y);
    u=cell2mat(u);
    wk=cell2mat(wk);
    figure
    plot(y(1,:))
    
    
      





