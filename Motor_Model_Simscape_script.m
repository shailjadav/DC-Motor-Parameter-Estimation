%% DC MOTOR PARAMETER ESTIMATION USING LEAST SQAURE AND SEARCH SPACE
% This code will help you to indentify parametrs of DC motors for better
% control. Idea is to have measuremnt of Voltage, Current and Motor
% Position to estimate motor parameters.

% Author: Shail Jadav
% Visit:  shailjadav.github.io
% December 10,2020
%% Intialise
close all
clear
clc
%% Simulation for Motor Behaviour
L = 0.55;
R = 3;
K = 0.085;
J = 0.6;
b = 0.25;

sim('Motor_Model_Simscape.mdl');
%% Measurement with Noise

V=Voltage.signals.values + rand(length(tout),1)/100 ;
I=Current.signals.values + rand(length(tout),1)/100;
dI=CurrentDerivative.signals.values + rand(length(tout),1)/100;
dTH=Velocity.signals.values + rand(length(tout),1)/100;
ddTH=Acc.signals.values + rand(length(tout),1)/100;
TH=Position.signals.values+ rand(length(tout),1)/100;

figure
subplot(231)
plot(tout,TH)
hold on
subplot(232)
plot(tout,dTH)
hold on
subplot(233)
plot(tout,ddTH)
hold on
subplot(234)
plot(tout,I)
hold on
subplot(235)
plot(tout,dI)
hold on
subplot(236)
plot(tout,V)
hold on

%% Find L R K

% Inital Least squares
for i=1:1:floor(length(I))
    Data=[dI(i); I(i); dTH(i)];
    IP=V(i);
    MAP(i,:)=IP*pinv(Data);
end

f=1;
for i=1:1:length(MAP)
    if(MAP(i,2)>R-0.05)
        if(MAP(i,1)>0)
            if(MAP(i,3)>0)
                MAPF(f,:)=MAP(i,:);
                f=f+1;
            end
        end
    end
end

Re=sum(MAPF(:,2))/length(MAPF);
Le=sum(MAPF(:,1))/length(MAPF);
Ke=sum(MAPF(:,3))/length(MAPF);
%% First Filtering
FG=[Le Re Ke];

L1=Le - (Le/2);

R1=Re - (Re/2);

K1=Ke - (Ke/2);

cnt=1;
L11=L1;
R11=R1;
K11=K1;

for i=1:1:20
    L11=L11+(Le/(20));
    R11=R1;
    for j=1:1:20
        R11=R11+(Re/(20));
        K11=K1;
        for k=1:1:20
            K11=K11+(Ke/(20));
            
            Data=[dI  I  dTH];
            Ve=Data*[L11 R11 K11]';
            E(cnt,1)=norm(V-Ve);
            E(cnt,2)=L11;
            E(cnt,3)=R11;
            E(cnt,4)=K11;
            cnt=cnt+1;
        end
    end
end

for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        FDATA=E(i,:);
    end
end

%% Second Filtering
FG=FDATA(1,2:end);
Le=FDATA(1,2);
Re=FDATA(1,3);
Ke=FDATA(1,4);

L1=Le - (Le/2);

R1=Re - (Re/2);

K1=Ke - (Ke/2);

cnt=1;
L11=L1;
R11=R1;
K11=K1;

for i=1:1:10
    L11=L11+(Le/(10));
    R11=R1;
    for j=1:1:10
        R11=R11+(Re/(10));
        K11=K1;
        for k=1:1:10
            K11=K11+(Ke/(10));
            
            Data=[dI  I  dTH];
            Ve=Data*[L11 R11 K11]';
            E(cnt,1)=norm(V-Ve);
            E(cnt,2)=L11;
            E(cnt,3)=R11;
            E(cnt,4)=K11;
            cnt=cnt+1;
        end
    end
end

for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        SDATA=E(i,:);
    end
end

clearvars -except SDATA FDATA V I dI ddTH dTH TH tout
%% Find J b
% Inital Least squares

for i=1:1:floor(length(I))
    Data=[ddTH(i); dTH(i)];
    IP=SDATA(1,4)*I(i);
    MAP(i,:)=IP*pinv(Data);
end

f=1;
for i=1:1:length(MAP)
    if(MAP(i,1)>0)
        if(MAP(i,2)>0)
            MAPF(f,:)=MAP(i,:);
            f=f+1;
        end
    end
end


Je=sum(MAPF(:,1))/length(MAPF);
be=sum(MAPF(:,2))/length(MAPF);


%% First Filtering
FG=[Je be];

J1=Je - (Je/2);
b1=be - (be/2);


cnt=1;
J11=J1;
b11=b1;


for i=1:1:20
    J11=J11+(Je/(20));
    b11=b1;
    for j=1:1:20
        b11=b11+(be/(20));
        
        
        Data=[ddTH  dTH];
        Ve=Data*[J11 b11]';
        E(cnt,1)=norm(SDATA(1,4)*I - Ve);
        E(cnt,2)=J11;
        E(cnt,3)=b11;
        cnt=cnt+1;
    end
end


for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        F1DATA=E(i,:);
    end
end

%% Second Filtering

FG=F1DATA(1,2:end);

Je=F1DATA(1,2);
be=F1DATA(1,3);

cnt=1;
J11=J1;
b11=b1;

for i=1:1:10
    J11=J11+(Je/(10));
    b11=b1;
    for j=1:1:10
        b11=b11+(be/(10));
        
        
        Data=[ddTH  dTH];
        Ve=Data*[J11 b11]';
        E(cnt,1)=norm(SDATA(1,4)*I - Ve);
        E(cnt,2)=J11;
        E(cnt,3)=b11;
        cnt=cnt+1;
    end
end


for i=1:1:length(E)
    if(min(E(:,1))==E(i,1))
        S1DATA=E(i,:);
    end
end

%% Display
clc
disp(strcat("Estimated L","=",num2str(SDATA(1,2))))
disp(strcat("Estimated R","=",num2str(SDATA(1,3))))
disp(strcat("Estimated K","=",num2str(SDATA(1,4))))
disp(strcat("Estimated J","=",num2str(S1DATA(1,2))))
disp(strcat("Estimated b","=",num2str(S1DATA(1,3))))


% Validation
L=(SDATA(1,2));
R=(SDATA(1,3));
K=(SDATA(1,4));
J=S1DATA(1,2);
b=S1DATA(1,3);

sim('Motor_Model_Simscape.mdl');

V=Voltage.signals.values ;
I=Current.signals.values ;
dI=CurrentDerivative.signals.values ;
dTH=Velocity.signals.values ;
ddTH=Acc.signals.values ;
TH=Position.signals.values;


subplot(231)
plot(tout,TH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular position(rad)','interpreter','latex','fontsize',14)

subplot(232)
plot(tout,dTH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular velocity(rad /s)','interpreter','latex','fontsize',14)

subplot(233)
plot(tout,ddTH)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Angular acceleration(rad /s*s)','interpreter','latex','fontsize',14)

subplot(234)
plot(tout,I)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Current (A)','interpreter','latex','fontsize',14)

subplot(235)
plot(tout,dI)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Current Derivative(A/s)','interpreter','latex','fontsize',14)

subplot(236)
plot(tout,V)
grid minor
xlabel('Time(s)','interpreter','latex','fontsize',14)
ylabel('Voltage(V)','interpreter','latex','fontsize',14)




