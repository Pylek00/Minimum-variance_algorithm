
%% INITIAL VALUES
clear
%/Simulation Options
disturbanceStrength = 0.001;
RegulatedValue = 5;
SampleTime = 1;
SimulationTime = 8001;
StepTime = 1;
startCalculationTime = 2000;
endCalculationTime = 6000;
jump = 10;
mvCalcLength = SampleTime*100;

%~/Simulation Options


%/Simulation Options
lambda = 0.05;
%~/Simulation Options

totalSteps = SimulationTime/SampleTime;
totalVariances = abs(startCalculationTime - endCalculationTime)-...
    mvCalcLength;

changeObjTime = startCalculationTime+...
    abs(startCalculationTime - endCalculationTime)/2;
    
x = startCalculationTime+mvCalcLength:SampleTime:...
    startCalculationTime+totalVariances+mvCalcLength-1;

%% Object structure
MVchoice = menu('Which simulation do you want to run?','MVC','MVC2','MVC3','MVC4','MVC5');

switch MVchoice
    case 1
        objNum = [1 -0.35 0 0];
        objDen = [-0.9 0.6 0];
        
        objNum2 = [1 -0.35 0 0];
        objDen2 = [-0.9 0.2 0];
        
        errorNum = [1 0 0 0];
        errorDen = [-0.9 0.2 0];
        
        P = 0.234;
        I = 0.1;
        D = 0;
        N = 1;
        
        P2 = 0.234;
        I2 = 0.1;
        D2 = 0;
        N = 1;
           
        delay = 3/SampleTime;
        kd = delay;
    case 2

        objNum = [0.5 -0.2 -0.12 0 0];
        objDen = [-1.2 0.5 -0.02 0];
        
        objNum2 = [0.5 -0.2 -0.12 0 0];
        objDen2 = [-1.2 0.5 -0.2 0.1];
        
        errorNum = [1 0 0 0 0];
        errorDen = [-1.2 0.5 -0.2 0.1];
        
        P = 2.82;
        I = 1;
        D = 0.25;
        N = 1;
        
        P2 = 2.82;
        I2 = 1;
        D2 = 0.25;
        N = 1;
        
        delay = 1/SampleTime;
        kd = delay;
    case 3
        objNum = [1 -0.5 0.1 0];
        objDen = [-0.8 0.5 0];
        
        objNum2 = [1 -0.5 0 0];
        objDen2 = [-0.7 0 0];
        
        errorNum = [1 0 0 0];
        errorDen = [-0.4 0 0];
        
        P = 0.33;
        I = 0.2;
        D = 0.05;
        N = 1;
        
        P2 = 0.33;
        I2 = 0.2;
        D2 = 0.05;
        N = 1;
        
        delay = 3/SampleTime;
        kd = delay;
    case 4
        objNum = [0.01 0];
        objDen = [-0.9];
        
        objNum2 = [0.01 0];
        objDen2 = [-0.7];
        
        errorNum = [1 0];
        errorDen = [-0.7];
        
        P = 60;
        I = 1.9;
        D = 0.3;
        N = 1;
        
        P2 = 60;
        I2 = 1.9;
        D2 = 0.3;
        N = 1;
        
        delay = 2/SampleTime;
        kd = delay;
    case 5
        objNum = [0.75 0.5 0 0]
        objDen = [-1.1 0.3 0]
        
        objNum2 = [0.75 0.5 0 0]
        objDen2 = [-1.1 0.3 -0.15];
        
        errorNum = [1 0 0 0];
        errorDen = [-1.1 0.3 -0.15];
        
        P = 0.84;
        I = 0.2;
        D = 0.075;
        N = 1;
        
        P2 = 0.84;
        I2 = 0.2;
        D2 = 0.075;
        N = 1;
        
        delay = 1/SampleTime;
        kd = delay;  
end
%% Symulation run
switch MVchoice
    case 1

sym1 = sim('MVC',SimulationTime);

    case 2
sym1 = sim('MVC2',SimulationTime);

    case 3
sym1 = sim('MVC3',SimulationTime);

    case 4
sym1 = sim('MVC4',SimulationTime);

    case 5
sym1 = sim('MVC5',SimulationTime);

end

sym2 = sim('PIDcontrol',SimulationTime);

%% NEW CALCULATION ALGORITHM FOR CONTROL ASSESSMENT
%---------------MINIMUM VARIANCE BENCHMARK INDEX------------------------

mvCalcLength = SampleTime*100*delay;
m=7;
n=mvCalcLength - kd - m;

y  = zeros(n,1);
y2 = zeros(n,1);
y3 = zeros(n,1);
y32 = zeros(n,1);

Ye1 = sym1.yout{2}.Values(1,1).data;
Ye2 = sym2.yout{2}.Values(1,1).data;
Ye3 = sym2.yout{3}.Values(1,1).data;


X1 = zeros(n,m);
X2 = zeros(n,m);
X3 = zeros(n,m);


length = floor(totalVariances/jump)

jmin1 = zeros(1,length);
j2 = zeros(1,length);

eta32 = zeros(1,length);

jmin1p = zeros(1,length);
j2p = zeros(1,length);

eta2 = zeros(1,length);
eta3 = zeros(1,length);

for k=1:length

for  i = 1:n
    y(i)  = sym1.yout{2}.Values(1,1).data...
        (startCalculationTime-i+1+(k-1)*jump,1);
    
    y2(i) = sym2.yout{2}.Values(1,1).data...
        (startCalculationTime-i+1+(k-1)*jump,1);
    %GMVC
    y3(i) = sym2.yout{3}.Values(1,1).data...
    (startCalculationTime-i+1+(k-1)*jump,1);
end

for  i = 1:n
    for  j = 1:m
        X1(i,j) = Ye1(startCalculationTime-kd-j+2-i+(k-1)*jump);
        d=size(X1);
        X2(i,j) = Ye2(startCalculationTime-kd-j+2-i+(k-1)*jump);
        %GMVC
        X3(i,j) = Ye3(startCalculationTime-kd-j+2-i+(k-1)*jump);
    end
end
%MVC
alfa1 = ((inv(X1'*X1)) * X1')*y;
%PID
alfa2 = ((inv(X2'*X2)) * X2')*y2;
%GMVC
alfa3 = ((inv(X3'*X3)) * X3')*y3;

%J_min MVC
jmin1(1,k) = 1/(n-kd-2*m+1)*((y-X1*alfa1)'*(y-X1*alfa1));
%J_min+J_0 MVC
j2(1,k) = 1/(n-kd-m+1)*(y'*y+mean(y)^2);
%J_min PID
jmin1p(1,k) = 1/(n-kd-2*m+1)*((y2-X2*alfa2)'*(y2-X2*alfa2));
%J_min+J_0 PID
j2p(1,k) = 1/(n-kd-m+1)*(y2'*y2+mean(y2)^2);
%MVC
eta2(1,k) = 1-(n-kd-m+1)/(n-kd-2*m+1)*((y2-X2*alfa2)'*(y2-X2*alfa2))/(y2'*y2+mean(y2)^2);

%GMVC
eta3(1,k) = 1-(n-kd-m+1)/(n-kd-2*m+1)*((y3-X3*alfa3)'*(y3-X3*alfa3))/(y3'*y3+mean(y3)^2);

end


%% DRAWING CHARTS
trans = 'G1'
 f1 = figure
 plot(jmin1, 'b'); hold on
 plot(j2, 'r');
 grid
 set(f1,'color','w');
 title('Wskaüniki regulacji J_{min} oraz J dla regulatora MVC')
 xlabel('Numer iteracji algorytmu');
% plot(ni3, 'g');
 legend('J_{min}','J')

f2 = figure
plot(jmin1p, 'b'); hold on
plot(j2p, 'r');
grid
set(f2,'color','w');
%title('Wskaüniki regulacji J_{min PID} oraz J_{PID}');
title(' ');
xlabel('Numer iteracji algorytmu');
%plot(ni3, 'g');
legend('J_{min}','J')

f3 = figure
plot(eta2, 'b'); hold on
grid
set(f3,'color','w');
%title('Wskaüniki regulacji J_{min PID} oraz J_{PID}');
title(' ');
xlabel('Numer iteracji algorytmu');
%plot(ni3, 'g');
legend('\eta')


f5 = figure
plot(eta3, 'b'); hold on
grid
set(f5,'color','w');
%title('Wskaüniki regulacji J_{min PID} oraz J_{PID}');
title(' ');
xlabel('Numer iteracji algorytmu');
legend('\eta GMVC')

