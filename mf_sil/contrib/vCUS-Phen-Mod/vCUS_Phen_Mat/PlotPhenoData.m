clear all;
close all;
clc;

% Set Variables
global F
currDir = cd;
hFoV = 80;

% Load Data
mdfds = mdfDatastore({[currDir,'\..\vCUS_Phen_CM\SimOutput\csv\SimulationResult.mf4']});
data = readall(mdfds);
load([currDir, '\LobeEvalFunction.mat']);
%load([currDir, '\AK2MeasShape_long6m.mat']);

% Rewrite Data
time = data.Time_1;

%Calculate plot data
for R=1:size(time,1)
    if ( data.SensData_0_Target_NPDir_Fr0_0(R) < (data.Vhcl_Yaw(R) + 270 + (hFoV/2)) ) &&...
            ( data.SensData_0_Target_NPDir_Fr0_0(R) > (270 - (hFoV/2)) )
        refl(1,R) = data.SensData_0_Target_distance_0(R);
        amp(1,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_0(R),data.SensData_0_Target_distance_0(R));
    else
        refl(1,R) = -99999;
        amp(1,R) = 0;
    end
    
    if ( data.SensData_0_Target_NPDir_Fr0_1(R) < (270 + (hFoV/2)) ) &&...
            ( data.SensData_0_Target_NPDir_Fr0_1(R) > (270 - (hFoV/2)) )
        refl(2,R) = data.SensData_0_Target_distance_1(R);
        amp(2,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_1(R),data.SensData_0_Target_distance_1(R));
    else
        refl(2,R) = -99999;
        amp(2,R) = 0;
    end
        
    if ( data.SensData_0_Target_NPDir_Fr0_2(R) < (270 + (hFoV/2)) ) &&...
            ( data.SensData_0_Target_NPDir_Fr0_2(R) > (270 - (hFoV/2)) )
        refl(3,R) = data.SensData_0_Target_distance_2(R);
        amp(3,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_2(R),data.SensData_0_Target_distance_2(R));
    else
        refl(3,R) = -99999;
        amp(3,R) = 0;
    end
        
    if ( data.SensData_0_Target_NPDir_Fr0_3(R) < (270 + (hFoV/2)) ) &&...
            ( data.SensData_0_Target_NPDir_Fr0_3(R) > (270 - (hFoV/2)) )
        refl(4,R) = data.SensData_0_Target_distance_3(R);
        amp(4,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_3(R),data.SensData_0_Target_distance_3(R));
    else
        refl(4,R) = -99999;
        amp(4,R) = 0;
    end
end

% for R=1:size(time,1)
%     refl(1,R) = data.SensData_0_Target_distance_0(R);
%     amp(1,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_0(R),data.SensData_0_Target_distance_0(R));
%     refl(2,R) = data.SensData_0_Target_distance_1(R);
%     amp(2,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_1(R),data.SensData_0_Target_distance_1(R));
%     refl(3,R) = data.SensData_0_Target_distance_2(R);
%     amp(3,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_2(R),data.SensData_0_Target_distance_2(R));
%     refl(4,R) = data.SensData_0_Target_distance_3(R);
%     amp(4,R) = getAmplitude(data.SensData_0_Target_NPDir_Fr0_3(R),data.SensData_0_Target_distance_3(R));
% end

figure;
% amp(1,1) = 0.05;
scatter(time,refl(1,:),10,amp(1,:),'filled');
hold on;
scatter(time,refl(2,:),10,amp(2,:),'filled');
scatter(time,refl(3,:),10,amp(3,:),'filled');
%scatter(time,refl(4,:),10,amp(3,:),'filled');
axis equal;
grid on;
xlim([0 20]);
ylim([-4 10]);
title('Scenario: Parallel Parking Scanning (10kph)');
xlabel('Time in s');
ylabel('Distance to reflection points in m');
colorbar;


%% Get Amplitude
function amplitude = getAmplitude(ang,dist)
    global F

    R = 1;
    RefLength = 0.45;
    Temp = 22.4;
    c = 331.5 + (0.6 * Temp);
    alpha = 0.1151/c;
    maxP = 8.598799071;
    
    corrAngle = deg2rad(ang + 90);
    
    p0 = F(corrAngle,0);
    G_tr = p0/maxP;

    amplitude = ( G_tr^2 * R * exp((alpha * -1) * (dist - RefLength)) ) / (p0 * (dist/RefLength));
end