clear
clc

M = [.198 .249 .450 .650 .500 .650 .800 .700 .800 .900];

a = [340.3 340.3 340.3 340.3 316.0 316.0 316.0 294.9 294.9 294.9];
g = 9.80665;

Yv = [-.0890 -.0997 -.143 -.197 -.0822 -.104 -.120 -.0488 -.0558 -.0606];
Yb = [-19.7 -27.8 -71.7 -143.0 -42.6 -70.4 -99.4 -33.1 -43.2 -52.8];
Lb = [-1.33 -1.63 -3.19 -5.45 -2.05 -2.96 -4.12 -1.45 -3.05 -1.32];
Nb = [.168 .247 .810 1.82 .419 .923 1.62 .404 .598 .971];

Lp = [-.975 -1.10 -1.12 -1.47 -.652 -.804 -.974 -.404 -.465 -.459];
Np = [-.166 -.125 -.0706 -.0214 -.0701 -.0531 -.0157 -.0366 -.0318 -.00284];

Lr = [.327 .198 .379 .256 .376 .317 .292 .312 .388 .280];
Nr = [-.217 -.229 -.246 -.344 -.140 -.193 -.232 -.0963 -.115 -.141];

Lda = [.227 .318 .229 .372 .128 .210 .310 .0964 .143 .186];
Nda = [.0264 .0300 .0285 .0371 .0177 .0199 .0127 .00875 .00775 -.00611];

Ydr = [.0148 .0182 .0226 .0213 .0131 .0142 .0124 .00777 .00729 .00464];
Ldr = [.0636 .110 .254 .318 .148 .211 .183 .115 .153 .100];
Ndr = [-.151 -.233 -.614 -.970 -.391 -.616 -.922 -.331 -.475 -.442];

FLIGHT_CONDITION_NOM = 1;
FLIGHT_CONDITION_PER = 10;

U_nom = a(FLIGHT_CONDITION_NOM)*M(FLIGHT_CONDITION_NOM); 
U_per = a(FLIGHT_CONDITION_PER)*M(FLIGHT_CONDITION_PER);
gamma0 = 0;

% Nominal system
A_nom = [Yb(FLIGHT_CONDITION_NOM)/U_nom 0 -1 g/U_nom*cos(gamma0) 0;
    Lb(FLIGHT_CONDITION_NOM) Lp(FLIGHT_CONDITION_NOM) Lr(FLIGHT_CONDITION_NOM) 0 0;
    Nb(FLIGHT_CONDITION_NOM) Np(FLIGHT_CONDITION_NOM) Nr(FLIGHT_CONDITION_NOM) 0 0;
    0 1 tan(gamma0) 0 0;
    0 0 sec(gamma0) 0 0];

B_nom = [0 Ydr(FLIGHT_CONDITION_NOM);
    Lda(FLIGHT_CONDITION_NOM) Ldr(FLIGHT_CONDITION_NOM);
    Nda(FLIGHT_CONDITION_NOM) Ndr(FLIGHT_CONDITION_NOM);
    0 0;
    0 0];

Cf = eye(5);

Df = zeros(5,2);

sys_f_nom = ss(A_nom,B_nom,Cf,Df);
set(sys_f_nom, 'inputname', {'aileron' 'rudder'},...
    'inputunit', {'rad';'rad'},...
    'outputname', {'sideslip' 'roll rate' 'yaw rate' 'bank angle' 'yaw angle'},...
    'outputunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'},...
    'statename', {'beta' 'p' 'r' 'phi' 'psi'},...
    'stateunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'});
sys_f_nom.InputGroup.controls = [1 2];

% Perturbed system
A_per = [Yb(FLIGHT_CONDITION_PER)/U_per 0 -1 g/U_per*cos(gamma0) 0;
    Lb(FLIGHT_CONDITION_PER) Lp(FLIGHT_CONDITION_PER) Lr(FLIGHT_CONDITION_PER) 0 0;
    Nb(FLIGHT_CONDITION_PER) Np(FLIGHT_CONDITION_PER) Nr(FLIGHT_CONDITION_PER) 0 0;
    0 1 tan(gamma0) 0 0;
    0 0 sec(gamma0) 0 0];

B_per = [0 Ydr(FLIGHT_CONDITION_PER);
    Lda(FLIGHT_CONDITION_PER) Ldr(FLIGHT_CONDITION_PER);
    Nda(FLIGHT_CONDITION_PER) Ndr(FLIGHT_CONDITION_PER);
    0 0;
    0 0];

sys_f_per = ss(A_per,B_per,Cf,Df);
set(sys_f_per, 'inputname', {'aileron' 'rudder'},...
    'inputunit', {'rad';'rad'},...
    'outputname', {'sideslip' 'roll rate' 'yaw rate' 'bank angle' 'yaw angle'},...
    'outputunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'},...
    'statename', {'beta' 'p' 'r' 'phi' 'psi'},...
    'stateunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'});
sys_f_per.InputGroup.controls = [1 2];

%Weight matrices
Q = diag(8*[.8001 .1288 .055 250.011 .01]);
Q(4,4) = 30*10^(-4);
Q(5,5) = 450*10^(-2);
R = diag([2 0.2]);

Lv = 533.3; % turbulence scale length, m (1750ft)
Lw = Lv;
sigmav = 1.542; % 0.1*W20,m/s (W20-wind speed at 20 feet, 30kts)
sigmaw = sigmav;
b = 59.64; % wingspan,m (195.68ft)

s = tf('s');
Hv_nom = sigmav*sqrt(Lv/(pi*U_nom))*(1+sqrt(3)*Lv/U_nom*s)/((1+Lv/U_nom*s)^2);
Hr_nom = (s/U_nom/(1+(3*b/(pi*U_nom))*s));
Hp_nom = sigmaw*sqrt(0.8/U_nom)*((pi/(4*b))^(1/6))/(Lw^(1/3)*(1+(4*b/(pi*U_nom))*s));

Hv_per = sigmav*sqrt(Lv/(pi*U_per))*(1+sqrt(3)*Lv/U_per*s)/((1+Lv/U_per*s)^2);
Hr_per = (s/U_per/(1+(3*b/(pi*U_per))*s));
Hp_per = sigmaw*sqrt(0.8/U_per)*((pi/(4*b))^(1/6))/(Lw^(1/3)*(1+(4*b/(pi*U_per))*s));

[numHv_nom,denHv_nom] = tfdata(Hv_nom,'v');
[numHv_per,denHv_per] = tfdata(Hv_per,'v');
[numHr_nom,denHr_nom] = tfdata(Hr_nom,'v');
[numHr_per,denHr_per] = tfdata(Hr_per,'v');
[numHp_nom,denHp_nom] = tfdata(Hp_nom,'v');
[numHp_per,denHp_per] = tfdata(Hp_per,'v');
 
Bex_nom = [-A_nom(1:3,1:3)
    0 0 0;
    0 0 0];
Bex_nom = [Bex_nom B_nom];

Bex_per = [-A_per(1:3,1:3)
   0 0 0;
   0 0 0];
Bex_per = [Bex_per B_per];

set(0,'defaultlinelinewidth',1.5)

Ts = 0.02; % 50 Hz
[Kd,S,e] = lqrd(A_nom,B_nom,Q,R,Ts);

sys_f_cloop_nom = feedback(sys_f_nom,Kd);
disp('Poles of closed loop nominal system')
pole(sys_f_cloop_nom)

sys_f_cloop_per = feedback(sys_f_per,Kd);
disp('Poles of closed loop perturbed system')
pole(sys_f_cloop_per)

disp('H2 of closed loop system')
NH2_f = [normh2(sys_f_cloop_nom); normh2(sys_f_cloop_per)]

disp('Hinf of closed loop system')
NHinf_f = [normhinf(sys_f_cloop_nom); normhinf(sys_f_cloop_per)]

load('/Users/ivan/Documents/MATLAB/MS/airdata_full_nominal.mat')
load('/Users/ivan/Documents/MATLAB/MS/airdata_full_perturbed.mat')

tf_nom = out_full_nom.time;
tf_per = out_full_per.time;

kf_nom = find(tf_nom == 100);
tSigmaf_nom = kf_nom:length(tf_nom);

kf_per = find(tf_per == 100);
tSigmaf_per = kf_per:length(tf_per);

daf_nom = out_full_nom.data(:,1);
daf_per = out_full_per.data(:,1);

drf_nom = out_full_nom.data(:,2);
drf_per = out_full_per.data(:,2);

betaf_nom = out_full_nom.data(:,3);
betaf_per = out_full_per.data(:,3);

pf_nom = out_full_nom.data(:,4);
pf_per = out_full_per.data(:,4);

rf_nom = out_full_nom.data(:,5);
rf_per = out_full_per.data(:,5);

phif_nom = out_full_nom.data(:,6);
phif_per = out_full_per.data(:,6);

psif_nom = out_full_nom.data(:,7);
psif_per = out_full_per.data(:,7);

sigmaDaf = [std(daf_nom(tSigmaf_nom)); std(daf_per(tSigmaf_per))];
sigmaDrf = [std(drf_nom(tSigmaf_nom)); std(drf_per(tSigmaf_per))];
sigmaBetaf = [std(betaf_nom(tSigmaf_nom)); std(betaf_per(tSigmaf_per))];
sigmaPf = [std(pf_nom(tSigmaf_nom)); std(pf_per(tSigmaf_per))];
sigmaRf = [std(rf_nom(tSigmaf_nom)); std(rf_per(tSigmaf_per))];
sigmaPhif = [std(phif_nom(tSigmaf_nom)); std(phif_per(tSigmaf_per))];
sigmaPsif = [std(psif_nom(tSigmaf_nom)); std(psif_per(tSigmaf_per))];
 
ssigmaAllf = [sigmaDaf sigmaDrf sigmaBetaf sigmaPf sigmaRf sigmaPhif sigmaPsif]

figure(1), plot(tf_nom,betaf_nom,tf_per,betaf_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Sideslip angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(2), plot(tf_nom,pf_nom,tf_per,pf_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Roll rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(3), plot(tf_nom,rf_nom,tf_per,rf_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Yaw rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(4), plot(tf_nom,phif_nom,tf_per,phif_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Bank angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(5), plot(tf_nom,psif_nom,tf_per,psif_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Heading angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(6), plot(tf_nom,daf_nom,tf_per,daf_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Ailerons deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(7), plot(tf_nom,drf_nom,tf_per,drf_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (full state)')
xlabel('Time, s')
ylabel('Rudder deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

C = [0 1 0 0 0;
    0 0 1 0 0;
    0 0 0 1 0;
    0 0 0 0 1];
D = zeros(4,2);

sys_nom = ss(A_nom,B_nom,C,D);
set(sys_nom, 'inputname', {'aileron' 'rudder'},...
    'inputunit', {'rad';'rad'},...
    'outputname', {'roll rate' 'yaw rate' 'bank angle' 'yaw angle'},...
    'outputunit', {'rad/s' 'rad/s' 'rad' 'rad'},...
    'statename', {'beta' 'p' 'r' 'phi' 'psi'},...
    'stateunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'});
sys_nom.InputGroup.controls = [1 2];

sys_per = ss(A_per,B_per,C,D);
set(sys_per, 'inputname', {'aileron' 'rudder'},...
    'inputunit', {'rad';'rad'},...
    'outputname', {'roll rate' 'yaw rate' 'bank angle' 'yaw angle'},...
    'outputunit', {'rad/s' 'rad/s' 'rad' 'rad'},...
    'statename', {'beta' 'p' 'r' 'phi' 'psi'},...
    'stateunit', {'rad' 'rad/s' 'rad/s' 'rad' 'rad'});
sys_per.InputGroup.controls = [1 2];

% Initialize description of LMI system
setlmis([]);

% Specify matrix variables in LMI problem
X1 = lmivar(1,[5 1]);
X2 = lmivar(1,[5 1]);
M = lmivar(2,[2 5]);
Z = lmivar(2,[5 4]);

% Specify term content of LMIs

% 1st LMI
% [ X1*A1' + A1*X1 + M'B' + B*M   X1*Q^1/2    M'*R^1/2
%   Q^1/2*X1                    -I          0           <   0
%   R^1/2*M                     0           -I      ]

% X1*A1' + A1*X1
lmiterm([1 1 1 X1],A_nom,1,'s');
% M'B' + B*M
lmiterm([1 1 1 M],B_nom,1,'s');
% X1*Q^1/2
lmiterm([1 1 2 X1],1,sqrt(Q));
% M'*R^1/2
lmiterm([1 1 3 -M],1,sqrt(R));
% -I
lmiterm([1 2 2 0],-1);
% -I
lmiterm([1 3 3 0],-1);

% 2nd LMI
% [ X1*A2' + A2*X1 + M'B' + B*M   X1*Q^1/2    M'*R^1/2
%   Q^1/2*X1                    -I          0           <   0
%   R^1/2*M                     0           -I      ]
% X1*A2' + A2*X1
lmiterm([2 1 1 X1],A_per,1,'s');
% M'B' + B*M
lmiterm([2 1 1 M],B_per,1,'s');
% X1*Q^1/2
lmiterm([2 1 2 X1],1,sqrt(Q));
% M'*R^1/2
lmiterm([2 1 3 -M],1,sqrt(R));
% -I
lmiterm([2 2 2 0],-1);
% -I
lmiterm([2 3 3 0],-1);


% 3rd LMI
% A1'*X2 + X2*A1 + C'*Z' + Z*C < 0
lmiterm([3 1 1 X2],1,A_nom,'s');
lmiterm([3 1 1 Z],1,C,'s');

% 4th LMI
% A2'*X2 + X2*A2 + C'*Z' + Z*C < 0
lmiterm([4 1 1 X2],1,A_per,'s');
lmiterm([4 1 1 Z],1,C,'s');

% % 5th LMI
% % X1 > 0
lmiterm([-5 1 1 X1],1,1);
%  
% % 6th LMI
% % X2 > 0
lmiterm([-6 1 1 X2],1,1);

lmis = getlmis;

[tmin,xfeas] = feasp(lmis);

X11 = dec2mat(lmis,xfeas,X1);
X21 = dec2mat(lmis,xfeas,X2);
M1 = dec2mat(lmis,xfeas,M);
Z1 = dec2mat(lmis,xfeas,Z);

K = M1/X11;
L = X21\Z1;

Areg = A_nom + L*C + B_nom*K;
Breg = L;
Creg = K;
Dreg = 0;
sysReg = ss(Areg,Breg,Creg,Dreg);
sysRegPlant_nom = series(sys_nom,sysReg);
cloop_nom = feedback(sysRegPlant_nom, eye(2));

disp('Poles of closed loop nominal system')
pole(cloop_nom)

sysRegPlant_per = series(sys_per,sysReg);
cloop_per = feedback(sysRegPlant_per, eye(2));
disp('Poles of closed loop perturbed system')
pole(cloop_per)

disp('H2 of closed loop system')
NH2 = [normh2(cloop_nom); normh2(cloop_per)]

disp('Hinf of closed loop system')
NHinf = [normhinf(cloop_nom); normhinf(cloop_per)]

load('/Users/ivan/Documents/MATLAB/MS/airdata_nominal.mat')
load('/Users/ivan/Documents/MATLAB/MS/airdata_perturbed.mat')

t_nom = out_nom.time;
t_per = out_per.time;

k_nom = find(t_nom == 100);
tSigma_nom = k_nom:length(t_nom);

k_per = find(t_per == 100);
tSigma_per = k_per:length(t_per);

da_nom = out_nom.data(:,1);
da_per = out_per.data(:,1);

dr_nom = out_nom.data(:,2);
dr_per = out_per.data(:,2);

beta_nom = out_nom.data(:,3);
beta_per = out_per.data(:,3);

p_nom = out_nom.data(:,4);
p_per = out_per.data(:,4);

r_nom = out_nom.data(:,5);
r_per = out_per.data(:,5);

phi_nom = out_nom.data(:,6);
phi_per = out_per.data(:,6);

psi_nom = out_nom.data(:,7);
psi_per = out_per.data(:,7);

sigmaDa = [std(da_nom(tSigma_nom)); std(da_per(tSigma_per))];
sigmaDr = [std(dr_nom(tSigma_nom)); std(dr_per(tSigma_per))];
sigmaBeta = [std(beta_nom(tSigma_nom)); std(beta_per(tSigma_per))];
sigmaP = [std(p_nom(tSigma_nom)); std(p_per(tSigma_per))];
sigmaR = [std(r_nom(tSigma_nom)); std(r_per(tSigma_per))];
sigmaPhi = [std(phi_nom(tSigma_nom)); std(phi_per(tSigma_per))];
sigmaPsi = [std(psi_nom(tSigma_nom)); std(psi_per(tSigma_per))];
 
sigmaAll = [sigmaDa sigmaDr sigmaBeta sigmaP sigmaR sigmaPhi sigmaPsi]

figure(8), plot(t_nom,beta_nom,t_per,beta_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Sideslip angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(9), plot(t_nom,p_nom,t_per,p_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Roll rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(10), plot(t_nom,r_nom,t_per,r_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Yaw rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(11), plot(t_nom,phi_nom,t_per,phi_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Bank angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(12), plot(t_nom,psi_nom,t_per,psi_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Heading angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(13), plot(t_nom,da_nom,t_per,da_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Ailerons deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(14), plot(t_nom,dr_nom,t_per,dr_per,'--')
title('B-747 lateral-directional motion control in the presence of external disturbances (with observer)')
xlabel('Time, s')
ylabel('Rudder deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

% Together
tSwitch = 70;

load('/Users/ivan/Documents/MATLAB/MS/airdata_together_nominal.mat')
load('/Users/ivan/Documents/MATLAB/MS/airdata_together_perturbed.mat')

tt_nom = out_together_nom.time;
tt_per = out_together_per.time;

dat_nom = out_together_nom.data(:,1);
dat_per = out_together_per.data(:,1);

drt_nom = out_together_nom.data(:,2);
drt_per = out_together_per.data(:,2);

betat_nom = out_together_nom.data(:,3);
betat_per = out_together_per.data(:,3);

pt_nom = out_together_nom.data(:,4);
pt_per = out_together_per.data(:,4);

rt_nom = out_together_nom.data(:,5);
rt_per = out_together_per.data(:,5);

phit_nom = out_together_nom.data(:,6);
phit_per = out_together_per.data(:,6);

psit_nom = out_together_nom.data(:,7);
psit_per = out_together_per.data(:,7);

figure(15), plot(tt_nom,betat_nom,tt_per,betat_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Sideslip angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(16), plot(tt_nom,pt_nom,tt_per,pt_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Roll rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(17), plot(tt_nom,rt_nom,tt_per,rt_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Yaw rate, deg/s')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(18), plot(tt_nom,phit_nom,tt_per,phit_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Bank angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(19), plot(tt_nom,psit_nom,tt_per,psit_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Heading angle, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(20), plot(tt_nom,dat_nom,tt_per,dat_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Ailerons deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on

figure(21), plot(tt_nom,drt_nom,tt_per,drt_per,'--')
vline(tSwitch,'-',sprintf('t = %d s',tSwitch))
title('B-747 lateral-directional motion control in the presence of external disturbances')
xlabel('Time, s')
ylabel('Rudder deflection, deg')
legend(sprintf('Nominal, U = %.1f m/s', U_nom), sprintf('Perturbed, U = %.1f m/s', U_per))
grid on