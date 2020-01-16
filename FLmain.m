%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Octavio Narvaez Aroche                                                  %
% Berkeley Center for Control and Identification                          %
% Spring 2019                                                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Implement Feedback Linearization for the control of the Sit-to-Stand 
% movement of a minimally actuated powered lower limb orthosis at the hips. 
% The complete algorithm is published in:
% O.Narvaez-Aroche, A. Packard, and M. Arcak, “Motion planning of the 
% sit-to-stand movement for powered lower limb orthoses”, ASME 2017 Dynamic 
% Systems & Control Conference, October 2017. Tysons, VA, USA.
% http://dx.doi.org/10.1115/DSCC2017-5289
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Set a clean workspace
clear
% clc
close all

%% Simulation parameters. 

% Total duration of ascension phase of the STS movement. 
tf = 3.5; % [s].

% Sitting position on the z-space for STS 1. 
zi = [-90*pi/180; 0.3099; 0.6678];

% Standing position on the z-space. 
zf = [-5*pi/180; 0; 0.9735];

% Nominal values for parameters of the system.
pnom = [9.68; 12.59; 44.57; 1.165; 0.519; 2.557; 0.533; 0.406; 0.52; 0.533/2; 0.406/2; 0.52/2];

% Real values for the parameters of the system.
punc = pnom + 0.01;

% Initial state.
x0 = [90; -90; 90; 0; 0; 0]*pi/180;

%% Perform Feedback Linearization.

% Solve ODEs under nominal conditions. 
fprintf('\nPerforming feedback linearization under nominal conditions...\n')
tic
[Tnom,Xnom] = ode45(@(t,x) FLThreeLink(t,tf,x,zi,zf,pnom,pnom),[0 tf],x0);
toc

% Solve ODEs under parameter uncertainty. 
fprintf('\nPerforming feedback linearization under parameter uncertainty...\n')
tic
[T,X] = ode45(@(t,x) FLThreeLink(t,tf,x,zi,zf,pnom,punc),[0 tf],x0);
toc

%% Plot simulations.

figure()
for i=1:numel(x0)
   subplot(2,3,i)
   plot(Tnom,Xnom(:,i)*180/pi,'r-',T,X(:,i)*180/pi,'b-','LineWidth',2)
   grid()
   xlabel('$t \, [s]$','Interpreter','Latex')
   if i<=3
       ylabel(['$\theta_{',num2str(i),'}(t) \, [^\circ]$'],'Interpreter','Latex')
   else
       ylabel(['$\dot{\theta}_{',num2str(i-3),'}(t) \, [^\circ/s]$'],'Interpreter','Latex')
   end
   legend({'$p = \bar{p}$','$ p \neq \bar{p}$'},'Interpreter','Latex','Location','Best')
   set(gca,'FontSize',14)
   xlim([Tnom(1),Tnom(end)])
end