% START STATE AS CROUCHED, AND END STATE AS STANDING

clear
close all

s = SLIP(0);
nStep = 100;
for i = 1:nStep
   % s.step();
   state = s.get_state();
   eom = s.get_eom();
   
end

% Get the mass matrix. H - H' = 0
H = reshape(eom.H, [s.nQ, s.nQ]);
if (H - H')
    disp('Mass matrix not symmetric');
end

% Get the h matrix for coriolis, centripetal, gravity and spring terms
h = reshape(eom.h, [s.nQ, 1]);
% need to make sure this is correct

% Get the contact Jacobian
J = reshape(eom.J, [6, s.nQ]);
% need to make sure this is correct
% the rows 1, 2, 3 are for contact site 0
% the rows 4, 5, 6 are for contact site 1

% J*Qdd + Jdot*Qdot = xdd, set qdd to zero, and get JdotQdot = xdd
Jdot_Qdot = reshape(eom.Jdot_Qdot, [6, 2]);

% Now I have everything I need to calculate qdd:
% These equations are directly from Wensing paper:
% "Generation of Dynamic Humanoid Behaviors Through Task-Space Contro ..."

Tau = zeors(1, nU);     % Torques
Sa = ones(1, s.nU);     % Selector matrix
I = eye(s.nQ);
Hinv = inv(H);          % Is this the correct way to take inverse?
JHinvJT = J*(Hinv\J');  % How to do pseudo-inverse?
Ns = I - J'*JHinvJT*J/Hinv;
gamma = J'*JHinvJT*Jdot_Qdot;

qdd = Hinv\(Ns'*Sa'*Tau - Ns'*h - gamma);      % maybe group Ns



s.close();

%%


	
gamma = Jeq.transpose()*JHinvJ*Jeqdot*qd;

%%

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                          Optimization Setup                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% INDEX
% (1) rootx
% (2) rootz
% (3) rot
% (4) left leg tau
% (5) left leg motor
% (6) left spring
% (7) left ankle
