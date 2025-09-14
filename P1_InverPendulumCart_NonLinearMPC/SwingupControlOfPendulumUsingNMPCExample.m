%% Swing-Up Control of Pendulum Using Nonlinear Model Predictive Control

% Create Nonlinear MPC Controller
nx = 4;
ny = 2;
nu = 1;
nlobj = nlmpc(nx, ny, nu);

%
Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

% Specify Nonlinear Plant Model
nlobj.Model.StateFcn = "pendulumDT0";

nlobj.Model.IsContinuousTime = false;

%
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = 'pendulumOutputFcn';

nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0; 0 0 1 0];



%% Define Cost and Constraints
nlobj.Weights.OutputVariables = [3 3];
nlobj.Weights.ManipulatedVariablesRate = 0.1;
%
nlobj.OV(1).Min = -10;
nlobj.OV(1).Max = 10;
%
nlobj.MV.Min = -100;
nlobj.MV.Max = 100;

%% Validate Nonlinear MPC Controller
% After designing a nonlinear MPC controller object, it is best practice to
% check the functions you defined for the prediction model, state function,
% output function, custom cost, and custom constraints, as well as their
% Jacobians. To do so, use the |validateFcns| command. This function
% detects any dimensional and numerical inconsistencies in these functions.
x0 = [0.1;0.2;-pi/2;0.3];
u0 = 0.4;
validateFcns(nlobj,x0,u0,[],{Ts});
%% Closed-Loop Simulation in Simulink
% Validate the nonlinear MPC controller with a closed-loop simulation in
% Simulink(R).
%
% Open the Simulink model.
mdl = 'mpc_pendcartNMPC';
open_system(mdl)

createParameterBus(nlobj,[mdl '/Nonlinear MPC Controller'],'myBusObject',{Ts});
