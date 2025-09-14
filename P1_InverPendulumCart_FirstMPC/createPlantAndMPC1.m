mdlPlant  = 'mpc_pendcartPlant';
open_system(mdlPlant )

%% 线性化模型
io(1) = linio([mdlPlant '/dF'],1,'openinput'); 
io(2) = linio([mdlPlant '/F'],1,'openinput'); 
io(3) = linio([mdlPlant '/Pendulum and Cart System'],1,'openoutput');
io(4) = linio([mdlPlant '/Pendulum and Cart System'],3,'openoutput');

opspec = operspec(mdlPlant);
opspec.States(1).Known = true; 
opspec.States(1).x = 0;
opspec.States(3).Known = true;
opspec.States(3).x = 0;

options = findopOptions('DisplayReport',false);
op = findop(mdlPlant,opspec,options);
plant = linearize(mdlPlant,op,io);

plant.InputName = {'dF';'F'};
plant.OutputName = {'x';'theta'};

%% 设计MPC控制器

plant = setmpcsignals(plant,'ud',1,'mv',2);
Ts = 0.01; 
PredictionHorizon = 50;
ControlHorizon = 5;
mpcobj = mpc(plant,Ts,PredictionHorizon,ControlHorizon);

% 力约束
mpcobj.MV.Min = -200; 
mpcobj.MV.Max = 200;
% ScaleFactor
mpcobj.MV.ScaleFactor = 100;
% 权重
mpcobj.Weights.MVRate = 1;
mpcobj.Weights.OV = [1.2 1];
% 输入扰动
disturbance_model = getindist(mpcobj); 
setindist(mpcobj,'model',disturbance_model*10);
% 观测扰动
disturbance_model = getoutdist(mpcobj); 
setoutdist(mpcobj,'model',disturbance_model*10);