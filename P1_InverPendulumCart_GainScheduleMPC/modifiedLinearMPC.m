load plant.mat

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

% 给输出增加软约束
mpcobj.OV(2).Min = -pi/2;  % 摆杆角度上限±90°（π/2弧度）
mpcobj.OV(2).Max = pi/2;
mpcobj.Weights.ECR = 100;  % 从1e5降到100，让约束更"软"（允许小幅度超调，但避免大幅偏移）

% 输入扰动
disturbance_model = getindist(mpcobj); 
setindist(mpcobj,'model',disturbance_model*10);
% 观测扰动
disturbance_model = getoutdist(mpcobj); 
setoutdist(mpcobj,'model',disturbance_model*10);