%% Time-Varying MPC Control of an Inverted Pendulum on a Cart

mdlMPC = 'mpc_pendcartLTVMPC';
open_system(mdlMPC);
%% Adaptive MPC Design
x0 = zeros(4,1);
u0 = zeros(1,1);
%% 
% Analytically obtain a linear plant model using the ODEs.
[~,~,A,B,C,D] = pendulumCT(x0, u0);
plant = ss(A,B,C([1 3],:),D([1 3],:)); % position and angle

%%
Ts = 0.01;
PredictionHorizon = 60; % 预测时域比我们第一个线性MPC稍微长了一些，原来是50
ControlHorizon = 3; % 控制时域比我们第一个线性MPC稍微短了一些，原来是5
mpcobj = mpc(c2d(plant,Ts),Ts,PredictionHorizon,ControlHorizon);

% 力约束=-
mpcobj.MV.Min = -100; % 比之前约束更严苛了，原来是 +-200
mpcobj.MV.Max = 100;
% scale factor
mpcobj.MV.ScaleFactor = 100;
% 权重
mpcobj.Weights.MVRate = 1; 
mpcobj.Weights.OV = [0.6 1.2];% % [x,theta]，调高了 theta 的相对权重，以稳定优先，原来是[1.2 1]

% 观测输出扰动模型
setoutdist(mpcobj,'model',[0;tf(1)]); %原来是白噪声，这里改为了延迟环节。告诉控制器测量输出有延迟
% 状态观测
setEstimator(mpcobj,'custom');% 使用自定义状态估计器（而非默认估计器）。

bdclose(mdlMPC);
