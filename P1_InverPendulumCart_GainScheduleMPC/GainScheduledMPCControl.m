%% Gain-Scheduled MPC Control of Inverted Pendulum on Cart
% Control Structure
mdlPlant = 'mpc_pendcartPlant';
open_system(mdlPlant)
% Linear Plant Model
io(1) = linio([mdlPlant '/dF'],1,'openinput');
io(2) = linio([mdlPlant '/F'],1,'openinput');
io(3) = linio([mdlPlant '/Pendulum and Cart System'],1,'openoutput');
io(4) = linio([mdlPlant '/Pendulum and Cart System'],3,'openoutput');
%
angles = [-4*pi/9 0 4*pi/9];
for ct=1:length(angles)
    opspec(ct) = operspec(mdlPlant);
    opspec(ct).States(1).Known = true;
    opspec(ct).States(1).x = 0;
    opspec(ct).States(2).SteadyState = false;
    opspec(ct).States(3).Known = true;
    opspec(ct).States(3).x = angles(ct);
    opspec(ct).States(4).SteadyState = false;
end
%
% Compute operating points using these specifications.
options = findopOptions('DisplayReport',false);
[op,opresult] = findop(mdlPlant,opspec,options);

%
% Obtain the linear plant model at the specified operating points.
plants = linearize(mdlPlant,op,io);
%
bdclose(mdlPlant)

%% 给每个工况点设计一个 MPC 控制器
% status = mpcverbosity('off');
for ct=1:length(angles)
    % Get a single plant model and set signals names.
    plant = plants(:,:,ct);
    plant.InputName = {'dF'; 'F'};
    plant.OutputName = {'x'; 'theta'};

    % 设计 MPC 控制器
    plant = setmpcsignals(plant,'ud',1,'mv',2);
    Ts = 0.01; % 采样时间
    PredictionHorizon = 50; %预测时域
    ControlHorizon = 5; % 控制时域
    mpcobj = mpc(plant,Ts,PredictionHorizon,ControlHorizon);

    % Specify nominal input and output values based on the operating point.
    mpcobj.Model.Nominal.Y = [0;opresult(ct).States(3).x];
    mpcobj.Model.Nominal.X = [0;0;opresult(ct).States(3).x;0];
    mpcobj.Model.Nominal.DX = [0;opresult(ct).States(2).dx;0;opresult(ct).States(4).dx];

    % 力约束
    mpcobj.MV.Min = -200;
    mpcobj.MV.Max = 200;
    % ScaleFactor
    mpcobj.MV.ScaleFactor = 100;
    % 权重
    mpcobj.Weights.MVRate = 1;
    mpcobj.Weights.OV = [1.2 1];

    % 输入噪音
    disturbance_model = getindist(mpcobj);
    setindist(mpcobj,'model',disturbance_model*10);
    % 观测噪音
    disturbance_model = getoutdist(mpcobj);
    setoutdist(mpcobj,'model',disturbance_model*10);

    % 保存结果 mpc1,mpc2,mpc3...
    assignin('base',['mpc' num2str(ct)],mpcobj);
end