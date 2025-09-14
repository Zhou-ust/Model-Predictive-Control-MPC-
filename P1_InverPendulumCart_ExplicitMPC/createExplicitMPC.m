%mpcobj = mpc1;

range = generateExplicitRange(mpcobj);
range.State.Min(:) = -20; % largest range comes from cart position x
range.State.Max(:) = 20;
range.Reference.Min = -20; % largest range comes from cart position x
range.Reference.Max = 20;
range.ManipulatedVariable.Min = -200;
range.ManipulatedVariable.Max = 200;

mpcobjExplicit = generateExplicitMPC(mpcobj,range);