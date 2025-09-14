function J = myCostFcnwSlackStageFirst(stage, x, u, dmv, price)

    J = 1e4*price*u(2) + 0.05*(dmv^2);

end