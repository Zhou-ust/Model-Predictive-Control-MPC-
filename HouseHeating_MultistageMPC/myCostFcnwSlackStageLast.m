function J = myCostFcnwSlackStageLast(stage, x, u, dmv, e, price)

    J = 1e5*(e(1) + e(2));

end