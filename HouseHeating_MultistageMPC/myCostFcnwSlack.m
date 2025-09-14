function J = myCostFcnwSlack(stage, x, u, dmv, e, price)

    J = 1e4*price*u(2) + 1e5*(e(1) + e(2)) + 0.05*(dmv^2);

end