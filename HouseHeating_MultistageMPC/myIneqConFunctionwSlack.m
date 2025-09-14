function C = myIneqConFunctionwSlack(stage, x, u, dmv, e, price)

    C = [-x+20-e(1); x-22-e(2)];

end