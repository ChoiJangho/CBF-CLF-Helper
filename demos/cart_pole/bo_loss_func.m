function loss = bo_loss_func(y, x)
    loss1 = -y^2;
    loss2 = -(x(3)-pi)^2;
    loss3 = -x(4)^2;
    loss4 = -x(2)^2;
    % fprintf("Loss: y_error: %.4f \t theta: %.4f \t dtheta: %.4f \t dx: %.4f \n", [loss1, loss2, loss3, loss4]);
    % c1 = 1000;
    c1 = 0;
    % c2 = 1000;
    % c2 = 200;
    c2 = 100;
    c3 = 1;
    % c4 = 100;
    c4 = 0;
    fprintf("Loss: y_error: %.4f \t theta: %.4f \t dtheta: %.4f \t dx: %.4f \n", [c1 * loss1, c2 * loss2, c3 *  loss3, c4 * loss4]);
    loss = c1 * loss1 + c2 * loss2 + c3 * loss3 + c4 * loss4;
    
end