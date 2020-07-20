% given are the functions 
%   r_BF_inB(alpha,beta,gamma) and
%   J_BF_inB(alpha,beta,gamma) 
% for the foot positon respectively Jacobian

r_BF_inB = @(alpha,beta,gamma)[...
    -sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
 
J_BF_inB = @(alpha,beta,gamma)[...
                                              0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
 cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
 sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];
 
% write an algorithm for the inverse kinematics problem to
% find the generalized coordinates q that gives the endeffector position rGoal =
% [0.2,0.5,-2]' and store it in qGoal
q0 = pi/180*([0,-30,60])';
rGoal = [0.2,0.5,-2]';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
q = q0;
r0 = r_BF_inB(q(1), q(2), q(3));
rn = r0;
counter = 0;

while norm(rn - rGoal) > 1e-8 
    rn = r_BF_inB(q(1), q(2), q(3));
    q = q + J_BF_inB(q(1), q(2), q(3)) \ (rGoal-rn);
    counter = counter + 1;
end

error = norm((rn-rGoal)/rGoal)*100;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
qGoal = double(q); 
disp(qGoal);
disp('Iterations: ' + string(counter));
disp('Error: ' + string(error));