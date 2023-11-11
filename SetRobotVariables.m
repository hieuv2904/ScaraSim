function SetRobotVariables()
    global a d alpha theta working_limit pos orien;
    % Set workspace values
    a = [0.2; 0.3; 0; 0];
    d = [0.179; 0; 0; 0];
    alpha = [0.00; 0.00; 0.00; 180];
    theta = [0.00; 90.0; 0.00; -180];
    working_limit = [125; 145; 0.15];
    [pos, orien] = ForwardKinematic(alpha, a, theta, d, working_limit);
    
    % Assign values to the MATLAB workspace
    assignin('base', 'a', a);
    assignin('base', 'd', d);
    assignin('base', 'alpha', alpha);
    assignin('base', 'theta', theta);
    assignin('base', 'working_limit', working_limit);
    assignin('base', 'pos', pos);
    assignin('base', 'orien', orien);
end