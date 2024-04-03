function param = plant_param()
    % dimensions
    param.xdim = 2;
    param.udim = 1;

    % parameters
    param.m = 1; % mass (kg)
    param.d = 1; % damping coefficient (N.s/m)
    param.k = 1; % spring constant (N/m)
end
