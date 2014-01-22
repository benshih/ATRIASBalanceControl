% Benjamin Shih
% 10/22/2013
% Solving the simple Point Mass Model system dynamics symbolically and
% applying the controller torques to the balancing model.

function PMMFeedbackLinearizationUnderactuated()
    clear
    sysT = SystemDynamics();
    % Obtain angle/angular velocity/angular acceleration data from simulation
    EvaluateTorques(sysT)
end


function sysT = SystemDynamics()
    % State space variables and their derivatives.
    syms thdd thd th p1dd p1d p1 p2dd p2d p2 ldd ld l

    % Model properties
    syms It Im m d 

    % Physical properties
    syms g

    % Body torques and forces
    syms Fext t1 t2

    % State space using gamma1, gamma2
    q = [thdd; 
         p1dd+thdd; 
         p2dd+thdd; 
         ldd];

    % Transformation between the two state spaces
    T = [1 0 0 0;
        -1 1 0 0;
        -1 0 1 0;
         0 0 0 1];

    % State space using phi1, phi2
    p = T*q;

    % M: dynamics matrix (state space coefficients)
    Mgamma = [m*d^2+It               m*l*d*cos(2*th+p1)  0   d*m*sin(-p1);
         m*l*d*cos(2*th+p1)     m*l^2               0   0;
         0                      0                   Im  0;
         m*d*sin(-p1)           0                   0   m];

    % b: dynamics matrix (constants)
    b = [m*d*(ld*(thd+p1d)*cos(2*th+p1) - ld*(thd+p1d)*cos(-p1) ...
            - l*(thd+p1d)^2*sin(2*th+p1) - g*sin(th));
         m*(2*(thd+p1d)*l*ld + ld*d*thd*cos(2*th+p1) - l*d*thd^2*sin(2*th+p1) ...
            + ld*d*thd*cos(-p1) + g*l*sin(th+p1));
         0;
         m*(d*thd*cos(-p1)*(-p1d) - cos(th+p1)*(d*(thd+p1d)*thd + g))];

    % Transform the M from state space q into state space p. (p is the one we
    % want to use because our forces are defined in terms of p.)
    Mphi = Mgamma*T^-1;

    % F: torque matrix
    F = [Fext*l*(cos(th+p1)-cos(th+p2));
         t1 + t2 + Fext*l*cos(th+p1);
         t2 - Fext*l*cos(th+p2);
         Fext*(sin(th+p1) - sin(th+p2))];

    % Assume for now (1/21) that Fext = 0 because that was just a test for
    % dynamics solving.
    F = [0; t1 + t2; t2; 0];


    % Calculate the system-parameter accelerations based on the system
    % dynamics (state space p = desAccel).
    desAccel = Mphi^-1*(F-b);

    % Symbolic variables for equating desired dynamics with system
    % dynamics.
    syms k1 k2 thdes p1des p2des ldes

    % Underactuated because I assume that there is no trunk torque (T_th = 0).
    % Also, as of 1/21, I assume that there is no prismatic leg movement (F_l =
    % 0). 
    T_th = 0;
    T_p1 = solve(desAccel(2) == k1*(p1des - p1) + k2*(-p1d), t1);
    T_p2 = solve(desAccel(3) == k3*(p2des - p2) + k4*(-p2d), t2);
    F_l = 0;

    % Desired dynamics. Gains are arbitrary at the moment.
    sysT = [T_th; T_p1; T_p2; F_l];

end

function EvaluateTorques(sysT)
    % Using 'From Workspace', I can import T_p1 and T_p2.
    % Using 'To Workspace', I can bring back the trunk-hip, leg1-hip, and
    % leg2-hip angles/angular velocities/angular accelerations.
%     T_th = sysT(1);
    T_p1 = sysT(2);
    T_p2 = sysT(3);
%     F_l = sysT(4);
    
    % Static system parameters
    % Assumptions: leg masses are negligible, leg rotational inertia are
    % negligible.
    l = 1; % leg length [m]
    m = 44; % mass of the torso [kg]
    It = 1.37; % rotational inertia about torso CoM [kg*m^2]
    Im = 
    d = 
    g = 9.81; % gravitational acceleration [m/s^2]
    
    % Desired
    p1des = 
    p2des = 
    
    % default units for angles: radians
    p1 = 
    p2 = 
    th = timeseries
    p1d =
    p2d = 
    thd = 
    % ld = 0 (currently don't have, wait until the prismatic is in
    % --are these timeseries gonna be valid for each time step? or do you
    % have to loop through the simulation
    
    % Gains (arbitrary)
    k1 = 
    k2 = 
    k3 = 
    k4 = 
    
    http://www.mathworks.com/help/simulink/slref/fromworkspace.html
    
    % Substitute and evaluate(can only subs one value at a time)
    blah = subs(
end