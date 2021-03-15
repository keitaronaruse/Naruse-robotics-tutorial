%   fk_3Link_Planar.m
%       Forward kinematics of a planr 3-link robot arm
%       Author: Keitaro Naruse
%       Date: 2021-03-15
%       Licensing: MIT Licenseq 

% Joint angles
q = [0.1, 0.4, 0.9];
% Forward kinematics calculation
p = fk(q);

% Plot poses
% Open a new plot window named Figure 1
figure(1);
% p(:,1): A vector of X positions
% p(:,2): A vector of Y positions
plot(p(:,1), p(:,2));
% Set plot range
xlim([-3, 3]); ylim([-3, 3]);
% Set aspect ratio between x, y, and z
pbaspect([1, 1, 1]);
% Set grid on plot
grid on;

% Definition of forward kinematics function
function p = fk(q)
% fk forward kinematics of 3-link planar robot arm
%   Argument: q, Join angle vector q = [q1, q2, q3]
%   Return: p, Pose vector p = [p1; p2; p3; p4]
%       pi = [xi, yi, qi]
%   p = [[x1, y1, q1]
%        [x2, y2, q2]
%        [x3, y3, q3]
%        [x4, y4, q4]]

% Robot arm parameters
L1 = 1.0; L2 = 1.0; L3 = 1.0;

% Pose calculation
% pi = [xi, yi, qi]
% A pose of the first joint = the base
p1 = [0,0,0];
% A pose of the second joint = the end of the first link 
p2 = p1 + [L1 * cos(q(1)), L1* sin(q(1)), q(1)];
% A pose of the third joint = the end of the second link 
p3 = p2 + [L2 * cos(q(1)+q(2)), L2* sin(q(1)+q(2)), q(2)];
% A pose of the hand tip = the end of the third link 
p4 = p3 + [L3 * cos(q(1)+q(2)+q(3)), L3* sin(q(1)+q(2)+q(3)), q(3)];

% Set the four poses in a pose vector p 
p = [p1; p2; p3; p4];
end
