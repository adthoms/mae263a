clc, clear, format short, format compact, close all

%---Input

% declare symbolic vars
syms a1 a2 a3 a4 a5 real
syms theta1 theta2 theta3 theta4 real

% declare DH parameter table
alpha = [0 sym(pi/2) 0 0 0]';
a = [0 0 a2 a3 a4]';
d = [0 0 0 0 0]';
theta = [theta1 theta2+sym(pi/2) theta3-sym(pi/2) theta4 0]';

% workspace limits and dimensions
n = 11; % discretization 
joint1_limit = linspace(-pi/2, pi/2, n);
joint2_limit = linspace(-pi/2, pi/2, n);
joint3_limit = linspace(-pi/2, pi/2, n);
joint4_limit = linspace(-pi/2, pi/2, n);
a2_val = 0.139;
a3_val = 0.140;
a4_val = 0.173;

%---Calculations

% Forward Kinematics
expand_transform = true; 
[OE_T, T] = SolveForwardKinematics(a,alpha,d,theta,expand_transform);
OE_T_function = matlabFunction(OE_T);

% Workspace Analysis

% init
P = nan(n^4,3);
counter = 1;

% loop through all possible discretized states
for theta_1_val = joint1_limit
  for theta_2_val = joint2_limit
    for theta_3_val = joint3_limit
      for theta_4_val = joint4_limit

        % calculate EE pose
        OE_T_val = OE_T_function(a2_val, a3_val, a4_val,...
                                 theta_1_val, theta_2_val, ...
                                 theta_3_val, theta_4_val);

        % get position
        [~, O_P_EORG] = GetPositionandRotationFromTranform(OE_T_val);
        P(counter,:) = O_P_EORG';

        % increment
        counter = counter + 1;
      end
    end
  end
end

plotWorkspace(P)

%% Helper Functions

% --- Plotting

% plot workspace from 3D, xy, xz perspectives
function plotWorkspace(P)

  % robot urdf file
  urdf_file = './urdf/doodlebot.urdf';

  % workspace figure files
  fig1_file = './img/workspace_3d';
  fig2_file = './img/workspace_xy';
  fig3_file = './img/workspace_xz';

  % Load the URDF file
  robot = importrobot(urdf_file);

  % Calculate the 3D convex hull (Figure 1)
  k = convhull(P);

  % % slice xy plane (Figure 2)
  P_xy_slice = [P(:,1), P(:,2)];
  k_xy_slice = convhull(P_xy_slice);

  % slice xz plane (Figure 3)
  P_xz_slice = [P(:,1), P(:,3)];
  k_xz_slice = convhull(P_xz_slice);

  % calculate axis limits
  x_min = min(P(:,1)); x_max = max(P(:,1));
  y_min = min(P(:,2)); y_max = max(P(:,2));
  z_min = min(P(:,3)); z_max = max(P(:,3));

  % --- Figure 1

  % define
  figure(1);
  axesHandle = axes;
  hold(axesHandle, 'on');

  % plot
  hold on;
  show(robot, 'Visuals', 'on', 'Parent', axesHandle);
  trisurf(k, P(:, 1), P(:, 2), P(:, 3), ...
    'Facecolor', 'cyan', 'FaceAlpha', 0.3);

  % properties
  xlim([x_min x_max])
  ylim([y_min y_max])
  zlim([z_min z_max])
  xlabel('X-axis');
  ylabel('Y-axis');
  zlabel('Z-axis');
  box on;
  axis equal;
  view(3)
  hold(axesHandle, 'off');

  printPlot(fig1_file);

  % --- Figure 2

  % plot Xy plane
  figure(2);
  axesHandle = axes;
  hold(axesHandle, 'on');

  % plot
  hold on;
  show(robot, 'Visuals', 'on', 'Parent', axesHandle);
  plot3(P_xy_slice(k_xy_slice,1), ...
    P_xy_slice(k_xy_slice, 2), ...
    zeros(length(k_xy_slice)),'b')

  % properties
  xlim([x_min x_max])
  ylim([y_min y_max])
  zlim([z_min z_max])
  xlabel('X-axis');
  ylabel('Y-axis');
  zlabel('Z-axis');
  box on;
  axis equal;
  view(0, 90);
  hold(axesHandle, 'off');

  printPlot(fig2_file);

  % --- Figure 3

  % plot XZ plane
  figure(3);
  axesHandle = axes;
  hold(axesHandle, 'on');

  % plot
  hold on;
  show(robot, 'Visuals', 'on', 'Parent', axesHandle);
  plot3(P_xz_slice(k_xz_slice,1), ...
    zeros(length(k_xz_slice)), ...
    P_xz_slice(k_xz_slice, 2),'b')

  % properties
  xlim([x_min x_max])
  ylim([y_min y_max])
  zlim([z_min z_max])
  xlabel('X-axis');
  ylabel('Y-axis');
  zlabel('Z-axis');
  box on;
  axis equal;
  view(0, 0);
  hold(axesHandle, 'off');

  printPlot(fig3_file);
end

% save figure file
function printPlot(saveFile)
  set(gca,'fontsize',7)
  set(gcf,'units','inch','position',[0,0,3,2])
  print(gcf,saveFile,'-r600','-dpng');
  close(gcf); % close after saving
end

%--- Homogeneous transformations

% Rotation matrices
function R = Rx(gamma)
  R = [...
  1 0            0
  0 cosd(gamma) -sind(gamma)
  0 sind(gamma)  cosd(gamma)
  ];
end

function R = Ry(beta)
  R = [...
  cosd(beta)  0 sind(beta)
  0           1 0
 -sind(beta)  0 cosd(beta)
  ];
end

function R = Rz(alpha)
  R = [...
  cosd(alpha) -sind(alpha)  0
  sind(alpha)  cosd(alpha)  0
  0            0            1
  ];
end

% rotation matrix wrt fixed angles (about reference frame)
function AB_R = R_ZYX(alpha, beta, gamma)
  AB_R = Rx(gamma)*Ry(beta)*Rz(alpha);
end

% rotation matrix wrt euler angles (about moving frame)
%
% Remember, for problems involving homogeneous transformations we concider
% the rotations (in alpha, beta, gamma order) that would rotate the body
% frame to its current orientation wrt to the base frame. For example, see
% cheat sheet, example from midterm practice. This way is the most
% consistant method
%
function AB_R = R_ZpYpXp(alpha, beta, gamma)
  AB_R = Rz(alpha)*Ry(beta)*Rx(gamma);
end

% homogeneous transformation matrices
%
% Once we solve for the rotation matrix using function R_ZpYpXp, we
% would like to get A_P_BORG. Remember, this vector represents the position
% of body frame {b} wrt to reference frame {A} (i.e. the expression of [x y
% z] coordinates is in the {A} frame
%
function AB_T = CreateTranform(AB_R, A_P_BORG)
  if all(size(A_P_BORG) ~= [3,1])
    error("A_P_BORG must be a 3x1 column vector")
  end
  AB_T = eye(4);
  AB_T(1:3,1:3) = AB_R;
  AB_T(1:3,end) = A_P_BORG;
end

function [AB_R, A_P_BORG] = GetPositionandRotationFromTranform(AB_T)
  if all(size(AB_T) ~= [4,4])
    error("AB_T must be a 4x4 matrix")
  end
  AB_R = AB_T(1:3,1:3);
  A_P_BORG = AB_T(1:3,end);
end

%--- Forward Kinematics

% calculate link transformations
%
% calculate the homogeneous transformation matrix as a function of DH
% parameters for value i such that:
% 
% i-1 | i | alpha_{i-1} | a_{i-1} | d_i | theta_i
% -----------------------------------------------
%           alpha         a         d     theta
%
% see cheat sheet for variable description
%
function T = TF(a,alpha,d,theta)
  T = ...
  [
  cos(theta)            -sin(theta)             0           a
  sin(theta)*cos(alpha)  cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d
  sin(theta)*sin(alpha)  cos(theta)*sin(alpha)  cos(alpha)  cos(alpha)*d
  0                      0                      0           1
  ];
end

% output link transformations
%
% output linke transforms as a cell array, where entries 1...N represent
% the transforms 01_T ... (N-1)N_T. Also, output the final transformation
% relating the origin 0 to the end effector E: OE_T. This function also
% prints the content out to screen
%
function [OE_T, T] = SolveForwardKinematics(a,alpha,d,theta,expand_transform)

  % Initialize
  N = length(alpha); % number of {i} frames
  OE_T = eye(4); % transformation from base frame {0} to end effector {E}
  
  % calculate and display homogeneous transformation matrices
  T = cell(N,1);
  for i = 1:N
    T{i} = simplify(TF(a(i), alpha(i), d(i), theta(i))); % store in cell i
    fprintf("T_[%i][%i]\n", i-1, i)
    disp(T{i})
    OE_T = OE_T * T{i};
  end
  
  % display
  fprintf("T_[%i][%i]\n", 0, N)
  OE_T = simplify(OE_T);
  if (expand_transform)
    OE_T = simplify(expand(OE_T));
  end
  disp(OE_T)
end


