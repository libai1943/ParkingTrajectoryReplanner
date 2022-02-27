% ==============================================================================
% Matlab demo for "Online trajectory replanning for sudden environmental
% changes during automated parking: A parallel stitching method". accepted
% in IEEE Transactions on Intelligent Vehicles.
% ==============================================================================
%   Copyright (C) 2022 Bai Li
%   Users of this demo are suggested to cite the following article. Bai Li,
%   Zhuyan Yin, Yakun Ouyang, et al., “Online trajectory replanning for
%   sudden environmental changes during automated parking: A parallel
%   stitching method,” IEEE Transactions on Intelligent Vehicles, accepted
%   on Feb. 26, 2022.
%   Wait for some time before a video begins to be created. The video file
%   can be found in the current folder.
%   Do not ask to open the *.p files. This is a demo rather than a share of
%   completely open source codes.
%   License GNU General Public License v3.0
% ==============================================================================
clear; close all; clc;

global params_
case_id = 1; % Alternative setting choices include 14, 20, 36, 39, 96, 100, 108
LoadCase(case_id);
InitializeParams();
PlanOriginalTrajectory();
if (~AssignSuddenObstacle())
    error 'Fail to add a valid new obstacle.';
end
if (~PlanEvasiveTrajectory())
    error 'Fail to find an evasive trajectory. Should switch to fail-safe mode.';
end
IdentifyConnectiveTrajectory();

if (isempty(params_.traj_replanned))
    error 'Fail to find any a valid connective trajectory. Should switch to fail-safe mode.';
end
CreateVideo();
close all;