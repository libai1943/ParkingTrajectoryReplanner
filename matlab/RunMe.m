% Chapter 7 of the CHINESE-LANGUAGE book
% 《非结构化场景自动驾驶轨迹规划技术》
% English title translation: Trajectory Planning Techniques for Autonomous
% Driving in Unstructured Environments (the book is written in Chinese).
%
% Please cite: Bai Li, Zhuyan Yin, Yakun Ouyang, Youmin Zhang, Xiang Zhong,
% and Shiqi Tang, "Online Trajectory Replanning for Sudden Environmental
% Changes During Automated Parking: A Parallel Stitching Method,"
% IEEE Transactions on Intelligent Vehicles, 7(3):748-757, 2022.
% DOI: 10.1109/TIV.2022.3156429.
%
% This release exposes the parallel-stitching architecture. Parking, LIOM,
% hybrid A*, geometry and rendering are encapsulated in protected P-code.
% Cases are consolidated in ../data/cases_data.mat. Noncommercial use only:
% see ../LICENSE (PolyForm Noncommercial 1.0.0).
% Windows MATLAB R2021b+, Navigation + Image Processing Toolboxes; AMPL/IPOPT.
% This desktop demo computes all 5x6 combinations with independent processes.
% enforce_deadline=true rejects results completed after the 1.2 s budget;
% MATLAB's synchronous evasive planner itself is not interrupted mid-solve.
% Default demonstration mode is not a real-time performance claim.
clear; close all; clc;
case_id = 1; % 1, 14, 20, 36, 39, 96, 100, 108
options = struct('workers',4,'enforce_deadline',false,'make_video',false);
result = RunParallelStitching(case_id,options);
