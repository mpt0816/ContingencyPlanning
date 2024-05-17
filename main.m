clc; clear all; close all;
%% 添加路径
current_folder = pwd;
addpath(genpath(current_folder));

%% planner Init
config = Configs();
planner = Planner();
speed = planner.Plan(config);
obstacle = planner.CutInObstacleST();

%% Plot
PlotResults(speed, obstacle);






