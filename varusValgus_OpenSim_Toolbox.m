%-------------------------------------------------------------------------%
%    Copyright (c) 2022 Sina Tabeiy                                       %
%    Author:   Sina Tabeiy,  2022                                         %
%    email:    stabeiy@gmail.com                                          %
% ----------------------------------------------------------------------- %
%% ----------- Start -----------
clear;
clc;
addpath('./Functions')
import org.opensim.modeling.*

%% ----------- Select Model -----------
altered_models_folder = '.';
OpenSim_Geometry_folder = './Geometry';
openSimModel = uigetfile('*.osim*', 'Select OpenSim Model');
osimModel = Model(openSimModel);

%% ----------- Get Bone Information -----------
[CORA, bone_to_deform, bone_side] = getCora(osimModel);

%% ----------- Get Deformity -----------
list = {'mMLDFA' , 'mMPTA'};
[index] = listdlg('PromptString', {'Select the type of deformity:'}, 'SelectionMode', 'single','ListString', list);
Deg = input('Please enter deviation angle: ');
if bone_side == 'r'
    deformity_angle = 90 - Deg;
else
    deformity_angle = Deg - 90;
end
torsion_doc_string = [char(list(index)), num2str(Deg), 'Deg'];

%% ----------- Calculate -----------
% suffix used for saving geometries
[osimModel, new_points] = applyTorsionToVTPBoneGeom(osimModel, CORA , bone_to_deform, deformity_angle, torsion_doc_string, OpenSim_Geometry_folder);

% Rotate joints
osimModel = applyTorsionToJoints(osimModel, CORA, bone_to_deform, bone_side, deformity_angle);
   
% deforming muscle attachments
osimModel = applyTorsionToMuscleAttachments(osimModel,CORA, bone_to_deform, deformity_angle);

% if there are markers rotate them
osimModel = applyTorsionToMarkers(osimModel, bone_to_deform, CORA, deformity_angle);

%% ----------- Save Model -----------
% save output model
[~, name,ext] = fileparts(openSimModel);
deformed_model_name = [name,'_', torsion_doc_string,ext];
output_model_path = fullfile(altered_models_folder, deformed_model_name);
osimModel.setName(deformed_model_name);

% save model
saveDeformedModel(osimModel, output_model_path);