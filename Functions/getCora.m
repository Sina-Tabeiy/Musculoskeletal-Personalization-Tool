%-------------------------------------------------------------------------%
%    Copyright (c) 2022 Sina Tabeiy                                       %
%    Author:   Sina Tabeiy,  2022                                         %
%    email:    stabeiy@gmail.com                                          %
% ----------------------------------------------------------------------- %
function [ CORA, bone_to_deform, bone_side] = getCora (osimModel) 

import org.opensim.modeling.*;
state = osimModel.initSystem(); %#ok<NASGU> 

cora ='cora';
CORA = [];

% Getting model markers
markers = osimModel.getMarkerSet();
N_markers = markers.getSize();

for n_marker = 0:N_markers-1
   
    curr_marker = markers.get(n_marker);
        if strcmpi(char(curr_marker.getName()), cora)
        markerLocVec3 =  curr_marker.get_location();
        CORA = [markerLocVec3.get(0),markerLocVec3.get(1),markerLocVec3.get(2)];
        bone_to_deform = char(curr_marker.getParentFrame().getName());
        bone_side = bone_to_deform(end);
        end
   
end

if isempty(CORA)
    error('Please add the CORA marker to model''s markers in OpenSim GUI.')
end
    

end