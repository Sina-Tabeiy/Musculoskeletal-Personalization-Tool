%-------------------------------------------------------------------------%
%    Copyright (c) 2021 Modenese L.                                       %
%    Author:   Luca Modenese,  2021; Modified by Sina Tabeiy, 2022        %
%    email:    l.modenese@imperial.ac.uk  //  stabeiy@gmail.com           %
% ----------------------------------------------------------------------- %
function osimModel = applyTorsionToMarkers(osimModel, aSegmentName, CORA, deformity_angle)

import org.opensim.modeling.*

% check if segment is included in the model
if osimModel.getBodySet().getIndex(aSegmentName)<0
    error('The specified segment is not included in the OpenSim model')
end

disp('-------------------');
disp(' ADJUSTING MARKERS ');
disp('-------------------');


% extracting MarkerSet
markers = osimModel.getMarkerSet();
N_markers = markers.getSize();

% loop through the muscles
for n_marker = 0:N_markers-1
    
    % current muscles
    curr_marker = markers.get(n_marker);
        
        % Body attached to each point of the PathPointSet
        if getOpenSimVersion()<4.0 %OpenSim 3.3
            attachBodyName = char(curr_marker.getBodyName());
        else %OpenSim 4.x
            attachBodyName = char(curr_marker.getParentFrame().getName());
        end
        
        if strcmp(attachBodyName, aSegmentName)
            
            disp(['processing ', char(curr_marker.getName())]);
            
            % point coordinates
            if getOpenSimVersion()<4.0 %OpenSim 3.3
                markerLocVec3 =  curr_marker.getOffset();
            else %OpenSim 4.x
                markerLocVec3 =  curr_marker.get_location();
            end

            % convert to Matlab var
            markerLocCoords = [markerLocVec3.get(0),markerLocVec3.get(1),markerLocVec3.get(2)];

            if  markerLocCoords(2) < CORA(2)

            deviation_angle = (abs(markerLocCoords(2) - CORA(2)) * deformity_angle) /abs(markerLocCoords(2));
            TorsRotMat = rotx(deviation_angle);
            new_markerLocCoords = (TorsRotMat*markerLocCoords')';
            
            else

            new_markerLocCoords = markerLocCoords;

            end

            % transform to MATLAB vector
            newOffset = Vec3(new_markerLocCoords(1), new_markerLocCoords(2), new_markerLocCoords(3));
            
            % setting the torsioned marker offset
            if getOpenSimVersion()<4.0 %OpenSim 3.3
                curr_marker.setOffset(newOffset);
            else %OpenSim 4.x
                curr_marker.set_location(newOffset);
            end
        end
end

