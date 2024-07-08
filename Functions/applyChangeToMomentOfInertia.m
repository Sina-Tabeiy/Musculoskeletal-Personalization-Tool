%-------------------------------------------------------------------------%
%    Copyright (c) 2021 Modenese L.                                       %
%    Author:   Luca Modenese,  2021; Modified by Sina Tabeiy, 2022        %
%    email:    l.modenese@imperial.ac.uk  //  stabeiy@gmail.com           %
% ----------------------------------------------------------------------- %
% function to deform the vtp bone geometries according to the specified
% torsional profile.
function [osimModel, new_points] = applyChangeToMomentOfInertia(osimModel, CORA , bone_to_deform, deformity_angle, torsion_doc_string, OSGeometry_folder)

% assign by default OpenSim 3.1 folder
if nargin<6
    [~, osim_version_string] = getOpenSimVersion();
    OSGeometry_folder = ['C:\OpenSim ',osim_version_string,'\Geometry'];
end

disp('--------------------------');
disp(' ADJUSTING VTP GEOMETRIES ');
disp('--------------------------');
% get VTP files for bone of interest
%disp(['Geometry folder: ', OSGeometry_folder])
disp(['VTP files attached to ', bone_to_deform,':']);
vtpNameSet = getBodyVTPFileNames(osimModel, bone_to_deform);

for n_vtp = 1:size(vtpNameSet,2)
    
    % current geometry
    curr_vtp = vtpNameSet{n_vtp};
    
    % print vtp file
    disp(['*', curr_vtp]);

    % vtp files with path
    vtp_file = fullfile(OSGeometry_folder,curr_vtp);
    
    % reads the original vtp file
    disp('   - reading VTP file');
    [normals, points] = readVTPfile_v2(vtp_file);
    
    j = 1;
    CORA_mat = [0 CORA(2) 0];
    for n = 1:size(points,1)
       
        % compute torsion matrix
        if  points(n,2) < CORA(2)
        TorsRotMat = rotx(deformity_angle);
        % New points and axis
        points(n,:) = points(n,:) - CORA_mat;
        new_points(j,:) = (TorsRotMat*points(n,:)')' + CORA_mat; %#ok<AGROW> 
        new_normals(j,:) = (TorsRotMat*normals(n,:)')'; %#ok<AGROW> 
        j = j + 1;
        else
        new_points(j,:) = points(n,:); %#ok<AGROW> 
        new_normals(j,:) = normals(n,:); %#ok<AGROW> 
        j = j + 1;
        end
    end
 
end

end

function vtpNameSet = getBodyVTPFileNames(aOsimModel, aBodyName)

import org.opensim.modeling.*

% check if body is included in the model
if aOsimModel.getBodySet().getIndex(aBodyName)<0
    error('The specified segment is not included in the OpenSim model')
end

% OpenSim 3.3
if getOpenSimVersion()<4.0
    % gets GeometrySet, where the display properties are located
    bodyGeometrySet = aOsimModel.getBodySet().get(aBodyName).getDisplayer().getGeometrySet();
    % Gets the element of the geometrySet
    N_vtp = bodyGeometrySet.getSize();
    % Loops and saved the names of the VTP geometry files
    for n_vtp = 0:N_vtp-1
        cur_geom = bodyGeometrySet.get(n_vtp);
        vtpNameSet(n_vtp+1) = {char(cur_geom.getGeometryFile())}; %#ok<AGROW>
    end
    
else
    body = aOsimModel.getBodySet().get(aBodyName);
    % get number of meshes
    N_vtp = body.getPropertyByName('attached_geometry').size();
    for n_vtp = 0:N_vtp-1 % here no -1
        cur_geom = body.get_attached_geometry(n_vtp);
        % transform to Mesh
        currentMesh = Mesh.safeDownCast(cur_geom);
        % extract file
        vtpNameSet(n_vtp+1) = {char(currentMesh.get_mesh_file())}; %#ok<AGROW>
    end
end

end

% function to read normals and points contained in a vtp file.
% points and normal define the geometry of the bone (assuming that topology 
% doesn't change).
function [normals, points] = readVTPfile_v2(vtp_file)

% open vtp file
fid = fopen(vtp_file);

% check on the file: is it open?
if fid == -1;        error('VTPfile not loaded');    end

% initialization
n_norm = 1;
n_p=1;

while ~feof(fid) % goes though the entire file
    % gets a line
    tline = fgetl(fid);
    % checks if there are three floating nr in a row (can be a normal or a
    % point)
    if ~isempty(sscanf(tline,'%f %f %f\n'))
        % it gets the vector as double
        data = sscanf(tline,'%f %f %f\n');
        
        % it si a normal if norm is close to one.
        % the code assumes that normals are listed before points.
        if (abs(norm(data)-1)<0.000001) && n_p==1
            % stores the normal
            normals(n_norm,:) = data; %#ok<AGROW> 
            n_norm = n_norm+1;
            % otherwise is a point
        elseif n_p<n_norm && (abs(norm(data))<2)
            % stores the point
            points(n_p,:) = data; %#ok<AGROW> 
            n_p = n_p+1;
        else 
            % if not point or normal than exit for loop, This avoids to go
            % through topology.
            break
        end
    end
end
% close files
fclose all;
end

