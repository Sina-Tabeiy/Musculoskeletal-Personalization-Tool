%-------------------------------------------------------------------------%
%    Copyright (c) 2021 Modenese L.                                       %
%    Author:   Luca Modenese,  2021; Modified by Sina Tabeiy, 2022        %
%    email:    l.modenese@imperial.ac.uk  //  stabeiy@gmail.com           %
% ----------------------------------------------------------------------- %
% function to deform the vtp bone geometries according to the specified
% torsional profile.
function [osimModel, new_points] = applyTorsionToVTPBoneGeom(osimModel, CORA , bone_to_deform, deformity_angle, torsion_doc_string, OSGeometry_folder)

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
        new_points(j,:)= points(n,:); %#ok<AGROW> 
        new_normals(j,:)= normals(n,:); %#ok<AGROW> 
        j = j + 1;
        end
    end
    
    % writes the deformed geometry
    disp('   - writing deformed VTP file');
    writeDeformedVTPGeometry(vtp_file, new_normals, new_points, torsion_doc_string);

end

% assign to model
osimModel =  assignDeformedVTPFileNamesToBody(osimModel, bone_to_deform, torsion_doc_string);
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

% this function updates the normals and points of a vtp file with some new
% normals and points given as input
% writes the deformed file in the same folder where the original vtp is
% locates
function writeDeformedVTPGeometry(vtp_file, new_normals, new_points, torsion_doc_string)

% Opens original VTP file
fid = fopen(vtp_file);
if fid == -1;      error('Vtp file not loaded');           end

[path, name, ~] = fileparts(vtp_file);
% Opens a VTP files to write the deformed geometry
file_name = [name,'_', torsion_doc_string,'.vtp'];
deformed_vtp_file = fullfile(path,file_name);
fidDef = fopen(deformed_vtp_file,'w+');

% Throws exception if there are problems with the files
if fidDef == -1;      error('Vtp deformed file not loaded');  end

% initializations
n_norm = 1;
n_p = 1;

% goes though the entire file
while ~feof(fid) 
    % initialization of the writing format
    format = '';
    % gets a line
    tline = fgetl(fid);
    % scans the file until it finds 3 double in a row. They can be a
    % normal, a point or topology.
    if ~ isempty(sscanf(tline,'%f %f %f\n'))
        % gets the floating numbers
        data = sscanf(tline,'%f %f %f\n');
        % They are normals if the norm is numerically zero and their index
        % is minor/equal to the first dimension of the given matrix
        if (abs(norm(data)-1)<0.000001) && n_norm<=length(new_normals)
            % substitution of the original values with the deformed one
            tline = new_normals(n_norm,:);
            % appropriate format for the normals
            format = '\t\t\t%-6.6f %-6.6f %-6.6f\r\n';
            n_norm = n_norm+1;
        end
        % if the vector has norm>1 and given points are not finished then
        % treat the vector as a point
        if (n_p<=length(new_points)) && (abs(norm(data)-1)>=0.000001) && (n_p < n_norm)
            % update the point coordinates
            tline = new_points(n_p,:);
            % same format as before
            format = '\t\t\t%-6.6f %-6.6f %-6.6f\r\n';
            n_p = n_p+1;
        end
        % if not point or normal the vector value just goes through and
        % format is not updated
    end
    if strcmp(format,'')
        % format used to copy the line of the original file as it is.
        format = '%s\r\n';
    end
    % just write the line (updated or not)
    fprintf(fidDef,format,tline);
end

% close files
fclose all;

% informs the user of the new geometry
disp(['   - saved as ''', deformed_vtp_file,  ''' in geometry folder.']);

end

function [aOsimModel,newVTPNames] =  assignDeformedVTPFileNamesToBody(aOsimModel, bone_to_deform, torsion_doc_string)

import org.opensim.modeling.*

% check if body is included in the model
if aOsimModel.getBodySet().getIndex(bone_to_deform)<0
    error('The specified segment is not included in the OpenSim model')
end

% OpenSim 3.3
if getOpenSimVersion()<4.0
    % gets GeometrySet, where the display properties are located
    bodyGeometrySet = aOsimModel.getBodySet().get(bone_to_deform).getDisplayer().getGeometrySet();
    
    % Gets the element of the geometrySet
    N_vtp = bodyGeometrySet.getSize();
    
    % Loops and updates the names of the VTP geometry files
    for n_vtp = 0:N_vtp-1
        cur_geom = bodyGeometrySet.get(n_vtp);
        % original name
        origName = char(cur_geom.getGeometryFile());
        % update the vtp file name
        updVTPName = [origName(1:end-4),torsion_doc_string,'.vtp'];
        % sets new file name for Geometry
        cur_geom.setGeometryFile(updVTPName);
        % stores name
        newVTPNames(n_vtp+1) = {updVTPName}; %#ok<AGROW> 
        % clear
        clear origName  newName
    end
    
else
    body = aOsimModel.getBodySet().get(bone_to_deform);
    % get number of meshes
    N_vtp = body.getPropertyByName('attached_geometry').size();
    
    % Loops and updates the names of the VTP geometry files
    for n_vtp = 0:N_vtp-1
        
        cur_geom = body.get_attached_geometry(n_vtp);
        
        % transform to Mesh
        currentMesh = Mesh.safeDownCast(cur_geom);
        
        % original name
        origName = char(currentMesh.get_mesh_file());

        % update the vtp file name
        %updVTPName = [bone_to_deform,'_', torsion_doc_string,'.vtp'];
        updVTPName = [origName(1:end-4),'_', torsion_doc_string,'.vtp'];
        
        % sets new file name for Geometry
        currentMesh.set_mesh_file(updVTPName);
        
        % stores name
        newVTPNames(n_vtp+1) = {updVTPName}; %#ok<AGROW> 
        
        % clear
        clear origName  newName
    end

end

end