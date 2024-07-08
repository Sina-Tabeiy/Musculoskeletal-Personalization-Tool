%-------------------------------------------------------------------------%
%    Copyright (c) 2022 Sina Tabeiy                                       %
%    Author:   Sina Tabeiy,  2022                                         %
%    email:    stabeiy@gmail.com                                          %
% ----------------------------------------------------------------------- %
function osimModel = applyTorsionToJoints(osimModel, CORA, bone_to_deform, bone_side, deformity_angle)
import org.opensim.modeling.*

%% ----------- Get Joint ----------- 
jointNameSet = getDistalJointNames(osimModel, bone_to_deform);
    for nj = 1:length(jointNameSet)
        % Get current joint
        cur_joint_name = jointNameSet{nj};
        curDistJoint = osimModel.getJointSet.get(cur_joint_name);
        % Extract joint parameters
        orientation = Vec3(0); %#ok<NASGU> 
        location    = Vec3(0); %#ok<NASGU> 
        orientation = curDistJoint.get_frames(0).get_orientation();
        XYZ_orient_vec = [orientation.get(0), orientation.get(1), orientation.get(2)];
        curDistJoint_name = char(curDistJoint);
    
%% ----------- Adjust Joint ----------- 
        % Adjust Rajagopal model
        % Since the orientation of coordinate system in knee joint is
        % compeletly different from other joints, we need to adjust knee
        % joint differently in Rajagopal model.

        if (strcmp(curDistJoint_name(1:4),'walk'))
            % Extract joint params
            location = curDistJoint.get_frames(0).get_translation();
            XYZ_location_vec =  [location.get(0), location.get(1), location.get(2)];
            % Get spatialTransform
            jointOffset = [0 0 0];
                if strcmp(char(curDistJoint.getConcreteClassName()), 'CustomJoint')
                % offset from the spatial transform
                % this is in parent, which is the bone of interest
                jointOffset = computeSpatialTransformTranslations(osimModel, curDistJoint);
                end
            % if CustomJoint add the translation from the CustomJoint
            XYZ_location_torsion = XYZ_location_vec + jointOffset;
            bone_to_deform(end-1:end) = [];
            bone_to_deform_rajagopal = [bone_side,'_',bone_to_deform]; 
            new_Loc = [0 0 0];
            jointRotMat = orientation2MatRot(XYZ_orient_vec);
            joint_deviation = (deformity_angle * abs(CORA(2) - XYZ_location_torsion(2))) / abs(XYZ_location_torsion(2));
            newJointRotMat =  jointRotMat * rotz(joint_deviation);
            
            % --------- Adjust CPA
            %newJointRotMat =  jointRotMat * rotz(0);
            new_OrientationInPar  = computeXYZAngleSeq(newJointRotMat);
            newOrientationInParent = Vec3(new_OrientationInPar(1), new_OrientationInPar(2), new_OrientationInPar(3));
            curDistJoint.get_frames(0).set_orientation(newOrientationInParent);
            % New location of femur: 
                if strcmpi('fem',bone_to_deform_rajagopal(3:5))
                joint_deviation = (deformity_angle * abs(CORA(2) - XYZ_location_vec(2))) / abs(XYZ_location_vec(2));
                new_Loc =  (XYZ_location_torsion * rotx(joint_deviation)) - XYZ_location_torsion;    
                end
                %{
            jointRotMat = orientation2MatRot(XYZ_orient_vec);
            newJointRotMat =  jointRotMat * rotz(joint_deviation);
            new_OrientationInPar  = computeXYZAngleSeq(newJointRotMat);
            newOrientationInParent = Vec3(new_OrientationInPar(1), new_OrientationInPar(2), new_OrientationInPar(3));
            curDistJoint.get_frames(0).set_orientation(newOrientationInParent);
                %}
%%  ------ Adjust other joints in the models 
            else
            % Extract joint params
            location    = curDistJoint.get_frames(0).get_translation();
            XYZ_location_vec =  [location.get(0), location.get(1), location.get(2)];
            jointOffset = [0 0 0];
                if strcmp(char(curDistJoint.getConcreteClassName()), 'CustomJoint')
                % offset from the spatial transform
                % this is in parent, which is the bone of interest
                jointOffset = computeSpatialTransformTranslations(osimModel, curDistJoint);
                end
            % if CustomJoint add the translation from the CustomJoint
            XYZ_location_torsion = XYZ_location_vec + jointOffset;
            jointRotMat = orientation2MatRot(XYZ_orient_vec);
            joint_deviation = (deformity_angle * abs(CORA(2) - XYZ_location_torsion(2))) / abs(XYZ_location_torsion(2));
            newJointRotMat =  jointRotMat * rotx(joint_deviation);
            new_OrientationInPar  = computeXYZAngleSeq(newJointRotMat);
            newOrientationInParent = Vec3(new_OrientationInPar(1), new_OrientationInPar(2), new_OrientationInPar(3));
            curDistJoint.get_frames(0).set_orientation(newOrientationInParent);
    
            if strcmpi('fem',bone_to_deform(1:3))
                joint_deviation = (deformity_angle * abs(CORA(2) - XYZ_location_torsion(2))) / abs(XYZ_location_torsion(2));
                new_Loc =  (XYZ_location_torsion * rotx(joint_deviation)) - XYZ_location_torsion;
            end
    
            % To adjust 2392/2354
            if (strcmp(curDistJoint_name(1:4),'knee'))
                new_Loc = [0 0 0];
            end
    
            if strcmpi('tib',bone_to_deform(1:3))
                joint_deviation = (deformity_angle * abs(CORA(2) - XYZ_location_torsion(2))) / abs(XYZ_location_torsion(2));
                new_Loc =  (XYZ_location_torsion * rotx(joint_deviation)) - XYZ_location_torsion;
            end
        end

    % Save to model
    newLocationInParent = Vec3(new_Loc(1), new_Loc(2), new_Loc(3));
    curDistJoint.get_frames(1).set_translation(newLocationInParent);
    
%% ----------- Adjust Hip joint ----------- 
    % Determine and getting side of the hip joint
    hip = ['femur_',bone_side];
    [hipJoint] = getBodyJoint(osimModel, hip, 0);
    hipOrientation = Vec3(0); %#ok<NASGU> 
    hipOrientation = hipJoint.get_frames(1).get_orientation();
    hip_orientation_vec = [hipOrientation.get(0), hipOrientation.get(1), hipOrientation.get(2)];
    
    % Calculate theangulation for hip
    jointRotMat = orientation2MatRot(hip_orientation_vec);   
    %hipRotMat =  jointRotMat * rotx(deformity_angle);
    % ------ Adjust hip angle here for now:
    hipRotMat =  jointRotMat * rotx(0);
    new_hip_orientation  = computeXYZAngleSeq(hipRotMat);
    newHipOrientation = Vec3(new_hip_orientation(1), new_hip_orientation(2), new_hip_orientation(3));
    hipJoint.get_frames(1).set_orientation(newHipOrientation);

    end

end