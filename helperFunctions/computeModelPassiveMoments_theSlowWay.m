function modelPassiveMoments = computeModelPassiveMoments_theSlowWay(params,passiveVals) ;

% Unpack params
osimModel = params.osimModel ; 
state = params.state ;
muscles = params.muscles ;
coords = params.coords ;
rMuscleInds = params.rMuscleInds ;
silderMoments = params.silderMoments ;
silderAngles = params.silderAngles ;
momentArms = params.momentArms ;
normFiberLengths = params.normFiberLengths ;
normFiberForceMultiplier = params.normFiberForceMultiplier ;
fiberForceCurves = params.fiberForceCurves ;
sagCoordNames = params.sagCoordNames ;


nSagCoords = length(sagCoordNames); 
nRMuscles = length(rMuscleInds) ;
nPoses = size(silderMoments,1) ;

passiveForces = zeros(1,nRMuscles) ;
modelPassiveMoments = zeros(nPoses,nSagCoords) ;

for pose = 1:nPoses
    if pose == 1 %Update the fiberForceCurveParameters on the first pose
        for musc = 1:length(rMuscleInds)
        try
         fiberForceCurves{musc}.setOptionalProperties(passiveVals(musc,3),passiveVals(musc,4),passiveVals(musc,5)) ;
         fiberForceCurves{musc}.setCurveStrains(passiveVals(musc,1),passiveVals(musc,2)) ;
        catch
         fiberForceCurves{musc}.setCurveStrains(passiveVals(musc,1),passiveVals(musc,2)) ;
         fiberForceCurves{musc}.setOptionalProperties(passiveVals(musc,3),passiveVals(musc,4),passiveVals(musc,5)) ;
        end
        end
        state = osimModel.initSystem ;
    end
        
    %Set coordinate for each pose
    for c = 1:nSagCoords
       if pose == 1 || silderAngles(pose,c) ~= silderAngles(pose-1,c)
        coords.get(char(sagCoordNames{c})).setValue(state,silderAngles(pose,c)) ;
       end
    end
    osimModel.equilibrateMuscles(state); 
    osimModel.realizeDynamics(state) ;
    % Get Passive Forces
    for musc = 1:length(rMuscleInds)
        passiveForces(1,musc) = muscles.get(rMuscleInds(musc)).getPassiveFiberForceAlongTendon(state) ;
    end
    modelPassiveMoments(pose,:) = passiveForces * momentArms(:,:,pose)  ;
end

