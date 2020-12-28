function modelPassiveMoments = computeModelPassiveMoments(params,passiveVals) ;

% Unpack params
rMuscleInds = params.rMuscleInds ;
silderMoments = params.silderMoments ;
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

% % TODO: make this updatable based on properties
% musc = org.opensim.modeling.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(i)) ;
% fiberForceCurve = musc.getFiberForceLengthCurve() ;
           
for pose = 1:nPoses
    % Compute Passive Forces
    for musc = 1:length(rMuscleInds)
        if pose == 1 %Update the fiberForceCurveParameters on the first pose
            try
             fiberForceCurves{musc}.setOptionalProperties(passiveVals(musc,3),passiveVals(musc,4),passiveVals(musc,5)) ;
             fiberForceCurves{musc}.setCurveStrains(passiveVals(musc,1),passiveVals(musc,2)) ;
            catch
             fiberForceCurves{musc}.setCurveStrains(passiveVals(musc,1),passiveVals(musc,2)) ;
             fiberForceCurves{musc}.setOptionalProperties(passiveVals(musc,3),passiveVals(musc,4),passiveVals(musc,5)) ;
            end
        end
        
        % This is slow in optimizer - could make a lookup table from
        % passive curve
        passiveForces(1,musc) = fiberForceCurves{musc}.calcValue(normFiberLengths(musc,pose)) * normFiberForceMultiplier(musc,pose) ;
    end
    modelPassiveMoments(pose,:) = passiveForces * momentArms(:,:,pose)  ;
end

