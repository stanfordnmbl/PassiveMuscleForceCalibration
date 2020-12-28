function costVal = CostPassiveCalibration(params,coeffs0)

passiveVals = computeNewPassiveParams(coeffs0,params.passiveValsInitial,params.paramsToModify,params.muscleInds2Modify_MLinds) ;

if params.ignoreTendonCompliance 
    modelPassiveMoments = computeModelPassiveMoments(params,passiveVals) ;
else 
    modelPassiveMoments = computeModelPassiveMoments_theSlowWay(params,passiveVals) ;
end

inds_mom = find(params.silderMoments) ;

torqueError = modelPassiveMoments(inds_mom) - params.silderMoments(inds_mom) ;
regularizationDif = reshape((coeffs0-params.coeffsModelInitial(params.muscleInds2Modify_MLinds,:)),1,[]) ;

costVal = params.w_regularization * mean(regularizationDif.^2) / mean(params.coeffsModelInitial(:).^2) + ...
          params.w_torqueError * mean(torqueError(:).^2) / mean(params.silderMoments(inds_mom).^2) ;
