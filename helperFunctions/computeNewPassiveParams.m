function passiveVals = computeNewPassiveParams(variableParameters,originalParameters,columnsToModify,rowsToModify) ;

passiveVals = originalParameters ;
passiveVals(rowsToModify,columnsToModify) = variableParameters ; % These are the design variables
passiveVals(rowsToModify,2) = max([passiveVals(rowsToModify,2),passiveVals(rowsToModify,1)+.4],[],2) ; % Make sure starting strain is .4 less than ending
passiveVals(rowsToModify,4) = 2./(passiveVals(rowsToModify,2)-passiveVals(rowsToModify,1)) ; % Making sure ending strain meets requirements of > 1/(strain_oneNormForce-strain_0force) ;
