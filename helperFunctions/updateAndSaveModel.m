function updateAndSaveModel(passiveVals,osimModel,updatedModelPath,muscles,rMuscleInds,muscNames)

fiberForceCurves = cell(muscles.getSize,1) ;
for i = 1:muscles.getSize
    thisMusc = org.opensim.modeling.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(i-1)) ;
    fiberForceCurves{i} = thisMusc.getFiberForceLengthCurve() ;
end

% finding left leg matching muscle indicies
muscleBaseNames = cellfun(@(x) x(1:end-2),muscNames,'uniformoutput',false) ;
muscleBaseNames_half = muscleBaseNames(rMuscleInds+1) ;
lMuscleInds = cellfun(@(x) strmatch([x '_l'],muscNames)-1,muscleBaseNames_half) ;

for musc = 1:length(rMuscleInds)
    try % need to try different orders, some of the parameters are dependent upon each other, so must be set in the correct order to avoid disallowed values
        fiberForceCurves{rMuscleInds(musc)+1}.setOptionalProperties(passiveVals(rMuscleInds(musc)+1,3),passiveVals(rMuscleInds(musc)+1,4),passiveVals(rMuscleInds(musc)+1,5)) ;
        fiberForceCurves{rMuscleInds(musc)+1}.setCurveStrains(passiveVals(rMuscleInds(musc)+1,1),passiveVals(rMuscleInds(musc)+1,2)) ;
    catch
        fiberForceCurves{rMuscleInds(musc)+1}.setCurveStrains(passiveVals(rMuscleInds(musc)+1,1),passiveVals(rMuscleInds(musc)+1,2)) ;
        fiberForceCurves{rMuscleInds(musc)+1}.setOptionalProperties(passiveVals(rMuscleInds(musc)+1,3),passiveVals(rMuscleInds(musc)+1,4),passiveVals(rMuscleInds(musc)+1,5)) ;
    end
    
    try
        fiberForceCurves{lMuscleInds(musc)+1}.setOptionalProperties(passiveVals(rMuscleInds(musc)+1,3),passiveVals(rMuscleInds(musc)+1,4),passiveVals(rMuscleInds(musc)+1,5)) ;
        fiberForceCurves{lMuscleInds(musc)+1}.setCurveStrains(passiveVals(rMuscleInds(musc)+1,1),passiveVals(rMuscleInds(musc)+1,2)) ;
    catch
        fiberForceCurves{lMuscleInds(musc)+1}.setCurveStrains(passiveVals(rMuscleInds(musc)+1,1),passiveVals(rMuscleInds(musc)+1,2)) ;
        fiberForceCurves{lMuscleInds(musc)+1}.setOptionalProperties(passiveVals(rMuscleInds(musc)+1,3),passiveVals(rMuscleInds(musc)+1,4),passiveVals(rMuscleInds(musc)+1,5)) ;
    end
end

state = osimModel.initSystem() ;

osimModel.print(updatedModelPath) ;
