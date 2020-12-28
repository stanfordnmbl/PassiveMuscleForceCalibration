function [c ceq] = NonlinConstraints(coeffs,params) ;

c = 0 ;

passiveVals = params.passiveValsInitial ;
passiveVals(:,params.paramsToModify) = coeffs;

