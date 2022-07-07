% This code uses passive joint moment data from Silder 2007 to tune the
% passive force curve parameters for a new model. This is currently set up
% for models that use the Millard2012EquillibriumMuscle model, and will
% have issues with Thelen2003Muscle model due to issues with the equilibrateMuscle
% functionality with the Thelen muscles. 

% Written by Scott Uhlrich. 12/28/2020. Stanford University. Please contact
% suhlrich@stanford.edu for questions, and cite our paper if you use this code in your work.

% Citation:
% Uhlrich, S.D., Jackson, R.W., Seth, A., Kolesar, J.A., Delp S.L. 
% Muscle coordination retraining  inspired by musculoskeletal simulations
% reduces knee contact force. Sci Rep 12, 9842 (2022). 
% https://doi.org/10.1038/s41598-022-13386-9

clc; clear all; close all; format compact
import org.opensim.modeling.*
myDir = pwd ;
addpath([myDir '\helperFunctions'])
ModelVisualizer.addDirToGeometrySearchPaths([myDir '\Geometry\']) ; % Add Rajagopal geometry to model search path to avoid 'couldn't find geometry' errors

%% Modifiable Parameters
modelPath = [myDir '\Rajagopal_modifiedAbductors.osim'] ; % full path to model
updatedModelName = 'Rajagopal_modifiedAbductors_passiveCal.osim' ; % file name for new model - will be in resultsFolder
silderDataPath = [myDir '\DigitizedPassiveMoments_Silder2007.mat'] ; % full path to Silder data
initialGuessFile = [myDir '\PreviousResults\RigidTendon\initialGuess.mat'] ; %Initial guess from previous optimization. Comment this out if you want to start from default values.
resultsFolder = [myDir '\Results\'] ; % The folder where you want your results and initial guess saved

silderCoordRanges = [-15 40 ; 0 75 ; -30 30] ; % 3x2 - coordinate upper and lower limits in degrees. 
                                               % These are the limits of the Silder data used for calibration. 
                                               % hip flexion, knee flexion, ankle dorsiflexion
silderDownsamplingFactor = 4 ; % Desired number of data points to skip in Silder digitized data. With 1, curves go by each degree.

paramsToModify = [1,2] ; % Muscle Parameters to modify [strainAtZeroForce,strainAtOneNormForce,stiffnessAtLowForce,stiffnessAtOneNormForce,Curviness] 
paramRange = [-.2 .2; .5 .9] ; % nParameters x 2

sagCoordNames = {'hip_flexion_r','knee_angle_r','ankle_angle_r'} ; % in the order hip, knee, ankle to match Silder data
calibrateAllMuscles = true ; % true if calibrating al muscles, false if using a subset defined in muscles2Calibrate cel
muscles2Calibrate = {'gasmed_r','gaslat_r'} ; % Which muscles to calibrate if calibrateAllMuscles is False 
ignoreTendonCompliance = true ; % True for rigid tendon assumption. It can be helpful to run with rigid tendons (fast) to generate 
                                % an initial guess for an optimization that uses compliant tendons (slow).
maxOptIterations = 2 ;  % make this small to prototype (3-4 iterations), likely requires several hundred iterations to converge

% Weights for terms in the cost function
params.w_regularization = 1 ; %cost function weight on multiplier terms being different from their original
params.w_torqueError = 2 ; % cost function weight on torque error

% For just plotting a previous solution. You need correct settings
% above (like tendon compliance). For the default example below,
% ignoreTendonComplaince = false
justPlotSolution = false ; % True if you want to skip optimization and just plot a saved solution
outputForPlotting = [myDir '\PreviousResults\CalibratedModelForPaper\passiveCalOutput.mat'] ; % Specify path to output file for plotting
initialGuessForPlotting = [myDir '\PreviousResults\CalibratedModelForPaper\initialGuess.mat'] ; % path to initialGuess.mat file for plotting
% % % End user inputs % % %

%% Get Passive Moments from Silder 2007
[silderMoments silderAngles momentLinearIndicies] = loadSilderPassiveMoments(silderDataPath,silderCoordRanges,silderDownsamplingFactor) ;
nPoses = size(silderAngles,1) ;

%% Load Model
osimModel = Model(modelPath);

%% Create results folder
if ~isfolder(resultsFolder) ; mkdir(resultsFolder) ; end

%% Get Coordinates
coords = osimModel.getCoordinateSet() ;
nCoords = coords.getSize() ;
nSagCoords = length(sagCoordNames) ;

%% Get Muscles
muscles = osimModel.updMuscles() ;
nMuscles = muscles.getSize() ;

for i = 1:nMuscles
    muscNames{i} = char(muscles.get(i-1).getName) ;
    if ignoreTendonCompliance
        muscles.get(i-1).set_ignore_tendon_compliance(1) ;    
    else
        muscles.get(i-1).set_ignore_tendon_compliance(0) ;
    end
end

rMuscleInds = find(cell2mat(strfind(muscNames,'_r')))-1 ;
nRMuscles = length(rMuscleInds) ;

%% Find indicies of muscles to modify
if calibrateAllMuscles
    muscleInds2Modify_MLinds = rMuscleInds + 1 ; 
else
    muscleInds2Modify_MLinds = zeros(length(muscles2Calibrate),1) ;
    for i = 1:length(muscles2Calibrate)
         muscleInds2Modify_MLinds(i) = strmatch(muscles2Calibrate(i),muscNames) ;
    end
end
nModifyMuscles = length(muscleInds2Modify_MLinds) ;

%% Initialize the state
state = osimModel.initSystem() ;

%% Make pre-computed matrices that are a function of pose
momentArms = zeros(nRMuscles,nSagCoords,nPoses) ;
maxIsoForces = zeros(nRMuscles,nPoses) ;
normFiberLengths = zeros(nRMuscles,nPoses) ;
cosAlphas = zeros(nRMuscles,nPoses) ;
fiberForceCurves = cell(nRMuscles,1) ;
passiveValsInitial = zeros(nRMuscles,5) ;

for pose = 1:nPoses
    %Set coordinate for each pose
    for c = 1:nSagCoords
        coords.get(char(sagCoordNames{c})).setValue(state,silderAngles(pose,c)) ;
    end  
    osimModel.equilibrateMuscles(state) ;

    % Loop through the coordinates and muscles to build the moment arm
    % matrix for this current pose
    for coord = 1:nSagCoords
        thisCoord = coords.get(char(sagCoordNames{coord})) ;
        for musc = 1:nRMuscles         
            momentArms(musc,coord,pose) = muscles.get(rMuscleInds(musc)).computeMomentArm(state,thisCoord) ;

            if coord == 1 ; % only need to do these once per muscle per pose
                maxIsoForces(musc,pose) = muscles.get(rMuscleInds(musc)).getMaxIsometricForce() ;
                normFiberLengths(musc,pose) = muscles.get(rMuscleInds(musc)).getNormalizedFiberLength(state) ;
                cosAlphas(musc,pose) = muscles.get(rMuscleInds(musc)).getCosPennationAngle(state) ;
                thisMusc = org.opensim.modeling.Millard2012EquilibriumMuscle.safeDownCast(muscles.get(rMuscleInds(musc))) ;
                fiberForceCurves{musc} = thisMusc.getFiberForceLengthCurve() ;
                fC = fiberForceCurves{musc} ;
                passiveValsInitial(musc,:) = [fC.getStrainAtZeroForce, fC.getStrainAtOneNormForce, fC.getStiffnessAtLowForceInUse, ...
                                              fC.getStiffnessAtOneNormForceInUse, fC.getCurvinessInUse] ;
            end
        end
    end
end

normFiberForceMultiplier = cosAlphas.*maxIsoForces ;

%% Fill params structure and find Initial Passive Moments
params.osimModel = osimModel ;
params.state = state ;
params.coords = coords ;
params.muscles = muscles ;
params.rMuscleInds = rMuscleInds ;
params.silderMoments = silderMoments ;
params.silderAngles = silderAngles ;
params.momentArms = momentArms ;
params.normFiberLengths = normFiberLengths ;
params.normFiberForceMultiplier = normFiberForceMultiplier ;
params.fiberForceCurves = fiberForceCurves ;
params.sagCoordNames = sagCoordNames ;
params.paramsToModify = paramsToModify ;
params.passiveValsInitial = passiveValsInitial ;
params.coeffsModelInitial = passiveValsInitial(:,paramsToModify) ;
params.ignoreTendonCompliance = ignoreTendonCompliance ;
params.muscleInds2Modify_MLinds = muscleInds2Modify_MLinds ;

modelMomentsInitial = computeModelPassiveMoments(params,passiveValsInitial) ;

%% Initialize design variable matrix, constraints, bounds

lb = ones(nModifyMuscles,1)*paramRange(:,1)' ;
ub = ones(nModifyMuscles,1)*paramRange(:,2)' ;

try
    load(initialGuessFile)
    coeffsInitial = initialGuess(muscleInds2Modify_MLinds,:) ;
catch
    coeffsInitial = passiveValsInitial(muscleInds2Modify_MLinds,paramsToModify) ;
end

% Set linear equality and inequality constraints
A = [] ;
Aeq = [] ;
b = [] ;
beq = [] ;
nonlcon = [] ;
% nonlcon = @(coeffs0,params) NonlinConstraints(coeffs0,params) ;

%% Set up optimizer

options_ip = optimoptions('fmincon','Display','notify-detailed', ...
     'TolCon',1e-4,'TolFun',1e-4,'TolX',1e-4,'MaxFunEvals',10000,...
     'MaxIter',maxOptIterations,'Algorithm','interior-point','Display','iter');
 
%% 
if ~justPlotSolution  
    
    %% Save this script in the results folder to reference settings
    FileNameAndLocation=[mfilename('fullpath')];
    newbackup=[resultsFolder '\Settings_PassiveCal.m'];
    currentfile=strcat(FileNameAndLocation, '.m');
    copyfile(currentfile,newbackup);

    %% Optimize
    tic
    disp('Beginning optimization')
    [coeffsFinal,fval,exitflag,output] = fmincon(@(coeffs0) CostPassiveCalibration(params,coeffs0), ...
    coeffsInitial,A,b,Aeq,beq,lb,ub,nonlcon,options_ip) ;

    toc
    output
    exitflag

    OUTPUT.coeffsFinal = coeffsFinal ;
    OUTPUT.exitflag = exitflag ;
    OUTPUT.output = output ;

    save([resultsFolder '/passiveCalOutput'],'OUTPUT') ;
    initialGuess = coeffsFinal ;
    save([resultsFolder '/initialGuess'],'initialGuess')
    
else % justPlotSolutions

    %% Load up old stuff - uncomment and run this block and all following blocks if you want to just plot a solution

    % Load output structure
    load(outputForPlotting) ;
    % Load results (saved as initialGuess.mat file)
    load(initialGuessForPlotting) ;

    coeffsFinal = OUTPUT.coeffsFinal;
    exitflag = OUTPUT.exitflag ;
    output = OUTPUT.output;

end % justPlotSolutions

%% Update and save model

passiveValsFinal = computeNewPassiveParams(coeffsFinal,passiveValsInitial,paramsToModify,muscleInds2Modify_MLinds) ;
modelMomentsFinal = computeModelPassiveMoments(params,passiveValsFinal) ;

updatedModelPath = [resultsFolder '\' updatedModelName] ;
updateAndSaveModel(passiveValsFinal,osimModel,updatedModelPath,muscles,rMuscleInds,muscNames)
%% Plot

figure(1)
set(gcf,'position',[50 50 1219 834])

indsJoint.j1 = find(silderMoments(:,1)) ;
indsJoint.j2 = find(silderMoments(:,2)) ;
indsJoint.j3 = find(silderMoments(:,3)) ;

yLabNames = {'passive hip flexion moment (Nm)','passive knee flexion moment (Nm)','passive ankle dorsiflexion moment (Nm)'} ;
styles = {'k-','r-','r--'} ;

for i = 1:3 
myInds = indsJoint.(['j',num2str(i)]) ;
subplot(2,3,i)
plot(silderMoments(myInds,i),'k-') ;
hold on
plot(modelMomentsInitial(myInds,i),'r-') ;
plot(modelMomentsFinal(myInds,i),'r--') ;
ylabel(yLabNames{i})
if i == 1 
legend('Silder','OriginalModel','CalibratedModel','location','southeast')
end

%angles
subplot(2,3,i+3)
plot(rad2deg(silderAngles(myInds,:)))
ylabel('hip flexion, knee flexion, ankle dorsiflexion')
xlabel('Sample')
if i == 1 
legend('Hip','Knee','Ankle')
end

end

%% Plot moments again

try; close(figure(20)) ; end
figure(20)
set(gcf,'position',[209.6667  338.3333  910.0000  279.3333])
set(gcf,'DefaultTextFontName','Arial','defaultAxesFontName','Arial')

indsJoint.j1 = find(silderMoments(:,1)) ;
indsJoint.j2 = find(silderMoments(:,2)) ;
indsJoint.j3 = find(silderMoments(:,3)) ;
xLabels = {'hip flexion angle (°)','knee flexion angle (°)','ankle dorsiflexion angle (°)'} ;
yLimits = [-33 40;-50 25;-55 7] ;
yTickVals = {-20:20:40,-50:25:25,-60:15:0} ;

nPoints = [14 19 13] ; % number of points per range - this isnt great hardcode
nRanges = [ 2 3 3] ; % number of range cycles - this isn't great hardcode

yLabNames = {{'passive hip flexion', 'moment (Nm)'},{'passive knee flexion', 'moment (Nm)'},{'passive ankle dorsiflexion','moment (Nm)'}} ;
styles = {'k-','r-','r--'} ;
colors = [0 121 196; 217, 78, 74; 46, 163, 0]/256 ;

for i = 1:3 
myInds = indsJoint.(['j',num2str(i)]) ;
subplot(1,3,i)
set(gca,'fontname','arial')
    
    plot([0 30],[0 0],'color',.5*[1 1 1],'linewidth',.5)
    hold on
    rIndex = myInds(1) ;
    for j = 1:nRanges(i)
    pHand(1) = plot(silderMoments(rIndex:rIndex+nPoints(i)-1,i),'-','linewidth',2,'color',colors(j,:)) ;
    pHand(2) = plot(modelMomentsInitial(rIndex:rIndex+nPoints(i)-1,i),'--','linewidth',2,'color',colors(j,:)) ;
    pHand(3) = plot(modelMomentsFinal(rIndex:rIndex+nPoints(i)-1,i),':','linewidth',2,'color',colors(j,:)) ;
    rIndex = nPoints(i)*(j) + myInds(1) ;
    end

xRg = [min(rad2deg(silderAngles(myInds,i))) ;max(rad2deg(silderAngles(myInds,i)))] ;
ylabel(yLabNames{i},'fontsize',12)
xlim([1 nPoints(i)]) ;
set(gca,'xtick',[1 nPoints(i)/2 nPoints(i)],'xticklabels',[xRg(1) mean(xRg) xRg(2)])
xlabel(xLabels{i},'fontsize',12)
ylim(yLimits(i,:))
set(gca,'ytick',yTickVals{i})
box off

if i == 1 
myLeg = legend(pHand,'Silder 2007','Original Model','Calibrated Model','location','southeast') ;
legend boxoff
end

end


%% Plot changes in muscle parameters
import org.opensim.modeling.*

osimModelPlot = Model(updatedModelPath) ;
musclesPlot = osimModelPlot.updMuscles() ;

muscleGroups.hipFlexion = {'psoas','iliacus','addlong','addbrev','tfl'} ;
muscleGroups.hipExtension = {'glmax1','glmax2','glmax3'} ;
muscleGroups.kneeFlexion = {'bflh','bfsh','semimem','semiten','grac','sart'};
muscleGroups.kneeExtension = {'vasmed','vaslat','vasint','recfem'};
muscleGroups.ankleFlexion = {'tibant'};
muscleGroups.ankleExtension = {'gasmed','gaslat','soleus','tibpost'};
muscleGroups.nonSagittal = {'addbrev','addmagDist','addmagIsch','addmagMid','addmagProx',...
                            'glmed1','glmed2','glmed3','glmin1','glmin2','glmin3','piri','perbrev','perlong'...
                            'edl','ehl','fdl','fhl'} ;
    
muscleFields = fields(muscleGroups) ;
nMuscleFields = length(muscleFields) ;
nPlottedMuscles = 0 ;
strains = [] ;
yVec = [0] ;
muscNamesPlotting = {};
for i = 1:nMuscleFields ;
    nPlottedMuscles = nPlottedMuscles + length(muscleGroups.(muscleFields{i})) ;
    yVec = [yVec,(max(yVec) + 3) : (max(yVec) +2+ length(muscleGroups.(muscleFields{i})))] ;
    for j = 1:length(muscleGroups.(muscleFields{i}))
        thisMuscName = [muscleGroups.(muscleFields{i}){j} '_r'] ;
        thisMusc = org.opensim.modeling.Millard2012EquilibriumMuscle.safeDownCast(musclesPlot.get(char(thisMuscName))) ;
        thisCurve = thisMusc.getFiberForceLengthCurve() ;
        strains(end+1,1) = thisCurve.getStrainAtZeroForce ;
        strains(end,2) = thisCurve.getStrainAtOneNormForce ;
        muscNamesPlotting{end+1} = muscleGroups.(muscleFields{i}){j} ;
    end
end
yVec = yVec(2:end) ;
yVec = abs(max(yVec)+1-yVec) ;

figure
set(gcf,'position',[20 40 400 600])
plot([1,1],[0 max(yVec)+5],'color',.5*[1,1,1])
hold on
plot(1.7*[1,1],[0 max(yVec)+5],'color',.5*[1 1 1])
for i = 1:nPlottedMuscles
    plot(strains(i,:)+1,yVec(i)*[1,1],'k')
    text(strains(i,2)+1.02,yVec(i),muscNamesPlotting{i},'fontsize',7)
end

xlabel('$\frac{l^m}{l_0^m}$','interpreter','latex','fontsize',18)
set(gca,'ytick',[])
ylim([0,max(yVec)+1])
xlim([min(strains(:,1))+.7,max(strains(:,2))+1.3])

text(.9,max(yVec)+2,'$F=0$','interpreter','latex')
text(1.6,max(yVec)+2,'$F=F_o^m$','interpreter','latex')

box off
axColor = get(gcf,'Color');
set(gca,'YColor',axColor,'TickDir','out','xtick',[1,1.7])
