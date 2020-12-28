# PassiveMuscleForceCalibration
Calibrates the passive muscle forces in an OpenSim model based on experimentally-collected passive joint moments from Silder et al. 2007. This procedure is detailed in the Supplemental Materials of Uhlrich et al., 2020. TODO add citation.

## <a href="https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1FiberForceLengthCurve.html">Passive MuscleForce Curve </a> from <a href="https://simtk.org/api_docs/opensim/api_docs/classOpenSim_1_1Millard2012EquilibriumMuscle.html">Millard2012EquilibriumMuscle </a>

The muscles in the Rajagopal2015 Full Body Model in OpenSim generate large passive forces at large hip flexion and knee flexion angles. This code calibrates the passive force length curves for the Millard2012EquillibriumMuscle muscles in this model. The curve is described by five parameters. In the current code, we only shift the strainAtZeroForce and strainAtOneNormForce parameters, effectively shifting and/or stretching the curve along the x-axis. Note: this code should work for other models that use Millard2012EquilibriumMuscles, but will likely not work for models (like gait2392.osim) that use the Thelen2003Muscle muscle class, due to an issue with using the equilibrateMuscles algorithm with the ThelenMuscles. There may be workarounds for this, but they have not been implemented in this code. 
<br>
<br>
![passiveCurveGeneric](https://github.com/stanfordnmbl/PassiveMuscleForceCalibration/blob/main/helperFunctions/fig_FiberForceLengthCurve.png?raw=true)
<br>
<br>
## Running the Code
Clone the repo and run PassiveCalibration.m. Below is a description of some of the options in the code:

* Use an initial guess (_initialGuessFile_ variable) for the optimization. Several examples are included in the PreviousResults folder.
* Change the joint angle ranges over which to calibrate the muscle parameters (_silderCoordRanges_ variable). You may want to tune these towards the activity of interest in your simulation. The current settings are for walking.
* Change which of the five parameters in the Millard2012EquilibriumMuscle model to modify (_paramsToModify_ variable).
* Change the upper and lower bounds for the parameters (_paramRange_ variable).
* Calibrate only specific muscles (_calibrateAllMuscles_ and _muscles2Calibrate_ variables).
* Account for tendon compliance (_ignoreTendonCompliance_ variable). Using rigid tendons speeds up the optimization substantially, so running an optimization with rigid tendons may be good for generating an initial guess for an optimization that uses compliant tendons.
* Different weights for passive torque error vs. a parameter regularization term (L2 penalty on changes in parameters, normalized to initial value).
* To only plot results, can change the justPlotSolution flag to true, which will skip the optimization.
<br>
<br>

## Example Results
Three example results are included. We include a solution that converged using rigid tendons, the results of an optimization with compliant tendons that did not converge after 100 iterations (this used the rigid tendon solution as an initial guess), and a solution with compliant tendons that converged with another 72 iterations (using the 100-iteration solution as an initial guess, stored in folder PreviousResults/CalibratedModelForPaper). The changes in passive joint moments and muscle parameters are shown below, from Uhlrich et al. 2020.
<br>
<br>

![passiveCurves](https://github.com/stanfordnmbl/PassiveMuscleForceCalibration/blob/main/helperFunctions/fig_UpdatedPassiveCurves.png?raw=true)
<br>
_Sagittal plane passive joint moment curves for the hip, knee, and ankle. Calibration improved the agreement in passive joint moments between the Rajagopal et al.35 model and experimentally-measured moments from Silder et al.55. Each joint was moved over the shown range of motion with other joints fixed at various angles (see Silder et al.). For example, the hip was moved from 15° of extension to 37° of flexion (left) with the knee fixed at 15° (blue) and 60° (red)._ 
<br>
<br>
![groupMuscleUpdates](https://github.com/stanfordnmbl/PassiveMuscleForceCalibration/blob/main/helperFunctions/fig_updatedMuscleGroupParameters.jpg?raw=true)
<br>
_Calibrated normalized muscle fiber lengths that parameterize the passive muscle force curve for sagittal plane muscle groups. At F^m=0, the default l^m⁄(l_o^m )=1, F^m=F_o^m, the default l^m⁄(l_o^m )=1.7. Muscle groupings are defined in Table S1. With the exception of the hip flexors, most muscle groups begin generating passive force at a longer fiber length after calibration._

<br>
<br>

_Calibrated muscle fiber lengths that parameterize the passive muscle force curve for each muscle in the Rajagopal et al.35 model._
![indivMuscleUpdateTable](https://github.com/stanfordnmbl/PassiveMuscleForceCalibration/blob/main/helperFunctions/fig_MuscleParameterTable.JPG?raw=true)


