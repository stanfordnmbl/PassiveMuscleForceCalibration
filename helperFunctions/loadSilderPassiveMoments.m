function [passiveMoments coordAngles momentsLinearIndicies] = loadSilderPassiveMoments(dataPath,coordinateRanges,downsamplingFactor) ;
% This function loads digitized passive moements from Silder 2007, and puts
% them into a nCoordCombinations x 3 matrix with columns being hip, knee,
% ankle.

% load Silder Moments
load(dataPath) ;

% pull out data
moments = PassiveM{1} * [1,0,0] ; % hip moments
moments = vertcat(moments, PassiveM{2} * [0,1,0]) ; % knee moments
moments = vertcat(moments, PassiveM{3} * [0,0,1]); % ankle moments

angles = JAngles{1}(:,[1,3,4]) ;
angles = vertcat(angles, JAngles{2}(:,[1,3,4])) ;
angles = vertcat(angles, JAngles{3}(:,[1,3,4])) ;

% downsample
angles = downsample(angles,downsamplingFactor) ;
moments = downsample(moments,downsamplingFactor) ;
nSamples = size(moments,1) ;

% cut off limits
lowerLimMat = (coordinateRanges(:,1) * ones(1,nSamples))';
upperLimMat = (coordinateRanges(:,2) * ones(1,nSamples))';

inclusionTrials = find(min(angles>=lowerLimMat,[],2) & min(angles<=upperLimMat,[],2)) ;

passiveMoments = moments(inclusionTrials,:) ;
coordAngles = deg2rad(angles(inclusionTrials,:)) ;

momentsLinearIndicies = find(passiveMoments) ;