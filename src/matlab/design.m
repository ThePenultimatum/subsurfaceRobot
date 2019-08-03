

%%% FUNCTIONS
  %%%% youngs modulus is that of the impactor material, units in Newtons/cm^2, this must be converted to (g*cm/s^2)/cm^2
  %%%% depth = depth of lowest face of robot in granular material, units cm
  %%%% density = density of impactor in g/cm^3
  %%%% areaImpactor is area in cm^2 of face striking the chassis
  %%%% areaRobot is area in cm^2 of surface in direction of motion in contact with granular material
  %%%% Vmax is defined by the impact mechanism, units cm/s, maximum velocity reached by impactor in the instant before impact
  %%%% massRobot is mass of the entire robot in g
  %%%% massImpactor is the mass of the entire impactor in g
  %%%% lengthImpactor is the length of the entire impactor in cm
  %%%% stress at depths which are taken experimentally from the paper on granular material legged locomotion is in N/cm^2 and must also be converted


function dx = dxHoriz(youngsModulus, density, areaImpactor, depth, areaRobot, Vmax, massRobot, massImpactor, lengthImpactor)
  newtonScalingFactor = 100000;
  convertedYoungsModulus = youngsModulus * newtonScalingFactor;
  convertedYieldStress = 0.3 * newtonScalingFactor;
  dtSquared = massImpactor * lengthImpactor / (convertedYoungsModulus * areaImpactor);
  aImpactor = Vmax * areaImpactor * sqrt(convertedYoungsModulus * density) / massRobot;
  aGranular = convertedYieldStress * depth * areaRobot / massRobot;
  withImpactorContribution = (aImpactor - aGranular)*dtSquared;
  withoutImpactorContribution = aGranular * (Vmax / aGranular) ^ 2; %% = Vmax / aGranular
  dx = withImpactorContribution + withoutImpactorContribution;
end

function dx = dxUp(youngsModulus, density, areaImpactor, depth, areaRobot, Vmax, massRobot, massImpactor, lengthImpactor)
  aGrav = 9.8;
  newtonScalingFactor = 100000;
  meterCMScalingFactor = 100;
  convertedYoungsModulus = youngsModulus * newtonScalingFactor;
  convertedYieldStress = 0.3 * newtonScalingFactor;
  convertedAGrav = aGrav * meterCMScalingFactor;
  dtSquared = massImpactor * lengthImpactor / (convertedYoungsModulus * areaImpactor);
  aImpactor = Vmax * areaImpactor * sqrt(convertedYoungsModulus * density) / massRobot;
  aGranular = convertedYieldStress * depth * areaRobot / massRobot;
  withImpactorContribution = (aImpactor - convertedAGrav - aGranular)*dtSquared;
  withoutImpactorContribution = aGranular * (Vmax / aGranular) ^ 2; %% = Vmax / aGranular
  dx = withImpactorContribution + withoutImpactorContribution;  
end

function dx = dxDown(youngsModulus, density, areaImpactor, depth, areaRobot, Vmax, massRobot, massImpactor, lengthImpactor)
  aGrav = 9.8;
  newtonScalingFactor = 100000;
  meterCMScalingFactor = 100;
  convertedYoungsModulus = youngsModulus * newtonScalingFactor;
  convertedYieldStress = 0.3 * newtonScalingFactor;
  convertedAGrav = aGrav * meterCMScalingFactor;
  dtSquared = massImpactor * lengthImpactor / (convertedYoungsModulus * areaImpactor);
  aImpactor = Vmax * areaImpactor * sqrt(convertedYoungsModulus * density) / massRobot;
  aGranular = convertedYieldStress * depth * areaRobot / massRobot;
  withImpactorContribution = (aImpactor + convertedAGrav - aGranular)*dtSquared;
  withoutImpactorContribution = aGranular * (Vmax / aGranular) ^ 2; %% = Vmax / aGranular
  dx = withImpactorContribution + withoutImpactorContribution;
end