/*
 * Copyright 2015 Google Inc. All Rights Reserved.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

var SensorSample = require('./sensor-sample.js');
var MathUtil = require('../math-util.js');
var Util = require('../util.js');

var DEBUG = false;

/**
 * An implementation of a simple complementary filter, which fuses gyroscope and
 * accelerometer data from the 'devicemotion' event.
 *
 * Accelerometer data is very noisy, but stable over the long term.
 * Gyroscope data is smooth, but tends to drift over the long term.
 *
 * This fusion is relatively simple:
 * 1. Get orientation estimates from accelerometer by applying a low-pass filter
 *    on that data.
 * 2. Get orientation estimates from gyroscope by integrating over time.
 * 3. Combine the two estimates, weighing (1) in the long term, but (2) for the
 *    short term.
 *
 *
 * An additional anti yaw drift measure is applied using the magnetometer
 * which gives us an absolute yaw reference but is very laggy
 * 
 * This fusion is relatively simple:
 * 1. Calculate the delta between the estimated yaw (gyro based) and the true yaw (compass based)
 * 2. Correct the estimated yaw with the true yaw. 
 *    The correction amount adapts to the radial velocity of the user to avoid the user noticing the correction
 *
 */
function ComplementaryFilter(kFilter, compassKFilter) {
  this.kFilter = kFilter;

  // Raw sensor measurements.
  this.currentAccelMeasurement = new SensorSample();
  this.currentGyroMeasurement = new SensorSample();
  this.previousGyroMeasurement = new SensorSample();
  this.currentOrientationMeasurement = new SensorSample();

  this.minRv = 0.5;  //minimal radial velocity to apply compass fusion
  this.maxRv = 15; //radial velocity to apply maximal compass fusion
  // compass compensation factor.
  // lower values give better accuracy but higher chance that the user will feel the correction
  compassKFilter = (typeof compassKFilter !== 'undefined') ?  compassKFilter : 0.7;
  this.maxFactor = (1-compassKFilter);

  // Set default look direction to be in the correct direction.
  if (Util.isIOS()) {
    this.filterQ = new MathUtil.Quaternion(-1, 0, 0, 1);
  } else {
    this.filterQ = new MathUtil.Quaternion(1, 0, 0, 1);
  }
  this.previousFilterQ = new MathUtil.Quaternion();
  this.previousFilterQ.copy(this.filterQ);

  // Orientation based on the accelerometer.
  this.accelQ = new MathUtil.Quaternion();
  // Whether or not the orientation has been initialized.
  this.isOrientationInitialized = false;
  // Running estimate of gravity based on the current orientation.
  this.estimatedGravity = new MathUtil.Vector3();
  // Measured gravity based on accelerometer.
  this.measuredGravity = new MathUtil.Vector3();

  // Debug only quaternion of gyro-based orientation.
  this.gyroIntegralQ = new MathUtil.Quaternion();
}

ComplementaryFilter.prototype.addAccelMeasurement = function(vector, timestampS) {
  this.currentAccelMeasurement.set(vector, timestampS);
};

ComplementaryFilter.prototype.addGyroMeasurement = function(vector, timestampS) {
  this.currentGyroMeasurement.set(vector, timestampS);

  var deltaT = timestampS - this.previousGyroMeasurement.timestampS;
  if (Util.isTimestampDeltaValid(deltaT)) {
    this.run_();
  }

  this.previousGyroMeasurement.copy(this.currentGyroMeasurement);
};

ComplementaryFilter.prototype.addOrientationAbsMeasurement = function(eulerV, timestampS) {
  this.currentOrientationMeasurement.set(eulerV, timestampS);
};

ComplementaryFilter.prototype.run_ = function() {

  if (!this.isOrientationInitialized) {
    this.accelQ = this.accelToQuaternion_(this.currentAccelMeasurement.sample);
    this.previousFilterQ.copy(this.accelQ);
    this.isOrientationInitialized = true;
    return;
  }

  var deltaT = this.currentGyroMeasurement.timestampS -
      this.previousGyroMeasurement.timestampS;

  // Convert gyro rotation vector to a quaternion delta.
  var gyroDeltaQ = this.gyroToQuaternionDelta_(this.currentGyroMeasurement.sample, deltaT);
  this.gyroIntegralQ.multiply(gyroDeltaQ);

  // filter_1 = K * (filter_0 + gyro * dT) + (1 - K) * accel.
  this.filterQ.copy(this.previousFilterQ);
  this.filterQ.multiply(gyroDeltaQ);


  // Calculate the delta between the current estimated gravity and the real
  // gravity vector from accelerometer.
  var invFilterQ = new MathUtil.Quaternion();
  invFilterQ.copy(this.filterQ);
  invFilterQ.inverse();

  this.estimatedGravity.set(0, 0, -1);
  this.estimatedGravity.applyQuaternion(invFilterQ);
  this.estimatedGravity.normalize();

  this.measuredGravity.copy(this.currentAccelMeasurement.sample);
  this.measuredGravity.normalize();

  // Compare estimated gravity with measured gravity, get the delta quaternion
  // between the two.
  var deltaQ = new MathUtil.Quaternion();
  deltaQ.setFromUnitVectors(this.estimatedGravity, this.measuredGravity);
  deltaQ.inverse();

  if (DEBUG) {
    console.log('Delta: %d deg, G_est: (%s, %s, %s), G_meas: (%s, %s, %s)',
                MathUtil.radToDeg * Util.getQuaternionAngle(deltaQ),
                (this.estimatedGravity.x).toFixed(1),
                (this.estimatedGravity.y).toFixed(1),
                (this.estimatedGravity.z).toFixed(1),
                (this.measuredGravity.x).toFixed(1),
                (this.measuredGravity.y).toFixed(1),
                (this.measuredGravity.z).toFixed(1));
  }

  // Calculate the SLERP target: current orientation plus the measured-estimated
  // quaternion delta.
  var targetQ = new MathUtil.Quaternion();
  targetQ.copy(this.filterQ);
  targetQ.multiply(deltaQ);

  // SLERP factor: 0 is pure gyro, 1 is pure accel.
  this.filterQ.slerp(targetQ, 1 - this.kFilter);

  if (this.currentOrientationMeasurement.sample &&
      this.currentOrientationMeasurement.sample.alpha !== null) {

    var compassQ = new MathUtil.Quaternion();
    compassQ.setFromEulerZXY(
      this.currentOrientationMeasurement.sample.beta,
      this.currentOrientationMeasurement.sample.gamma,
      this.currentOrientationMeasurement.sample.alpha);

    if (false) {
      //simple, non-adaptive fusion
      this.filterQ.slerp(compassQ, 1 - this.kFilter);  
    } else {
      //adaptive fusion

      //calculate earth yaw delta quaternion
      //there's probably a better way of doing this, but this one works...
      var estimatedNorth = new MathUtil.Vector3();
      estimatedNorth.set(0, 1, 0);
      estimatedNorth.applyQuaternion(invFilterQ);
      estimatedNorth.normalize();

      var invCompassQ = new MathUtil.Quaternion();
      invCompassQ.copy(compassQ);
      invCompassQ.inverse();

      var measuredNorth = new MathUtil.Vector3();
      measuredNorth.set(0, 1, 0);
      measuredNorth.applyQuaternion(invCompassQ);
      measuredNorth.normalize();

      // Compare estimated compass with measured compass, get the delta quaternion
      // between the two.
      deltaQ = new MathUtil.Quaternion();
      deltaQ.setFromUnitVectors(estimatedNorth, measuredNorth);
      deltaQ.inverse();

      targetQ = new MathUtil.Quaternion();
      targetQ.copy(this.filterQ);
      targetQ.multiply(deltaQ);

      //earth frame radial velocity from gyro
      var earthGyro = new MathUtil.Vector3();
      earthGyro.copy(this.currentGyroMeasurement.sample);
      earthGyro.applyQuaternion(this.filterQ);

      //calculate adaptive factor, relative to user's yaw velocity
      var kFactor = scaleClamp(this.minRv, this.maxRv, 0, this.maxFactor, Math.abs(earthGyro.z));

      //adaptive factor, relative to user's combined radial velocity
      // var kFactor = scale(this.minRv, this.maxRv, 0, this.maxFactor, this.currentGyroMeasurement.sample.length());

      this.filterQ.slerp(targetQ, kFactor);
    }
  }  

  this.previousFilterQ.copy(this.filterQ);
};

/**
 * Scale a value from range to range + clamp
 * @param sMin min val of source range
 * @param sMax max val of source range
 * @param tMin min val of target range
 * @param tMax max val of target range
 * @param val value to map
 */
function scaleClamp(sMin, sMax, tMin, tMax, val){
  //map
  var mapped = (val-sMin)/(sMax-sMin) * (tMax-tMin) + tMin;
  //clamp
  return Math.max(tMin, Math.min(mapped, tMax));
}

ComplementaryFilter.prototype.getOrientation = function() {
  return this.filterQ;
};

ComplementaryFilter.prototype.accelToQuaternion_ = function(accel) {
  var normAccel = new MathUtil.Vector3();
  normAccel.copy(accel);
  normAccel.normalize();
  var quat = new MathUtil.Quaternion();
  quat.setFromUnitVectors(new MathUtil.Vector3(0, 0, -1), normAccel);
  quat.inverse();
  return quat;
};

ComplementaryFilter.prototype.gyroToQuaternionDelta_ = function(gyro, dt) {
  // Extract axis and angle from the gyroscope data.
  var quat = new MathUtil.Quaternion();
  var axis = new MathUtil.Vector3();
  axis.copy(gyro);
  axis.normalize();
  quat.setFromAxisAngle(axis, gyro.length() * dt);
  return quat;
};


module.exports = ComplementaryFilter;
