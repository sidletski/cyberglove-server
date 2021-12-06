class Madgwick {
  constructor() {
    this.sampleFreq = 80000.0 // sample frequency in Hz
    this.betaDef = 0.02 // 2 * proportional gain

    this.beta = this.betaDef // 2 * proportional gain (Kp)
    this.q0 = 1.0
    this.q1 = 0.0
    this.q2 = 0.0
    this.q3 = 0.0 // quaternion of sensor frame relative to auxiliary frame
  }

  update(gx, gy, gz, ax, ay, az, mx, my, mz) {
    let recipNorm
    let s0, s1, s2, s3
    let qDot1, qDot2, qDot3, qDot4
    let hx, hy
    let _2q0mx,
      _2q0my,
      _2q0mz,
      _2q1mx,
      _2bx,
      _2bz,
      _4bx,
      _4bz,
      _2q0,
      _2q1,
      _2q2,
      _2q3,
      _2q0q2,
      _2q2q3,
      q0q0,
      q0q1,
      q0q2,
      q0q3,
      q1q1,
      q1q2,
      q1q3,
      q2q2,
      q2q3,
      q3q3

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if (mx == 0.0 && my == 0.0 && mz == 0.0) {
      this.updateIMU(gx, gy, gz, ax, ay, az)
      return
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-this.q1 * gx - this.q2 * gy - this.q3 * gz)
    qDot2 = 0.5 * (this.q0 * gx + this.q2 * gz - this.q3 * gy)
    qDot3 = 0.5 * (this.q0 * gy - this.q1 * gz + this.q3 * gx)
    qDot4 = 0.5 * (this.q0 * gz + this.q1 * gy - this.q2 * gx)

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!(ax == 0.0 && ay == 0.0 && az == 0.0)) {
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az)
      ax *= recipNorm
      ay *= recipNorm
      az *= recipNorm

      // Normalise magnetometer measurement
      recipNorm = invSqrt(mx * mx + my * my + mz * mz)
      mx *= recipNorm
      my *= recipNorm
      mz *= recipNorm

      // Auxiliary variables to avoid repeated arithmetic
      _2q0mx = 2.0 * this.q0 * mx
      _2q0my = 2.0 * this.q0 * my
      _2q0mz = 2.0 * this.q0 * mz
      _2q1mx = 2.0 * this.q1 * mx
      _2q0 = 2.0 * this.q0
      _2q1 = 2.0 * this.q1
      _2q2 = 2.0 * this.q2
      _2q3 = 2.0 * this.q3
      _2q0q2 = 2.0 * this.q0 * this.q2
      _2q2q3 = 2.0 * this.q2 * this.q3
      q0q0 = this.q0 * this.q0
      q0q1 = this.q0 * this.q1
      q0q2 = this.q0 * this.q2
      q0q3 = this.q0 * this.q3
      q1q1 = this.q1 * this.q1
      q1q2 = this.q1 * this.q2
      q1q3 = this.q1 * this.q3
      q2q2 = this.q2 * this.q2
      q2q3 = this.q2 * this.q3
      q3q3 = this.q3 * this.q3

      // Reference direction of Earth's magnetic field
      hx =
        mx * q0q0 -
        _2q0my * this.q3 +
        _2q0mz * this.q2 +
        mx * q1q1 +
        _2q1 * my * this.q2 +
        _2q1 * mz * this.q3 -
        mx * q2q2 -
        mx * q3q3
      hy =
        _2q0mx * this.q3 +
        my * q0q0 -
        _2q0mz * this.q1 +
        _2q1mx * this.q2 -
        my * q1q1 +
        my * q2q2 +
        _2q2 * mz * this.q3 -
        my * q3q3
      _2bx = invSqrt(hx * hx + hy * hy)
      _2bz =
        -_2q0mx * this.q2 +
        _2q0my * this.q1 +
        mz * q0q0 +
        _2q1mx * this.q3 -
        mz * q1q1 +
        _2q2 * my * this.q3 -
        mz * q2q2 +
        mz * q3q3
      _4bx = 2.0 * _2bx
      _4bz = 2.0 * _2bz

      // Gradient decent algorithm corrective step
      s0 =
        -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) +
        _2q1 * (2.0 * q0q1 + _2q2q3 - ay) -
        _2bz *
          this.q2 *
          (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
        (-_2bx * this.q3 + _2bz * this.q1) *
          (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
        _2bx *
          this.q2 *
          (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
      s1 =
        _2q3 * (2.0 * q1q3 - _2q0q2 - ax) +
        _2q0 * (2.0 * q0q1 + _2q2q3 - ay) -
        4.0 * this.q1 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
        _2bz *
          this.q3 *
          (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
        (_2bx * this.q2 + _2bz * this.q0) *
          (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
        (_2bx * this.q3 - _4bz * this.q1) *
          (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
      s2 =
        -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) +
        _2q3 * (2.0 * q0q1 + _2q2q3 - ay) -
        4.0 * this.q2 * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) +
        (-_4bx * this.q2 - _2bz * this.q0) *
          (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
        (_2bx * this.q1 + _2bz * this.q3) *
          (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
        (_2bx * this.q0 - _4bz * this.q2) *
          (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
      s3 =
        _2q1 * (2.0 * q1q3 - _2q0q2 - ax) +
        _2q2 * (2.0 * q0q1 + _2q2q3 - ay) +
        (-_4bx * this.q3 + _2bz * this.q1) *
          (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) +
        (-_2bx * this.q0 + _2bz * this.q2) *
          (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) +
        _2bx *
          this.q1 *
          (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
      s0 *= recipNorm
      s1 *= recipNorm
      s2 *= recipNorm
      s3 *= recipNorm

      // Apply feedback step
      qDot1 -= this.beta * s0
      qDot2 -= this.beta * s1
      qDot3 -= this.beta * s2
      qDot4 -= this.beta * s3
    }

    // Integrate rate of change of quaternion to yield quaternion
    this.q0 += qDot1 * (1.0 / this.sampleFreq)
    this.q1 += qDot2 * (1.0 / this.sampleFreq)
    this.q2 += qDot3 * (1.0 / this.sampleFreq)
    this.q3 += qDot4 * (1.0 / this.sampleFreq)

    // Normalise quaternion
    recipNorm = invSqrt(
      this.q0 * this.q0 +
        this.q1 * this.q1 +
        this.q2 * this.q2 +
        this.q3 * this.q3
    )
    this.q0 *= recipNorm
    this.q1 *= recipNorm
    this.q2 *= recipNorm
    this.q3 *= recipNorm

    return [this.q0, this.q1, this.q2, this.q3]
  }

  updateIMU(gx, gy, gz, ax, ay, az) {
    let recipNorm
    let s0, s1, s2, s3
    let qDot1, qDot2, qDot3, qDot4
    let _2q0,
      _2q1,
      _2q2,
      _2q3,
      _4q0,
      _4q1,
      _4q2,
      _8q1,
      _8q2,
      q0q0,
      q1q1,
      q2q2,
      q3q3

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-this.q1 * gx - this.q2 * gy - this.q3 * gz)
    qDot2 = 0.5 * (this.q0 * gx + this.q2 * gz - this.q3 * gy)
    qDot3 = 0.5 * (this.q0 * gy - this.q1 * gz + this.q3 * gx)
    qDot4 = 0.5 * (this.q0 * gz + this.q1 * gy - this.q2 * gx)

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!(ax == 0.0 && ay == 0.0 && az == 0.0)) {
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az)
      ax *= recipNorm
      ay *= recipNorm
      az *= recipNorm

      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0 * this.q0
      _2q1 = 2.0 * this.q1
      _2q2 = 2.0 * this.q2
      _2q3 = 2.0 * this.q3
      _4q0 = 4.0 * this.q0
      _4q1 = 4.0 * this.q1
      _4q2 = 4.0 * this.q2
      _8q1 = 8.0 * this.q1
      _8q2 = 8.0 * this.q2
      q0q0 = this.q0 * this.q0
      q1q1 = this.q1 * this.q1
      q2q2 = this.q2 * this.q2
      q3q3 = this.q3 * this.q3

      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
      s1 =
        _4q1 * q3q3 -
        _2q3 * ax +
        4.0 * q0q0 * this.q1 -
        _2q0 * ay -
        _4q1 +
        _8q1 * q1q1 +
        _8q1 * q2q2 +
        _4q1 * az
      s2 =
        4.0 * q0q0 * this.q2 +
        _2q0 * ax +
        _4q2 * q3q3 -
        _2q3 * ay -
        _4q2 +
        _8q2 * q1q1 +
        _8q2 * q2q2 +
        _4q2 * az
      s3 =
        4.0 * q1q1 * this.q3 -
        _2q1 * ax +
        4.0 * q2q2 * this.q3 -
        _2q2 * ay
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) // normalise step magnitude
      s0 *= recipNorm
      s1 *= recipNorm
      s2 *= recipNorm
      s3 *= recipNorm

      // Apply feedback step
      qDot1 -= this.beta * s0
      qDot2 -= this.beta * s1
      qDot3 -= this.beta * s2
      qDot4 -= this.beta * s3
    }

    // Integrate rate of change of quaternion to yield quaternion
    this.q0 += qDot1 * (1.0 / this.sampleFreq)
    this.q1 += qDot2 * (1.0 / this.sampleFreq)
    this.q2 += qDot3 * (1.0 / this.sampleFreq)
    this.q3 += qDot4 * (1.0 / this.sampleFreq)

    // Normalise quaternion
    recipNorm = invSqrt(
      this.q0 * this.q0 +
        this.q1 * this.q1 +
        this.q2 * this.q2 +
        this.q3 * this.q3
    )
    this.q0 *= recipNorm
    this.q1 *= recipNorm
    this.q2 *= recipNorm
    this.q3 *= recipNorm
  }
}

module.exports = Madgwick

function invSqrt(number) {
  return 1 / Math.sqrt(number)
  //   var i
  //   var x2, y
  //   const threehalfs = 1.5

  //   x2 = number * 0.5
  //   y = number
  //   //evil floating bit level hacking
  //   var buf = new ArrayBuffer(4)
  //   new Float32Array(buf)[0] = number
  //   i = new Uint32Array(buf)[0]
  //   i = 0x5f3759df - (i >> 1) //What the fuck?
  //   new Uint32Array(buf)[0] = i
  //   y = new Float32Array(buf)[0]
  //   y = y * (threehalfs - x2 * y * y) // 1st iteration
  //   y = y * (threehalfs - x2 * y * y) // 2nd iteration, this can be removed

  //   return y
}
