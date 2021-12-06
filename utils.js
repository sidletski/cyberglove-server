const config = require('./config')

const formatFingersData = function (data) {
  return data.map((i) => Math.min(1, Math.max(0, (i - 400) / 350)))
}

const arrToObject = (coords) => {
  const [x, y, z] = coords

  return {
    x,
    y,
    z
  }
}

const formatOrientationData = function (data) {
  const accelIndex = config.imu.order.indexOf('accel') * 3
  const gyroIndex = config.imu.order.indexOf('gyro') * 3
  const magIndex = config.imu.order.indexOf('mag') * 3

  return {
    accel: arrToObject(data.slice(accelIndex, accelIndex + 3)),
    gyro: arrToObject(data.slice(gyroIndex, gyroIndex + 3)),
    mag: arrToObject(data.slice(magIndex, magIndex + 3))
  }
}

const formatOrientationData2 = function (data) {
  const accelIndex = config.imu.order.indexOf('accel') * 3
  const gyroIndex = config.imu.order.indexOf('gyro') * 3
  const magIndex = config.imu.order.indexOf('mag') * 3

  return {
    accel: arrToObject(
      data.slice(accelIndex, accelIndex + 3).map((i) => i / 32767)
    ),
    gyro: arrToObject(
      data.slice(gyroIndex, gyroIndex + 3).map((i) => i / 32767)
    ),
    mag: arrToObject(
      data.slice(magIndex, magIndex + 3).map((i) => i / 32767)
    )
  }
}

module.exports = {
  formatFingersData,
  formatOrientationData,
  formatOrientationData2
}
