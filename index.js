const WebSocket = require('ws')
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const config = require('./config')
const {
  formatFingersData,
  formatOrientationData
} = require('./utils')
const Madgwick = require('./madgwick')

const madgwick = new Madgwick()

let socketClient = null
const wsServer = new WebSocket.Server({ port: config.ws.port })

const sendData = function (data) {
  if (!socketClient) return
  socketClient.send(JSON.stringify(data))
}

const initSocket = function () {
  console.log(`WebSocket was started at port ${config.ws.port}`)

  const onConnect = function (client) {
    socketClient = client

    client.on('message', function (message) {
      console.log(message)
    })

    client.on('close', function () {
      console.log('Client was disconnected')
    })
  }

  wsServer.on('connection', onConnect)
}

const initPort = function ({ path }) {
  const serialPort = new SerialPort(path, {
    baudRate: config.board.baudrate
  })

  console.log('Connected to the serial port')

  const parser = serialPort.pipe(new Readline({ delimiter: '\r\n' }))

  serialPort.on('close', (msg) => {
    setTimeout(() => {
      connectBoard()
    }, 500)
  })

  parser.on('data', (msg) => {
    try {
      const data = msg.trim().split('\t')
      const type = data[0]

      if (type == 'IMU') {
        const { accel, gyro, mag } = formatOrientationData(
          data.slice(1)
        )

        const qs = madgwick.update(
          gyro.x,
          gyro.y,
          gyro.z,
          accel.x,
          accel.y,
          accel.z,
          mag.x,
          mag.y,
          mag.z
        )

        if (config.logInput) {
          console.log(
            'IMU values:\t',
            `gx=${gyro.x}\tgy=${gyro.y}\tgz=${gyro.z}\tax=${accel.x}\tay=${accel.y}\tax=${accel.z}\tmx=${mag.x}\tmy=${mag.y}\tmz=${mag.z}`
          )
          console.log(
            'Quaternion:\t',
            qs
              .map((i, index) => `q${index}=${i.toFixed(3)}`)
              .join('\t')
          )
        }

        console.log(qs.map((i) => i.toFixed(3)).join('\t'))

        sendData({
          orientation: qs
        })
      } else if (type == 'Fingers') {
        const fingersData = formatFingersData(data.slice(1))
        sendData({
          fingers: fingersData
        })
      } else if (type == 'Quat') {
        sendData({
          orientation: data.slice(1).map((i) => parseFloat(i))
        })
      }
    } catch (e) {
      console.log('Wrong message format', e)
    }
  })

  initSocket()
}

function connectBoard() {
  SerialPort.list().then(function (ports) {
    const port = ports.find(
      ({ manufacturer }) => manufacturer == 'SparkFun'
    )

    if (port) {
      initPort(port)
    } else {
      console.log('Board is not connected')
      setTimeout(() => {
        connectBoard()
      }, 500)
    }
  })
}

connectBoard()
