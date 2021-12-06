const WebSocket = require('ws')
const SerialPort = require('serialport')
const Readline = require('@serialport/parser-readline')
const config = require('./config')
const {
  formatFingersData,
  formatOrientationData2
} = require('./utils')
const AHRS = require('ahrs')

const madgwick = new AHRS({
  sampleInterval: 500,
  algorithm: 'Mahony',
  kp: 0.3,
  ki: 0,
  doInitialisation: false
})

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
        const { accel, gyro, mag } = formatOrientationData2(
          data.slice(1)
        )

        madgwick.update(
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

        const { heading, pitch, roll } = madgwick.getEulerAngles()

        sendData({
          orientation: [heading, pitch, roll]
        })
      } else if (type == 'Fingers') {
        const fingersData = formatFingersData(data.slice(1))
        sendData({
          fingers: fingersData
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
