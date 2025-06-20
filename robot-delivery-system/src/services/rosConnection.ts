import * as ROSLIB from 'roslib'

export interface RobotPosition {
  x: number
  y: number
  z: number
  orientation: {
    x: number
    y: number
    z: number
    w: number
  }
}

export interface MapData {
  width: number
  height: number
  resolution: number
  origin: {
    position: {
      x: number
      y: number
      z: number
    }
    orientation: {
      x: number
      y: number
      z: number
      w: number
    }
  }
  data: number[]
}

export class ROSConnection {
  private ros: ROSLIB.Ros | null = null
  private mapTopic: ROSLIB.Topic | null = null
  private tfTopic: ROSLIB.Topic | null = null
  private robotStateTopic: ROSLIB.Topic | null = null
  // 新增导航指令话题
  private multiNavTopic: ROSLIB.Topic | null = null
  private nextSignalTopic: ROSLIB.Topic | null = null
  
  private mapCallback: ((map: MapData) => void) | null = null
  private robotPositionCallback: ((position: RobotPosition) => void) | null = null
  
  private trackingInterval: number | null = null
  private isTracking = false
  
  constructor(private rosUrl: string = 'ws://192.168.44.129:9090') {}

  async connect(): Promise<boolean> {
    return new Promise((resolve, reject) => {
      this.ros = new ROSLIB.Ros({
        url: this.rosUrl
      })

      this.ros.on('connection', () => {
        console.log('Connected to ROS bridge')
        this.setupTopics()
        resolve(true)
      })

      this.ros.on('error', (error: Error) => {
        console.error('Error connecting to ROS:', error)
        reject(error)
      })

      this.ros.on('close', () => {
        console.log('Connection to ROS closed')
      })
    })
  }

  private setupTopics() {
    if (!this.ros) return

    // 订阅地图话题
    this.mapTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    })

    this.mapTopic.subscribe((message: any) => {
      if (this.mapCallback) {
        const mapData: MapData = {
          width: message.info.width,
          height: message.info.height,
          resolution: message.info.resolution,
          origin: {
            position: {
              x: message.info.origin.position.x,
              y: message.info.origin.position.y,
              z: message.info.origin.position.z
            },
            orientation: {
              x: message.info.origin.orientation.x,
              y: message.info.origin.orientation.y,
              z: message.info.origin.orientation.z,
              w: message.info.origin.orientation.w
            }
          },
          data: message.data
        }
        this.mapCallback(mapData)
      }
    })

    // 订阅TF话题进行位置追踪
    this.tfTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/tf',
      messageType: 'tf2_msgs/TFMessage'
    })

    this.tfTopic.subscribe((message: any) => {
      if (this.robotPositionCallback && message.transforms && this.isTracking) {
        // 查找机器人的transform
        const robotTransform = message.transforms.find((transform: any) => 
          transform.child_frame_id === 'base_link' || 
          transform.child_frame_id === 'robot_base' ||
          transform.child_frame_id === 'base_footprint' ||
          transform.child_frame_id.includes('robot')
        )
        
        if (robotTransform) {
          const position: RobotPosition = {
            x: robotTransform.transform.translation.x,
            y: robotTransform.transform.translation.y,
            z: robotTransform.transform.translation.z,
            orientation: {
              x: robotTransform.transform.rotation.x,
              y: robotTransform.transform.rotation.y,
              z: robotTransform.transform.rotation.z,
              w: robotTransform.transform.rotation.w
            }
          }
          this.robotPositionCallback(position)
        }
      }
    })

    // 备选方案：订阅机器人状态话题
    this.robotStateTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/robot_state',
      messageType: 'geometry_msgs/PoseStamped'
    })

    this.robotStateTopic.subscribe((message: any) => {
      if (this.robotPositionCallback && message.pose && this.isTracking) {
        const position: RobotPosition = {
          x: message.pose.position.x,
          y: message.pose.position.y,
          z: message.pose.position.z,
          orientation: {
            x: message.pose.orientation.x,
            y: message.pose.orientation.y,
            z: message.pose.orientation.z,
            w: message.pose.orientation.w
          }
        }
        this.robotPositionCallback(position)
      }
    })

    // 新增：订阅多目标导航话题
    this.multiNavTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/move_base/goal',
      messageType: 'geometry_msgs/PoseStamped'
    })

    this.multiNavTopic.subscribe((message: any) => {
      console.log('Received multi-goal navigation message:', message)
    })

    // 新增：订阅下一步信号话题
    this.nextSignalTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/next_signal',
      messageType: 'std_msgs/String'
    })

    this.nextSignalTopic.subscribe((message: any) => {
      console.log('Received next signal:', message.data)
    })
  }

  onMapReceived(callback: (map: MapData) => void) {
    this.mapCallback = callback
  }

  onRobotPositionUpdate(callback: (position: RobotPosition) => void) {
    this.robotPositionCallback = callback
  }

  disconnect() {
    this.stopTracking()
    
    if (this.ros) {
      this.ros.close()
      this.ros = null
    }
    
    this.mapTopic = null
    this.robotStateTopic = null
    this.tfTopic = null
    this.multiNavTopic = null
    this.nextSignalTopic = null
  }

  // 获取单次地图数据
  async getMapOnce(): Promise<MapData> {
    return new Promise((resolve, reject) => {
      if (!this.ros) {
        reject(new Error('ROS not connected'))
        return
      }

      const mapService = new ROSLIB.Service({
        ros: this.ros,
        name: '/static_map',
        serviceType: 'nav_msgs/GetMap'
      })

      const request = new ROSLIB.ServiceRequest({})

      mapService.callService(request, (result: any) => {
        const mapData: MapData = {
          width: result.map.info.width,
          height: result.map.info.height,
          resolution: result.map.info.resolution,
          origin: {
            position: {
              x: result.map.info.origin.position.x,
              y: result.map.info.origin.position.y,
              z: result.map.info.origin.position.z
            },
            orientation: {
              x: result.map.info.origin.orientation.x,
              y: result.map.info.origin.orientation.y,
              z: result.map.info.origin.orientation.z,
              w: result.map.info.origin.orientation.w
            }
          },
          data: result.map.data
        }
        resolve(mapData)
      }, (error: any) => {
        reject(error)
      })
    })
  }

  // 开始追踪机器人位置
  startTracking() {
    if (this.isTracking) return
    
    this.isTracking = true
    console.log('开始追踪机器人位置')
  }
  
  // 停止追踪机器人位置
  stopTracking() {
    this.isTracking = false
    console.log('停止追踪机器人位置')
  }

  // 发布多目标导航指令
  async publishMultiNavCommand(locations: string[]): Promise<void> {
    if (!this.ros) {
      throw new Error('ROS not connected')
    }

    if (!this.multiNavTopic) {
      this.multiNavTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/multi_nav_command',
        messageType: 'std_msgs/String'
      })
    }

    const message = {
      data: JSON.stringify(locations)
    }

    this.multiNavTopic.publish(message)
    console.log('Published multi-nav command:', locations)
  }

  // 发布next信号
  async publishNextSignal(command: string = 'start'): Promise<void> {
    if (!this.ros) {
      throw new Error('ROS not connected')
    }

    if (!this.nextSignalTopic) {
      this.nextSignalTopic = new ROSLIB.Topic({
        ros: this.ros,
        name: '/next',
        messageType: 'std_msgs/String'
      })
    }

    const message = {
      data: command
    }

    this.nextSignalTopic.publish(message)
    console.log('Published next signal:', command)
  }
}

export const rosConnection = new ROSConnection()
