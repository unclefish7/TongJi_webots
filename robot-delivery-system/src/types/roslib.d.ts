declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string })
    on(event: string, callback: (data?: any) => void): void
    close(): void
  }

  export class Topic {
    constructor(options: { ros: Ros; name: string; messageType: string })
    subscribe(callback: (message: any) => void): void
    unsubscribe(): void
    publish(message: any): void
  }

  export class Service {
    constructor(options: { ros: Ros; name: string; serviceType: string })
    callService(request: any, callback: (result: any) => void, failedCallback?: (error: any) => void): void
  }

  export class ServiceRequest {
    constructor(values?: any)
  }
}
