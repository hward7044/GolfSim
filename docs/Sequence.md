@startuml
skinparam maxMessageSize 150
skinparam ParticipantPadding 20
skinparam BoxPadding 10
autonumber

actor "Golfer" as Golfer

box "Concurrency & HAL" #LightCyan
participant "ThreadManager\n(Orchestrator)" as Orchestrator
participant "Producer Thread" as Producer
participant "Camera Array\n(OV9281)" as Camera
end box

queue "AtomicRingBuffer" as Buffer

box "Business Logic & Math" #LightYellow
participant "Consumer Thread" as Consumer
participant "SessionStateMachine" as FSM
participant "Vision\n(OpenCV)" as Vision
participant "SpatialSolver\n(Stereo Map)" as Spatial
participant "KinematicsSolver\n(Eigen SVD)" as Physics
participant "TcpTransmitter" as Network
end box

box "Diagnostics" #LightGreen
participant "GlobalLogger\n(Singleton)" as Logger
participant "FlightRecorder" as Recorder
end box

participant "OpenGolfSim" as UI

== Initialization Phase ==
Orchestrator -> Producer: startProducerThread()
Orchestrator -> Consumer: startConsumerThread()
Consumer -> FSM: loadCalibration()
FSM -> Logger: LOG_INFO("System ARMED")

== Asynchronous Capture & Trigger Loop ==
par Lock-Free Execution
    loop 120+ FPS
        Producer -> Camera: grabRawFrame()
        note right: Native UVC via V4L2 (Linux)\nor MediaFoundation (Windows)
        Camera --> Producer: FrameSet
        Producer -> Buffer: push(FrameSet)
    end
else Background Consumer Loop
    loop
        Consumer -> Buffer: Poll Latest FrameSet
        Consumer -> FSM: processNextFrame(FrameSet)
        FSM -> FSM: evaluateState(ROI Trigger)
    end
end

== Launch Event ==
Golfer -> Camera: Club Impacts Ball
FSM -> FSM: Trigger Detected! (State = FLIGHT_CAPTURE)
FSM -> Logger: LOG_INFO("Trigger SAD threshold exceeded")
FSM -> Buffer: getHistory(Flight Frames)

== Modular Mathematical Pipeline ==
FSM -> Vision: extractFeatures(Flight Frames)
activate Vision
note right: Sub-pixel centroids: x_c, y_c
Vision --> FSM: 2D Feature Map
deactivate Vision

FSM -> Spatial: triangulate3D(2D Feature Map)
activate Spatial
note right: Z = (f * b) / d
Spatial --> FSM: 3D Trajectory Coordinates
deactivate Spatial

FSM -> Physics: calculateBallistics(3D Trajectory)
activate Physics
Physics -> Logger: LOG_INFO("SVD Matrix Computed")
note right: Orthogonal Procrustes (R = USV^T)
Physics --> FSM: LaunchData
deactivate Physics

== Telemetry Transmission ==
FSM -> Network: transmitPayload(LaunchData)
activate Network
Network -> UI: TCP Push (Port 3111 or 49152)
deactivate Network

== Post-Shot Diagnostics ==
opt if --record flag is active
    FSM -> Recorder: saveSession(Flight Frames)
    activate Recorder
    note right: Saves to disk AFTER\nthe TCP transmission
    Recorder -> Logger: LOG_INFO("Flight frames saved to /logs")
    deactivate Recorder
end

FSM -> FSM: Reset State = ARMED
@enduml