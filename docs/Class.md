@startuml
skinparam classAttributeIconSize 0
skinparam packageStyle rectangle
top to bottom direction

' =============================================================
' ROW 1: CORE SENSING TOPOGRAPHY
' =============================================================
package "1. Hardware Abstraction Layer (HAL)" as p1 {
    interface IUsbVideoDriver <<Interface>> {
        +grabRawFrame() : cv::Mat
        +setHardwareExposure(microseconds: int) : void
        +injectImmediateRegisterWrite(reg: uint16_t, value: uint8_t) : void
    }
    note right of IUsbVideoDriver::injectImmediateRegisterWrite
        CRITICAL: Bypasses video stream queue to send
        direct USB Control Transfers (XU commands).
        Allows microsecond I2C register updates.
    end note

    class V4L2Driver {
        -fileDescriptor : int
        +grabRawFrame() : cv::Mat
        +injectImmediateRegisterWrite(reg: uint16_t, value: uint8_t) : void
    }
    note top of V4L2Driver : Compiled via #ifdef __linux__

    class MediaFoundationDriver {
        -source : IMFMediaSource
        +grabRawFrame() : cv::Mat
        +injectImmediateRegisterWrite(reg: uint16_t, value: uint8_t) : void
    }
    note top of MediaFoundationDriver : Compiled via #ifdef _WIN32

    IUsbVideoDriver <|.. V4L2Driver
    IUsbVideoDriver <|.. MediaFoundationDriver
}

package "2. Camera Subsystem" as p2 {
    enum CameraRole {
        STEREO_LEFT
        STEREO_RIGHT
    }

    class FrameSet {
        +timestamp : uint64_t
        -frames : std::unordered_map<CameraRole, cv::Mat>
        +getFrame(role: CameraRole) : cv::Mat
    }

    interface ICameraNode <<Interface>> {
        +captureFrame() : cv::Mat
        +enableHardwareStrobeMode() : void
        +getRole() : CameraRole
    }

    class OV9281CameraNode {
        -usbDriver : unique_ptr<IUsbVideoDriver>
        +captureFrame() : cv::Mat
        +enableHardwareStrobeMode() : void
    }
    note right of OV9281CameraNode::enableHardwareStrobeMode
        Triggers driver->injectImmediateRegisterWrite()
        to pull physical STROBE pin high on chip.
    end note

    class PlaybackCameraNode {
        -mockFilePath : string
        +captureFrame() : cv::Mat
        +enableHardwareStrobeMode() : void
    }

    interface ICameraSystem <<Interface>> {
        +captureSynchronizedFrames() : FrameSet
    }

    class HardwareSyncedCameraSystem {
        -cameras : std::vector<shared_ptr<ICameraNode>>
        +captureSynchronizedFrames() : FrameSet
    }

    ICameraNode <|.. OV9281CameraNode
    ICameraNode <|.. PlaybackCameraNode
    ICameraSystem <|.. HardwareSyncedCameraSystem
    HardwareSyncedCameraSystem o-- ICameraNode
}

' =============================================================
' ROW 2: TELEMETRY & DIAGNOSTICS DETACHED
' =============================================================
package "3. Diagnostics & Telemetry" as p3 {
    class GlobalLogger <<Singleton>> {
        -asyncQueue : spdlog::thread_pool
        -{static} instance : GlobalLogger*
        +{static} getInstance() : GlobalLogger*
        +logInfo(msg: string) : void
        +logError(msg: string) : void
        +setLevel(level: LogLevel) : void
    }
    note right of GlobalLogger : Accessed via pragmatic macro:\nLOG_INFO("Matrix: {}", mat);

    class FlightRecorder {
        -outputDirectory : string
        +saveSession<FrameRange>(frames: FrameRange) : void
    }
    note right of FlightRecorder
        T.3 — express containers & ranges\n
        template<typename FrameRange>\n
        requires std::ranges::range<FrameRange>\n
        void saveSession(FrameRange&& frames);\n
        Accepts span, deque, ring-buffer view, etc.
    end note
}

' =============================================================
' ROW 3: DETAILED COMPUTE HOT-PATH (ISOLATED ROW FOR WIDTH)
' =============================================================
package "4. Segregated Processing & Math Subsystem" as p4 {
    class "LaunchData<AngleUnit, SpeedUnit>" as LaunchData {
        +ballSpeed : SpeedUnit
        +verticalLaunchAngle : AngleUnit
        +horizontalLaunchAngle : AngleUnit
        +spinRPM : double
        +spinAxis : Eigen::Vector3d
    }
    note right of LaunchData
        T.4 — express syntax tree / strong types\n
        template<typename AngleUnit,\n
                 typename SpeedUnit>\n
        struct LaunchData { ... };\n
        Units enforced at compile time;\n
        implicit conversions are errors.
    end note

    interface "IBufferManager<T>" as IBufferManager <<Interface>> {
        +push(item: T) : void
        +pop() : std::optional<T>
    }

    class "AtomicRingBuffer<T, N>" as AtomicRingBuffer {
        -buffer : std::array<T, N>
        +push(item: T) : void
        +pop() : std::optional<T>
    }
    note right of AtomicRingBuffer
        T.1 — raise level of abstraction\n
        template<typename T, std::size_t N>\n
        class AtomicRingBuffer\n
          : public IBufferManager<T> { ... };\n
        Caches burst of IR strobe frames lock-free.
    end note

    interface ITriggerDetector <<Interface>>
    
    class OpticalGateTrigger {
        -gateROI : cv::Rect
        -backgroundRef : cv::Mat
        +checkOpticalGate(currentFrame: cv::Mat) : bool
    }
    note right of OpticalGateTrigger
        Executes lightweight cv::absdiff exclusively
        on the downrange 3-inch bounding box.
    end note

    interface IComputerVision <<Interface>>
    class OpenCVMomentsTracker

    interface ISpatialSolver <<Interface>>
    class StereoTriangulator

    interface IKinematicsSolver <<Interface>>
    class EigenBallisticsEngine

    interface INetworkTransmitter <<Interface>>
    class TcpJsonTransmitter

    IBufferManager <|.. AtomicRingBuffer
    ITriggerDetector <|.. OpticalGateTrigger
    IComputerVision <|.. OpenCVMomentsTracker
    ISpatialSolver <|.. StereoTriangulator
    IKinematicsSolver <|.. EigenBallisticsEngine
    INetworkTransmitter <|.. TcpJsonTransmitter
}

' =============================================================
' ROW 4: SYSTEM ORCHESTRATION ANCHOR
' =============================================================
package "5. Segregated Orchestration" as p5 {
    class "SessionStateMachine\n<Trigger, Vision, Spatial, Kinematics, Net>" as SessionStateMachine {
        -trigger : Trigger
        -vision : Vision
        -spatial : Spatial
        -kinematics : Kinematics
        -network : Net
        -recorder : FlightRecorder
        +processNextFrame(set: FrameSet) : void
    }
    note right of SessionStateMachine
        T.2 — algorithms for many argument types\n
        T.5 — combine generic + OO techniques\n\n
        template<ITriggerDetector  Trigger,\n
                 IComputerVision   Vision,\n
                 ISpatialSolver    Spatial,\n
                 IKinematicsSolver Kinematics,\n
                 INetworkTransmitter Net>\n
        class SessionStateMachine { ... };\n\n
        Static dispatch on hot path;\n
        concept constraints preserve\n
        interface contracts for testing.
    end note

    class ThreadManager {
        -cameraSystem : ICameraSystem
        -buffer : IBufferManager<FrameSet>
        -stateMachine : SessionStateMachine
        +startProducerThread() : void
        +startConsumerThread() : void
    }
    note bottom of ThreadManager
        Producer Thread: Runs low-power Idle Loop -> 
        trips OpticalGateTrigger -> injects I2C Strobe command ->
        pushes rapid burst of FrameSets to AtomicRingBuffer.
        
        Consumer Thread: Pops burst FrameSets -> passes to
        SessionStateMachine -> transmits payload < 15ms.
    end note

    ThreadManager o-- ICameraSystem
    ThreadManager o-- IBufferManager
    ThreadManager o-- SessionStateMachine
    SessionStateMachine o-- FlightRecorder

    SessionStateMachine ..> GlobalLogger : <<Uses>>
    ThreadManager ..> GlobalLogger : <<Uses>>
    EigenBallisticsEngine ..> GlobalLogger : <<Uses>>
}

' =============================================================
' INVISIBLE VERTICAL ALIGNMENT CONSTRAINTS
' Stacks p1/p2 on Row 1, p3 on Row 2, p4 completely isolated 
' on Row 3, and p5 on Row 4.
' =============================================================
p1 -[hidden]down-> p3
p3 -[hidden]down-> p4
p4 -[hidden]down-> p5

@enduml