# 02 — Interface Segregation

**Principle:** ISP — an interface should contain only what every implementer can honour and every caller needs. LSP — a derived class must be usable through the base without surprises.
**Effort:** Small–Medium. **Risk:** Low.

---

## Findings

### I1 — `IUsbVideoDriver` carries a method no implementation can honour
[IUsbVideoDriver.hpp:15-17](../../include/HAL/IUsbVideoDriver.hpp#L15-L17): `injectImmediateRegisterWrite(reg, value)` is declared "CRITICAL … microsecond I2C register updates". Both drivers implement it as an empty body with a comment saying the hardware cannot do it. The interface promises a capability that the abstraction does not have, so any future caller is silently a no-op.

**Fix:** delete it (also [01 A6](01_Dead_Code_And_Leftovers.md)). If register access ever exists, it is a separate `IRegisterAccess` that a driver *may* implement; callers `dynamic_cast` or query for it.

### I2 — Two abstractions for the same role: `ITriggerDetector` vs `CTriggerDetector`
[ITriggerDetector.hpp](../../include/Math/ITriggerDetector.hpp) is a virtual interface. [SessionStateMachine.hpp:31-35](../../include/Orchestration/SessionStateMachine.hpp#L31-L35) does **not** use it — it defines a duck-typed concept `CTriggerDetector` that only requires `checkTrigger` and `reset`. Meanwhile the other four concepts (`CComputerVision` …) are `std::derived_from<Interface>`. Result: the trigger is the one component that can be a mock struct with no base class ([tests/Mocks.hpp](../../tests/Mocks.hpp) `MockStrobeTrigger`), while everything else must inherit. Two rules for five slots.

**Fix:** pick one. Recommended: concepts everywhere *or* interfaces everywhere, not both. Given [03](03_Dependency_Inversion_Open_Closed.md) argues for virtual interfaces at the SSM boundary, make `CTriggerDetector = std::derived_from<ITriggerDetector>` and give the mock the base class.

### I3 — `ITriggerDetector::checkOpticalGate` is a non-virtual helper that a subclass hides
[ITriggerDetector.hpp:11-14](../../include/Math/ITriggerDetector.hpp#L11-L14) defines `checkOpticalGate(frame)` as a *non-virtual* forwarding helper. [BallPresenceTrigger.hpp:58](../../include/Math/BallPresenceTrigger.hpp#L58) declares its own `checkOpticalGate` with the same signature — name hiding, not overriding. Calling it through an `ITriggerDetector&` runs the base helper (which calls `checkTrigger(f, f)`), calling it on the concrete type runs the real one. LSP violation; it only works because nothing calls it through the base.

**Fix:** delete the base helper ("legacy single-frame … " by its own comment). Goes away with [01 A1](01_Dead_Code_And_Leftovers.md).

### I4 — `IDiagnosticProvider` exists but the consumer duck-types instead
[IDiagnosticProvider.hpp](../../include/Diagnostics/IDiagnosticProvider.hpp) declares `getLatestDiagnostics()`. [SessionStateMachine.hpp:93-99](../../include/Orchestration/SessionStateMachine.hpp#L93-L99) `getTelemetry()` ignores the interface and uses `if constexpr (requires { obj.getLatestDiagnostics(); })`. So a component can implement the interface and not be diagnosable through it, or not implement it and still be. The interface is decoration.

**Fix:** the shape of diagnostics is the real problem ([03 D3](03_Dependency_Inversion_Open_Closed.md) — stringly typed). Whatever replaces it, use one mechanism: either the interface (`dynamic_cast<const IDiagnosticProvider*>`) or the concept, not both. Prefer a typed `std::optional<Diagnostics>` return from the call that produced them (see 03).

### I5 — `ICameraNode` lacks `setExposure`, so the debug viewer holds concrete types
[OV9281CameraNode.hpp:14](../../include/Camera/OV9281CameraNode.hpp#L14) has `setExposure(int)` outside the interface. [main.cpp](../../src/main.cpp) `runCameraDebugViewer` therefore keeps `std::shared_ptr<OV9281CameraNode> nodeL, nodeR` next to the `HardwareSyncedCameraSystem` that also owns them, just to call `setExposure`. Two owners, two views of the same object, and the viewer cannot work with any other `ICameraNode`.

**Fix:** either `ICameraNode::setExposure(std::chrono::microseconds)` (every camera has an exposure) or a `ICameraSystem::forEachNode(...)`. Then the viewer holds only the system.

### I6 — `ICameraNode::getRole()` is not `const`
[ICameraNode.hpp:12](../../include/Camera/ICameraNode.hpp#L12). A getter that mutates nothing, declared non-const, forces every holder to be non-const. Same for `IBufferManager::push(T&)` taking a non-const ref only because it swaps — that one is deliberate; document it.

**Fix:** `virtual CameraRole getRole() const = 0;`.

### I7 — `ICameraSystem::captureSynchronizedFrames` promises synchronisation it does not perform
[HardwareSyncedCameraSystem.cpp:3-17](../../src/Camera/HardwareSyncedCameraSystem.cpp#L3-L17) grabs left then right, sequentially and blocking. The two frames may be up to one frame period apart; nothing checks. The name and the doc-comment ("Capture synchronized frames") describe a guarantee the class does not have. With a strobe fired from *one* camera's STROBE pin ([firmware](../../firmware/strobe_controller/strobe_controller.ino) `STROBE_INPUT_PIN`), the other camera's exposure may not even overlap the pulses.

**Fix (interface part):** return a `std::optional<FrameSet>`-style result that carries both timestamps and a `bool synchronized` (|Δt| ≤ tolerance); rename to `captureFramePair`. The policy part (reject / retry / warn) belongs in [08](08_Ownership_Lifetime_Concurrency.md).

---

## Plan

1. I1, I3 — delete with [01](01_Dead_Code_And_Leftovers.md).
2. I6 — one-line change, do with I5.
3. I5 — add `setExposure` to `ICameraNode`; `OV9281CameraNode` overrides; viewer drops the concrete pointers.
4. I2, I4 — decide together with [03](03_Dependency_Inversion_Open_Closed.md), which changes how the SSM sees its components.
5. I7 — signature change with [08](08_Ownership_Lifetime_Concurrency.md).

## Verification
- Build + `ctest`.
- `grep -rn "shared_ptr<OV9281CameraNode>" src` → nothing after I5.
- No `requires { obj.getLatestDiagnostics(); }` left after I4.

**Depends on:** 01 for I1/I3. **Feeds:** 03, 08.
