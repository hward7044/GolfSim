# 11 — Documentation Drift

**Principle:** documentation describes the code that exists. A doc that describes a design that was never built (or was built and later replaced) is worse than no doc, because the next reader — human or AI — will build on it.
**Effort:** Small. **Risk:** Low. Do last, after the code sections settle.

---

## Findings

| ID | Document | Claims | Reality |
| :--- | :--- | :--- | :--- |
| W1 | [docs/Class.md](../Class.md) (246 lines), [ClassDiagram.puml](../../ClassDiagram.puml) (428 lines) | `OpticalGateTrigger`, `GlobalLogger` (with `LOG_INFO` macros), `Win32Serial`, "injects I2C strobe command" | All deleted or dead ([01](01_Dead_Code_And_Leftovers.md)). The diagrams predate the three trigger generations and the SSM template. |
| W2 | [docs/Sequence.md](../Sequence.md) | `GlobalLogger` participant; trigger → I2C flow | Same. |
| W3 | [docs/ReplayFeature.md](../ReplayFeature.md) | "high-speed (1000 FPS)"; `OpticalGateTrigger` implements `IDiagnosticProvider`; "Trigger Box (Orange)" overlay | 100 fps design; class deleted; the overlay never draws for the production trigger ([03 D3](03_Dependency_Inversion_Open_Closed.md)). |
| W4 | [docs/refactor/05](../refactor/05_Shot_Detection_IR_Safety.md) §4.1, [refactor README](../refactor/README.md) row 05 | "Hardware Timer CTC driver", "Timer1 hardware 50 µs clamp" | No Timer1 code; software `digitalRead` clamp ([10 F2](10_Firmware.md)). **Safety-relevant.** |
| W5 | [docs/refactor/03](../refactor/03_Hardware_Trigger_Handshake.md), README row 03 | "Continuous low-power strobing … eliminating USB trigger latency" | Firmware fires a 3-pulse train **per camera STROBE edge** — frame-synchronous, not continuous. That is actually the better design (it is what makes intra-frame timing exact, [07 T1](07_Data_Model_And_Time_Base.md)), but the doc should say so. |
| W6 | [ToolsAndLibraries.md:48](../ToolsAndLibraries.md#L48), [.agents skill](../../.agents/skills/camera-ball-tracker-analysis/SKILL.md) §4 | "structured logging subsystem … `GlobalLogger`"; "update instantiation in `src/main.cpp`, rebuild with MSVC (hard-coded `C:\Github\GolfSim` path)" | `spdlog` directly; Linux build exists; parameters should come from a params struct ([06 C1](06_Configuration_And_Magic_Numbers.md)) not `main.cpp` literals. |

Also: `docs/refactor/02`'s timing table ("Solves after Frame 2 across 6 pulses") is correct *as a design* but the code does not implement the inter-frame time gap it implies ([07 T1](07_Data_Model_And_Time_Base.md)) — worth a note in 02 once 07 lands.

---

## Plan

1. **Delete** `Class.md`, `Sequence.md`, `ClassDiagram.puml`. Hand-maintained UML has drifted three times; it will drift again. If a class diagram is wanted, generate it (`clang-uml` or Doxygen + Graphviz) from a CMake target so it cannot lie.
2. **Rewrite** `ReplayFeature.md` after [03 D3](03_Dependency_Inversion_Open_Closed.md)/[04 S2](04_Single_Responsibility.md) change what is drawn; until then, remove the two false sentences.
3. **Correct** 05 §4.1 and the README rows for 03 and 05 to describe the firmware as it is; re-correct after [10](10_Firmware.md) is implemented.
4. **Update** the skill: parameters live in the params struct; build on either platform per `BuildInstructions.md`; no absolute Windows path.
5. **Adopt a rule** (add to `BuildInstructions.md` or a `CONTRIBUTING.md`): a refactor doc's status column may say COMPLETED only when its verification section has been run; design docs that describe hardware behaviour cite the firmware function that implements it.

## Verification
- `grep -rn -i "OpticalGateTrigger\|GlobalLogger\|Win32Serial\|1000 FPS\|Timer1" docs README.md .agents` → only historical mentions inside `docs/refactor2/` and the git-history narrative in `docs/refactor/`.
- Every claim in `docs/refactor/README.md`'s status table has a code or scope reference.

**Depends on:** 01, 03, 10 (for the final wording). The deletions in step 1 can happen any time.
