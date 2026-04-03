const PORTAL_STEERING_GEAR_RATIO = 12.5;
const PORTAL_ENCODER_COUNT_PER_STEERING_DEG = (PORTAL_STEERING_GEAR_RATIO * 48000) / 360;
const PORTAL_PULSE_PER_STEERING_DEG = PORTAL_STEERING_GEAR_RATIO / 0.003;

const portalData = {
  kpis: [
    { label: "Loop period", value: "1 ms", note: "SysTick based control loop" },
    { label: "Runtime split", value: "main -> app", note: "boot entry and app supervisor are now separated" },
    { label: "Encoder mode", value: "TIM2 + virtual option", note: "PA0/PB3 real path with optional pulse-integrated bench feedback" },
    { label: "Bench default", value: "ENCDBG ON", note: "keyboard bench + real TIM2 diag, virtual feedback OFF, Doc/putty live viewer와 MATLAB/Simulink 적용 계획 문서가 준비돼 있다" }
  ],
  latency: [
    { name: "Sense", avg: 3.344, p99: 3.372 },
    { name: "Control", avg: 3.594, p99: 3.594 },
    { name: "Actuate", avg: 7.839, p99: 7.844 },
    { name: "Comms", avg: 2.233, p99: 2.289 }
  ],
  actualTrace: {
    labels: ["112.17", "112.27", "112.37", "112.47", "112.57"],
    target: [40, 40, 40, 40, 40],
    current: [43.707, 43.699, 43.674, 43.648, 43.623]
  },
  evaluation: {
    score: 72,
    verdict: "runtime 구조와 telemetry는 설명 가능하지만, 현재 bench 기준 최종 병목은 real encoder truth와 startup safety closure다.",
    summary:
      "현업 관점에서 이 프로젝트는 단순한 모터 데모가 아니라 센서, 제어, 액추에이터, 로그, 문서를 한 프레임으로 묶은 steering controller baseline으로 설명할 수 있다. 최근 baseline은 `main.c -> app_runtime.c` 분리, `position_control_diag.c` / `position_control_safety.c` 분리, `TIM2(PA0/PB3)` encoder path, real `[ENCDBG]`와 optional virtual feedback 경로까지 갖췄다. 다만 production-complete steering subsystem 기준으로는 real encoder truth, startup auto-enable 제거, watchdog 정책 정렬이 아직 남아 있어 현재 평점은 약 72%가 적절하다.",
    interviewer:
      "면접에서는 '이미 완성된 조향 제어기'보다 '구조와 증거 체계를 갖춘 bring-up baseline'으로 설명하는 편이 더 강하다. 특히 command lifecycle, pulse status, real TIM2 encoder diag, virtual feedback bench path를 분리해서 설명하면 설계 감각을 보여주기 좋다. 대신 지금은 real encoder truth가 아직 authoritative하지 않고 startup auto-enable도 남아 있다는 점을 같이 말해야 신뢰가 높아진다.",
    categories: [
      {
        name: "Architecture / Readability",
        score: 91,
        detail: "main.c를 얇은 부트 엔트리로 남기고 app_runtime.c와 position_control_diag.c로 책임을 분리하면서 구조가 한 단계 정리됐다. 다음 단계는 app_runtime 안의 bench console과 telemetry를 다시 쪼개는 것이다."
      },
      {
        name: "Observability / Evidence",
        score: 90,
        detail: "CSV, keyboard snapshot, command lifecycle log, latency batch, portal, change log에 더해 Doc/putty live viewer와 bridge launcher까지 이어져 있어 분석 가능한 증거 체계가 강하다. 특히 diagnostic 계층 분리로 상태 문자열과 debug var 경로가 더 읽기 쉬워졌다."
      },
      {
        name: "Actuator Interface",
        score: 76,
        detail: "requested/applied Hz, reverse guard, direction polarity macro가 추가되며 actuator 계약이 좋아졌다. 다만 real motion proof는 encoder truth와 함께 다시 닫아야 한다."
      },
      {
        name: "Sensor Truth",
        score: 49,
        detail: "현재 sensor path는 `TIM2(PA0/PB3)` real path와 optional virtual feedback path로 나뉜다. 구조는 설명 가능하지만, commanded motion 기준 authoritative real truth는 아직 닫히지 않았다."
      },
      {
        name: "Safety / Startup / Fail-safe",
        score: 54,
        detail: "ESTOP, timeout, lifecycle trace는 존재한다. 반면 startup auto-enable, latched fault, clear policy, cross-check fault는 아직 미완성이다."
      },
      {
        name: "Verification Closure",
        score: 69,
        detail: "명령 단위 추적과 pulse/encoder 상태 관찰은 좋아졌지만, 드라이브가 입력 pulse를 실제 motion으로 받아들이는 최종 증거 패키지가 아직 없다."
      }
    ],
    reasons: [
      {
        title: "구조와 증거 체계는 분명히 좋아졌다",
        detail: "command lifecycle, encoder unwrap, pulse status, latency evidence, portal 동기화에 더해 `main.c -> app_runtime.c` 분리와 `position_control_diag.c` 분리가 들어가면서 현재 코드 상태를 설명하기 쉬워졌다."
      },
      {
        title: "현재 병목이 더 구체적으로 드러났다",
        detail: "현재는 software pulse path와 controller 구조는 설명 가능하고, 남은 핵심 병목이 real encoder truth와 hardware signal integrity 쪽이라는 점이 더 분명해졌다."
      },
      {
        title: "startup safety와 fault policy는 아직 설명형 수준이다",
        detail: "구조 분리는 좋아졌지만 boot-time auto-enable이 남아 있고, fault latch / clear policy / stale sensor taxonomy가 아직 정식 계약으로 닫히지 않았다."
      },
      {
        title: "production steering claim에는 마지막 motion closure가 필요하다",
        detail: "PE9 pulse와 encoder unwrap만으로는 충분하지 않다. drive가 input pulse를 받아 실제로 움직였다는 반복 가능한 evidence pack이 있어야 100%에 가깝다."
      }
    ],
    to100: [
      {
        title: "Real encoder chain을 bench에서 닫아야 한다",
        detail: "`TIM2(PA0/PB3)`, `[ENCDBG]`, scope waveform, line receiver output을 같은 장면으로 묶어 authoritative real feedback을 증명해야 한다."
      },
      {
        title: "Startup / Arm contract를 만들어야 한다",
        detail: "INIT -> READY -> ARMED -> RUN -> ESTOP_LATCH 형태로 startup auto-enable을 제거하고, readiness 확인 전에는 enable되지 않도록 바꿔야 한다."
      },
      {
        title: "Fault latch와 clear policy를 정식화해야 한다",
        detail: "ESTOP, tracking limit, timeout, stale sensor, implausible motion을 reason code와 clear 절차까지 포함해 운영 계약으로 정리해야 한다."
      },
      {
        title: "Actuator contract를 하나로 통일해야 한다",
        detail: "`constants.h`의 1 MHz 상수와 `pulse_control.c`의 100 kHz clamp를 하나의 current runtime contract로 정리하고, 이후 design target은 분리해서 문서화해야 한다."
      },
      {
        title: "Async logging으로 strict timing 검증을 분리해야 한다",
        detail: "blocking UART 대신 DMA + ring buffer를 넣어 motion bring-up과 strict latency measurement를 서로 덜 방해하게 만들어야 한다."
      }
    ]
  },
  analysisFlow: {
    steps: [
      {
        step: "Step 1",
        title: "Upper command와 단위를 먼저 맞춘다",
        summary: "상위 입력은 `steering_deg`이고, app runtime에서 `motor_deg`로 변환되어 controller로 들어간다. 먼저 gear ratio와 pulse conversion contract가 같은 기준을 쓰는지 본다.",
        vars: [
          "AutoDrive_Packet_t.steering_angle",
          "STEERING_GEAR_RATIO = 12.5",
          "SteeringDegToMotorDeg()",
          "DEG_PER_PULSE = 0.003 motor_deg"
        ],
        files: ["ethernet_communication.c", "constants.h", "app_runtime.c"],
        status: "적용됨. 외부 steering_deg -> 내부 motor_deg 경로는 정리됐지만 naming consistency는 더 다듬을 수 있다."
      },
      {
        step: "Step 2",
        title: "센서 truth를 먼저 확인한다",
        summary: "현재는 `enc_raw`만 보지 않고 `enc_cnt`, `GetAngleDeg()`, `[ENCDBG] cnt/delta/A/B`를 함께 봐야 한다. 코드에는 `TIM2(PA0/PB3)` real path와 optional virtual feedback path가 같이 존재한다.",
        vars: [
          "EncoderReader_GetRawCounter()",
          "EncoderReader_GetCount()",
          "EncoderReader_GetAngleDeg()",
          "ADC_Pot_GetAngle()"
        ],
        files: ["encoder_reader.c", "adc_potentiometer.c", "position_control.c"],
        status: "부분 적용. TIM2 real path와 virtual bench path는 존재하지만, commanded motion 기준 authoritative sensor truth closure는 아직 남아 있다."
      },
      {
        step: "Step 3",
        title: "error -> output -> direction -> drive monitor를 비교한다",
        summary: "현재는 state.output만 보는 것이 아니라 requested/applied Hz, reverse guard, direction, real `[ENCDBG]`와 scope를 함께 봐야 한다. 그래야 motion bottleneck이 controller 쪽인지 real sensor truth 쪽인지 분리된다.",
        vars: [
          "state.target_angle / current_angle / error",
          "PulseControl_SetFrequency()",
          "PulseControl_GetStatus()",
          "PE9 pulse / PE10 direction / [ENCDBG] / scope"
        ],
        files: ["position_control.c", "pulse_control.c", "app_runtime.c"],
        status: "적용됨. reverse guard와 pulse status는 들어갔고, 현재 bench 병목은 real hardware closure 쪽으로 좁혀졌다."
      },
      {
        step: "Step 4",
        title: "lifecycle / timing / fail-safe trace를 붙인다",
        summary: "마지막에는 command lifecycle, CSV, latency batch를 함께 본다. 현재는 event trace가 좋아졌고 다음 단계는 richer fault taxonomy와 startup clear policy다.",
        vars: [
          "CSV_HEADER / CSV rows",
          "CMD_START / CMD_REACHED / CMD_TIMEOUT / CMD_ABORT / CMD_FAULT",
          "LATENCY_BATCH_BEGIN / LATENCY_STAGE / LATENCY_BATCH_END",
          "PositionControl_EmergencyStop()"
        ],
        files: ["app_runtime.c", "position_control.c", "position_control_diag.c", "latency_profiler.c"],
        status: "적용됨. evidence skeleton은 충분히 좋아졌고, 최근에는 runtime/diagnostic 분리로 trace path도 더 읽기 쉬워졌다."
      }
    ],
    phases: [
      {
        name: "Command In",
        description: "UDP 또는 keyboard에서 목표값과 mode를 받는다.",
        vars: ["steering_angle", "mode", "misc", "g_last_rx_tick"]
      },
      {
        name: "Unit Convert",
        description: "상위 steering_deg를 내부 motor_deg로 변환한다.",
        vars: ["STEERING_GEAR_RATIO", "SteeringDegToMotorDeg()", "MotorDegToSteeringDeg()"]
      },
      {
        name: "Sense",
        description: "TIM2 raw와 누적 count를 읽어 current angle을 만든다. bench에서는 optional virtual feedback도 선택할 수 있고, ADC는 아직 startup/homing 보조 경로다.",
        vars: ["enc_raw", "enc_cnt", "EncoderReader_GetAngleDeg()", "ADC_Pot_GetAngle()"]
      },
      {
        name: "Control",
        description: "오차 계산, 안전 체크, PID 계산을 1ms loop 안에서 수행한다.",
        vars: ["state.error", "PositionControl_CheckSafety()", "PID_Calculate()"]
      },
      {
        name: "Actuate",
        description: "signed output을 pulse_hz와 direction으로 바꿔 TIM1과 line-driver로 보낸다.",
        vars: ["requested_frequency_hz", "applied_frequency_hz", "reverse_guard_active", "PE9", "PE10"]
      },
      {
        name: "Evidence",
        description: "CSV, lifecycle event, latency batch, keyboard snapshot, debug vars로 현재 상태를 바깥에 남긴다.",
        vars: ["CSV rows", "CMD_*", "LATENCY_STAGE", "[KB][snapshot]", "dbg_*"]
      }
    ],
    logs: [
      {
        title: "CSV trace 예시",
        snippet:
          "CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc_cnt,enc_raw,req_hz,applied_hz,out_active,rev_guard,cmd_id,cmd_state,cmd_result\nCSV,196670,1,45.000,-0.604,45.604,10000,1,-966,31802,10000,10000,1,0,202,1,0",
        note: "현재 코드 기준 CSV는 command lifecycle과 pulse/encoder 상태를 한 줄에서 같이 보여준다. 단, virtual feedback이 켜지면 CSV의 `current/enc_*`는 real TIM2가 아니라 bench 적분값일 수 있으므로 `[ENCDBG]`를 같이 봐야 한다."
      },
      {
        title: "Latency batch 예시",
        snippet:
          "LATENCY_BATCH_BEGIN,seq=526,samples=2000,core_hz=180000000,deadline_miss=0\nLATENCY_STAGE,seq=526,name=Sense,count=2000,avg_us=0.928,p99_us=0.961,max_us=0.967\nLATENCY_STAGE,seq=526,name=Control,count=2000,avg_us=2.557,p99_us=2.583,max_us=2.590\nLATENCY_BATCH_END,seq=526",
        note: "historical latency evidence는 여전히 유효하다. 다만 strict measurement와 interactive UART logging을 같은 실험으로 섞어 해석하면 안 된다."
      },
      {
        title: "Lifecycle event 예시",
        snippet:
          "CMD_START,id=202,src=KEYBOARD,target_deg=45.000,target_motor_deg=562.500,start_ms=196650,start_deg=-0.604,start_error_deg=45.604\nCMD_TIMEOUT,id=202,end_ms=199651,elapsed_ms=3001,error_deg=45.604",
        note: "현재 lifecycle event는 충분히 명확하다. 다음 단계는 timeout / fault / stale sensor를 더 세분화해 drive-side 문제와 sensor-side 문제를 reason code로 구분하는 것이다."
      }
    ]
  },
  reqs: [
    {
      id: "REQ-P0-001",
      priority: "P0",
      owner: "Sensor / C",
      title: "unwrap count 장기 검증과 stale sensor 진단을 추가한다.",
      detail: "unwrap 자체는 구현됐으므로, 이제는 long-run wrap 구간과 commanded motion 대비 stale sensor를 판별하는 진단을 넣어야 한다."
    },
    {
      id: "REQ-P0-002",
      priority: "P0",
      owner: "Actuator / D",
      title: "real encoder chain과 signal integrity를 bench에서 닫는다.",
      detail: "`TIM2(PA0/PB3)`, line receiver output, scope waveform, `[ENCDBG]`를 한 세트로 비교해 authoritative real encoder truth를 닫아야 한다."
    },
    {
      id: "REQ-P0-003",
      priority: "P0",
      owner: "Startup / B",
      title: "boot-time auto enable을 제거하고 startup state machine을 만든다.",
      detail: "INIT -> HOMING -> READY -> ARMED -> RUN -> ESTOP_LATCH 구조로 정리하고, homing 성공 전 enable 금지를 강제한다."
    },
    {
      id: "REQ-P0-004",
      priority: "P0",
      owner: "Control / A",
      title: "fault latch / clear policy를 command lifecycle과 연결한다.",
      detail: "timeout, estop, tracking fault, stale sensor, startup inhibit를 reason code와 clear 절차까지 포함해 운영 계약으로 닫아야 한다."
    },
    {
      id: "REQ-P1-001",
      priority: "P1",
      owner: "Actuator / D",
      title: "pulse contract를 문서와 코드에서 하나로 통일한다.",
      detail: "`constants.h`의 1 MHz와 `pulse_control.c`의 100 kHz clamp를 current runtime contract와 future design target으로 분리해 정리해야 한다."
    },
    {
      id: "REQ-P1-002",
      priority: "P1",
      owner: "Sensor / C",
      title: "encoder-ADC cross-check와 implausible motion 진단을 넣는다.",
      detail: "commanded direction과 encoder delta 부호가 일정 시간 이상 맞지 않으면 sensor 또는 actuator fault 후보로 남겨야 한다."
    },
    {
      id: "REQ-P1-003",
      priority: "P1",
      owner: "Integration / E",
      title: "keyboard bench, UDP mode, timeout, ESTOP를 하나의 system contract로 묶는다.",
      detail: "현재는 keyboard mode 기본값과 UDP mode가 공존한다. mode transition과 recovery 조건을 같은 상태 모델로 정리해야 한다."
    },
    {
      id: "REQ-P1-004",
      priority: "P1",
      owner: "Validation / F",
      title: "CSV, latency, drive monitor, change log를 evidence pack으로 자동 연결한다.",
      detail: "실험 baseline, plots, raw logs, command lifecycle, drive monitor 값을 같은 태그로 묶으면 인수/포트폴리오 설명력이 높아진다."
    },
    {
      id: "REQ-P2-001",
      priority: "P2",
      owner: "Actuator / D",
      title: "direction reversal setup/hold를 scope evidence로 남긴다.",
      detail: "1 ms reverse guard가 실제 line-driver와 드라이브 입력에서 어떻게 보이는지 wave capture로 남겨야 한다."
    },
    {
      id: "REQ-P2-002",
      priority: "P2",
      owner: "Validation / F",
      title: "UART logging을 async path로 분리한다.",
      detail: "DMA + ring buffer로 바꾸어 motion bring-up과 strict timing 측정을 서로 덜 방해하게 해야 한다."
    }
  ],
  teams: [
    {
      owner: "Team A",
      title: "Motion Control & Fault Manager",
      scope: "position_control.c, lifecycle, timeout, fault reason",
      deliverable: "fault latch, clear policy, wrong-direction / stale sensor diagnostics",
      evidence: "state transition log, timeout case, fault injection trace",
      brief: "../members/position_control.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team B",
      title: "Startup / Homing / Relay Safety",
      scope: "homing.c, relay_control.c, startup gating",
      deliverable: "INIT -> READY -> ARMED -> RUN contract and recovery sequence",
      evidence: "startup GPIO trace, ready/arm log, homing fallback flow",
      brief: "../members/homing_relay.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team C",
      title: "Sensor Integrity",
      scope: "encoder_reader.c, adc_potentiometer.c, count/angle health",
      deliverable: "unwrap validation, stale detection, encoder-ADC cross-check",
      evidence: "long-run encoder log, mismatch detection trace, sign verification",
      brief: "../members/adc_encoder.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team D",
      title: "Pulse / Direction / Driver Timing",
      scope: "pulse_control.c, TIM1 contract, PF/PR path, drive monitor",
      deliverable: "requested/applied Hz contract, real encoder `[ENCDBG]` closure, reverse-guard wave proof",
      evidence: "scope capture, drive monitor table, commanded vs applied frequency report",
      brief: "../members/pulse_control.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team E",
      title: "Communication & System Integration",
      scope: "ethernet_communication.c, app_runtime.c mode transitions, timeout behavior",
      deliverable: "keyboard/UDP/system mode contract and recovery gating",
      evidence: "mode transition log, timeout fail-safe proof, packet-to-target trace",
      brief: "../members/ethernet_integration.html",
      briefLabel: "상세 할당서"
    },
    {
      owner: "Team F",
      title: "Verification & Tooling",
      scope: "latency_profiler.c, position_control_diag.c, debug_vars, plotting scripts, documentation portal",
      deliverable: "evidence automation, async logging support, portal maintenance, PuTTY live viewer and bridge upkeep",
      evidence: "latency report, PNG plots, portal snapshot, live viewer recording JSON, bridge launcher, submission-ready artifact set",
      brief: "../members/verification_tooling.html",
      briefLabel: "상세 할당서"
    }
  ],
  modules: [
    {
      id: "main",
      title: "main.c",
      subtitle: "CubeMX boot wrapper and top-level entrypoint",
      owner: "Integration / Runtime",
      inputs: ["HAL reset", "SystemClock_Config()", "MX_* peripheral initialization completion"],
      outputs: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "__io_putchar() UART bridge"],
      risks: ["blocking __io_putchar()", "CubeMX merge points still need discipline"],
      files: ["../../Core/Src/main.c", "../../Core/Inc/main.h"],
      doxygen: "../doxygen/html/main_8c_source.html",
      snippet: "MX_GPIO_Init();\\nMX_USART3_UART_Init();\\nMX_TIM1_Init();\\nMX_TIM2_Init();\\nMX_LWIP_Init();\\nAppRuntime_Init();\\nwhile (1) {\\n    AppRuntime_RunIteration();\\n}",
      notes: "최근 리팩터링 이후 main.c는 CubeMX 재생성 경계와 앱 진입점 역할만 남겼다. 실제 startup, super-loop, bench telemetry는 app_runtime.c가 맡는다."
    },
    {
      id: "runtime",
      title: "app_runtime.c",
      subtitle: "Application startup, super-loop services, bench console, CSV and DIAG",
      owner: "Integration / Runtime",
      inputs: ["interrupt_flag", "keyboard UART bytes", "EthComm mode/data", "PositionControl/Pulse/Encoder state"],
      outputs: ["PositionControl_Enable()/Update()", "PositionControl_SetTargetWithSource()", "CSV/DIAG logs", "IWDG refresh"],
      risks: ["startup auto-enable still present", "bench console and telemetry are still combined", "blocking UART logging"],
      files: ["../../Core/Src/app_runtime.c", "../../Core/Inc/app_runtime.h"],
      doxygen: null,
      snippet: "void AppRuntime_RunIteration(void) {\\n    MX_LWIP_Process();\\n    AppRuntime_KeyboardProcessInput();\\n    AppRuntime_ServiceUdpComms();\\n    if (interrupt_flag != 0U) {\\n        interrupt_flag = 0U;\\n        AppRuntime_ServiceFastTick();\\n    }\\n    AppRuntime_ServicePeriodicCsv();\\n    HAL_IWDG_Refresh(&hiwdg);\\n}",
      notes: "현재 앱 운영의 중심 파일이다. startup 시퀀스, keyboard bench, UDP mode transition, fast tick, telemetry가 한데 모여 있어 이후 `bench_console`과 `telemetry`로 더 쪼갤 후보이기도 하다."
    },
    {
      id: "position",
      title: "position_control.c",
      subtitle: "PID, safety check, lifecycle, emergency stop",
      owner: "Control / A",
      inputs: ["target_angle[motor_deg]", "current_angle from encoder", "dt from HAL_GetTick"],
      outputs: ["state.output -> PulseControl_SetFrequency()", "fault -> Relay_Emergency()"],
      risks: ["fault latch 부족", "startup contract 미완성", "HAL_GetTick 기반 dt 정밀도 한계"],
      files: ["../../Core/Src/position_control.c", "../../Core/Inc/position_control.h"],
      doxygen: "../doxygen/html/position__control_8c_source.html",
      snippet: "state.current_angle = EncoderReader_GetAngleDeg();\\nstate.error = state.target_angle - state.current_angle;\\nif (!PositionControl_CheckSafety()) {\\n    PositionControl_CommandFinish(CMD_FAULTED, fault_result, HAL_GetTick());\\n    PositionControl_EmergencyStop();\\n    return;\\n}\\nstate.output = PID_Calculate(state.error, dt);\\nPulseControl_SetFrequency((int32_t)state.output);",
      notes: "현재 lifecycle trace는 충분히 좋아졌다. 다음 핵심은 stale sensor와 startup gating을 같은 fault policy로 묶는 것이다."
    },
    {
      id: "positiondiag",
      title: "position_control_diag.c",
      subtitle: "Command string helpers, debug var mirroring, state summary",
      owner: "Validation / F",
      inputs: ["PositionControl_State_t", "CommandLifecycle_t", "control_enabled/mode/fault_flag"],
      outputs: ["dbg_* globals", "bench summary printf", "readable lifecycle/error strings"],
      risks: ["stats/callback APIs are still placeholder stubs", "printf path is still blocking"],
      files: ["../../Core/Src/position_control_diag.c", "../../Core/Inc/position_control_diag.h"],
      doxygen: null,
      snippet: "void PositionControlDiag_UpdateDebugVars(const PositionControl_State_t* state, ...) {\\n    dbg_enc_raw = (int32_t)EncoderReader_GetRawCounter();\\n    dbg_pos_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->current_angle));\\n    dbg_target_mdeg = PositionControlDiag_DegToMilliDeg(MotorDegToSteeringDeg(state->target_angle));\\n    dbg_fault_flags = PositionControlDiag_BuildDebugFaultFlags(...);\\n}",
      notes: "진단 책임을 따로 뺀 덕분에 position_control.c 핫패스가 더 얇아졌다. 현재는 문자열/출력 계층이고, 앞으로는 async telemetry adapter 후보가 될 수 있다."
    },
    {
      id: "encoder",
      title: "encoder_reader.c",
      subtitle: "TIM2 raw counter / optional virtual feedback -> motor angle",
      owner: "Sensor / C",
      inputs: ["TIM2 raw counter", "optional virtual feedback count"],
      outputs: ["current_angle[motor_deg]", "enc_cnt", "enc_raw"],
      risks: ["long-run wrap validation 미완료", "raw와 count 의미 혼동 가능", "commanded motion proof 미완료"],
      files: ["../../Core/Src/encoder_reader.c", "../../Core/Inc/encoder_reader.h"],
      doxygen: "../doxygen/html/encoder__reader_8c_source.html",
      snippet: "uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&ENCODER_TIMER);\\nint16_t delta = (int16_t)(raw - encoder_last_raw);\\nencoder_count += (int32_t)delta;\\nencoder_last_raw = raw;\\nreturn encoder_count - encoder_offset;",
      notes: "이제 encoder는 centered raw 직접 해석이 아니라 unwrap count를 사용한다. 현재 과제는 commanded motion 기준의 sensor truth closure다."
    },
    {
      id: "pulse",
      title: "pulse_control.c",
      subtitle: "PE9 pulse, PE10 direction, reverse guard, status API",
      owner: "Actuator / D",
      inputs: ["signed pulse_hz", "step count", "direction enum"],
      outputs: ["TIM1 PWM", "GPIO direction", "applied Hz status", "reverse_guard_active"],
      risks: ["100 kHz current clamp", "real encoder truth unresolved", "logging load와 bench 상호작용"],
      files: ["../../Core/Src/pulse_control.c", "../../Core/Inc/pulse_control.h"],
      doxygen: "../doxygen/html/pulse__control_8c_source.html",
      snippet: "requested_frequency_hz = freq_hz;\\nPulseControl_ServiceReverseGuard();\\nif (target_direction != current_direction) {\\n    PulseControl_BeginReverseGuard(target_direction, target_frequency_hz);\\n    return;\\n}\\nPulseControl_StartContinuousOutput(target_frequency_hz);",
      notes: "최근 변경으로 requested/applied Hz와 reverse guard를 볼 수 있게 됐다. 이제 남은 핵심은 drive-side input recognition을 bench에서 닫는 것이다."
    },
    {
      id: "comms",
      title: "ethernet_communication.c",
      subtitle: "UDP parsing, mode handling, steering unit conversion",
      owner: "Integration / E",
      inputs: ["UDP command text / packets"],
      outputs: ["SteerMode_t", "AutoDrive_Packet_t", "status text"],
      risks: ["binary CRC contract 미완료", "keyboard default와 UDP mode 통합 필요"],
      files: ["../../Core/Src/ethernet_communication.c", "../../Core/Inc/ethernet_communication.h"],
      doxygen: "../doxygen/html/ethernet__communication_8c_source.html",
      snippet: "if (g_current_mode == STEER_MODE_MANUAL) {\\n    g_latest_pkt.steering_angle = joy_to_deg(joy_y);\\n    g_new_data = true;\\n} else if (g_current_mode == STEER_MODE_ESTOP) {\\n    g_emergency_request = true;\\n}\\n...\\nif (pc_speed == SPEED_ESTOP_SENTINEL) {\\n    g_emergency_request = true;\\n}",
      notes: "외부 명령 계약은 steering_deg 기준으로 꽤 정리됐다. 현재 app_runtime.c가 이 파일의 mode/data를 소비해 runtime 상태전이를 수행한다."
    },
    {
      id: "homing",
      title: "relay_control.c + homing.c",
      subtitle: "Power/safety outputs and startup zero-reference skeleton",
      owner: "Safety / B",
      inputs: ["ADC angle", "startup event", "operator reset"],
      outputs: ["encoder offset", "SVON/EMG relay action"],
      risks: ["main runtime에 실제 gating 미통합", "failure clear path 약함"],
      files: ["../../Core/Src/homing.c", "../../Core/Src/relay_control.c"],
      doxygen: "../doxygen/html/homing_8c_source.html",
      snippet: "float pot_angle = ADC_Pot_GetAngle();\\nEncoderReader_Reset();\\nint32_t offset_count = (int32_t)(pot_angle * ENCODER_COUNT_PER_REV / FULL_ROTATION_DEG);\\nEncoderReader_SetOffset(offset_count);\\nhoming_status = HOMING_STATUS_COMPLETE;",
      notes: "현재는 startup safety의 skeleton이다. 실제 운영 계약으로 보려면 ready/arm gating과 recovery까지 묶어야 한다."
    },
    {
      id: "latency",
      title: "latency_profiler.c + debug_vars.h",
      subtitle: "Measured evidence and debug observability",
      owner: "Validation / F",
      inputs: ["DWT cycle counter", "debug globals", "1 ms deadline tick"],
      outputs: ["LATENCY_STAGE logs", "dbg_* watch variables", "plots"],
      risks: ["strict measurement session과 interactive session 분리 필요"],
      files: ["../../Core/Src/latency_profiler.c", "../../Core/Inc/debug_vars.h"],
      doxygen: "../doxygen/html/latency__profiler_8c_source.html",
      snippet: "dt = DWT->CYCCNT - buf->start_cycle;\\nbuf->samples[buf->sample_count] = dt;\\n...\\nout_stats->avg_us = cycles_to_us(avg_cycles);\\nout_stats->p99_us = cycles_to_us(p99_cycles);\\nout_stats->max_us = cycles_to_us(buf->max_cycles);",
      notes: "이 프로젝트의 강점은 코드뿐 아니라 evidence pipeline이다. 현재는 motion bring-up과 timing measurement 조건을 분리해 설명하는 것이 중요하다."
    }
  ],
  runtimeAtlas: [
    {
      title: "main.c",
      file: "Core/Src/main.c",
      role: "CubeMX가 관리하는 초기화 코드와 앱 런타임 호출 경계를 유지한다.",
      receives: ["HAL reset + clock init", "MX_GPIO/MX_TIM/MX_LWIP 초기화 완료"],
      sends: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "USART3 기반 __io_putchar()"],
      functions: ["main()", "SystemClock_Config()", "__io_putchar()"],
      variables: ["사용자 static state 거의 없음", "UART 리다이렉션은 huart3에 의존"]
    },
    {
      title: "app_runtime.c",
      file: "Core/Src/app_runtime.c",
      role: "실제 애플리케이션 startup과 super-loop를 운영하는 통합 supervisor다.",
      receives: ["interrupt_flag", "keyboard UART bytes", "EthComm latest packet / mode", "PositionControl/Pulse/Encoder 상태"],
      sends: ["PositionControl_Update()", "PositionControl_SetTargetWithSource()", "CSV/DIAG/UART 로그", "IWDG refresh"],
      functions: ["AppRuntime_Init()", "AppRuntime_RunIteration()", "AppRuntime_ServiceFastTick()", "AppRuntime_ServiceUdpComms()"],
      variables: ["g_debug_print_divider", "g_latency_report_seq", "g_keyboard_target_steer_deg", "g_periodic_csv_enabled"]
    },
    {
      title: "ethernet_communication.c",
      file: "Core/Src/ethernet_communication.c",
      role: "UDP 패킷을 runtime이 소비할 수 있는 mode/data 상태로 변환한다.",
      receives: ["LwIP UDP payload", "manual joy_y / PC steer packet", "service text command"],
      sends: ["AutoDrive_Packet_t", "SteerMode_t", "emergency request flag", "last_rx_tick"],
      functions: ["EthComm_Init()", "EthComm_UDP_Init()", "EthComm_HasNewData()", "EthComm_GetLatestData()"],
      variables: ["g_latest_pkt", "g_current_mode", "g_emergency_request", "g_last_rx_tick"]
    },
    {
      title: "position_control.c",
      file: "Core/Src/position_control.c",
      role: "PID 계산, lifecycle 상태전이, safety check, ESTOP를 수행하는 제어 코어다.",
      receives: ["target_angle[motor_deg]", "EncoderReader current angle", "HAL_GetTick() based dt"],
      sends: ["PulseControl_SetFrequency()", "Relay_Emergency()", "CommandLifecycle_t update"],
      functions: ["PositionControl_Update()", "PositionControl_SetTargetWithSource()", "PositionControl_Enable()", "PositionControl_EmergencyStop()"],
      variables: ["state", "command_lifecycle", "pending_command_source", "fault_flag"]
    },
    {
      title: "position_control_diag.c",
      file: "Core/Src/position_control_diag.c",
      role: "제어 코어에서 분리된 diagnostic helper 레이어다.",
      receives: ["PositionControl_State_t snapshot", "command lifecycle", "control mode / fault info"],
      sends: ["dbg_* globals", "상태 요약 printf", "readable enum labels"],
      functions: ["PositionControlDiag_UpdateDebugVars()", "PositionControlDiag_PrintStateSummary()", "PositionControlDiag_CommandStateString()"],
      variables: ["diag_level", "stats", "dbg_enc_raw", "dbg_fault_flags"]
    },
    {
      title: "encoder_reader.c",
      file: "Core/Src/encoder_reader.c",
      role: "TIM2 raw counter를 unwrap count와 motor angle로 바꾸고, bench에서는 optional virtual feedback도 받을 수 있다.",
      receives: ["TIM2->CNT raw counter", "virtual feedback count", "offset_count"],
      sends: ["current motor_deg", "enc_cnt", "enc_raw"],
      functions: ["EncoderReader_Init()", "EncoderReader_GetAngleDeg()", "EncoderReader_GetCount()", "EncoderReader_SetOffset()"],
      variables: ["encoder_last_raw", "encoder_count", "encoder_offset", "encoder_initialized"]
    },
    {
      title: "pulse_control.c",
      file: "Core/Src/pulse_control.c",
      role: "signed pulse_hz를 TIM1 PWM과 direction GPIO로 바꾸는 액추에이터 인터페이스다.",
      receives: ["signed pulse_hz", "step count", "target direction"],
      sends: ["PE9 pulse", "PE10 direction", "requested/applied Hz status", "reverse guard state"],
      functions: ["PulseControl_SetFrequency()", "PulseControl_SendSteps()", "PulseControl_Stop()", "PulseControl_GetStatus()"],
      variables: ["requested_frequency_hz", "applied_frequency_hz", "pending_direction", "pending_frequency_hz", "remaining_steps"]
    },
    {
      title: "relay_control.c + homing.c",
      file: "Core/Src/relay_control.c / Core/Src/homing.c",
      role: "SVON/EMG 릴레이 출력과 startup zero-reference skeleton을 담당한다.",
      receives: ["operator reset", "ADC absolute angle", "startup event"],
      sends: ["Relay_ServoOn/Off()", "Relay_Emergency()", "encoder offset"],
      functions: ["Relay_Init()", "Relay_ServoOn()", "Relay_Emergency()", "Homing_Start()"],
      variables: ["homing_status", "offset_count", "GPIO relay state"]
    }
  ]
};

function makeEl(tag, className, text) {
  const el = document.createElement(tag);
  if (className) el.className = className;
  if (text !== undefined) el.textContent = text;
  return el;
}

function renderKpis() {
  const host = document.getElementById("kpi-strip");
  portalData.kpis.forEach((kpi) => {
    const card = makeEl("article", "kpi-card");
    card.append(makeEl("span", "metric-label", kpi.label));
    card.append(makeEl("strong", "", kpi.value));
    card.append(makeEl("p", "caption", kpi.note));
    host.append(card);
  });
}

function renderLatencyBars() {
  const host = document.getElementById("latency-bars");
  const max = Math.max(...portalData.latency.map((stage) => stage.p99));
  portalData.latency.forEach((stage) => {
    const row = makeEl("div", "latency-row");
    row.append(makeEl("strong", "", stage.name));
    const meter = makeEl("div", "latency-meter");
    const fill = makeEl("div", "latency-fill");
    fill.style.width = `${(stage.p99 / max) * 100}%`;
    meter.append(fill);
    row.append(meter);
    row.append(makeEl("span", "caption", `avg ${stage.avg.toFixed(3)} / p99 ${stage.p99.toFixed(3)} us`));
    host.append(row);
  });
}

function drawTrace(svgId, labels, target, current, options = {}) {
  const svg = document.getElementById(svgId);
  const width = 640;
  const height = svgId === "ideal-trace" ? 240 : 260;
  const margin = { top: 24, right: 20, bottom: 36, left: 48 };
  const innerW = width - margin.left - margin.right;
  const innerH = height - margin.top - margin.bottom;
  const minY = -50;
  const maxY = 50;
  const ns = "http://www.w3.org/2000/svg";

  svg.innerHTML = "";

  const root = document.createElementNS(ns, "g");
  root.setAttribute("transform", `translate(${margin.left},${margin.top})`);

  for (let i = 0; i <= 4; i += 1) {
    const line = document.createElementNS(ns, "line");
    const y = (innerH / 4) * i;
    line.setAttribute("x1", "0");
    line.setAttribute("x2", innerW);
    line.setAttribute("y1", y);
    line.setAttribute("y2", y);
    line.setAttribute("stroke", "rgba(33,92,147,0.10)");
    root.append(line);
  }

  const xScale = (index) => (labels.length <= 1 ? 0 : (innerW * index) / (labels.length - 1));
  const yScale = (value) => innerH - ((value - minY) / (maxY - minY)) * innerH;
  const buildPath = (series) =>
    series.map((value, index) => `${index === 0 ? "M" : "L"} ${xScale(index).toFixed(2)} ${yScale(value).toFixed(2)}`).join(" ");

  const targetPath = document.createElementNS(ns, "path");
  targetPath.setAttribute("d", buildPath(target));
  targetPath.setAttribute("fill", "none");
  targetPath.setAttribute("stroke", "#215c93");
  targetPath.setAttribute("stroke-width", "3");
  root.append(targetPath);

  const currentPath = document.createElementNS(ns, "path");
  currentPath.setAttribute("d", buildPath(current));
  currentPath.setAttribute("fill", "none");
  currentPath.setAttribute("stroke", "#0f7b79");
  currentPath.setAttribute("stroke-width", "3");
  if (options.dashed) currentPath.setAttribute("stroke-dasharray", "8 6");
  root.append(currentPath);

  labels.forEach((label, index) => {
    if (index % Math.max(1, Math.floor(labels.length / 6)) !== 0 && index !== labels.length - 1) return;
    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", xScale(index));
    text.setAttribute("y", innerH + 24);
    text.setAttribute("text-anchor", "middle");
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = label;
    root.append(text);
  });

  [50, 25, 0, -25, -50].forEach((value) => {
    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", "-8");
    text.setAttribute("y", yScale(value) + 4);
    text.setAttribute("text-anchor", "end");
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = `${value}`;
    root.append(text);
  });

  const legend = document.createElementNS(ns, "g");
  legend.setAttribute("transform", `translate(${innerW - 180}, 0)`);
  [
    { color: "#215c93", text: options.targetLabel || "target" },
    { color: "#0f7b79", text: options.currentLabel || "current" }
  ].forEach((entry, index) => {
    const y = 12 + index * 20;
    const line = document.createElementNS(ns, "line");
    line.setAttribute("x1", "0");
    line.setAttribute("x2", "18");
    line.setAttribute("y1", y);
    line.setAttribute("y2", y);
    line.setAttribute("stroke", entry.color);
    line.setAttribute("stroke-width", "3");
    legend.append(line);

    const text = document.createElementNS(ns, "text");
    text.setAttribute("x", "24");
    text.setAttribute("y", y + 4);
    text.setAttribute("font-size", "12");
    text.setAttribute("fill", "#56728c");
    text.textContent = entry.text;
    legend.append(text);
  });

  root.append(legend);
  svg.append(root);
}

function formatSigned(value, digits = 1, unit = "deg") {
  return `${value >= 0 ? "+" : ""}${value.toFixed(digits)} ${unit}`;
}

function setDialRotation(el, degrees) {
  el.style.transform = `translate(-50%, -88%) rotate(${degrees}deg)`;
}

function setDialMarker(el, degrees) {
  el.style.transform = `rotate(${degrees}deg) translateY(-78px)`;
}

function buildIdealTrace(targetDeg, durationSec) {
  const points = 32;
  const effectiveDuration = Math.max(durationSec, 0.2);
  const labelDigits = effectiveDuration >= 10 ? 1 : 2;
  const labels = Array.from({ length: points }, (_, index) =>
    `${(effectiveDuration * (index / (points - 1))).toFixed(labelDigits)}s`
  );
  const targetSeries = Array(points).fill(targetDeg);
  const currentSeries = Array.from({ length: points }, (_, index) => targetDeg * (index / (points - 1)));
  drawTrace("ideal-trace", labels, targetSeries, currentSeries, {
    targetLabel: "target (ideal)",
    currentLabel: "ideal response"
  });
}

function renderReqs() {
  const host = document.getElementById("req-table");
  portalData.reqs.forEach((req) => {
    const row = makeEl("article", "req-row");
    row.append(makeEl("div", "req-id", req.id));
    row.append(makeEl("span", `prio ${req.priority.toLowerCase()}`, req.priority));
    row.append(makeEl("div", "", req.owner));
    const detailBox = makeEl("div", "");
    detailBox.append(makeEl("strong", "", req.title));
    detailBox.append(makeEl("p", "caption", req.detail));
    row.append(detailBox);
    host.append(row);
  });
}

function renderEvaluation() {
  const data = portalData.evaluation;
  const summaryHost = document.getElementById("evaluation-summary");
  const interviewerHost = document.getElementById("evaluation-interviewer");
  const barsHost = document.getElementById("evaluation-bars");
  const reasonsHost = document.getElementById("evaluation-reasons");
  const to100Host = document.getElementById("evaluation-to100");

  summaryHost.innerHTML = `
    <p class="eyebrow">Interview Verdict</p>
    <div class="score-value">${data.score}%</div>
    <p><strong>현재 평점:</strong> ${data.verdict}</p>
    <p class="score-note">${data.summary}</p>
  `;

  interviewerHost.innerHTML = `
    <p class="eyebrow">Why This Score</p>
    <h3>면접관 코멘트</h3>
    <p>${data.interviewer}</p>
    <p class="caption">참고로 이 점수는 "production-complete autonomous steering subsystem" 기준이다. bring-up / debug / tooling 프로젝트만 놓고 보면 더 높게 평가될 수 있다.</p>
  `;

  data.categories.forEach((entry) => {
    const row = makeEl("div", "eval-row");
    row.innerHTML = `
      <div class="eval-head">
        <strong>${entry.name}</strong>
        <span class="eval-score">${entry.score} / 100</span>
      </div>
      <div class="eval-meter">
        <div class="eval-fill" style="width:${entry.score}%;"></div>
      </div>
      <p class="caption">${entry.detail}</p>
    `;
    barsHost.append(row);
  });

  data.reasons.forEach((entry) => {
    const note = makeEl("article", "eval-note");
    note.innerHTML = `<strong>${entry.title}</strong><p class="caption">${entry.detail}</p>`;
    reasonsHost.append(note);
  });

  data.to100.forEach((entry) => {
    const note = makeEl("article", "eval-note");
    note.innerHTML = `<strong>${entry.title}</strong><p class="caption">${entry.detail}</p>`;
    to100Host.append(note);
  });
}

function renderAnalysisFlow() {
  const data = portalData.analysisFlow;
  const stepHost = document.getElementById("analysis-step-grid");
  const loopHost = document.getElementById("analysis-loop-view");
  const logHost = document.getElementById("analysis-log-grid");

  data.steps.forEach((entry) => {
    const card = makeEl("article", "analysis-step-card");
    card.innerHTML = `
      <span class="step-pill">${entry.step}</span>
      <h3>${entry.title}</h3>
      <p>${entry.summary}</p>
      <div class="badge-row">${entry.vars.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      <p class="caption" style="margin-top:12px;">Files: ${entry.files.join(", ")}</p>
      <p class="caption">${entry.status}</p>
    `;
    stepHost.append(card);
  });

  loopHost.innerHTML = `
    <div class="analysis-phase-grid">
      ${data.phases.map((entry) => `
        <article class="analysis-phase">
          <h4>${entry.name}</h4>
          <p>${entry.description}</p>
          <div class="badge-row">${entry.vars.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
        </article>
      `).join("")}
    </div>
  `;

  data.logs.forEach((entry) => {
    const card = makeEl("article", "log-card");
    card.innerHTML = `
      <h3>${entry.title}</h3>
      <pre class="log-snippet">${entry.snippet}</pre>
      <p class="caption">${entry.note}</p>
    `;
    logHost.append(card);
  });
}

function renderTeams() {
  const host = document.getElementById("team-grid");
  portalData.teams.forEach((team) => {
    const card = makeEl("article", "team-card");
    card.append(makeEl("span", "team-owner", team.owner));
    card.append(makeEl("h3", "", team.title));
    card.append(makeEl("p", "", team.scope));
    card.append(makeEl("p", "caption", `Deliverable: ${team.deliverable}`));
    card.append(makeEl("p", "caption", `Evidence: ${team.evidence}`));
    if (team.brief) {
      const link = makeEl("a", "team-brief-link", team.briefLabel || "상세 할당서");
      link.href = team.brief;
      card.append(link);
    }
    host.append(card);
  });
}

function renderRuntimeAtlas() {
  const flowHost = document.getElementById("runtime-flow");
  const gridHost = document.getElementById("runtime-atlas-grid");

  portalData.runtimeAtlas.forEach((entry) => {
    const stage = makeEl("article", "runtime-stage");
    stage.innerHTML = `
      <span class="runtime-file-tag">${entry.title}</span>
      <h3>${entry.role}</h3>
      <p class="caption">${entry.file}</p>
    `;
    flowHost.append(stage);

    const card = makeEl("article", "runtime-atlas-card");
    card.innerHTML = `
      <span class="runtime-file-tag">${entry.title}</span>
      <p class="runtime-role">${entry.role}</p>
      <div class="runtime-io-grid">
        <div class="runtime-io-box">
          <h4>받는 정보</h4>
          <ul class="plain-list">${entry.receives.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
        <div class="runtime-io-box">
          <h4>보내는 정보</h4>
          <ul class="plain-list">${entry.sends.map((item) => `<li>${item}</li>`).join("")}</ul>
        </div>
      </div>
      <div class="runtime-list-block">
        <h4>핵심 함수</h4>
        <div class="badge-row">${entry.functions.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      </div>
      <div class="runtime-list-block">
        <h4>핵심 변수 / 상태</h4>
        <div class="badge-row">${entry.variables.map((item) => `<span class="code-badge">${item}</span>`).join("")}</div>
      </div>
    `;
    gridHost.append(card);
  });
}

function selectModule(moduleId) {
  const module = portalData.modules.find((entry) => entry.id === moduleId);
  const host = document.getElementById("module-detail");
  if (!module) return;

  document.querySelectorAll("[data-module-id]").forEach((el) => {
    el.classList.toggle("active", el.dataset.moduleId === moduleId);
  });

  host.innerHTML = `
    <h3>${module.title}</h3>
    <p class="module-meta">${module.subtitle}</p>
    <p><strong>Owner:</strong> ${module.owner}</p>
    <p>${module.notes}</p>
    <div class="dual-grid">
      <article class="card">
        <h3>Inputs</h3>
        <ul class="plain-list">${module.inputs.map((item) => `<li>${item}</li>`).join("")}</ul>
      </article>
      <article class="card">
        <h3>Outputs / Risks</h3>
        <ul class="plain-list">${module.outputs.map((item) => `<li>${item}</li>`).join("")}</ul>
        <p class="caption" style="margin-top:12px;">Risks: ${module.risks.join(", ")}</p>
      </article>
    </div>
    <pre class="code-snippet">${module.snippet}</pre>
    <div class="module-links">
      ${module.doxygen ? `<a href="${module.doxygen}">Doxygen source</a>` : ""}
      ${module.files.map((file) => `<a href="${file}">${file.split("/").slice(-2).join("/")}</a>`).join("")}
    </div>
  `;
}

function renderModules() {
  const mapHost = document.getElementById("module-map");
  const listHost = document.getElementById("module-list");
  portalData.modules.forEach((module, index) => {
    const chip = makeEl("button", "module-chip");
    chip.type = "button";
    chip.dataset.moduleId = module.id;
    chip.innerHTML = `<strong>${module.title}</strong><p class="caption">${module.subtitle}</p>`;
    chip.addEventListener("click", () => selectModule(module.id));
    mapHost.append(chip);

    const button = makeEl("button", "module-button");
    button.type = "button";
    button.dataset.moduleId = module.id;
    button.innerHTML = `<strong>${module.title}</strong><p class="caption">${module.owner}</p>`;
    button.addEventListener("click", () => selectModule(module.id));
    listHost.append(button);

    if (index === 0) selectModule(module.id);
  });
}

function setupSimulation() {
  const targetSlider = document.getElementById("target-slider");
  const freqSlider = document.getElementById("freq-slider");
  const replayButton = document.getElementById("replay-sim");
  const targetReadout = document.getElementById("target-readout");
  const freqReadout = document.getElementById("freq-readout");
  const motorTargetValue = document.getElementById("motor-target-value");
  const encoderTargetValue = document.getElementById("encoder-target-value");
  const pulseTargetValue = document.getElementById("pulse-target-value");
  const timeTargetValue = document.getElementById("time-target-value");
  const simStatus = document.getElementById("sim-status");
  const simProgressFill = document.getElementById("sim-progress-fill");
  const steeringNeedle = document.getElementById("steering-needle");
  const motorNeedle = document.getElementById("motor-needle");
  const steeringTargetMarker = document.getElementById("steering-target-marker");
  const motorTargetMarker = document.getElementById("motor-target-marker");
  const steeringCaption = document.getElementById("steering-caption");
  const motorCaption = document.getElementById("motor-caption");
  const freqStatus = document.getElementById("freq-status");

  let simRafId = 0;
  let simRestartTimer = 0;
  const simState = {
    targetDeg: Number(targetSlider.value),
    freqHz: Number(freqSlider.value),
    targetMotorDeg: Number(targetSlider.value) * PORTAL_STEERING_GEAR_RATIO,
    pulseCount: Math.round(Math.abs(Number(targetSlider.value)) * PORTAL_PULSE_PER_STEERING_DEG),
    encoderCount: Math.round(Math.abs(Number(targetSlider.value)) * PORTAL_ENCODER_COUNT_PER_STEERING_DEG),
    timeSec: 0,
    durationMs: 0,
    startTs: 0
  };

  const cancelSimulation = () => {
    if (simRafId) {
      cancelAnimationFrame(simRafId);
      simRafId = 0;
    }
    if (simRestartTimer) {
      clearTimeout(simRestartTimer);
      simRestartTimer = 0;
    }
  };

  const refreshMetrics = () => {
    const target = Number(targetSlider.value);
    const freq = Number(freqSlider.value);
    const motorDeg = target * PORTAL_STEERING_GEAR_RATIO;
    const encoderCount = Math.round(Math.abs(target) * PORTAL_ENCODER_COUNT_PER_STEERING_DEG);
    const pulseCount = Math.round(Math.abs(target) * PORTAL_PULSE_PER_STEERING_DEG);
    const timeSec = pulseCount / Math.max(freq, 1);
    const motorRev = motorDeg / 360;

    simState.targetDeg = target;
    simState.freqHz = freq;
    simState.targetMotorDeg = motorDeg;
    simState.encoderCount = encoderCount;
    simState.pulseCount = pulseCount;
    simState.timeSec = timeSec;
    simState.durationMs = Math.max(timeSec * 1000, 220);

    targetReadout.textContent = `${target.toFixed(1)} deg`;
    freqReadout.textContent = `${freq.toLocaleString()} Hz`;
    motorTargetValue.textContent = `${motorDeg.toFixed(0)} deg`;
    encoderTargetValue.textContent = `${encoderCount.toLocaleString()}`;
    pulseTargetValue.textContent = `${pulseCount.toLocaleString()}`;
    timeTargetValue.textContent = `${timeSec.toFixed(2)} s`;
    setDialMarker(steeringTargetMarker, target * 2);
    setDialMarker(motorTargetMarker, motorDeg);
    buildIdealTrace(target, timeSec);

    if (freq >= 100000) {
      freqStatus.textContent = "현재 firmware의 상한값 부근이다. 실제 코드에서는 100000 Hz clamp와 1 ms reverse guard를 사용하며, final bench closure는 real encoder truth를 authoritative하게 닫는 일이다.";
    } else {
      freqStatus.textContent = "현재 firmware contract 안쪽 값이다. 포털 시뮬레이션은 현재 코드의 pulse contract를 설명하기 위한 모델이며, 실제 bench에서는 drive monitor와 함께 해석해야 한다.";
    }
    steeringCaption.textContent = `Target ${formatSigned(target)} / Current ${formatSigned(0)}`;
    motorCaption.textContent = `Target ${formatSigned(motorDeg, 0)} / Current ${formatSigned(0, 0)} / ${motorRev.toFixed(2)} rev target`;
  };

  const renderProgress = (progress) => {
    const currentSteeringDeg = simState.targetDeg * progress;
    const currentMotorDeg = simState.targetMotorDeg * progress;
    const currentMotorRev = currentMotorDeg / 360;
    const errorDeg = simState.targetDeg - currentSteeringDeg;
    const currentPulse = Math.round(simState.pulseCount * progress);
    const elapsedSec = simState.timeSec * progress;

    setDialRotation(steeringNeedle, currentSteeringDeg * 2);
    setDialRotation(motorNeedle, currentMotorDeg);
    simProgressFill.style.width = `${(progress * 100).toFixed(1)}%`;
    steeringCaption.textContent = `Target ${formatSigned(simState.targetDeg)} / Current ${formatSigned(currentSteeringDeg)} / Error ${formatSigned(errorDeg)}`;
    motorCaption.textContent = `Target ${formatSigned(simState.targetMotorDeg, 0)} / Current ${formatSigned(currentMotorDeg, 0)} / ${currentMotorRev.toFixed(2)} rev`;

    if (progress >= 1) {
      simStatus.textContent = `complete · ${simState.timeSec.toFixed(2)} s · ${simState.pulseCount.toLocaleString()} pulse`;
    } else {
      simStatus.textContent = `running · ${elapsedSec.toFixed(2)} / ${simState.timeSec.toFixed(2)} s · ${currentPulse.toLocaleString()} / ${simState.pulseCount.toLocaleString()} pulse`;
    }
  };

  const animate = (timestamp) => {
    if (!simState.startTs) simState.startTs = timestamp;
    const elapsed = timestamp - simState.startTs;
    const progress = simState.durationMs <= 0 ? 1 : Math.min(elapsed / simState.durationMs, 1);
    renderProgress(progress);

    if (progress < 1) {
      simRafId = requestAnimationFrame(animate);
    } else {
      simRafId = 0;
      simState.startTs = 0;
    }
  };

  const startSimulation = () => {
    cancelSimulation();
    refreshMetrics();
    simState.startTs = 0;
    renderProgress(0);
    simRafId = requestAnimationFrame(animate);
  };

  const scheduleSimulation = () => {
    refreshMetrics();
    simStatus.textContent = "armed · slider 값 반영 후 재생 준비";
    simProgressFill.style.width = "0%";
    if (simRestartTimer) clearTimeout(simRestartTimer);
    simRestartTimer = setTimeout(() => {
      simRestartTimer = 0;
      startSimulation();
    }, 180);
  };

  targetSlider.addEventListener("input", scheduleSimulation);
  freqSlider.addEventListener("input", scheduleSimulation);
  targetSlider.addEventListener("change", startSimulation);
  freqSlider.addEventListener("change", startSimulation);
  replayButton.addEventListener("click", startSimulation);

  refreshMetrics();
  startSimulation();
}

function setupNav() {
  document.querySelectorAll(".nav-link").forEach((button) => {
    button.addEventListener("click", () => {
      const target = document.getElementById(button.dataset.target);
      if (target) target.scrollIntoView({ behavior: "smooth", block: "start" });
    });
  });

  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      document.querySelectorAll(".nav-link").forEach((button) => {
        button.classList.toggle("active", button.dataset.target === entry.target.id);
      });
    });
  }, { threshold: 0.28 });

  document.querySelectorAll("main section[id]").forEach((section) => observer.observe(section));
}

function init() {
  renderKpis();
  renderLatencyBars();
  drawTrace("actual-trace", portalData.actualTrace.labels, portalData.actualTrace.target, portalData.actualTrace.current, {
    targetLabel: "logged target",
    currentLabel: "logged current"
  });
  renderAnalysisFlow();
  renderEvaluation();
  renderReqs();
  renderTeams();
  renderRuntimeAtlas();
  renderModules();
  setupSimulation();
  setupNav();
}

document.addEventListener("DOMContentLoaded", init);
