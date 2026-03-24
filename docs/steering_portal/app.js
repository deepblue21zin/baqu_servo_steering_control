const portalData = {
  kpis: [
    { label: "Control samples", value: "154", note: "measured from latest PuTTY logs" },
    { label: "Unique encoder raw", value: "1", note: "all snapshots stayed at 32769" },
    { label: "Latency batches", value: "220", note: "Sense / Control / Actuate / Comms" },
    { label: "Max deadline miss", value: "65", note: "interactive logging condition" }
  ],
  latency: [
    { name: "Sense", avg: 0.928, p99: 0.961 },
    { name: "Control", avg: 2.557, p99: 2.583 },
    { name: "Actuate", avg: 2.281, p99: 2.294 },
    { name: "Comms", avg: 1.995, p99: 1.917 }
  ],
  actualTrace: {
    labels: ["0", "-1", "-2", "-3", "-4", "-5", "-4", "-3", "-30", "0", "-45", "45", "44", "0"],
    target: [0, -1, -2, -3, -4, -5, -4, -3, -30, 0, -45, 45, 44, 0],
    current: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  },
  evaluation: {
    score: 67,
    verdict: "조향 제어 bring-up, 계측 체계, 명령 lifecycle 추적은 꽤 좋아졌지만, production-complete steering subsystem으로 보려면 아직 센서 진실성과 readiness closure 공백이 남아 있다.",
    summary:
      "현업 면접관 기준으로 이 프로젝트는 '코드가 있는 데모'가 아니라 '실제 하드웨어를 붙여 계측하고 구조화한 steering controller 프로젝트'라는 점에서 분명 강점이 있다. command lifecycle이 들어가면서 검증 폐쇄성과 설명력이 한 단계 좋아졌지만, 100%를 자율주행 조향 서브컨트롤러 완성형으로 정의하면 현재 상태는 약 67% 수준이 적절하다.",
    interviewer:
      "면접에서는 이 프로젝트를 완성품으로 포장하기보다, 센서-제어-액추에이터-로그를 한 프레임으로 묶은 engineering baseline으로 설명하는 편이 더 강하다. 특히 command_id 기반 start/reached/timeout/abort/fault 추적이 생긴 점은 좋다. 다만 closed-loop truth와 fail-safe closure는 아직 마지막 문턱이다.",
    categories: [
      {
        name: "Architecture / Readability",
        score: 82,
        detail: "모듈 책임과 문서화가 꽤 잘 보인다. 다만 main.c에 runtime orchestration, bench UI, logging이 몰려 있어 최종 구조로는 더 분리할 필요가 있다."
      },
      {
        name: "Observability / Evidence",
        score: 86,
        detail: "UART CSV, latency logs, dbg vars, plotting, change log, portal, Doxygen까지 갖춰져 있어 증거 체계는 강한 편이다."
      },
      {
        name: "Actuator Interface",
        score: 71,
        detail: "pulse/direction path와 PF/PR differential 개념은 정리돼 있다. 하지만 100kHz clamp, 500kHz 희망 bench mode, 1MHz 설계 상수가 충돌해 계약 통일이 필요하다."
      },
      {
        name: "Sensor Truth",
        score: 34,
        detail: "latest logs에서 encoder raw가 고정이라 closed-loop feedback truth를 증명하지 못한다. production steering에서는 이 항목이 가장 중요하다."
      },
      {
        name: "Safety / Startup / Fail-safe",
        score: 49,
        detail: "ESTOP, timeout, command fault/abort trace는 좋아졌지만 startup gating, fault latch, recovery contract가 runtime에 완전히 닫혀 있지 않다."
      },
      {
        name: "Verification Closure",
        score: 66,
        detail: "CMD_START/REACHED/TIMEOUT/ABORT 로그가 추가되면서 명령 단위 검증은 분명 좋아졌다. 다만 step response, wrong-direction, long-run stability 같은 완료 증거는 아직 부족하다."
      }
    ],
    reasons: [
      {
        title: "설계와 계측의 골격은 분명히 있다",
        detail: "1ms loop, pulse/direction path, gear ratio 기준, UART/latency logging, plot 생성, 변경 이력 관리까지 이어지는 구조는 이미 프로젝트다운 모습을 갖췄다. 그래서 30~40% 수준의 미완성 프로토타입으로 보이지는 않는다."
      },
      {
        title: "하지만 steering current truth가 아직 서지 않았다",
        detail: "현재 로그에서는 target만 변하고 current/encoder가 고정돼 있어, 면접관 입장에서는 '실제로 steering axis가 제어됐는가'라는 핵심 질문에 아직 예라고 답하기 어렵다."
      },
      {
        title: "production steering은 command lifecycle과 fault closure가 핵심이다",
        detail: "자율주행 조향은 목표값을 받아 움직이는 것만으로는 부족하다. 도달, 미도달, timeout, 반대 방향 진행, sensor stale, estop latch가 상태기와 reason code로 닫혀야 한다. 지금은 lifecycle 1차 구현이 들어갔지만 readiness closure는 아직 남아 있다."
      },
      {
        title: "startup safety가 아직 설명형 구현에 머물러 있다",
        detail: "homing과 relay control 파일은 존재하지만 실제 runtime gating으로 완전히 연결되지 않았다. boot-time auto enable은 면접에서 가장 먼저 지적받을 수 있는 리스크다."
      }
    ],
    to100: [
      {
        title: "Sensor truth를 authoritative source로 세워야 한다",
        detail: "TIM4 raw만 보는 수준에서 끝나지 말고 unwrap, accum_count, sign verification, ADC cross-check, stale detection을 넣어야 한다. 이유는 steering 제품에서 현재각이 틀리면 그 위 모든 제어 논리가 무의미해지기 때문이다."
      },
      {
        title: "Move state machine을 완성해야 한다",
        detail: "IDLE, ARMED, MOVING, REACHED, TIMEOUT, FAULT_LATCH, ESTOP_LATCH를 운영 계약으로 정의해야 한다. 지금은 no-homing baseline에서 command lifecycle이 들어갔고, 다음 단계는 startup gating과 latched fault까지 닫는 것이다."
      },
      {
        title: "Startup / Homing / Recovery를 runtime에 통합해야 한다",
        detail: "INIT -> HOMING -> READY -> RUN 구조와 clear policy가 있어야 한다. production steering은 전원 인가 직후 움직이지 않고, 센서 기준과 안전 조건이 맞을 때만 arm되는 것이 기본이다."
      },
      {
        title: "Actuator contract를 하나로 통일해야 한다",
        detail: "TIM1 capability, clamp, fixed-speed bench mode, direction settle time, stop-before-reverse를 하나의 문서와 코드 계약으로 정리해야 한다. 그래야 하드웨어 인터페이스와 요구사항이 일치한다."
      },
      {
        title: "완료 증거 패키지를 만들어야 한다",
        detail: "step response, reached time, steady-state error, timeout case, estop case, startup inhibit, long-run stability를 로그/플롯/영상으로 남겨야 한다. 100%는 기능이 아니라 반복 가능한 evidence pack까지 포함한 상태다."
      }
    ]
  },
  analysisFlow: {
    steps: [
      {
        step: "Step 1",
        title: "Upper command / 단위 기준부터 맞춘다",
        summary: "상위 입력은 `steering_deg` 기준으로 들어오고, main runtime에서 `motor_deg`로 변환되어 position controller로 전달된다. 이 단계에서는 기어비와 pulse 환산식이 같은 기준을 쓰는지 먼저 본다.",
        vars: [
          "AutoDrive_Packet_t.steering_angle : float steering_deg",
          "STEERING_GEAR_RATIO = 12.0",
          "SteeringDegToMotorDeg() / MotorDegToSteeringDeg()",
          "DEG_PER_PULSE = 0.003 motor_deg"
        ],
        files: ["ethernet_communication.c", "constants.h", "main.c"],
        status: "적용됨. 다만 코드 전반에 steering_deg / motor_deg 명칭이 완전히 통일된 상태는 아니다."
      },
      {
        step: "Step 2",
        title: "센서 truth를 먼저 확인한다",
        summary: "제어기보다 먼저 봐야 하는 것은 `enc_raw`, `current_deg`, `ADC angle`이다. 현재 구조상 debug vars와 sensor API는 존재하지만, latest logs에서는 encoder current가 고정이라 sensor truth가 아직 미완료다.",
        vars: [
          "dbg_enc_raw / dbg_pos_mdeg / dbg_err_mdeg",
          "EncoderReader_GetAngleDeg()",
          "ADC_Pot_GetAngle()",
          "DBG_FAULT_POS_LIMIT / DBG_FAULT_TRACKING"
        ],
        files: ["encoder_reader.c", "adc_potentiometer.c", "debug_vars.h", "position_control.c"],
        status: "부분 적용. debug hooks는 있으나 runtime authoritative sensor path는 아직 닫히지 않았다."
      },
      {
        step: "Step 3",
        title: "error -> output -> dir -> pulse를 비교한다",
        summary: "센서값이 믿을 수 있다는 가정 아래, current를 읽고 error를 계산하고 safety check를 통과한 뒤 PID output을 pulse frequency로 내보낸다. 여기서 `dir`과 `pulse`가 error와 같은 방향인지 비교하는 것이 핵심이다.",
        vars: [
          "state.target_angle / state.current_angle / state.error",
          "PID_Calculate(error, dt)",
          "PulseControl_SetFrequency((int32_t)state.output)",
          "DIR pin + signed pulse_hz"
        ],
        files: ["position_control.c", "pulse_control.c"],
        status: "적용됨. no-homing command lifecycle은 반영됐고 actuator contract와 fault latch는 더 보강이 필요하다."
      },
      {
        step: "Step 4",
        title: "latency / deadline miss / fail-safe trace를 붙인다",
        summary: "마지막에는 제어 결과만 보지 않고 timing evidence와 fail-safe trace를 함께 본다. 현재는 CSV, latency batch, ESTOP path, command lifecycle event log까지 존재하지만 richer fault taxonomy와 clear policy는 더 필요하다.",
        vars: [
          "CSV_HEADER / CSV rows",
          "CMD_START / CMD_REACHED / CMD_TIMEOUT / CMD_ABORT / CMD_FAULT",
          "LATENCY_BATCH_BEGIN / LATENCY_STAGE / LATENCY_BATCH_END",
          "ETHCOMM_RX_TIMEOUT_MS = 300",
          "PositionControl_EmergencyStop()"
        ],
        files: ["main.c", "latency_profiler.c", "ethernet_communication.h"],
        status: "적용됨. deterministic evidence의 골격은 갖췄고, 다음 단계는 richer fault taxonomy와 clear policy다."
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
        vars: ["STEERING_GEAR_RATIO", "SteeringDegToMotorDeg()", "TargetSteeringDegToMotorDeg()"]
      },
      {
        name: "Sense",
        description: "엔코더 current angle과 debug raw 값을 읽는다. ADC angle은 startup/homing 보조 센서 역할이다.",
        vars: ["EncoderReader_GetAngleDeg()", "dbg_enc_raw", "dbg_pos_mdeg", "ADC_Pot_GetAngle()"]
      },
      {
        name: "Control",
        description: "오차 계산, 안전 체크, PID 계산을 1ms loop 안에서 수행한다.",
        vars: ["state.error", "PositionControl_CheckSafety()", "PID_Calculate()", "dbg_err_mdeg"]
      },
      {
        name: "Actuate",
        description: "signed output을 pulse_hz와 direction으로 바꿔 line-driver로 전달한다.",
        vars: ["state.output", "PulseControl_SetFrequency()", "PE9 pulse", "PE10 direction"]
      },
      {
        name: "Evidence",
        description: "CSV, latency, snapshot, fault flags로 현재 loop 상태를 외부에 남긴다.",
        vars: ["CSV rows", "LATENCY_STAGE", "dbg_fault_flags", "[KB][snapshot]"]
      }
    ],
    logs: [
      {
        title: "CSV trace 예시",
        snippet:
          "CSV_HEADER,ms,mode,target_deg,current_deg,error_deg,output,dir,enc,cmd_id,cmd_state,cmd_result\nCSV,1234,1,5.000,0.000,5.000,7400,1,32769,12,1,0",
        note: "한 줄에서 target, current, error, output, dir, enc와 command lifecycle 상태를 같이 본다. 현업에서는 이 로그로 단위, 부호, 명령 진행 상태가 맞는지 먼저 확인한다."
      },
      {
        title: "Latency / deadline miss 예시",
        snippet:
          "LATENCY_BATCH_BEGIN,seq=526,samples=2000,core_hz=180000000,deadline_miss=0\nLATENCY_STAGE,seq=526,name=Sense,count=2000,avg_us=0.928,p99_us=0.961,max_us=0.967\nLATENCY_STAGE,seq=526,name=Control,count=2000,avg_us=2.478,p99_us=2.478,max_us=2.478\nLATENCY_BATCH_END,seq=526",
        note: "Sense, Control, Actuate, Comms 단계별 timing과 deadline miss를 함께 기록해 deterministic behavior를 설명한다."
      },
      {
        title: "Fail-safe / safety 적용 예시",
        snippet:
          "CMD_START,id=12,src=KEYBOARD,target_deg=5.000,target_motor_deg=60.000,start_ms=12345,start_deg=0.000,start_error_deg=5.000\nCMD_REACHED,id=12,end_ms=12780,settling_ms=435,final_deg=4.940,final_error_deg=0.060\nCMD_ABORT,id=13,reason=ESTOP,end_ms=13020,error_deg=1.200",
        note: "현재는 timeout ESTOP 경로와 command lifecycle event log가 적용돼 있다. 다만 timeout/fault taxonomy와 clear policy는 더 세분화해야 한다."
      }
    ]
  },
  reqs: [
    {
      id: "REQ-P0-001",
      priority: "P0",
      owner: "Sensor / C",
      title: "Encoder unwrap와 accum_count를 도입해 ±45 deg steering full range를 설명 가능하게 만든다.",
      detail: "16-bit TIM4 raw만으로는 full steering range를 닫을 수 없으므로 raw_count, delta_count, accum_count, motor_deg, steering_deg를 분리해 제공해야 한다."
    },
    {
      id: "REQ-P0-002",
      priority: "P0",
      owner: "Control / A",
      title: "command lifecycle 다음 단계로 fault latch / clear policy를 닫는다.",
      detail: "no-homing baseline에서 target/reached/timeout/abort/fault event는 추가됐다. 다음 단계는 latched fault, startup readiness, wrong-direction/stale sensor abort 정책까지 운영 계약으로 닫는 것이다."
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
      owner: "Actuator / D",
      title: "pulse contract를 하나로 통일한다.",
      detail: "1 MHz constant, 100 kHz clamp, 500 kHz desired bench mode가 서로 충돌하므로 current firmware capability와 target design capability를 분리해서 계약해야 한다."
    },
    {
      id: "REQ-P1-001",
      priority: "P1",
      owner: "Integration / E",
      title: "upper command, mode, ESTOP, timeout을 하나의 system contract로 묶는다.",
      detail: "UDP packet, keyboard test, comm timeout, recovery clear 조건을 같은 state model에서 다뤄야 한다."
    },
    {
      id: "REQ-P1-002",
      priority: "P1",
      owner: "Validation / F",
      title: "CSV, latency, plot, change log, 영상, Doxygen source를 하나의 evidence pack으로 자동 연결한다.",
      detail: "포트폴리오와 인수기준 설명이 가능하도록 측정 결과와 요구사항 ID를 함께 남기는 체계가 필요하다."
    },
    {
      id: "REQ-P1-003",
      priority: "P1",
      owner: "Sensor / C",
      title: "encoder-ADC cross-check와 implausible motion 진단을 넣는다.",
      detail: "commanded direction과 encoder delta 부호가 일정 시간 이상 안 맞으면 sensor/actuator fault 후보로 판단해야 한다."
    },
    {
      id: "REQ-P1-004",
      priority: "P1",
      owner: "Control / A",
      title: "fault taxonomy를 정식 코드 체계로 만든다.",
      detail: "sensor stale, tracking error, over-limit, timeout, estop, startup inhibit를 reason code와 clear 조건으로 함께 관리해야 한다."
    },
    {
      id: "REQ-P2-001",
      priority: "P2",
      owner: "Actuator / D",
      title: "direction reversal setup/hold와 residual pulse count를 scope evidence로 남긴다.",
      detail: "stop-before-reverse, guard time, actual waveform 검증이 있어야 driver interface를 설명할 수 있다."
    },
    {
      id: "REQ-P2-002",
      priority: "P2",
      owner: "Validation / F",
      title: "MClab 제출용 PDF와 측정 리포트를 자동 생성 가능한 형태로 정리한다.",
      detail: "사이트, PNG plot, raw log, 핵심 표를 같은 baseline 태그로 묶으면 제출 품질이 높아진다."
    }
  ],
  teams: [
    {
      owner: "Team A",
      title: "Motion Control & Fault Manager",
      scope: "position_control.c, control contract, reached-state, fault latch",
      deliverable: "target/reached/timeout/fault state machine, actual dt/saturation diagnostics",
      evidence: "step response log, state transition trace, fault injection log"
    },
    {
      owner: "Team B",
      title: "Startup / Homing / Relay Safety",
      scope: "homing.c, relay_control.c, startup and recovery sequence",
      deliverable: "INIT->HOMING->READY->RUN state machine, latched ESTOP clear procedure",
      evidence: "startup GPIO sequence, homing success/failure log, recovery checklist"
    },
    {
      owner: "Team C",
      title: "Sensor Integrity",
      scope: "encoder_reader.c, adc_potentiometer.c, unit conversion and health",
      deliverable: "unwrap, accum_count, ADC validity, cross-check, sign verification",
      evidence: "long-run encoder log, ADC raw/angle CSV, mismatch detection trace"
    },
    {
      owner: "Team D",
      title: "Pulse / Direction / Driver Timing",
      scope: "pulse_control.c, TIM1 frequency contract, PF/PR path",
      deliverable: "applied_hz contract, fixed-speed move mode, stop-before-reverse policy",
      evidence: "scope capture, commanded vs measured frequency table, residual pulse count"
    },
    {
      owner: "Team E",
      title: "Communication & System Integration",
      scope: "ethernet_communication.c, main.c mode transitions, timeout behavior",
      deliverable: "UDP/keyboard/system mode contract, command validation, recovery gating",
      evidence: "mode transition log, timeout fail-safe proof, packet-to-target trace"
    },
    {
      owner: "Team F",
      title: "Verification & Tooling",
      scope: "latency_profiler.c, debug_vars, plotting scripts, documentation portal",
      deliverable: "evidence automation, plot generation, portal maintenance, measurement pack",
      evidence: "latency report, PNG plots, Doxygen portal, submission-ready artifact set"
    }
  ],
  modules: [
    {
      id: "main",
      title: "main.c",
      subtitle: "System init, scheduler, keyboard test, CSV/latency logging",
      owner: "Integration / Runtime",
      inputs: ["UDP packets", "keyboard UART", "interrupt_flag", "watchdog tick"],
      outputs: ["PositionControl_SetTargetWithSource()", "Relay_ServoOn()", "Periodic CSV", "LATENCY_STAGE"],
      risks: ["boot-time auto enable", "multiple responsibilities in one file"],
      files: ["../../Core/Src/main.c", "../../Core/Inc/main.h"],
      doxygen: "../doxygen/html/main_8c_source.html",
      snippet: "HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);\\nRelay_Init();\\nPulseControl_Init();\\nEncoderReader_Init();\\nPositionControl_Init();\\n...\\nEncoderReader_Reset();\\nPositionControl_SetTargetWithSource(TargetSteeringDegToMotorDeg(0.0f), CMD_SRC_LOCALTEST);\\nPositionControl_Enable();",
      notes: "main.c는 initialization, mode handling, bench UI, logging까지 맡고 있어 runtime orchestration의 중심이지만, state machine 관점에서는 책임이 과하다."
    },
    {
      id: "position",
      title: "position_control.c",
      subtitle: "PID, safety check, debug vars, emergency stop",
      owner: "Control / A",
      inputs: ["target_angle[motor_deg]", "current_angle from encoder", "dt from HAL_GetTick"],
      outputs: ["state.output -> PulseControl_SetFrequency()", "fault -> Relay_Emergency()"],
      risks: ["fault latch 부족", "boot-time auto-enable 연계 미완성", "HAL_GetTick precision limitation"],
      files: ["../../Core/Src/position_control.c", "../../Core/Inc/position_control.h"],
      doxygen: "../doxygen/html/position__control_8c_source.html",
      snippet: "state.current_angle = EncoderReader_GetAngleDeg();\\nstate.error = state.target_angle - state.current_angle;\\nif (!PositionControl_CheckSafety()) {\\n    PositionControl_CommandFinish(CMD_FAULTED, fault_result, HAL_GetTick());\\n    PositionControl_EmergencyStop();\\n    return;\\n}\\nstate.output = PID_Calculate(state.error, dt);\\nPulseControl_SetFrequency((int32_t)state.output);",
      notes: "현재 구조는 error 기반 PID output, command_id, start/reached/timeout/abort/fault event까지 연결됐다. 다음 단계는 fault latch, stale sensor, startup readiness와의 closure다."
    },
    {
      id: "encoder",
      title: "encoder_reader.c",
      subtitle: "TIM4 16-bit encoder raw -> angle conversion",
      owner: "Sensor / C",
      inputs: ["TIM4 raw counter"],
      outputs: ["current_angle[motor_deg]", "raw_count"],
      risks: ["unwrap missing", "latest log 기준 raw stuck"],
      files: ["../../Core/Src/encoder_reader.c", "../../Core/Inc/encoder_reader.h"],
      doxygen: "../doxygen/html/encoder__reader_8c_source.html",
      snippet: "uint16_t raw = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);\\nencoder_count = (int32_t)raw - 32768;\\nint32_t adjusted_count = encoder_count - encoder_offset;\\nreturn (float)adjusted_count * DEG_PER_COUNT;",
      notes: "full steering range를 설명하려면 raw_count만이 아니라 accum_count와 steering/motor 축 변환을 함께 보여줘야 한다."
    },
    {
      id: "pulse",
      title: "pulse_control.c",
      subtitle: "PE9 pulse, PE10 direction, PF/PR line-driver path",
      owner: "Actuator / D",
      inputs: ["signed pulse_hz", "step count", "direction enum"],
      outputs: ["TIM1 PWM", "GPIO direction", "step busy state"],
      risks: ["current clamp 100kHz", "hard-coded timer clock", "reverse timing policy 없음"],
      files: ["../../Core/Src/pulse_control.c", "../../Core/Inc/pulse_control.h"],
      doxygen: "../doxygen/html/pulse__control_8c_source.html",
      snippet: "if (freq_hz > 0) {\\n    PulseControl_ApplyDirection(DIR_CW);\\n} else {\\n    PulseControl_ApplyDirection(DIR_CCW);\\n    freq_hz = -freq_hz;\\n}\\nif (freq_hz > 100000) freq_hz = 100000;\\nif (freq_hz < 10) freq_hz = 10;",
      notes: "하드웨어는 pulse/direction line driver가 분리돼 있으므로, 펌웨어도 frequency contract와 direction timing을 더 명시적으로 관리해야 한다."
    },
    {
      id: "comms",
      title: "ethernet_communication.c",
      subtitle: "UDP parsing, status reporting, steering unit conversion",
      owner: "Integration / E",
      inputs: ["UDP command text / packets"],
      outputs: ["SteerMode_t", "AutoDrive_Packet_t", "status text"],
      risks: ["binary CRC contract 미완료", "state machine 통합 필요"],
      files: ["../../Core/Src/ethernet_communication.c", "../../Core/Inc/ethernet_communication.h"],
      doxygen: "../doxygen/html/ethernet__communication_8c_source.html",
      snippet: "target_motor_deg = SteeringDegToMotorDeg(cmd->f32_value);\\nif (PositionControl_SetTargetWithSource(target_motor_deg, CMD_SRC_SERVICE) != 0) {\\n    return EthComm_SendString(\\\"ERR TARGET\\\\r\\\\n\\\");\\n}\\n...\\nfloat cur = MotorDegToSteeringDeg(PositionControl_GetCurrentAngle());",
      notes: "외부 명령은 steering_deg, 내부는 motor_deg로 바꾸는 경로가 이미 일부 들어가 있다. 이 계약을 state machine 전체와 연결하는 것이 다음 단계다."
    },
    {
      id: "homing",
      title: "homing.c + relay_control.c",
      subtitle: "Startup safety skeleton",
      owner: "Safety / B",
      inputs: ["ADC angle", "operator reset or startup event"],
      outputs: ["encoder offset", "relay GPIO action"],
      risks: ["main runtime에서 실제 통합 안 됨", "failure path가 약함"],
      files: ["../../Core/Src/homing.c", "../../Core/Src/relay_control.c"],
      doxygen: "../doxygen/html/homing_8c_source.html",
      snippet: "float pot_angle = ADC_Pot_GetAngle();\\nEncoderReader_Reset();\\nint32_t offset_count = (int32_t)(pot_angle * ENCODER_COUNT_PER_REV / FULL_ROTATION_DEG);\\nEncoderReader_SetOffset(offset_count);\\nhoming_status = HOMING_STATUS_COMPLETE;",
      notes: "현업 기준에서는 startup safety를 설명하는 핵심인데, 현재는 개념 구현에 가까워 실제 runtime gating까지는 연결되지 않았다."
    },
    {
      id: "latency",
      title: "latency_profiler.c + debug_vars.h",
      subtitle: "Measured evidence and debug observability",
      owner: "Validation / F",
      inputs: ["DWT cycle counter", "debug globals", "1ms deadline tick"],
      outputs: ["LATENCY_STAGE logs", "dbg_* watch variables", "plots"],
      risks: ["strict measurement session과 interactive session 구분 필요"],
      files: ["../../Core/Src/latency_profiler.c", "../../Core/Inc/debug_vars.h"],
      doxygen: "../doxygen/html/latency__profiler_8c_source.html",
      snippet: "dt = DWT->CYCCNT - buf->start_cycle;\\nbuf->samples[buf->sample_count] = dt;\\n...\\nout_stats->avg_us = cycles_to_us(avg_cycles);\\nout_stats->p99_us = cycles_to_us(p99_cycles);\\nout_stats->max_us = cycles_to_us(buf->max_cycles);",
      notes: "이 프로젝트의 강점은 코드 자체뿐 아니라 지연 시간과 debug variables를 evidence로 남긴다는 점이다."
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
    host.append(card);
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
      <a href="${module.doxygen}">Doxygen source</a>
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
    targetMotorDeg: Number(targetSlider.value) * 12,
    pulseCount: Math.round(Math.abs(Number(targetSlider.value)) * 4000),
    encoderCount: Math.round(Math.abs(Number(targetSlider.value)) * 1600),
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
    const motorDeg = target * 12;
    const encoderCount = Math.round(Math.abs(target) * 1600);
    const pulseCount = Math.round(Math.abs(target) * 4000);
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

    if (freq > 100000) {
      freqStatus.textContent = "현재 코드 기준으로는 100000 Hz clamp가 있으므로 이 값은 설명용 설계 목표다. 포털 시뮬레이션은 500000 Hz까지 재생하지만, 실제 firmware는 TIM1 / pulse contract 재설계가 필요하다.";
    } else {
      freqStatus.textContent = "현재 firmware clamp 안쪽 값이다. 포털에서는 fixed-speed move를 이상화해 보여주며, latest logs 기준 실제 current feedback은 아직 고정 상태라 별도 검증이 더 필요하다.";
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
  renderModules();
  setupSimulation();
  setupNav();
}

document.addEventListener("DOMContentLoaded", init);
