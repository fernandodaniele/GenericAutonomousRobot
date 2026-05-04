# Arquitectura de GenericAutonomousRobot (GAR)

## Diagrama de Arquitectura General

```mermaid
graph TB
    subgraph Hardware["🔧 Hardware"]
        STM32["STM32F407VG<br/>ARM Cortex-M4<br/>168MHz"]
        ESP32["ESP32-S3<br/>Dual-Core Xtensa<br/>240MHz"]
        Motors["🔄 Motores DC<br/>TB6612FNG Driver"]
        IMU["📡 MPU6050<br/>IMU"]
        Ultrasonic["📏 HC-SR04<br/>Ultrasónico"]
        IR["🔴 TCRT5000 x5<br/>Infrarrojo"]
    end
    
    subgraph STM32_FW["⚙️ STM32 Firmware (FreeRTOS)"]
        Tasks["<b>RTOS Tasks</b><br/>- Motor Control<br/>- Sensor Polling<br/>- Navigation Logic<br/>- DSP Filters<br/>- Black Box"]
        DSP["🔌 DSP Module<br/>FIR/IIR Filters"]
        Kinematics["📐 Kinematics<br/>Motor Commands"]
        Logger["📝 Black Box Logger<br/>Flash Storage"]
    end
    
    subgraph ESP32_FW["📡 ESP32 Firmware (ESP-IDF)"]
        Gateway["WiFi Gateway"]
        Telemetry["📊 Telemetry Manager"]
        UART["UART Bridge<br/>STM32 ↔ ESP32"]
    end
    
    subgraph Analysis["🧮 Analysis & Tools"]
        MATLAB["MATLAB/Octave<br/>Filter Design<br/>Signal Analysis"]
        Python["Python Scripts<br/>Data Processing<br/>Pericias"]
    end
    
    subgraph Sensors_Network["Sensor Network"]
        IMU ---|I2C| STM32
        Ultrasonic ---|GPIO/PWM| STM32
        IR ---|GPIO| STM32
    end
    
    subgraph ActuatorNetwork["Actuator Network"]
        STM32 ---|PWM| Motors
    end
    
    subgraph RTOS_Internal["RTOS Communication"]
        Tasks ---|Queues<br/>Semaphores| DSP
        Tasks ---|Data| Kinematics
        Tasks ---|Log Events| Logger
    end
    
    subgraph Comm["Communication"]
        STM32 ---|UART| UART
        UART ---|Data| Gateway
        Gateway ---|HTTP| Telemetry
    end
    
    subgraph LogAnalysis["Post-Mortem Analysis"]
        Logger ---|Flash Data| MATLAB
        Logger ---|Flash Data| Python
    end
    
    style Hardware fill:#e1f5ff
    style STM32_FW fill:#fff3e0
    style ESP32_FW fill:#f3e5f5
    style Analysis fill:#e8f5e9
    style Sensors_Network fill:#fce4ec
    style ActuatorNetwork fill:#fce4ec
    style RTOS_Internal fill:#fff8e1
    style Comm fill:#f0f4c3
    style LogAnalysis fill:#e0f2f1
```

---

## Diagrama de Flujo de Datos

```mermaid
sequenceDiagram
    participant Sensors as Sensores<br/>(IMU, US, IR)
    participant STM32 as STM32F4<br/>(FreeRTOS)
    participant ESP32 as ESP32-S3<br/>(Gateway)
    participant Cloud as Cloud/Monitor
    participant Motors as Motores
    
    loop Ciclo Principal (~10ms)
        Sensors->>STM32: Enviar datos crudos
        STM32->>STM32: DSP Filters
        STM32->>STM32: Cinemática
        STM32->>STM32: Lógica de navegación
        STM32->>Parser: Registrar en Black Box
        STM32->>Motors: Comandos PWM
    end
    
    alt Cada 100ms
        STM32->>ESP32: UART - Telemetría
        ESP32->>Cloud: HTTP/MQTT
        Cloud-->>Monitor: Panel WiFi
    end
    
    alt Evento de Error
        STM32->>Parser: Log de Error
        Parser->>Cloud: Trigger análisis
    end
```

---

## Diagrama de Capas del Software

```mermaid
graph LR
    subgraph STM32_Layers["STM32 - Capas de Software"]
        Hardware_Abs["Hardware Abstraction Layer<br/>(HAL)"]
        Drivers["Device Drivers<br/>(Motor, Ultrasound, I2C)"]
        Middleware["Middleware<br/>(DSP, Kinematics, Logger)"]
        Tasks["RTOS Tasks<br/>(Main Control Logic)"]
    end
    
    Hardware_Abs --> Drivers
    Drivers --> Middleware
    Middleware --> Tasks
    
    style Hardware_Abs fill:#ffccbc
    style Drivers fill:#ffcc80
    style Middleware fill:#ffd54f
    style Tasks fill:#fff59d
```

---

## Diagrama de Estados del Sistema

```mermaid
stateDiagram-v2
    [*] --> Boot
    
    Boot --> Initialization: Power On
    Initialization --> Idle: Init Complete
    
    Idle --> Autonomous: Start Command
    Idle --> Manual: Manual Mode
    
    Autonomous --> Obstacle: Obstacle Detected
    Autonomous --> Cliff: Cliff Detected
    Autonomous --> Autonomous: Continue
    
    Obstacle --> Avoidance: Ultrasonic Alert
    Avoidance --> Autonomous: Path Clear
    
    Cliff --> Stop: IR Alert
    Stop --> Idle: Safety Mode
    
    Manual --> Idle: Stop Command
    Autonomous --> Idle: Stop Command
    
    Idle --> [*]: Power Off
    
    note right of Autonomous
        - DSP Filters On
        - Motor Control Active
        - Black Box Logging
    end note
    
    note right of Obstacle
        - Obstacle Avoidance
        - Ultrasonic Ranging
    end note
    
    note right of Cliff
        - Immediate Stop
        - Safety Protocol
    end note
```

---

## Diagrama de Interfaces de Comunicación

```mermaid
graph TB
    subgraph External["Dispositivos Externos"]
        MPU["MPU6050<br/>I2C"]
        US["HC-SR04<br/>GPIO/PWM"]
        IR["TCRT5000<br/>GPIO"]
        TIM["TB6612FNG<br/>PWM"]
    end
    
    subgraph STM32_Periph["Periféricos STM32"]
        I2C["I2C1 Bus"]
        GPIO["GPIO Port"]
        PWM["Timer PWM"]
        UART["UART3"]
    end
    
    subgraph ESP32_Periph["Periféricos ESP32"]
        UART_ESP["UART0"]
        WiFi["WiFi Module"]
    end
    
    subgraph Network["Red"]
        Router["WiFi Router/<br/>Gateway"]
    end
    
    MPU ---|I2C| I2C
    US ---|GPIO| GPIO
    IR ---|GPIO| GPIO
    TIM ---|PWM| PWM
    
    I2C --> STM32_Periph
    GPIO --> STM32_Periph
    PWM --> STM32_Periph
    
    UART ---|Serial| UART_ESP
    UART_ESP ---|WiFi| WiFi
    WiFi --> Router
    
    style External fill:#ffebee
    style STM32_Periph fill:#fff3e0
    style ESP32_Periph fill:#f3e5f5
    style Network fill:#e8f5e9
```

---

## Diagrama de Tareas RTOS (STM32)

```mermaid
graph TB
    Idle["Idle Task<br/>Priority: 0"]
    Sense["Sensor Task<br/>Priority: 4<br/>10ms"]
    Control["Control Task<br/>Priority: 3<br/>20ms"]
    DSP["DSP Filter Task<br/>Priority: 4<br/>10ms"]
    Motor["Motor Task<br/>Priority: 3<br/>20ms"]
    Logger["Logger Task<br/>Priority: 1<br/>100ms"]
    Comm["Comm Task<br/>Priority: 2<br/>50ms"]
    
    Idle --> Sense
    Sense --> DSP
    DSP --> Control
    Control --> Motor
    Motor --> Comm
    Comm --> Logger
    Logger --> Idle
    
    style Idle fill:#e0e0e0
    style Sense fill:#bbdefb
    style Control fill:#c8e6c9
    style DSP fill:#ffe0b2
    style Motor fill:#f8bbd0
    style Logger fill:#e1bee7
    style Comm fill:#c0cae4
```

---

## Diagrama de Módulos del Proyecto

```mermaid
graph LR
    Firmware["📦 Firmware"]
    Hardware_Des["📦 Hardware<br/>(KiCad)"]
    Mechanical["📦 Mechanical<br/>(CAD/BOM)"]
    Analysis["📦 Analysis<br/>(MATLAB/Python)"]
    Docs["📦 Docs<br/>(Doxygen)"]
    
    subgraph Dependencies
        FreeRTOS["FreeRTOS"]
        ESPIDF["ESP-IDF"]
        HAL["STM32 HAL"]
        KiCad["KiCad"]
    end
    
    Firmware --> FreeRTOS
    Firmware --> HAL
    Firmware --> ESPIDF
    Hardware_Des --> KiCad
    
    style Firmware fill:#fff3e0
    style Hardware_Des fill:#e0f2f1
    style Mechanical fill:#f3e5f5
    style Analysis fill:#e8f5e9
    style Docs fill:#fce4ec
    style Dependencies fill:#e0e0e0
```

---

**Fecha de creación**: 8 de marzo de 2026
