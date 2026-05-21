# stm32-rtos-sensor-hub

A real-time embedded firmware project using STM32 and FreeRTOS demonstrating multitasking, interrupt handling, ADC acquisition, inter-task communication, and synchronization mechanisms.

---

## Features

- FreeRTOS task scheduling
- Multi-task execution
- Interrupt-driven ADC
- Queue-based communication
- Binary semaphore synchronization
- UART debugging interface
- Real-time sensor processing

---

## Technologies Used

- STM32 Microcontroller
- Embedded C
- FreeRTOS
- UART Communication
- ADC Peripheral
- Interrupt Handling

---

## Project Architecture

ADC Interrupt → Queue → Processing Task → UART Output

---

## FreeRTOS Concepts Implemented

- Tasks
- Scheduler
- Queues
- Binary Semaphores
- ISR (Interrupt Service Routine)
- Task Synchronization

---

## Project Objective

The objective of this project is to demonstrate real-time multitasking and peripheral interaction using FreeRTOS on STM32
