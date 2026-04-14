# PLFM RTOS Firmware (Phase 2 Integration)

This module provides a layered RTOS migration path for the STM32 firmware:

- HAL/BSP bindings via `plfm_hal_port.*`
- Drivers and ISR handoff
- DSP and protocol services
- Application task orchestration

## Phase 2 Status

- RTOS startup path is wired from legacy `main.cpp`.
- Timer and DMA ISR hooks are bridged into `plfm_tasks_notify_*`.
- Communication task emits framed binary telemetry.
- Watchdog servicing moved into monitor task.

## HAL Port Bindings

Implement (or override weak defaults):
- `plfm_hal_port_start_acq_dma(...)`
- `plfm_hal_port_start_pri_timer(...)`
- `plfm_hal_port_stream_tx(...)`
- `plfm_hal_port_watchdog_kick()`
- `plfm_hal_port_get_time_us()`

Weak defaults exist in `src/plfm_hal_port.c` for portability.

## Simulation Fallback

When acquisition DMA is not yet mapped, `task_acq` generates synthetic I/Q data,
allowing end-to-end scheduler, DSP, queue, and protocol testing before board ADC
integration is complete.

