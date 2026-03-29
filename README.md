# mhi-ac-ctrl-esp32
Control MHI Airconditioning locally with ESPHome!

This code is inspired by [@absalom-muc's MHI-AC-Ctrl](https://github.com/absalom-muc/MHI-AC-Ctrl), [@mriksman's esp32 homekit implementation](https://github.com/mriksman/esp32_homekit_mhi/blob/e4a8a4382b990c8e64463411c47e911d1741d9d1/main/main.c) and [@ginkage's MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome). Compared to the popular implementation of @absalom-muc and @ginkage, this repository uses the hardware SPI peripheral instead of a software based implementation. That improves reliability and frees up CPU resources for other tasks. 

## Hardware

### Officialy supported (tested by @hberntsen):
* [ESP-C3-32S(4M)-KIT](https://nl.aliexpress.com/item/1005002983050962.html).
  Requires an additional PCB for the AC interface. See the [kicad folder](kicad) folder for the PCB design (manufactured via [Aisler](https://aisler.net)).
* ESP32-S3 (design your own PCB). Putting the SPI clock on pins other than 43 (UART TX) [is not stable](https://esp32.com/viewtopic.php?t=47883).
* 2022 models of these indoor AC units:
  * SRK50ZS-W
  * SRK25ZS-W

### Community supported:
* [ESP32-C6](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/pull/25)
* Other ESP32-C3 boards (e.g. [ESP32-C3 super mini](https://github.com/xangin/mhi-ac-ctrl-esp32-c3/blob/master/Hardware.md)).
* The ESP32 [has been reported to work](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/issues/12#issuecomment-2594627493). Make sure you use the right pins, as discussed in [#11](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3/issues/11).
* The AC units listed at https://github.com/absalom-muc/MHI-AC-Ctrl?tab=readme-ov-file#prerequisites should work. Some older units have to use the `use_long_frame: false` setting.

### Not supported
[ESP32 chips without RMT peripheral](https://github.com/esphome/esphome/blob/dev/esphome/components/esp32_rmt/__init__.py#L7). At the moment of writing that's the ESP32-C2 and ESP32-C61. The RMT is used to derive an SPI CS signal from the clock. Users of these chips might try an older version of the code, i.e. commit 3ddb8cb, that uses timers and interrupts for this.

## Getting started

This project is used in ESPHome as external component. The [ESPHome documentation](https://esphome.io/guides/getting_started_hassio) will help you getting started with ESPHome. You can base off your configuration from the [`example.yaml`](esphome/example.yaml) included in this repository. Once you got that going and want to an external temperature sensor, have a look at [`example-external-sensor.yaml`](esphome/example-external-sensor.yaml).

In general, the latest version of ESPHome should work. See the [build workflow](.github/workflows/build-example.yml#L17) for the latest version of ESPHome that was tested.
