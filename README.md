# mhi-ac-ctrl-esp32-c3
Control MHI Airconditioning locally using your own [ESP-C3-32S(4M)-KIT](https://www.aliexpress.com/item/1005003152986418.html)!

This code is based on [@absalom-muc's MHI-AC-Ctrl](https://github.com/absalom-muc/MHI-AC-Ctrl), [@mriksman's esp32 homekit implementation](https://github.com/mriksman/esp32_homekit_mhi/blob/e4a8a4382b990c8e64463411c47e911d1741d9d1/main/main.c) and [@ginkage's MHI-AC-Ctrl-ESPHome](https://github.com/ginkage/MHI-AC-Ctrl-ESPHome).

The kicad folder contains a board design for using with the [ESP-C3-32S(4M)-KIT](https://www.aliexpress.com/item/1005003152986418.html). The PCB works great (ordered via https://aisler.net).

The code is running reliably at three AC units at @hberntsen's home :).

Note that you'll need to enable active mode in Home Assistant before sending commands to the AC will work. Without that enabled, the ESP will passively listen to data and you'll be able to use timers again via the IR remote. It is off after boot. You could automate turning on/off the active mode when something is wrong. An automation blueprint to automatically enable it is included in the homeassistant directory.

