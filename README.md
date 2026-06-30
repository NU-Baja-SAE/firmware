# NU Baja SAE — Firmware

PlatformIO multi-env monorepo for all NU Baja SAE board firmware. All boards are ESP32 (`espressif32` / `nodemcu-32s`, matching [Dingo_Wheels](https://github.com/NU-Baja-SAE/Dingo_Wheels)).

## Structure

```
firmware/
├── platformio.ini           # one [env:name] per board
├── boards/
│   ├── ecvt-controller/src/
│   ├── daq-node/src/
│   └── hud-node/src/
├── lib/
│   ├── baja-can/             # MCP2515 driver wrapper, send/recv helpers
│   ├── baja-messages/        # generated from DBC (cantools codegen)
│   └── baja-common/          # shared utils
├── dbc/baja-can.dbc          # single source of truth for CAN messages
├── scripts/gen_messages.py   # dbc -> lib/baja-messages headers
└── .github/workflows/build.yml
```

## Uploading firmware

`default_envs` in `platformio.ini` is intentionally left unset, so a bare `pio run` will not silently flash the wrong board. Always specify the environment:

```
pio run -e <board-name> -t upload
```

Valid board names: `ecvt-controller`, `daq-node`, `hud-node`.

## Regenerating CAN messages

`dbc/baja-can.dbc` is the single source of truth for CAN messages. Run `scripts/gen_messages.py` to regenerate `lib/baja-messages` headers after editing the DBC.
