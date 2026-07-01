# Board3 MCP2515 Pin Map Patch — 2026-07-01

Changed Board3 MCP2515 control pins as requested:

| Function | Old | New |
|---|---|---|
| MCP2515 CS | PA10 | PB12 |
| MCP2515 INT | PA9 / EXTI9 | PB4 / EXTI4 |

SPI2 data pins stay unchanged:

| Function | Pin |
|---|---|
| SCK | PB13 |
| MISO | PB14 |
| MOSI | PB15 |

Updated files: `spi.c`, `mcp2515.c`, `main.c`, README/patch notes.
