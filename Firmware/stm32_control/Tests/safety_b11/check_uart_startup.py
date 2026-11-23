"""Guard the boot call site too: the native harness does not execute HAL main."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[2]
main = (root / 'Core/Src/main.c').read_text(encoding='utf-8')
main = re.sub(r'/\*.*?\*/|//[^\n]*', '', main, flags=re.S)
assert re.search(r'\bDrvUart_Init\s*\(\s*&esp_uart\s*,\s*&huart2\s*\)', main)
assert not re.search(r'\bDrvUart_StartReceive\s*\(', main), 'RX must not start before the scheduler in main'
assert not re.search(r'\bxTaskGetTickCountFromISR\s*\(', main)
print('PASS UART boot guard: main initializes the driver without arming its ISR receive path')
