import asyncio, time
LOG = []
SCAN_S = 0.0
class BleakScanner:
    @staticmethod
    async def find_device_by_name(name, timeout=20.0):
        await asyncio.sleep(SCAN_S)
        return "AA:BB"
class BleakClient:
    mtu_size = 247
    def __init__(self, dev, disconnected_callback=None):
        self.dev = dev
    async def __aenter__(self):
        await asyncio.sleep(0.02); LOG.append(("connect", time.perf_counter())); return self
    async def __aexit__(self, *a):
        LOG.append(("disconnect", time.perf_counter()))
    async def start_notify(self, uuid, cb):
        pass
    async def write_gatt_char(self, uuid, data, response=False):
        LOG.append(("write", data[8], time.perf_counter()))
