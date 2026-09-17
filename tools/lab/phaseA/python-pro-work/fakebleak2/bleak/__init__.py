import asyncio, time
LOG = []
MODE = {"found": True, "connect_s": 0.02, "drop_after": None, "write_fails": False, "scan_s": 0.0}
class BleakScanner:
    @staticmethod
    async def find_device_by_name(name, timeout=20.0):
        await asyncio.sleep(MODE["scan_s"])
        return "AA:BB" if MODE["found"] else None
class BleakClient:
    mtu_size = 247
    def __init__(self, dev, disconnected_callback=None):
        self.cb = disconnected_callback
    async def __aenter__(self):
        await asyncio.sleep(MODE["connect_s"]); LOG.append("connect")
        if MODE["drop_after"] is not None:
            asyncio.get_running_loop().call_later(MODE["drop_after"], self.cb, self)
        return self
    async def __aexit__(self, *a):
        LOG.append("disconnect")
    async def start_notify(self, uuid, cb):
        LOG.append("notify")
    async def write_gatt_char(self, uuid, data, response=False):
        if MODE["write_fails"]:
            raise OSError("GATT write refused")
        LOG.append(("write", data[8]))
