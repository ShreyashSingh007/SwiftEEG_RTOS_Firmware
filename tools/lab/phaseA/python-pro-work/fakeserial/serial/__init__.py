import time
MODE = {"read_fails_after": None, "write_fails": False, "data": b""}
OPEN = []
class Serial:
    def __init__(self, port, baud, timeout=0.05):
        self.port, self.timeout, self.closed, self.t0, self.written = port, timeout, False, time.time(), []
        self.data = MODE["data"]; OPEN.append(self)
    def read(self, n):
        if self.closed: raise OSError("port closed")
        after = MODE["read_fails_after"]
        if after is not None and time.time() - self.t0 > after: raise OSError("device unplugged")
        if self.data:
            chunk, self.data = self.data[:n], self.data[n:]
            return chunk
        time.sleep(self.timeout); return b""
    def write(self, b):
        if MODE["write_fails"]: raise OSError("write refused")
        self.written.append(bytes(b))
    def close(self): self.closed = True
