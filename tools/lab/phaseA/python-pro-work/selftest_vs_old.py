# Run the new self-test with the old FrameParser swapped in: it must fail.
import sys
sys.path.insert(0, r"D:/Electronics Projects/EEG Project/SwiftEEG/RTOS Firmware/RTOS/tools")
sys.path.insert(0, r"C:/Users/shrey/AppData/Local/Temp/claude/D--Electronics-Projects-EEG-Project-SwiftEEG-RTOS-Firmware/005c4da0-af1b-4f26-97b9-4ee4c4c7cdce/scratchpad/phaseA/python-pro-work")
import swifteeg_link as new, old_link as old
class OldParser(old.FrameParser):
    def discard(self):
        if self.buf: self.buf.clear(); self.bad += 1
new.FrameParser = OldParser
try:
    new._self_test()
    print("OLD PARSER PASSED - test has no teeth")
except AssertionError as e:
    print("old parser fails the self-test as expected:", e)
