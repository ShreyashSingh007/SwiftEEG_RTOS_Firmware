"""Guarded edits to swifteeg_app.py: filters on the PC or on the device."""
import pathlib

P = pathlib.Path(r"D:\Electronics Projects\EEG Project\SwiftEEG\RTOS Firmware\RTOS\tools\swifteeg_app.py")

EDITS = [
# 1. docstring
('''  * The filters are built from the same biquad forms the firmware uses, so a
    chain that works here transfers to the device as coefficients rather than
    as a rewrite. That is what makes moving the DSP on-chip a configuration
    step and not a second implementation.
''',
'''  * The same filters can run on the device instead. They are designed here
    and sent to it as sections, and it then streams raw and filtered side by
    side: raw for the recording, filtered for the plot. Moving the DSP
    on-chip is a switch, not a second implementation.
'''),

# 2. no encoding picker: the app chooses the stream it needs
('''        rc.bind("<<ComboboxSelected>>", lambda e: self._set_rate())

        self._label(row, "  format").pack(side=tk.LEFT)
        # "device filtered" is what makes the on-device DSP visible. "raw"
        # sends counts straight off the converter, before the device touches
        # them - so with it selected, the on-device notch correctly appears
        # to do nothing. There is no 32-bit raw here: the converter is
        # 24-bit, and a fourth byte would only repeat the sign.
        self.enc_var = tk.StringVar(value="raw 24-bit")
        ec = ttk.Combobox(row, textvariable=self.enc_var, width=14,
                          state="readonly",
                          values=["raw 24-bit", "device filtered"])
        ec.pack(side=tk.LEFT, padx=6)
        ec.bind("<<ComboboxSelected>>", lambda e: self._set_encoding())
''',
'''        rc.bind("<<ComboboxSelected>>", lambda e: self._set_rate())
'''),

# 3. the separate device notch goes: the device runs the whole chain now
('''        # -- device notch --
        f = self._section(inner, "on-device filter")
        row = tk.Frame(f, bg=PANEL)
        row.pack(fill=tk.X)
        self._label(row, "mains notch").pack(side=tk.LEFT)
        self.dev_notch = tk.StringVar(value="50 Hz")
        nc = ttk.Combobox(row, textvariable=self.dev_notch, width=8,
                          state="readonly", values=["off", "50 Hz", "60 Hz"])
        nc.pack(side=tk.LEFT, padx=6)
        nc.bind("<<ComboboxSelected>>", lambda e: self._set_dev_notch())

        # -- motion sensor --
''',
'''        # -- motion sensor --
'''),

# 4. where the filters run
('''        # -- host filters --
        f = self._section(inner, "host filters (display)")
        self.hp_var = tk.StringVar(value="0.5")
''',
'''        # -- filters --
        f = self._section(inner, "filters (display)")

        # Where the chain runs. On the device it is the same chain, sent as
        # sections; the recording stays raw either way.
        self.site_var = tk.StringVar(value="PC")
        self._combo_row(f, "run on", self.site_var, ["PC", "device"],
                        self._set_filter_site, "")
        self.site_label = self._label(f, "", fg=DIM, font=("Consolas", 8))
        self.site_label.pack(fill=tk.X, pady=(0, 4))

        self.hp_var = tk.StringVar(value="0.5")
'''),

# 5. state
('''        self.rec_rows = 0

        # Display state.
''',
'''        self.rec_rows = 0

        # Where the filters run. On the device, what it was last sent is
        # kept per stage, so an unchanged stage is never sent - and so never
        # restarted - again.
        self.filters_on_device = False
        self._dev_sent: dict = {}
        self._dev_settle_t = -math.inf
        self._dev_applied_seq: int | None = None

        # Display state.
'''),

# 6. disconnect forgets what the device holds
('''            self.link = None
            self.btn_conn.config(text="Connect")
            self.status.config(text="not connected", fg=DIM)
            return
''',
'''            self.link = None
            self._dev_sent = {}
            self.btn_conn.config(text="Connect")
            self.status.config(text="not connected", fg=DIM)
            return
'''),

# 7. a rate change clears the device's sections
('''        if was_streaming:
            # The device is stopping a thread and reconfiguring the AFE.
            # Asking it to stream again before that finishes is ignored.
            self.after(2000, self._resume_after_rate)

    def _resume_after_rate(self) -> None:
        self._send(link.CMD_STREAM_START)
        self._send(link.CMD_GET_CONFIG)
''',
'''        # The device drops any sections it was sent: designed for the old
        # rate, they would be different filters at the new one.
        self._dev_sent = {}

        if was_streaming:
            # The device is stopping a thread and reconfiguring the AFE.
            # Asking it to stream again before that finishes is ignored.
            self.after(2000, self._resume_after_rate)
        elif self.filters_on_device:
            self.after(2000, self._push_device_chain)

    def _resume_after_rate(self) -> None:
        self._send(link.CMD_STREAM_START)
        self._send(link.CMD_GET_CONFIG)
        self._push_device_chain()
'''),

# 8. link reset
('''        self._was_connected = False
        self.chain.reset()
        self._reset_traces()
''',
'''        self._was_connected = False
        self._dev_sent = {}
        self.chain.reset()
        self._reset_traces()
'''),

# 9. site switching, replacing the encoding picker's handler
('''    def _set_encoding(self) -> None:
        pick = self.enc_var.get()
        enc = {"raw 24-bit": link.ENC_RAW_I24,
               "device filtered": link.ENC_UV_F32}.get(pick, link.ENC_RAW_I24)
        self._send(link.CMD_SET_ENCODING, enc)
        self.chain.reset()
''',
'''    def _set_filter_site(self) -> None:
        """
        Run the chain here, or on the device.

        On the device it is this chain, sent as sections, and the stream
        carries raw counts and the device's microvolts side by side: raw for
        the recording and the limit checks, filtered for the plot. Back on
        the PC the device returns to its own default, the mains notch alone,
        and is left as any other host would expect to find it.
        """
        self.filters_on_device = self.site_var.get() == "device"
        self._dev_sent = {}
        self._dev_applied_seq = None
        self.chain.reset()

        if self.filters_on_device:
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_UV)
            self._push_device_chain()
            self.site_label.config(text="sending the filters to the device...",
                                   fg="#ffd866")
        else:
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
            self._restore_device_chain()
            self.site_label.config(text="", fg=DIM)

    def _push_device_chain(self, keep_state: bool = False) -> None:
        """
        Send the device whatever part of the chain it does not already have.

        A stage whose sections are unchanged is not sent, so changing the
        low-pass does not restart the high-pass - the rule the chain here
        follows too. `keep_state` is for the notch following the mains: a
        retune small enough that restarting for it would only add a
        transient.
        """
        if not (self.filters_on_device and self.link and self.link.connected):
            return

        pre, post = self.chain.device_stages()
        for stage, sections in ((link.STAGE_PRE, pre), (link.STAGE_POST, post)):
            key = (self.rate, tuple(sections))
            previous = self._dev_sent.get(stage)
            if previous == key:
                continue
            keep = (keep_state and previous is not None
                    and len(previous[1]) == len(sections))
            self._send(link.CMD_SET_FILTER,
                       *link.filter_args(stage, sections, self.rate, keep))
            self._dev_sent[stage] = key

        car = (bool(self.chain.car), self.chain.car_bits)
        if self._dev_sent.get("car") != car:
            self._send(link.CMD_SET_CAR, 1 if car[0] else 0, car[1])
            self._dev_sent["car"] = car

    def _restore_device_chain(self) -> None:
        """The device's own default: its mains notch, no average, no low-pass."""
        hz = {"off": 0, "50 Hz": 50, "60 Hz": 60}[self.hnotch_var.get()]
        self._send(link.CMD_SET_FILTER,
                   *link.filter_args(link.STAGE_POST, [], self.rate))
        self._send(link.CMD_SET_NOTCH, hz)
        self._send(link.CMD_SET_CAR, 0, 0xFF)

    def _back_to_pc(self, why: str) -> None:
        """Filters back on the PC, saying why - never a silently wrong plot."""
        self.site_var.set("PC")
        self.filters_on_device = False
        self._dev_sent = {}
        self.chain.reset()
        self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
        self.site_label.config(text=why, fg="#ff6b6b")
'''),

# 10. the device notch control's handler goes with it
('''    def _set_dev_notch(self) -> None:
        hz = {"off": 0, "50 Hz": 50, "60 Hz": 60}[self.dev_notch.get()]
        self._send(link.CMD_SET_NOTCH, hz)

''',
''''''),

# 11. settings reach the device as they change
('''        self.chain.notch_track = self.track_var.get()
        self.chain.rebuild()

    def _set_car_mask(self) -> None:
        self.chain.car_mask = np.array([v.get() for v in self.in_avg],
                                       dtype=bool)
''',
'''        self.chain.notch_track = self.track_var.get()
        self.chain.rebuild()
        self._push_device_chain()

    def _set_car_mask(self) -> None:
        self.chain.car_mask = np.array([v.get() for v in self.in_avg],
                                       dtype=bool)
        self._push_device_chain()
'''),

# 12. recordings are raw, always
('''        # Raw counts, not microvolts: the scale depends on the gain, and a
        # recording that has already been filtered cannot be un-filtered.
        what = ("device filtered uV" if self.enc_var.get() == "device filtered"
                else "raw counts")
        self.recorder.writerow(
            [f"# SwiftEEG {what}", f"rate={self.rate}", f"gain={self.gain}",
             f"lsb_uv={link.lsb_uv(self.gain):.9f}"])
''',
'''        # Raw counts, not microvolts: the scale depends on the gain, and a
        # recording that has already been filtered cannot be un-filtered.
        # With the filters on the device this still holds - it sends raw
        # alongside, and raw is what is kept.
        self.recorder.writerow(
            ["# SwiftEEG raw counts", f"rate={self.rate}", f"gain={self.gain}",
             f"lsb_uv={link.lsb_uv(self.gain):.9f}"])
'''),

# 13. data frames carry raw, and the device's microvolts when it filters
('''                ts, seq, enc, vals = got
                self.frames += 1

                if self.last_seq is not None and seq != self.last_seq:
                    self.gaps += 1
                self.last_seq = seq + len(vals)

                block_raw.append((ts, seq, vals))
''',
'''                ts, seq, _, counts, uv = got
                if counts is None:
                    # Microvolts alone, left by another host: nothing to
                    # record or check until the config reply fixes it.
                    continue
                self.frames += 1

                if self.last_seq is not None and seq != self.last_seq:
                    self.gaps += 1
                self.last_seq = seq + len(counts)

                if f.flags & link.FLAG_SETTLING:
                    self._dev_settle_t = now

                block_raw.append((ts, seq, counts, uv))
'''),

# 14. responses
('''    def _on_response(self, p: bytes) -> None:
        if len(p) < 2 or p[0] != link.CMD_GET_CONFIG or p[1] != 0:
            return
        if len(p) < 7:
            return

        sps = p[4] | (p[5] << 8)
        if sps in (250, 500, 1000):
            self.rate_var.set(str(sps))
            if sps != self.rate:
                self.rate = sps
                self._reset_traces()
            self.chain.set_rate(sps)

        enc = p[3]
        if enc == link.ENC_RAW_I32:
            # The same samples as 24-bit plus a byte of sign extension. A
            # board flashed before 24-bit became its default still starts here.
            self._send(link.CMD_SET_ENCODING, link.ENC_RAW_I24)
            enc = link.ENC_RAW_I24
        self.enc_var.set({link.ENC_RAW_I24: "raw 24-bit",
                          link.ENC_UV_F32: "device filtered"}.get(
                              enc, "raw 24-bit"))
        self.dev_notch.set({0: "off", 50: "50 Hz", 60: "60 Hz"}.get(p[6], "off"))

        if len(p) >= 15:
            chset = p[7:15]
            code = (chset[0] >> 4) & 0x07
            self.gain = link.GAIN_FROM_CODE.get(code, 24)
            self.gain_var.set(str(self.gain))
            mux = chset[0] & 0x07
            self.src_var.set({link.MUX_NORMAL: "Electrodes",
                              link.MUX_SHORTED: "Shorted (noise)",
                              link.MUX_TEST: "Test signal"}.get(mux,
                                                                "Electrodes"))

        # Motion sensor, from firmware that has one: flags (bit 0 on, bit 1
        # fitted), rate, ranges. Older firmware sends none of it.
        if len(p) >= 21:
            self.imu_present = bool(p[15] & 0x02)
            hz = p[16] | (p[17] << 8)
            self.imu_rate_var.set(str(hz) if p[15] & 0x01 else "off")
            self.imu_acc_var.set(str(p[18]))
            self.imu_gyro_var.set(str(p[19] | (p[20] << 8)))

        self.status.config(text=f"connected - {sps} SPS, gain {self.gain}",
                           fg="#5ed18b")
''',
'''    def _on_response(self, p: bytes) -> None:
        if len(p) < 2:
            return
        if p[0] in (link.CMD_SET_FILTER, link.CMD_SET_CAR):
            self._on_filter_response(p)
            return

        cfg = link.decode_config(p)
        if cfg is None:
            return

        sps = cfg["rate"]
        if sps in (250, 500, 1000):
            self.rate_var.set(str(sps))
            if sps != self.rate:
                self.rate = sps
                self._reset_traces()
                self._dev_sent = {}
            self.chain.set_rate(sps)

        # The stream this app needs: raw, plus the device's microvolts when
        # the filters run there. A board left on anything else - an older
        # firmware's 32-bit default, another host's choice - is moved to it.
        want = link.ENC_RAW_UV if self.filters_on_device else link.ENC_RAW_I24
        if cfg["encoding"] != want:
            self._send(link.CMD_SET_ENCODING, want)

        if "gains" in cfg:
            self.gain = cfg["gains"][0] or 24
            self.gain_var.set(str(self.gain))
            self.src_var.set({link.MUX_NORMAL: "Electrodes",
                              link.MUX_SHORTED: "Shorted (noise)",
                              link.MUX_TEST: "Test signal"}.get(
                                  cfg["mux"][0], "Electrodes"))

        # Motion sensor, from firmware that has one. Older firmware sends
        # none of it.
        if "imu_rate" in cfg:
            self.imu_present = cfg["imu_fitted"]
            self.imu_rate_var.set(str(cfg["imu_rate"]) if cfg["imu_on"]
                                  else "off")
            self.imu_acc_var.set(str(cfg["imu_accel_g"]))
            self.imu_gyro_var.set(str(cfg["imu_gyro_dps"]))

        if self.filters_on_device:
            if "pre_crc" not in cfg:
                self._back_to_pc("this firmware cannot run the filters")
            else:
                # What the device holds against what it should. A rate change
                # or a restart clears its copy, which is then sent again.
                pre, post = self.chain.device_stages()
                if (cfg["pre_crc"] != link.sections_crc(pre)
                        or cfg["post_crc"] != link.sections_crc(post)
                        or cfg["pre_count"] != len(pre)
                        or cfg["post_count"] != len(post)):
                    self._dev_sent = {}
                    self._push_device_chain()

        self.status.config(text=f"connected - {sps} SPS, gain {self.gain}",
                           fg="#5ed18b")

    def _on_filter_response(self, p: bytes) -> None:
        if not self.filters_on_device:
            return  # restoring the device's default; nothing to show

        if p[1] == link.STATUS_OK:
            seq = link.response_seq(p)
            if seq is not None:
                self._dev_applied_seq = seq
                self.site_label.config(
                    text=f"on the device from sample {seq}", fg="#5ed18b")
            return

        what = "filters" if p[0] == link.CMD_SET_FILTER else "common average"
        self._back_to_pc(f"the device refused the {what} - back on the PC")
'''),

# 15. the plot shows the device's output when it filters
('''    def _consume(self, blocks) -> None:
        counts = np.vstack([v for _, _, v in blocks])

        # "device filtered" arrives already in microvolts, having been
        # through the device's own chain. Scaling it by the LSB again would
        # divide it by 45 million.
        already_uv = self.enc_var.get() == "device filtered"
        scale = 1.0 if already_uv else link.lsb_uv(self.gain)

        self.samples += len(counts)
''',
'''    def _consume(self, blocks) -> None:
        counts = np.vstack([c for _, _, c, _ in blocks])
        scale = link.lsb_uv(self.gain)

        self.samples += len(counts)
'''),

('''            for ts0, seq0, vals in blocks:
''',
'''            for ts0, seq0, vals, _ in blocks:
'''),

('''        # Only meaningful on raw counts; the filtered form has had its DC
        # removed on the device and can no longer show the input limit.
        if not already_uv:
            self._check_limits(counts)
        for ch in range(link.CHANNELS):
''',
'''        self._check_limits(counts)
        for ch in range(link.CHANNELS):
'''),

('''        # a re-aim; restarting it here was putting each electrode's whole
        # offset back through the high-pass every few seconds.
        if not already_uv:
            self.mains_buf.extend(uv.mean(axis=1))
            now = time.time()
            if now >= self.mains_next and len(self.mains_buf) >= self.rate * 4:
                self.mains_next = now + 5.0
                self.chain.update_mains(np.array(self.mains_buf))

        out = self.chain.process(uv)

        # Where the newest batch sits on the device clock. The motion lanes
        # are placed by time, and this is what gives them the EEG's axis.
        last_ts, _, last_vals = blocks[-1]
''',
'''        # a re-aim; restarting it here was putting each electrode's whole
        # offset back through the high-pass every few seconds. On the device
        # the notch is retuned the same way.
        self.mains_buf.extend(uv.mean(axis=1))
        now = time.time()
        if now >= self.mains_next and len(self.mains_buf) >= self.rate * 4:
            self.mains_next = now + 5.0
            if self.chain.update_mains(np.array(self.mains_buf)):
                self._push_device_chain(keep_state=True)

        # The device's own output when it is filtering; this chain otherwise,
        # and for any batch still arriving in the old form just after a
        # switch.
        device = [u for _, _, _, u in blocks]
        if self.filters_on_device and all(u is not None for u in device):
            out = np.vstack(device).astype(np.float64)
        else:
            out = self.chain.process(uv)

        # Where the newest batch sits on the device clock. The motion lanes
        # are placed by time, and this is what gives them the EEG's axis.
        last_ts, _, last_vals, _ = blocks[-1]
'''),

# 16. settling label
('''        settled = (time.time() - self.started_at) > self.chain.settling_seconds
        self._cfg(lay["settle"], text=(
            f"filters settling ({self.chain.settling_seconds:.0f} s)"
            if not settled and self.samples else ""))
''',
'''        if self.filters_on_device:
            # The device flags the samples inside its filters' settling time.
            settling = now - self._dev_settle_t < 0.5
            text = "device filters settling" if settling and self.samples else ""
        else:
            settled = ((time.time() - self.started_at)
                       > self.chain.settling_seconds)
            text = (f"filters settling ({self.chain.settling_seconds:.0f} s)"
                    if not settled and self.samples else "")
        self._cfg(lay["settle"], text=text)
'''),

# 17. stats
('''        self.stats.config(text="\\n".join([
            f"{self.samples} samples  {self.samples / el:6.1f} SPS",
            f"{self.frames} frames  {bad} bad  {self.gaps} gaps{rec}",
            f"DC mV: {dc}",
            motion,
            display]))
''',
'''        site = "filters on the device" if self.filters_on_device \\
            else "filters on the PC"

        self.stats.config(text="\\n".join([
            f"{self.samples} samples  {self.samples / el:6.1f} SPS",
            f"{self.frames} frames  {bad} bad  {self.gaps} gaps{rec}",
            f"DC mV: {dc}",
            motion,
            site,
            display]))
'''),
]


def main():
    raw = P.read_bytes()
    crlf = b"\r\n" in raw
    s = raw.decode("utf-8").replace("\r\n", "\n")

    for i, (old, new) in enumerate(EDITS, 1):
        n = s.count(old)
        assert n == 1, f"edit {i}: found {n} times"
        s = s.replace(old, new)

    for gone in ("enc_var", "dev_notch", "already_uv", "_set_encoding",
                 "_set_dev_notch"):
        assert gone not in s, f"{gone} still referenced"

    if crlf:
        s = s.replace("\n", "\r\n")
    P.write_bytes(s.encode("utf-8"))
    print(f"applied {len(EDITS)} edits ({'CRLF' if crlf else 'LF'})")


if __name__ == "__main__":
    main()
