/*
 * Host command handling.
 *
 * Decodes protocol CMD frames arriving over USB or BLE and acts on them.
 * Every device control is here, and every one is answered while streaming.
 *
 * Every command is answered with an RSP frame carrying the same opcode and
 * a status byte, so the host can tell "rejected" from "lost".
 */
#ifndef SWIFTEEG_TRANSPORT_COMMAND_H
#define SWIFTEEG_TRANSPORT_COMMAND_H

/*
 * Not a real opcode a host ever sends: what a reply to a CMD frame with no
 * payload at all carries in place of one, since there is no opcode byte to
 * echo back (R5-CONTRACT-06).
 */
#define CMD_OP_NONE      0x00u

/* Opcodes, first byte of a CMD payload. */
#define CMD_PING         0x01u
#define CMD_STREAM_START 0x02u
#define CMD_STREAM_STOP  0x03u
#define CMD_SET_ENCODING 0x04u /* payload[1] = STREAM_ENC_* */
#define CMD_TEST_SIGNAL  0x05u /* payload[1] = on, payload[2] = cal_freq */
#define CMD_GET_INFO     0x06u
#define CMD_READ_REG     0x07u /* payload[1] = address; RSP carries the value */
#define CMD_SET_INPUT    0x08u /* payload[1] = ADS1299_MUX_*, [2] = cal freq */
#define CMD_SET_RATE     0x09u /* payload[1..2] = SPS, little-endian */
#define CMD_SET_CHANNEL  0x0Au /* [1]=ch or 0xFF, [2]=gain code 0-6, [3]=mux
                                *   0-7, [4]=power down, [5]=srb2 */
#define CMD_SET_BIAS     0x0Bu /* [1]=enable, [2]=sensp mask, [3]=sensn mask */
#define CMD_SET_NOTCH    0x0Cu /* [1]=nominal Hz (0 removes it, 50 or 60),
                                *   [2]=Q, [3]=flags (bit 0 harmonic, bit 1
                                *   follow the measured mains); RSP carries
                                *   the sequence number it applies from */
#define CMD_SET_LEADOFF  0x0Du /* [1]=enable, [2]=sensp, [3]=sensn */
#define CMD_GET_CONFIG   0x0Eu /* no args; RSP carries the whole state */
#define CMD_SET_IMU      0x0Fu /* [1]=enable, [2..3]=rate Hz, [4]=accel g,
                                *   [5..6]=gyro dps */
#define CMD_SET_FILTER   0x10u /* [1]=stage, [2]=flags (bit 0 keep state),
                                *   [3..4]=SPS the sections were designed
                                *   for, [5]=count, then count x (g, k, m0,
                                *   m1, m2) as float32; RSP carries the u32
                                *   sequence number it applies from */
#define CMD_SET_CAR      0x11u /* [1]=enable, [2]=channel mask; RSP carries
                                *   the sequence number */
#define CMD_RESET_CHAIN  0x12u /* no args; RSP carries the sequence number
                                *   the restart applies from */

/* Status byte in an RSP payload. */
#define CMD_OK        0x00u
#define CMD_EBADARG   0x01u
#define CMD_EFAILED   0x02u
#define CMD_EUNKNOWN  0x03u

/* Starts the thread that services host commands. */
int command_init(void);

#endif /* SWIFTEEG_TRANSPORT_COMMAND_H */
