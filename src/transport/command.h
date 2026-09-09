/*
 * Host command handling.
 *
 * Decodes protocol CMD frames arriving over USB and acts on them. This is
 * the smallest useful command set - enough to drive acquisition and switch
 * the AFE's test generator on from the host - not the full surface, which
 * arrives with the rest of the protocol work.
 *
 * Every command is answered with an RSP frame carrying the same opcode and
 * a status byte, so the host can tell "rejected" from "lost".
 */
#ifndef SWIFTEEG_TRANSPORT_COMMAND_H
#define SWIFTEEG_TRANSPORT_COMMAND_H

/* Opcodes, first byte of a CMD payload. */
#define CMD_PING         0x01u
#define CMD_STREAM_START 0x02u
#define CMD_STREAM_STOP  0x03u
#define CMD_SET_ENCODING 0x04u /* payload[1] = STREAM_ENC_* */
#define CMD_TEST_SIGNAL  0x05u /* payload[1] = on, payload[2] = cal_freq */
#define CMD_GET_INFO     0x06u
#define CMD_READ_REG     0x07u /* payload[1] = address; RSP carries the value */
#define CMD_SET_INPUT    0x08u /* payload[1] = ADS1299_MUX_*, [2] = cal freq */

/* Status byte in an RSP payload. */
#define CMD_OK        0x00u
#define CMD_EBADARG   0x01u
#define CMD_EFAILED   0x02u
#define CMD_EUNKNOWN  0x03u

/* Starts the thread that services host commands. */
int command_init(void);

#endif /* SWIFTEEG_TRANSPORT_COMMAND_H */
