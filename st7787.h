/* Nanosecond timings from ST7787 datasheet. */
/* Parallel interface timings. */
#define T_AST 10
#define T_RC 160
#define T_RDH 90
#define T_DST 20
#define T_DHT 20
#define T_RAT 40

/* Serial interface timings. */
#define T_CSS 60
#define T_SCC 20
#define T_CHW 40
#define T_SHW 20
#define T_SLW 20
#define T_SHR 60
#define T_SLR 60


#define C_NOP 0x00
#define C_SWRESET 0x01
#define C_RDDID 0x04
#define C_RDDST 0x09
#define C_SLPOUT 0x11
#define C_DISPON 0x29
#define C_CASET 0x2A
#define C_RASET 0x2B
#define C_RAMWR 0x2C
#define C_COLMOD 0x3A
#define C_VSYNCOUT 0xBC
#define C_VSYNCIN 0xBD
