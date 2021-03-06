#define BOOL uint8_t //Define a datatype

#define DS1307_SLA_W 0xD0 //Write address of the DS1307
#define DS1307_SLA_R 0xD1 //Read address

#define TRUE	1 //Define return values from Soft I2C functions
#define FALSE	0

extern void DS1307Init(void);
extern BOOL DS1307Read(uint8_t,uint8_t *);
extern BOOL DS1307Write(uint8_t,uint8_t);
extern void TXRegisters(void);
extern void TXTime(void);
extern void TXTimeX(void);
extern void TXBinTime(void);
extern int GetBinMinutes(void);
extern int GetBinHours(void);
BOOL SetTime(uint8_t hour,uint8_t min,uint8_t sec);
BOOL SetDate(uint8_t day,uint8_t date,uint8_t month,uint8_t year);
uint8_t ds1307_sw_read(uint8_t address);
