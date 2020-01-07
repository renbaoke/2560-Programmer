/*
   2560 Programmer by Buck
*/

//////////////// global definitions ////////////////
#define BUF_SIZE 256
#define CMD_ARG 16

#define BS 0x08
#define CR 0x0d
#define DEL 0x7f

#define XMODEM_DATA_SIZE 128

#define SOH 0x01
#define EOT 0x04
#define ACK 0x06
#define NAK 0x15
#define ETB 0x17
#define CAN 0x18

const int P[40] =
{
  11, 10, 8, 6, 4, 53, 52, 51, 50, 49,
  48, 47, 46, 39, 40, 41, 42, 43, 45, 44,
  32, 33, 34, 35, 36, 37, 38, 31, 30, 29,
  28, 27, 26, 25, 24, 23, 22, 5, 7, 9
};

enum error_check
{
  CRC, CHECKSUM
};

typedef struct xmodem_packet
{
  unsigned char  soh;
  unsigned char  pno;
  unsigned char  npno;
  unsigned char  data[XMODEM_DATA_SIZE];
  union {
    unsigned short crc;
    unsigned char checksum;
  } check;
} xmodem_packet;

typedef void (*dev_init_func)();
typedef unsigned int (*dev_read_func)(unsigned int addr);
typedef void (*dev_write_func)(unsigned int addr, unsigned int data);
typedef struct eeprom_dev
{
  char* name;
  unsigned int size;
  dev_init_func init_func;
  dev_read_func read_func;
  dev_write_func write_func;
} eeprom_dev;

void set_addr(unsigned int addr, int* A, unsigned char A_size)
{
  unsigned int mask = 1;
  for (unsigned char i = 0; i < A_size; i++, mask = mask << 1)
    digitalWrite(P[A[i]], addr & mask ? HIGH : LOW);
}

void set_data(unsigned int data, int* D, unsigned char D_size)
{
  for (unsigned char i = 0; i < D_size; i++)
    pinMode(P[D[i]], OUTPUT);

  unsigned int mask = 1;
  for (unsigned char i = 0; i < D_size; i++, mask = mask << 1)
    digitalWrite(P[D[i]], data & mask ? HIGH : LOW);
}

unsigned int get_data(int *D, unsigned char D_size)
{
  for (int i = 0; i < D_size; i++)
    pinMode(P[D[i]], INPUT);

  unsigned int data = 0;
  unsigned int mask = 1;
  for (unsigned char i = 0; i < D_size; i++, mask = mask << 1)
    data |= digitalRead(P[D[i]]) == HIGH ? mask : 0;

  return data;
}

//////////////// all supported devices ////////////////
int at28c256_A[15] = {29, 28, 27, 26, 25, 24, 23, 22, 16, 15, 12, 14, 21, 17, 20};
int at28c256_D[8] = {30, 31, 32, 6, 7, 8, 9, 10};
#define at28c256_GND 33
#define at28c256_VCC 19
#define at28c256_WE 18
#define at28c256_OE 13
#define at28c256_CE 11

void at28c256_init()
{
  pinMode(P[at28c256_VCC], OUTPUT);
  pinMode(P[at28c256_GND], OUTPUT);
  digitalWrite(P[at28c256_VCC], HIGH);
  digitalWrite(P[at28c256_GND], LOW);

  pinMode(P[at28c256_CE], OUTPUT);
  pinMode(P[at28c256_OE], OUTPUT);
  pinMode(P[at28c256_WE], OUTPUT);
  digitalWrite(P[at28c256_CE], LOW);
  digitalWrite(P[at28c256_OE], HIGH);
  digitalWrite(P[at28c256_WE], HIGH);

  for (int i = 0; i < 15; i++)
    pinMode(P[at28c256_A[i]], OUTPUT);
}

unsigned int at28c256_read(unsigned int addr)
{
  unsigned int data = 0;
  
  set_addr(addr, at28c256_A, 15);
  digitalWrite(P[at28c256_OE], LOW);
  delayMicroseconds(1);
  data = get_data(at28c256_D, 8);
  digitalWrite(P[at28c256_OE], HIGH);
  
  return data;
}

void at28c256_write(unsigned int addr, unsigned int data)
{
  set_addr(addr, at28c256_A, 15);
  set_data(data, at28c256_D, 8);
  digitalWrite(P[at28c256_WE], LOW);
  delayMicroseconds(1);
  digitalWrite(P[at28c256_WE], HIGH);
  delay(6);
}

eeprom_dev devs[] =
{
  {"at28c256", 32 * 1024, at28c256_init, at28c256_read, at28c256_write}
};

//////////////// global variables ////////////////
unsigned char buf[BUF_SIZE];
unsigned char cnt = 0;

char* cmd_argv[CMD_ARG];
unsigned char cmd_argc;

error_check check = CRC;
eeprom_dev* dev = &devs[0];
unsigned int offset = 0;

//////////////// xmodem protocol ////////////////
unsigned short do_crc(const unsigned char *data, int size)
{
  unsigned short crc = 0;

  while (size--)
  {
    crc = crc ^ (int) * data++ << 8;

    for (int i = 0; i < 8; i++)
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
  }

  return (crc >> 8) | (crc << 8);
}

unsigned char do_checksum(const unsigned char *data, int size)
{
  unsigned char checksum = 0;

  while (size--)
    checksum += *data++;

  return checksum;
}

void xmodem_read()
{
  int c = 0;

  // wait for 'C' or NAK
  while (c != 'C' && c != NAK)
  {
    while (!Serial1.available())
      ;
    c = Serial1.read();
  }

  bool crc = (c == 'C');

  for (int i = offset; i < dev->size / XMODEM_DATA_SIZE; i++)
  {
    //read from eeprom
    xmodem_packet *p = (xmodem_packet*)buf;
    cnt = 0;
    for (int j = 0; j < XMODEM_DATA_SIZE; j++)
      p->data[cnt++] = dev->read_func(i * XMODEM_DATA_SIZE + j);

    p->soh = SOH;
    p->pno = i + 1;
    p->npno = ~(p->pno);

    if (crc)
    {
      p->check.crc = do_crc(p->data, XMODEM_DATA_SIZE);
      Serial1.write(buf, sizeof(xmodem_packet));
    }
    else
    {
      p->check.checksum = do_checksum(p->data, XMODEM_DATA_SIZE);
      Serial1.write(buf, sizeof(xmodem_packet) - 1);
    }

    // wait for ACK or NAK
    while (!Serial1.available())
      ;

    c = Serial1.read();
    if (c == ACK)
      ;
    else if (c == NAK)
      i--;
    else
      ;
  }

  // send EOT
  Serial1.write(EOT);

  // wait for ACK or NAK
  while (!Serial1.available())
    ;

  c = Serial1.read();
  if (c == ACK)
    ;
  else if (c == NAK)
    ;
  else
    ;
}

void xmodem_write()
{
  while (!Serial1.available())
  {
    Serial1.write(check == CRC ? 'C' : NAK);
    delay(3000);
  }

  for (int i = offset; i < dev->size / XMODEM_DATA_SIZE; i++)
  {
    cnt = 0;
    while (cnt < (check == CRC ? sizeof(xmodem_packet) : sizeof(xmodem_packet) - 1))
    {
      while (!Serial1.available())
        ;

      if (cnt == 0)
      {
        int c = Serial1.read();
        if (c == EOT)
        {
          Serial1.write(ACK);
          goto W_END;
        }
        else
          buf[cnt++] = c;
      }
      else
        buf[cnt++] = Serial1.read();
    }

    xmodem_packet *p = (xmodem_packet*)buf;

    bool check_ok = false;

    if (check == CRC)
      check_ok = do_crc(p->data, XMODEM_DATA_SIZE) == p->check.crc;
    else
      check_ok = do_checksum(p->data, XMODEM_DATA_SIZE) == p->check.checksum;

    if (check_ok)
    {
      for (int j = 0; j < XMODEM_DATA_SIZE; j++)
        dev->write_func(i * XMODEM_DATA_SIZE + j, p->data[j]);

      Serial1.write(ACK);
    }
    else
    {
      Serial1.write(NAK);
      i--;
    }
  }

W_END:;
}

//////////////// all commands ////////////////
int echo_func(int argc, char *argv[])
{
  for (int i = 1; i < argc; i++)
    Serial1.println(argv[i]);

  return 0;
}

int list_func(int argc, char *argv[])
{
  for (int i = 0; i < sizeof(devs) / sizeof(eeprom_dev); i++)
  {
    Serial1.print(devs[i].name);
    Serial1.print(", size : ");
    Serial1.print(devs[i].size);
    Serial1.println(" bytes");
  }
}

int set_func(int argc, char *argv[])
{
  if (argc < 2)
    return -1;

  if (strcmp(argv[1], "device") == 0)
  {
    if (argc < 3)
      return -1;

    dev = NULL;
    for (int i = 0; i < sizeof(devs) / sizeof(eeprom_dev); i++)
      if (strcmp(argv[2], devs[i].name) == 0)
      {
        dev = &devs[0] + i;
        break;
      }

    if (dev == NULL)
      Serial1.println("error, device not found");
    else
      Serial1.println("okay");
  }
  else if (strcmp(argv[1], "offset") == 0)
  {
    if (argc < 3)
      return -1;

    // todo
    offset = 0;
  }

  return 0;
}

int read_func(int argc, char *argv[])
{
  if (dev == NULL)
    return -1;

  dev->init_func();

  if (argc > 1)
  {
    unsigned int addr = strtoul(argv[1], NULL, 16);
    Serial1.print(addr, HEX);
    Serial1.print(" : ");
    Serial1.println(dev->read_func(addr), HEX);
  }
  else    
    xmodem_read();

  return 0;
}

int write_func(int argc, char *argv[])
{
  if (dev == NULL)
    return -1;

  dev->init_func();

  if (argc > 1)
  {
    if (argc < 3)
      return -1;

    unsigned int addr = strtoul(argv[1], NULL, 16);
    unsigned int data = strtoul(argv[2], NULL, 16);
    dev->write_func(addr, data);
  }
  else
    xmodem_write();

  return 0;
}

typedef int (*cmd_func) (int argc, char *argv[]);
typedef struct cmd
{
  char* name;
  cmd_func func;
  char* desc;
} cmd;

cmd cmds[] =
{
  {"echo", echo_func, "echo what you input"},
  {"list", list_func, "list all supported devices"},
  {"set", set_func, "set device/offset"},
  {"read", read_func, "read from eeprom"},
  {"write", write_func, "write to eeprom"}
};

//////////////// command interpreter ////////////////
void shell()
{
  cmd_argc = 0;

  cmd_argv[cmd_argc++] = buf;
  for (int i = 0; i < cnt; i++)
  {
    if (buf[i] == ' ')
    {
      buf[i] = '\0';
      cmd_argv[cmd_argc++] = buf + i + 1;
    }
  }

  if (strcmp(cmd_argv[0], "help") == 0)
  {
    for (int i = 0; i < sizeof(cmds) / sizeof(cmd); i++)
    {
      Serial1.print(cmds[i].name);
      Serial1.print(" : ");
      Serial1.println(cmds[i].desc);
    }

    return;
  }

  for (int i = 0; i < sizeof(cmds) / sizeof(cmd); i++)
  {
    if (strcmp(cmd_argv[0], cmds[i].name) == 0)
    {
      cmds[i].func(cmd_argc, cmd_argv);
      break;
    }
  }
}

//////////////// setup and loop ////////////////
void setup()
{
  Serial1.begin(115200);
  Serial1.println();
  Serial1.println("Welcome to 2560 Programmer!");
  Serial1.println();
}

void loop()
{
  while (!Serial1.available())
    ;

  int c = Serial1.read();
  Serial1.write(c);
  
  if (c == CR)
  {
    buf[cnt++] = 0;
    Serial1.println();
    if (cnt > 1)
      shell();
    cnt = 0;
  }
  else if (c == DEL)
    cnt--;
  else
    buf[cnt++] = c;
}
