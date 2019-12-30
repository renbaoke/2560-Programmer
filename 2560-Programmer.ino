#define A14 51
#define A12 49
#define A7  47
#define A6  45
#define A5  43
#define A4  41
#define A3  39
#define A2  37
#define A1  35
#define A0  33
#define D0  31
#define D1  29
#define D2  27
#define GND 25

#define VCC 50
#define WE  48
#define A13 46
#define A8  44
#define A9  42
#define A11 40
#define OE  38
#define A10 36
#define CE  34
#define D7  32
#define D6  30
#define D5  28
#define D4  26
#define D3  24

#define RW  2

int A[15] = {A0, A1, A2, A3, A4, A5, A6, A7,
             A8, A9, A10, A11, A12, A13, A14
            };

int D[8] = {D0, D1, D2, D3, D4, D5, D6, D7};

void set_addr(unsigned short addr)
{
  unsigned short mask = 0x0001;
  for (int i = 0; i < 15; i++, mask = mask << 1)
    digitalWrite(A[i], addr & mask ? HIGH : LOW);
}

void set_data(unsigned char data)
{
  for (int i = 0; i < 8; i++)
    pinMode(D[i], OUTPUT);

  unsigned char mask = 0x01;
  for (int i = 0; i < 8; i++, mask = mask << 1)
    digitalWrite(D[i], data & mask ? HIGH : LOW);
}

unsigned char get_data()
{
  for (int i = 0; i < 8; i++)
    pinMode(D[i], INPUT);

  unsigned char data = 0x00;
  unsigned char mask = 0x01;
  for (int i = 0; i < 8; i++, mask = mask << 1)
    data |= digitalRead(D[i]) == HIGH ? mask : 0x00;

  return data;
}

unsigned char read(unsigned short addr)
{
  unsigned char data = 0;

  set_addr(addr);
  digitalWrite(OE, LOW);
  delayMicroseconds(1);
  data = get_data();
  digitalWrite(OE, HIGH);

  return data;
}

void write(unsigned short addr, unsigned char data)
{
  set_addr(addr);
  set_data(data);
  digitalWrite(WE, LOW);
  delayMicroseconds(1);
  digitalWrite(WE, HIGH);
  delay(6);
}

void data_protection_off()
{
  write(0x5555, 0xAA);
  write(0x2AAA, 0x55);
  write(0x5555, 0x80);
  write(0x5555, 0xAA);
  write(0x2AAA, 0x55);
  write(0x5555, 0x20);
}

void data_protection_on()
{
  write(0x5555, 0xAA);
  write(0x2AAA, 0x55);
  write(0x5555, 0xA0);
}

void setup() {
  for (int i = 0; i < 16; i++)
    pinMode(A[i], OUTPUT);

  pinMode(VCC, OUTPUT);
  pinMode(GND, OUTPUT);
  digitalWrite(VCC, HIGH);
  digitalWrite(GND, LOW);

  pinMode(CE, OUTPUT);
  pinMode(OE, OUTPUT);
  pinMode(WE, OUTPUT);
  digitalWrite(CE, LOW);
  digitalWrite(OE, HIGH);
  digitalWrite(WE, HIGH);

  pinMode(RW, INPUT);

  Serial.begin(115200);
  Serial1.begin(115200);
}

unsigned char buf[256];
unsigned char cnt = 0;

#define SOH   0x01
#define EOT   0x04
#define ACK   0x06
#define NAK   0x15
#define ETB   0x17
#define CAN   0x18

struct xmodem_packet
{
  unsigned char  soh;
  unsigned char  pno;
  unsigned char  npno;
  unsigned char  data[128];
  union {
    unsigned short crc;
    unsigned char checksum;
  } check;
};

unsigned short do_crc(const unsigned char *data, int size)
{
  unsigned short crc = 0;

  while (size--)
  {
    crc = crc ^ (int) * data++ << 8;

    for (int i = 0; i < 8; i++)
    {
      if (crc & 0x8000)
        crc = crc << 1 ^ 0x1021;
      else
        crc = crc << 1;
    }
  }

  return (crc >> 8) | (crc << 8);
}

unsigned char do_checksum(const unsigned char *data, int size)
{
  unsigned char checksum = 0;

  while (size--)
  {
    checksum += *data++;
  }

  return checksum;
}

void read_rom()
{
  int c = 0;

  // wait for 'C' or NAK
  Serial.println("waitting");
  while (c != 'C' && c != NAK)
  {
    while (!Serial1.available())
    ;
    c = Serial1.read();
  }

  bool crc = (c == 'C');

  Serial.print("connected, check : ");
  Serial.println( crc ? "crc" : "checksum");

  for (int i = 0; i < 32768 / 128; i++)
  {
    //read from eeprom
    xmodem_packet *p = (xmodem_packet*)buf;
    cnt = 0;
    for (int j = 0; j < 128; j++)
      p->data[cnt++] = read(i * 128 + j);

    p->soh = SOH;
    p->pno = i + 1;
    p->npno = ~(p->pno);

    Serial.print("sending packet ");
    Serial.println(p->pno);
    if (crc)
    {
      p->check.crc = do_crc(p->data, 128);
      Serial1.write(buf, sizeof(xmodem_packet));
    }
    else
    {
      p->check.checksum = do_checksum(p->data, 128);
      Serial1.write(buf, sizeof(xmodem_packet) - 1);
    }

    // wait for ACK or NAK
    while (!Serial1.available())
      ;

    c = Serial1.read();
    if (c == ACK)
      Serial.println("got ack");
    else if (c == NAK)
    {
      Serial.println("got nak");
      i--;
    }
    else
      ;
  }

  // send EOT
  Serial.println("sending eot");
  Serial1.write(EOT);
  Serial.println("okay");
 
  while (true)
    ;
}

void write_rom()
{
  bool crc = true;
  //Serial1.write(crc ? 'C' : NAK);
  Serial1.write('C');

  //data_protection_off();

  for (int i = 0; i < 32768 / 128; i++)
  {
    cnt = 0;
    while (cnt < (crc ? sizeof(xmodem_packet) : sizeof(xmodem_packet) - 1))
    {
      while (!Serial1.available())
        ;

      if (cnt == 0)
      {
        int c = Serial1.read();
        if (c == EOT)
        {
          Serial.println("end of transmation, sending ack");
          Serial1.write(ACK);
          Serial.println("okay");
          goto END;
        }
        else
          buf[cnt++] = c;
      }
      else
        buf[cnt++] = Serial1.read();
    }

    xmodem_packet *p = (xmodem_packet*)buf;
    Serial.print("got packet : ");
    Serial.println(p->pno);

    bool check_ok = false;

    if (crc)
      check_ok = do_crc(p->data, 128) == p->check.crc;
    else
      check_ok = do_checksum(p->data, 128) == p->check.checksum;

    if (check_ok)
    {
      Serial.println("checking okay, writing");
      for (int j = 0; j < 128; j++)
      {
        write(i * 128 + j, p->data[j]);
      }

      Serial.println("sending ack");
      Serial1.write(ACK);
    }
    else
    {
      Serial.println("checking failed, sending nak");
      Serial1.write(NAK);
      i--;
    }
  }

END:;
}

void loop() {
  if (digitalRead(RW) == HIGH)
    write_rom();
  else
    read_rom();
}
