//  panel_fware - 2/13/2025 - set up c++ structures
//
//  Reformat in VSCode: shift-option-f
//  print a file: prx <file> | lpx
//
//  2/14/2025 1;30 pm byte object function works
//  add bit object 2:40 function works
//  02/15/2025 - add register object - problems expected/now fixed
//  02/16/2025 - expand register from one bit to four
//  02/18/2025 3:30 - finally works
//  02/20/2025 - expand to 16 bits
//	02/21/2025 - 10 am - works, but nibble reversed
//  02/21/2025 - change nibble order
//  02/22/2025 - start on switch input features
//  02/23/2025 - more input features
//  02/24/2025 - input 16 works on most bits. Some hw issues.
//  02/24/2025 - started updating front panel register with input bits
//  02/26/2025 - pushbutton input changes work
//  03/07/2025 - commands a and b change the idle display pattern
//  03/07/2025 - commands c and d for register display
//  03/08/2025 - commands c, d, and e work for register display
//  03/12/2025 - cleanup debug outputs
//  03/14/2025 - change command input from USB
//  03/17/2025 - flush input bytes
//  03/27/2025 - tested wdt reboot code, started to add radio and onoff buttons
//  03/29/2025 - more cleanup
//  04/04/2025 - test version for new parts of hardware
//  05/03/2025 - platformio version
//  05/06/2025 - platformio version works same as arduino version
//  05/07/2025 - add support for 2nd I2C
//  05/09/2025 - more add support for 2nd I2C
//  05/12/2025 - fix bit numbering bug when dreg 16
//  05/15/2025 - fixed problem with "clear" button push
//  05/15/2025 - more changes
//  05/19/2025 - fix failure when commend "b" used
//  05/22/2025 - continue changes for "clear" button
//  05/23/2025 - add support for "clear" button
//  05/26/2025 - cmd "c" works w/o clear defined
//  05/27/2025 - moved some function to D_base
//  05/28/2025 - D_clear button works - at least prints msg
//  05/29/2025 - Start customizing D_clear
//
/****** for H316 front panel board 3/2024. ******/
//  sixteen register bit top row
//  Note: wiring is not consecutive - this simplified board layout
//  Grove 3 connector
//  i/o expander PCF8574TS
//  0x20 bits 1-4 on front panel labels
//  0x24 bits 5-8
//  0x22 bits 9-12
//  0x26 bits 13-16
//  0x21 clear button
//
//  sixteen register bit bottom row
//  Note: wiring is not consecutive - this simplified board layout
//  Grove 4 connector
//  i/o expander PCF8574TS
//  0x20 SS1 through 4
//  0x24
//  0x22
//  0x26  6
//
//  #include <iostream>

#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "hardware/structs/psm.h"

#include "pico/bootrom.h"

using namespace std;
#include <Wire.h>
// #define WIRE Wire  // not needed for two I2C busses
#define DEBUGX 0
#define CLEAR 1 // define to use the clear button object

arduino::MbedI2C xwire0(I2C_SDA, I2C_SCL);
arduino::MbedI2C xwire1((uint8_t)6, (uint8_t)7);

static int rev4[16] = {0x00, 0x08, 0x04, 0x0c, 0x02, 0x0a, 0x06, 0x0e,
                       0x01, 0x09, 0x05, 0x0d, 0x03, 0x0b, 0x07, 0x0f}; // reverse four bits msb/lsb (i/o expander oddity)

/************** class definitions *********************************/

class D_byte;
class D_bit;
class D_base;
class D_reg;

int xouch()
{
  int i;
  Serial.println("Xouch!");
  while (1)
    ;
  return (0);
};

void clear_register();

/**
 * @brief D_io class provides a single object to controll both/all I2C busses
 *       and all i/o expanders on the front panel.
 * @param D_byte class provides one object to control a single i/o expander
 *
 **/
class D_io
{ // this object represents all bytes on H316 front panel
private:
  D_byte *D_bytes[10]; /** @  pointers to all byte objects */
  int dbyte_index = 0; // number of bytes defined
  D_byte *temp;

public:
  D_io();
  D_byte *make(MbedI2C *, int); // create byte, add to table, return ptr
  void W_wordf();               // force writes on all bytes
  int R_scan();                 // scan for changed inputs on all bytes
};
/**
 * @brief D_byte class provides a single object to control a single i/o expander
 *
 */
class D_byte
{ /* : public  D_io */ // one byte-sized register on a PCF8574TS
public:
  D_byte(MbedI2C *ptr, int addr);        // constructor
  void W_byte(int mask, int data);       // update out byte
  void W_bytef();                        // flush out byte to hw
  int R_bytef();                         // return most recent input data
  int R_find_changes();                  // find changes and call D_bit object
  D_bit *mbit(D_byte *parent, int mask); /** make a D_bit object and add to D_byte **/

  /** D_byte is the heart of this system.
   * It is related on a one-to-one basis to the i2c bus and the i/o expander chip.
   */
private:
  MbedI2C *i2c_bus = NULL;
  int i2caddr;
  int dbit_index = 0;
  int data_out;      // saved last value output to hardware
  int data_in;       // last value read from hw
  int previous_in;   // previous input to find changes
  int changed_in;    // bit mask for changes
  int defined_bits;  // set only for bits in use
  int change_flag;   // mark when changed for output
  D_bit *D_bits[16]; // pointer to bits in this byte
  int junk;          // scratch variable
  D_bit *temp2;
};
//
//
/** @brief D_bit class provides an object to control a single i/o bit.
 * @param in D_byte parent - pointer to the i/o expander byte.

 *
 */
class D_bit
{ // one bit on an i/o expander byte
private:
public:
  /** constructor: parameters are D_byte and bit mask */
  D_bit(D_byte *parent, int mask); // constructor
  /** parent - link to i/o expander register D_byte. */
  D_byte *parent; // link to i/o expander register "byte"
  /** bit_no_in_byte - bit number in the i/o expander byte. */
  int bit_no_in_byte; // bit number in byte
  /** mask - bit field in byte   */
  int mask;
  /** D_reg_link - pointer to front panel register or button */
  D_base *D_reg_link; // pointer to front panel register or button
  /** D_reg_bitno - bit number in the front panel register (LSB=0) or button. */
  int D_reg_bitno; // bit no in front panel panel reg (LSB=0) or button

  void W_bit(int bitno, int mask);

  virtual int R_bit(int);
  int R_changed_bit(int data); // process a changed bit
};

/** D_base base class for front panel registers and buttons */
class D_base
{
public:
  D_bit *parent;  // pointer to D_byte object
  long value;     // latest value
  long old_value; // previous value
  int D_reg_bitno;
  D_base *D_reg_link;
  // virtual int action(); // execute when the bit changes
  // virtual int clear();  // reset value
  virtual int R_bit(int);
  virtual int W_bit(int, int) { return (0); };

  virtual int BN_changed_bit(int) { return (0); }; // test
};

//  class D_button - base class for all single button input
/** @brief D_button is a base class for all single button input
 * parent - pointer to D_bit object
 */
class D_button : public D_base
{
public:
  D_bit *parent; // pointer to D_bit object
  int R_bit(int);
  int BN_changed_bit(int) { return (0); };

private:
};
//
/** @brief class D_reg provides a single object for a front panel register
 *  This is the single 16-bit register displayed on the top row of LEDs.
 *  (It can also be used to test the bottom 16 LEDs.)
 */

class D_reg : D_base
{ // front panel register - consists of bits - PUBLIC is needed
public:
  D_reg()
  {
    size = change_flag = value = old_value = 0;
    Serial.println("D_reg constructor");
  };
  /** pointers to the D_bit objects that make up the register.
   */
  D_bit *regptr[32];
  /** size - number of bits in front panel reg (16) */
  /** size - normally 16 bits (unless testing). */
  int size;
  /** register current contents */
  long value;     // current register value
  long old_value; // detect changes
  /** flag if value has changed and needs output */
  int change_flag; // set if output is needed
  /** D_reg_addbit - creates a new instance of D_bit object and adds to register */
  void D_reg_addbit(D_bit *xnew); // add a bit to the register
  /** D_reg_addbit call this object after all bits are added to register */
  void D_reg_end_bits(); // call after last bit defined by addbit
  /** write register using D_byte objects */
  void D_reg_write_word(int in);
  /** return register value */
  int D_reg_read_word();
  /** XXX needs work */
  int R_bit(int);
  /** XXX needs work */
  int BN_changed_bit(int) { return (0); }; // test
};
//
/** @brief class D_clear provides an object for the front panel "clear" button
 *
 */

class D_clear : public D_button
{ // front panel clear button
public:
  D_clear(D_bit *xparent)
  {
    parent = xparent;
    Serial.println("D_clear constructor");
  }
  int R_bit(int in)
  {
    Serial.println("D_clear::R_bit");
    Serial.println("clear display register");
    clear_register(); // clear the display register
    return (0);
  }
  // int BN_changed_bit(int ) {return(0);};  // test
};
//
/** @brief class usbio provides keyboard input from the host computer.
 * This is not a very sophisticatd class.
 */
// character i/o on USB - looks like an async terminal
class usbio
{
public:
  int in_available();
  int in_char();
  int out_char();
  int inCharx; // last character read
};

/********************* end of class definitions ******************/

/** Random, uncategorized multi-line comment in D_io construct
 * or method.
 */
D_io::D_io()
{ // D_io constructor
  Serial.println("D_io constructor");
  dbyte_index = 0;
}
//
//  create a new byte in an i2c expander
//
/** Create a new byte for this i/o expander
 * The hardware i/o expander provides four input and four output bits.
 */
D_byte::D_byte(MbedI2C *ptr, int addr)
{ // constructor
  Serial.println("D_byte constructor ");
  Serial.println(addr, HEX);
  i2c_bus = ptr; // save pointer to I2C bus (0 or1)
  i2caddr = addr;
  i2c_bus->begin();
  i2c_bus->beginTransmission(i2caddr);
  junk = i2c_bus->write(0xf0); // set high 4 bits into input mode
  i2c_bus->endTransmission();
  data_out = 0;
  data_in = 0xff;
  previous_in = 0;
  changed_in = 0;
  defined_bits = 0; // no bits defined yet
};

/** Write one bit to i/o expander byte, but do not flush
 *
 */
void D_byte::W_byte(int addmask, int data_outx)
{
  if (DEBUGX)
    Serial.print("W_byte: ");
  if (DEBUGX)
    Serial.print(addmask, HEX);
  data_out = (data_out & (~addmask)) | (data_outx & addmask); // update saved value
  change_flag++;
};
/** return input bits from last read.
 * Bits are shifted and re-ordered to match the output bits.
 * This means that the bit for switch 1 and led 1 use the same mask.
 */
int D_byte::R_bytef()
{ // return last input bits (shifted)
  int i;
  i = ((data_in & 0xf0) >> 4); //  mask and shift input bit
  return (rev4[i]);            // reverse bits to match input bit order
};

/** perform hardware write via I2C to write byte to i/o expander.
 *  Also reads input bytes and saves in byte object.
 * W_bytef is called by D_io::W_wordf to flush all bytes.
 */
void D_byte::W_bytef()
{ // byte flush - write byte to hw/read from hw
  int work;
  if (DEBUGX)
    Serial.print("W_bytef: ");
  if (DEBUGX)
    Serial.println(data_out, HEX);
  i2c_bus->beginTransmission(i2caddr);
  i2c_bus->write((0xf0 | ((data_out & 0x0f) ^ 0x0f))); // invert bits
  if (DEBUGX)
    Serial.println(data_out, HEX); //
  i2c_bus->endTransmission();
  change_flag = 0; // clear change flag
  //
  // add logic to read input
  //
  i2c_bus->requestFrom(i2caddr, 1);
  previous_in = data_in;
  data_in = i2c_bus->read(); // read new data
  if (((data_in ^ previous_in) & 0xf0) != 0)
  {
    changed_in = (((data_in ^ previous_in) & 0xf0) >> 4); // find changed bits - mask and shift
    work = rev4[defined_bits];
    work = defined_bits;
    changed_in = rev4[changed_in] & work ;         // reverse bits to match input bit order

    Serial.print("W_bytef: in/prev/changed: ");
    Serial.print(i2caddr, HEX);
    Serial.print(" ");
    Serial.print(data_in, HEX);
    Serial.print(" ");
    Serial.print(previous_in, HEX);
    Serial.print(" ");
    Serial.println(changed_in, HEX);
  }
  // Serial.print("W_bytef read data:");
  // Serial.println(data_in,HEX);
};

/** Create D_byte object and add to D_io object.
 * This is how D_io can control all bytes.
 */
D_byte *D_io::make(MbedI2C *bus, int addr)
{ // add the byte register in an i/o expander
  //    Serial.println("D_io make");
  temp = new D_byte(bus, addr);
  D_bytes[dbyte_index++] = temp;
  return (temp);
};

/** Flush all D_byte objects by writing values to i/o expander hw
 *
 */
void D_io::W_wordf()
{ // flush out writes to hw
  int i;
  if (DEBUGX)
    Serial.print("D_io W_wordf called - dbyte_index = ");
  if (DEBUGX)
    Serial.println(dbyte_index);
  if (dbyte_index == 0)
    return;
  for (i = 0; i < dbyte_index; i++)
  {
    D_bytes[i]->W_bytef();
  }
}

/**
 * D_io::R_scan checks each byte for changes by calling D_byte::R_find_changes.
 *
 */
int D_io::R_scan()
{ // scan inputs for changes.
  int i;
  int changes = 0;

  if (DEBUGX)
    Serial.print("D_io R_scan() called - dbyte_index = ");
  if (DEBUGX)
    Serial.println(dbyte_index);
  if (dbyte_index == 0)
    return (0);
  for (i = 0; i < dbyte_index; i++)
  {
    changes = changes + D_bytes[i]->R_find_changes();
  }
  return (changes);
}

D_bit *D_byte::mbit(D_byte *parentp, int mask)
{ // make a D_bit object
  D_bit *temp2;
  temp2 = new D_bit(this, mask);
  D_bits[dbit_index++] = temp2;
  defined_bits |= mask; // define this bit as being used
  return (temp2);
}

/** R_bit for base class D_base
 * This is a virtual function to be redefined in subclasses.
 */
int D_base::R_bit(int in)
{
  Serial.print("D_base::R_bit: ");
  Serial.print((long unsigned int)this, HEX);
  Serial.print(" ");
  Serial.println(in, HEX);
  return (0);
};

/** D_byte - find which bits changed in this byte.
 * Do this via the four D_bit objects in this byte.
 * @param in - the changed_in word in the D_byte object is compared
 * against the mask in the D_bit object.
 * Action - call R_bit and W_bit in the D_bit object if appropriate.
 */
int D_byte::R_find_changes()
{ // find bit changed in this byte -- what about non D_reg?
  int i;

  while (changed_in != 0)
  { // does this work with multiple bit changes?

    // Serial.print("R_find_changes: before/after ");
    // Serial.println("R_find_changes");
    // Serial.println(changed_in,HEX)
    // Serial.print(changed_in,HEX);
    // Serial.println(" ");
    if (dbit_index == 0) // nothing here, return
      return (0);
    for (i = 0; i < dbit_index; i++)
    {                                   // scan for changed bit
      if (D_bits[i]->mask & changed_in) // compare bits changed in byte with D_mask.
      {

        // D_bits[i]->D_reg_bitno;  // this bit was changed  NOOP
        D_bits[i]->R_bit(changed_in); // BJD DEBUG
        D_bits[i]->W_bit(i, 1);       // Write output bit via byte

        D_bits[i]->R_changed_bit(0); // update registers/buttons using this bit BJD

        changed_in = changed_in & (~changed_in);
        // Serial.println(changed_in,HEX);
        break;
      }
      else if (changed_in == 0)
        return (0);
    } // end scan
  } // end while there are still changed bits
  return (0);
}

/** D_bit constructor takes pointer to D_byte object and bit mask identifying bit  */
D_bit::D_bit(D_byte *parentx, int maskx)
{                   // constructor - D_bit is one bit in front panel reg
  mask = maskx;     // mask to select bit in 8-bit hw register
  parent = parentx; // pointer to W_byte object
  Serial.println("D_bit constructor");
};

//  dummy R_bit
/** Update bit in D_reg or D_button?  */
int D_bit::R_bit(int in)
{
  Serial.print("D_bit::R_bit: ");
  Serial.print((long unsigned int)this, HEX);
  Serial.print(" ");
  Serial.println(D_reg_bitno, HEX);
  if (D_reg_bitno > 33)
    xouch();
  D_reg_link->R_bit(D_reg_bitno);
  return (0);
}

//  set/reset bit in D_reg front panel register
//  called from D_bit
/** Update bit in D_reg  */
int D_reg::R_bit(int bitno)
{ // set/reset bit in i/o expander sw object
  int temp;

  Serial.print("D_reg::R_bit: ");
  Serial.print((long unsigned int)this, HEX);
  Serial.print(" ");
  Serial.println(bitno, HEX);

  // temp = 0x10000 >> D_reg_bitno;  // note oddball bit position
  // temp = 1 << (size-bitno); // bit no
  // temp = 1 << bitno; // bit no
  temp = (1 << size); // starting point 10000 hex for 16 bit size
  temp = temp >> (bitno + 1);
  value |= temp; // update value
  // W_bit(D_reg_bitno,1); // update bit to be output

  return (size - bitno); // return bit mask in the byte
}

int D_button::R_bit(int in)
{ // set/reset bit in i/o expander sw object

  Serial.print("D_button::R_bit: ");
  Serial.print((long unsigned int)this, HEX);
  Serial.print(" ");
  Serial.println(in, HEX);


  // Serial.print(" ");
  // Serial.println(D_reg_bitno);

  // temp = 0x10000 >> D_reg_bitno;  // note oddball bit position
  // temp = 1 << (D_reg_link->size-D_reg_bitno); // bit no ------------------------xxx BJD
  // D_reg_link->value |= temp;  // update value
  // W_bit(D_reg_bitno,1); // update bit to be output

  return (0); // return bit mask in the byte
}

/** D_bit object routine to write one bit via the D_byte parent
 *
 */
void D_bit::W_bit(int bitno, int output_bit)
{ // write one bit to object, but don't flush to hw
  if (output_bit)
  {
    parent->W_byte(mask, 0xff);
  }
  else
  {
    parent->W_byte(mask, 0);
  }
};

/** method to update one bit in the D_reg object
 * This is experimental and may be half-baked.
 */
int D_bit::R_changed_bit(int data)
{ // experimental update fp register
  int i;
  i = parent->R_bytef() & data;  // find - changed to 0 or 1?
  D_reg_link->BN_changed_bit(i); // notify the register or button object
  return (0);
};

//
//  add new bit (previously defined as part of an i/o expander byte)
//  to this front panel register
//
void D_reg::D_reg_addbit(D_bit *xnew)
{                          // create bit object, add to D_reg table
  xnew->D_reg_link = this; // link to D_reg object
  // xnew->D_reg_bitno = (size-16) * -1; // save D_reg bit no (MSB=0?) --FIX THIS
  regptr[size++] = xnew; // link to the D_bit being added
};

//
//  end adding bits - this updates bit numbers
//
void D_reg::D_reg_end_bits()
{
  int i;
  for (i = 0; i < size; i++)
  {
    regptr[i]->D_reg_bitno = size - (i + 1); // bits numbered 1 to n
  }
};

//
//  write a word to a front panel register
//
void D_reg::D_reg_write_word(int in)
{ // write word to bit objects but don't flush to hw
  int bit_select;
  int i;
  int work;

  if (DEBUGX)
    Serial.print("write D_reg_write_word --- ");
  if (DEBUGX)
    Serial.println(in, HEX);
  value = in; // save value
              //   do {  // go through all bits in lsb-msb order
  bit_select = 1;
  for (i = 0; i < size; i++)
  {
    if ((in & bit_select))
    {
      regptr[i]->W_bit(i, 1);
    }
    else
    {
      regptr[i]->W_bit(i, 0);
    }
    bit_select = bit_select << 1;
  }

  if (DEBUGX)
    Serial.println("return from write word\n");
  if (DEBUGX)
    Serial.println(work, HEX);
};

//  read word from register

int D_reg::D_reg_read_word()
{
  return (value);
};

//  usb character i/o object(s)

int usbio::in_available()
{
  if (Serial.available() > 0)
    return (1);
  return (-1);
};

int usbio::in_char()
{
  //  wait for output to complete - see if it helps
  Serial.flush();
  // check for incoming serial data:
  if (Serial.available() > 0)
  {
    // read incoming serial data:
    inCharx = Serial.read();

    // Serial.print("usbio::in_char  ");
    // Serial.println((int) inCharx,HEX);

    if ((inCharx >= 0x41) && (inCharx <= 0x7a))
    { // was 7a
      return (inCharx);
    }
    else
      return (-1);
  }
  return (-1);
};

/************************* end of object member functions ************************/

//
/************************* start of "real" code ************************************/
//

static D_io *D_io_base;
D_byte *dbyte_save;
D_bit *dbit_save;
usbio usbio2; // i/o via USB to FrontPanelH316.c
D_reg *fp_dreg;
D_base *fp_clear; // clear button object

// D_reg front_panel_reg;
int display_value = 0;

void setup()
{
  // put your setup code here, to run once:
  // initialize digital pin LED_BUILTIN as an output.

  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  //  while (!Serial)
  //  delay((unsigned long) 10);

  if (DEBUGX)
  {
    while (Serial.available() <= 0)
      delay((unsigned long)10);
    Serial.println("\nPanel Test 03/27/2025 - enter anything");
    Serial.read();
  }
  D_io_base = new D_io;  // base of all i/o expander i/o
  fp_dreg = new D_reg(); // define the front-panel register
  // fp_clear = new D_clear(NULL); // define clear register button object

  /***  define top 16 bits */

  dbyte_save = D_io_base->make(&xwire0, 0x26); // make one i/o expander
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to front panel register   // bits labeled 1-4
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);

  dbyte_save = D_io_base->make(&xwire0, 0x22); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to register   // bits labeled 5-8
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);

  dbyte_save = D_io_base->make(&xwire0, 0x24); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to register   // bits labeled 9-12
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);
  dbyte_save = D_io_base->make(&xwire0, 0x20); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)

  //  define bits and add to register   // bits 13-16
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);

//
/*** define 5th byte for clear button ** */
//
#ifdef CLEAR
  dbyte_save = D_io_base->make(&xwire0, 0x21); // make one i/o expander (on SDA, SCL
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  // dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  //  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register **** BJD DUMMY
  fp_clear = dbit_save->D_reg_link = new D_clear(dbit_save);
  // dbit_save->D_reg_bitno = 0;
//
#endif

/***  define bottom 16 bits */
#define testnow
#ifdef testnow

  dbyte_save = D_io_base->make(&xwire1, 0x26); // make one i/o expander
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to front panel register   // bits labeled 1-4
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);

  dbyte_save = D_io_base->make(&xwire1, 0x22); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to register   // bits labeled 5-8
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);

  dbyte_save = D_io_base->make(&xwire1, 0x24); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)(20,22,24,26)
  //  define bits and add to register   // bits labeled 9-12
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_dreg->D_reg_addbit(dbit_save);
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_dreg->D_reg_addbit(dbit_save);
  dbyte_save = D_io_base->make(&xwire1, 0x20); // i2c addr of i/o expander chip
  //      (on SDA, SCL = ,5)

  // //  define bits and add to register   // bits 13-16
  // dbit_save = dbyte_save->mbit(dbyte_save,0x08);
  // fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  // dbit_save = dbyte_save->mbit(dbyte_save,0x04);
  // fp_dreg->D_reg_addbit(dbit_save);
  // dbit_save = dbyte_save->mbit(dbyte_save,0x02);
  // fp_dreg->D_reg_addbit(dbit_save); // add a D_bit to the front panel register
  // dbit_save = dbyte_save->mbit(dbyte_save,0x01);
  // fp_dreg->D_reg_addbit(dbit_save);
  fp_dreg->D_reg_end_bits(); // update bit number after last bit added

#endif

  if (DEBUGX)
  {
    while (Serial.available() <= 0) // wait - enter c/r to start test
      delay((unsigned long)10);
    Serial.println("\nPanel Test - enter anything 2 start");
    Serial.read();
  }
}

/********************************* counter display pattern *************************/
void count(unsigned long delay_time)
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // dbyte_save->W_byte(0xf); // old code from bit level tests
  // dbit_save->W_bit(1);

  fp_dreg->D_reg_write_word(display_value++);
  D_io_base->W_wordf();

  delay(delay_time); // wait for a second

  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  // dbyte_save->W_byte(0);
  // dbit_save->W_bit(0);

  fp_dreg->D_reg_write_word(0x0);
  D_io_base->W_wordf();

  delay(delay_time); // wait for a second
}

/********************************* rotate display pattern *************************/
static unsigned long pattern = 072727;
static unsigned long pattern_reset = 0x80000000;

void rotate(unsigned long delay_time)
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // dbyte_save->W_byte(0xf);
  // dbit_save->W_bit(1);

  fp_dreg->D_reg_write_word(pattern);

  pattern = pattern >> 1;
  if (pattern == 0)
    pattern = pattern_reset;

  D_io_base->W_wordf();

  delay(delay_time); // wait for 1/10th second

  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  // dbyte_save->W_byte(0);
  // dbit_save->W_bit(0);

  fp_dreg->D_reg_write_word(0x0);
  D_io_base->W_wordf();

  if (D_io_base->R_scan())
  {
    D_io_base->W_wordf();
    delay((unsigned long)2);
  } // scan for changes in data read earlier

  delay(delay_time); // wait for a second
}

/********************************* test display and update fp register  *************************/

void display_register(unsigned long delay_time)
{
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  // dbyte_save->W_byte(0xf);
  // dbit_save->W_bit(1);
  fp_dreg->D_reg_write_word(fp_dreg->value);

  D_io_base->W_wordf(); // write and read all bytes

  delay((unsigned long)2); // wait

  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  // dbyte_save->W_byte(0);
  // dbit_save->W_bit(0);

  if (D_io_base->R_scan())
  {
    D_io_base->W_wordf();
    //     Serial.print("display_register2: ");
    // Serial.println(fp_dreg->value,HEX);
    delay((unsigned long)2);
  } // scan for changes in data read earlier

  delay(delay_time); // wait
}

/*********************************/

void save_register()
{ // pass register value to simh simulator
  Serial.print("save_register: simh command to update register ");
  Serial.println(fp_dreg->value, OCT);
}

void clear_register()
{ // zero out register
  fp_dreg->D_reg_write_word(0x0);
  D_io_base->W_wordf();
}

int command(int i)
{
  int return_value;
  if (1)
    Serial.print("command: from USB - ");
  if (1)
    Serial.println(i, HEX);
  return_value = (i - 0x61);

  return (return_value);
}

void reboot()
{
  Serial.println("Reboot now:");
  watchdog_reboot((uint32_t)0, (uint32_t)0, (uint32_t)0x7fffff);
};

void loop()
{
  static int current_cmd;
  static int inChar;
  static unsigned long delay_time = 1000; // delay before continue loop

  // put your main code here, to run repeatedly:

  // the loop function runs over and over again forever

  inChar = usbio2.in_char();
  if (inChar != -1)
  {
    current_cmd = command(inChar);
    // Serial.println(current_cmd,HEX);
    delay_time = 1000;
  }
  switch (current_cmd)
  {
  case 0: // a
    count(delay_time);
    if (usbio2.in_available() > 0)
      delay_time = 50;
    else
      delay_time = 1000;
    break;
  case 1: // b
    rotate(delay_time);
    // if (usbio2.in_available() > 0) delay_time = 50; else delay_time = 1000;
    delay_time = 100;
    break;
  case 2: // c
    display_register(delay_time);
    break;
  case 3: // d
    clear_register();
    current_cmd = 2; // back to display registeer
    break;
  case 4: // e
    save_register();
    current_cmd = 2; // back to display register
    break;
  case 5:     // f
    reboot(); // force reboot via wdt
    delay(9000);
    break;
  default:
    count(50);
    current_cmd = 1; // default to rotate
    break;
  }
  //  process inputs

  // }
  // /Users/bdietz/Library/Arduino15/packages/arduino/hardware/mbed_rp2040/4.2.4
  //  /cores/arduino/mbed/targets/TARGET_RASPBERRYPI/TARGET_RP2040/pico-sdk/rp2_common/hardware_watchdog/include/hardware
}
