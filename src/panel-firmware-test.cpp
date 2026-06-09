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
//  05/30/2025 - Add lower buttons
//  06/03/2025 - All buttons defined. Fix bugs.
//  06/08/2025 - checkpoint version
//  06/10/2025 - Release candidate for Demo 1
//  06/10/2025 - changes post release candidate
//  06/15/2025 - JSON changes decode properly, but fp register not set
//  06/17/2025 - JSON works. Add register select
//  06/20/2025 - define R_select objects for 5 registers
//  06/23/2025 - refine R_select functionality
//  06/25/2025 - more updates
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
#include "cJSON.h" // JSON utilities
// #define WIRE Wire  // not needed for two I2C busses
#define DEBUGX 0

arduino::MbedI2C xwire0(I2C_SDA, I2C_SCL);
arduino::MbedI2C xwire1((uint8_t)6, (uint8_t)7);

int status_reg(const char *const monitor, char *reg_name);

static int rev4[16] = {0x00, 0x08, 0x04, 0x0c, 0x02, 0x0a, 0x06, 0x0e,
                       0x01, 0x09, 0x05, 0x0d, 0x03, 0x0b, 0x07, 0x0f}; // reverse four bits msb/lsb (i/o expander oddity)

/************** class definitions *********************************/

class D_byte;
class D_bit;
class D_base;
class D_reg;
int step_cmd(); //
D_reg *fp_dreg; // define this early

int xouch() // catistrophic error routing
{
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
  D_byte *make(MbedI2C *, int, int); // create byte, add to table, return ptr
  void W_wordf();                    // force writes on all bytes
  int R_scan();                      // scan for changed inputs on all bytes
};
/**
 * @brief D_byte class provides a single object to control a single i/o expander
 *
 */
class D_byte
{ /* : public  D_io */ // one byte-sized register on a PCF8574TS
public:
  D_byte(MbedI2C *ptr, int addr, int debug); // constructor
  void W_byte(int mask, int data);           // update out byte
  void W_bytef();                            // flush out byte to hw
  int R_bytef();                             // return most recent input data
  int R_find_changes();                      // find changes and call D_bit object
  D_bit *mbit(D_byte *parent, int mask);     /** make a D_bit object and add to D_byte **/

  /** D_byte is the heart of this system.
   * It is related on a one-to-one basis to the i2c bus and the i/o expander chip.
   */
private:
  MbedI2C *i2c_bus = NULL;
  int i2caddr;
  int dbit_index = 0;
  int data_out;              // saved last value output to hardware
  int data_in;               // last value read from hw
  unsigned long change_time; // time of last change
  int previous_in;           // previous input to find changes
  int changed_in;            // bit mask for changes
  int defined_bits;          // set only for bits in use
  int change_flag;           // mark when changed for output
  int debug_flag;
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
  // int bit_no_in_byte; // bit number in byte
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
  D_base() { D_reg_bitno = 0; }; // constructor
  char *name = nullptr;
  D_bit *parent;  // pointer to D_byte object
  long value;     // latest value
  long old_value; // previous value
  int D_reg_bitno;
  int button_value; // 0 = off, nz = on
  D_base *D_reg_link;
  // virtual int action(); // execute when the bit changes
  // virtual int clear();  // reset value
  virtual int R_bit(int);
  virtual int W_bit(int, int) { return (0); };
  /** @brief - BN_changed_bit exists in both D_button and D_base. Beware.  */
  virtual int BN_changed_bit(int j) //
  {
  //   Serial.print("D_base::BN_changed_bit: ");
    return (0);
  }
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
  virtual void R_bit_on() { /* Serial.println("D_button::R_bit_on"); */ };
  virtual void R_bit_off() { /* Serial.println("D_button::R_bit_off"); */ };
  //
  /** @brief - BN_changed_bit exists in both D_button, R_select,  and D_base. Beware.  */
  int BN_changed_bit(int j)
  {
    // int temp;

   //  temp = parent->parent->R_bytef() & parent->mask; // get latest input bit value -- never zero
    // Serial.print("D_button::BN_changed_bit: ");
    // Serial.print("button_value = ");
    // Serial.print(button_value);
    // Serial.print(" data ");
    // Serial.print(temp, HEX);
    // Serial.print(" j = ");
    // Serial.print(j);

    if (j == 0)
    { // process Switch ON events only (ignore OFF)
      if (button_value == 0)
      {
        // LED was OFF, turn it on
        // Serial.println("ON");
        button_value = 1; // set switch state to ON
        // Serial.print("D_button BN_changed_bit \n");
        parent->W_bit(0, 0);
        R_bit_on();
      }
      else
      {
        // LED was ON, turn it off
        // Serial.println("OFF");
        button_value = 0; // set switch statte to OFF
       //  Serial.print("D_button BN_changed_bit \n");
        parent->W_bit(0, 1);
        R_bit_off();
      }
    }
    return (0);
  }

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
};
//
/** @brief class D_clear provides an object for the front panel "clear" button
 *
 */

class D_clear : public D_button
{ // front panel clear button
public:
  D_clear(D_bit *xparent, char *namex)
  {
    parent = xparent;
    Serial.println("D_clear constructor");
    name = namex;
    button_value = 0; // clear/set off
  }
  int R_bit(int in)
  {
    Serial.print("D_clear::R_bit ");
    button_value++;     // set light on
    Serial.print(name); // printable name
    Serial.print(" ");
    clear_register(); // clear the display register
    return (0);
  }
};

class D_demo : public D_button
{ // front panel clear button
public:
  D_demo(D_bit *xparent, char *namex)
  {
    parent = xparent;
    Serial.println("D_demo constructor");
    name = namex;
    button_value = 0; // clear/set off
  }
  int R_bit(int in)
  {
    // Serial.print("D_demo::R_bit ");
    // // button_value++;            // set light on
    // Serial.print("DEMO MODE"); // printable name
    // Serial.print(" ");
    // Serial.println(name); // printable name
    return (0);
  }
};

class D_spare : public D_button
{ // spare front panel button
public:
  D_spare(D_bit *xparent, char *namex)
  {
    parent = xparent;
    Serial.println("D_spare constructor");
    name = namex;
    button_value = 0; // clear/set off
  }
  int R_bit(int in)
  {
    Serial.print("D_spare::R_bit ");
    // button_value++;            // set light on
    Serial.print("DEMO MODE"); // printable name
    Serial.print(" ");
    Serial.print(name); // printable name
    Serial.print(" ");
    // if ((button_value & 2) == 0)
    //   step_cmd(); // change demo mode
    return (0);
  }
  void R_bit_on()
  {
    Serial.println("D_spare::R_bit_on");
    step_cmd(); // change demo mode
  }
};

//
/** @brief class R_select defines one register select button.
 * (A, B, X, P ...)
 * In includes methods to act on one button or the group.
 */
class R_select : public D_button
{
public:
  R_select() {}; // initialize statics
  R_select(D_bit *bin, char *in)
  {
    // this constructor adds this object to the static table of
    // R_select objects.
    Serial.println("R_select constructor");
    parent = bin;                 // save parent D_bit object
    reg_select[reg_max++] = this; // save pointer to this object
    selected_reg = this;          // default selected register
    value = 0;                    // set register value to 0
    D_reg_bitno = 0;              //
    name = in;                    // strcpy(in,name);  // save name
  };

  int display_current()
  { // set this register to be displayed
    fp_dreg->D_reg_write_word(selected_reg->value);
    return 0;
  };

  int parse_json(char *in); // extract register values from json
  char *create_json();      // create json text msg

  static R_select *reg_select[6]; // pointer to select buttons
  static int reg_max;             // number of registers
  static R_select *selected_reg;  // displayed register
                                  // int value;                  value is defined in D_base
  char *name;                     // register name for json

  /** @brief - BN_changed_bit exists in  D_button, R_select, and D_base. Beware.  */
  int BN_changed_bit(int j) // this is the R_select version
  {
    if (j == 0)
    { // process Switch ON events only (ignore OFF)

      // LED was OFF, turn it on
     // Serial.println("ON");
      button_value = 1; // set switch state to ON
     // Serial.print("R_select::BN_changed_bit \n");
      // parent->W_bit(0, 0);
      R_bit_on();
      selected_reg = this; // save which register is selected
      // Serial.print("R_select: register ");
      // Serial.print(name);
      // Serial.println(" selected");
      // Serial.println(value);
      fp_dreg->D_reg_write_word(value); // display contents
    }
    return (0);
  }

  //
  //  select this register and display on front panel
  //
  virtual void R_bit_on()
  {
    int i;
    // Serial.println("R_select::R_bit_on");
    fp_dreg->D_reg_write_word(value); // set value
    for (i = 0; i < reg_max; i++)
    {
      reg_select[i]->parent->W_bit(0, 0); // turn all R_select light off
    };
    parent->W_bit(0, 0); // turn selected regiter on
  };
  virtual void R_bit_off()
  {
  };
  //
  //
  int R_bit(int in)
  {
    // Serial.print("R_select::R_bit ");
    // // // button_value++;            // set light on
    // // Serial.print("DEMO MODE"); // printable name
    // // Serial.print(" ");
    // Serial.println(name); // printable name
    return (0);
  }

private: // name of this register
};

//  outside of objects -- initialize static variables in R_select

int R_select::reg_max = 0;               // initialize
R_select *R_select::reg_select[6];       //
R_select *r_sel_ptr;                     // debug
R_select *R_select::selected_reg = NULL; // initialize

//
/** @brief class usbio provides keyboard input from the host computer.
 * This is not a very sophisticatd class.
 */
// character i/o on USB - looks like an async terminal
class usbio
{
public:
  usbio() { index = 0; };
  // constructor
  int in_available();
  int in_char();
  int out_char();
  char *in_string();
  int inCharx; // last character read
private:
  char buff[256];
  int index;
};

/** @brief: class lcontrol provides variables/methods for controlling the main loop  */
//
class lcontrol
{
public:
#define LOOP_0_count 0
#define LOOP_1_rotate 1
#define LOOP_2_display 2
#define LOOP_3_clear 3
#define LOOP_4_save 4
#define LOOP_5_reboot 5
#define LOOP_6_json 6

  int current_cmd_global;
  int current_cmd;
  int inChar;
  unsigned long delay_time = 1000;

private:
};

/********************* end of class definitions ******************/

static usbio usbio2; // i/o via USB to FrontPanelH316.c
// D_reg *fp_dreg;
D_base *fp_clear;  // clear button object
D_base *fp_ss1;    // ss1 switch
D_base *fp_rsel;   // last register select switch
D_base *fp_rsel_A; // register select A object
D_base *fp_rsel_B;
D_base *fp_rsel_OP;
D_base *fp_rsel_PY;
D_base *fp_rsel_M;

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
D_byte::D_byte(MbedI2C *ptr, int addr, int debug)
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
  defined_bits = 0;   // no bits defined yet
  debug_flag = debug; // save this for optional debug printouts
};

/** Write one bit to i/o expander byte, but do not flush
 *
 */
void D_byte::W_byte(int addmask, int data_outx)
{
  data_out = (data_out & (~addmask)) | (data_outx & addmask); // update saved value
  if (debug_flag)
  {
    // Serial.print("W_byte: ");
    // Serial.println(data_out, HEX);
  };
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
  unsigned long time_now;
  unsigned int debounce = 100; // minimum milliseconds between reads
  // if (debug_flag)
  //   Serial.print("W_bytef: ");
  // if (debug_flag)
  //   Serial.println(data_out, HEX);
  i2c_bus->beginTransmission(i2caddr);
  work = (0xf0 | ((data_out & 0x0f) ^ 0x0f));
  i2c_bus->write((0xf0 | ((data_out & 0x0f) ^ 0x0f))); // invert bits
  // if (debug_flag)
  //   Serial.println(work, HEX); //
  i2c_bus->endTransmission();
  change_flag = 0; // clear change flag
  //
  // add logic to read input
  //
  time_now = millis();
  if ((time_now - change_time) < debounce)
    return;

  i2c_bus->requestFrom(i2caddr, 1);
  previous_in = data_in;
  data_in = i2c_bus->read(); // read new data
  if (((data_in ^ previous_in) & 0xf0) != 0)
  {
    change_time = time_now;                               // save last time of change
    changed_in = (((data_in ^ previous_in) & 0xf0) >> 4); // find changed bits - mask and shift
    work = rev4[defined_bits];
    work = defined_bits;
    changed_in = rev4[changed_in] & work; // reverse bits to match input bit order

    // Serial.print("W_bytef: in/prev/changed: ");
    // Serial.print(i2caddr, HEX);
    // Serial.print(" ");
    // Serial.print(data_in, HEX);
    // Serial.print(" ");
    // Serial.print(previous_in, HEX);
    // Serial.print(" ");
    // Serial.println(changed_in, HEX);
  }
  // Serial.print("W_bytef read data:");
  // Serial.println(data_in,HEX);
};

/** Create D_byte object and add to D_io object.
 * This is how D_io can control all bytes.
 */
D_byte *D_io::make(MbedI2C *bus, int addr, int debugx)
{ // add the byte register in an i/o expander
  //    Serial.println("D_io make");
  temp = new D_byte(bus, addr, debugx);
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
        //     D_bits[i]->W_bit(i, 1);       // Write output bit via byte (old code)
        D_bits[i]->W_bit(i, (data_in & (D_bits[i]->mask))); // Write output bit via byte

        D_bits[i]->R_changed_bit(0); // update registers/buttons using this bit BJD

        changed_in = changed_in & (~(D_bits[i]->mask));
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
  D_reg_bitno = 0;
};

//  dummy R_bit
/** Update bit in D_reg or D_button?  */
int D_bit::R_bit(int in)
{
  // Serial.print("D_bit::R_bit: ");
  // Serial.print((long unsigned int)this, HEX);
  // Serial.print(" ");
  // Serial.println(D_reg_bitno, HEX);
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

  // Serial.print("D_reg::R_bit: ");
  // Serial.print((long unsigned int)this, HEX);
  // Serial.print(" ");
  // Serial.println(bitno, HEX);

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
  i = parent->R_bytef() & mask; // find - changed to 0 or 1?
  // Serial.println("D_bit::R_changed_bit ");
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

    // while(Serial.available() > 0) Serial.read();

    // Serial.print("usbio::in_char  ");
    // Serial.println((int) inCharx,HEX);
    if (inCharx == '<')
    { // < means start string
      while ((inCharx = Serial.read()) != '>')
      {
        if (inCharx > 0)
        {
          buff[index++] = inCharx;
          buff[index] = 0;
        }
      }

      Serial.print("<string> ");
      Serial.println(buff);
      return ('g'); // key for case statement
    }
    // not start of string
    if ((inCharx >= 0x41) && (inCharx <= 0x7a))
    { // was 7a
      return (inCharx);
    }
    else
      return (-1);
  }
  return (-1);
}
char *usbio::in_string()
{
  buff[index] = 0;
  index = 0;
  return (buff);
};

/** @brief process a json command enclosed in <>
 * <{"name":"H316 Front Panel Status","A":668,"B":1024}> (test data)
 */
void process_json()
{
  int temp;
  char *inputx = usbio2.in_string();
  temp = status_reg(inputx, (char *)"A");
  if (temp >= 0)
  {
    fp_dreg->value = temp;
    fp_rsel_A->value = temp; //
    fp_dreg->D_reg_write_word(temp);
  };
  //
  temp = status_reg(inputx, (char *)"B");
  if (temp >= 0)
  {
    fp_rsel_B->value = temp; //
  };
  //
  temp = status_reg(inputx, (char *)"OP");
  if (temp >= 0)
  {
    fp_rsel_OP->value = temp; //
  };
  //
  temp = status_reg(inputx, (char *)"P/Y");
  if (temp >= 0)
  {
    fp_rsel_PY->value = temp; //
  };
  //
  temp = status_reg(inputx, (char *)"M-reg");
  if (temp >= 0)
  {
    fp_rsel_M->value = temp; //
  };
  R_select *tempptr = (R_select *)fp_rsel_A;
  tempptr->display_current(); // update front panel reg
};

/** @brief: process json command to set A register
 *
 */
int status_reg(const char *const monitor, char *reg_name)
{
  const cJSON *a_ptr = NULL;
  const cJSON *name = NULL;
  int status = -1;

  cJSON *monitor_json = cJSON_Parse(monitor);
  if (monitor_json == NULL)
  {
    const char *error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != NULL)
    {
      fprintf(stderr, "Error before: %s\n", error_ptr);
    }
    status = -1;
    goto end;
  }

  name = cJSON_GetObjectItemCaseSensitive(monitor_json, "name");
  if (cJSON_IsString(name) && (name->valuestring != NULL))
  {
    printf("Checking monitor \"%s\"\n", name->valuestring);
  }

  a_ptr = cJSON_GetObjectItemCaseSensitive(monitor_json, reg_name);
  if (cJSON_IsNumber(a_ptr))
  {
    status = a_ptr->valuedouble;
    goto end;
  }

end:
  cJSON_Delete(monitor_json);
  // Serial.print("json values ");
  // Serial.print(status);
  // Serial.print(reg_name);
  // Serial.println();
  return status;
};

/************************* end of object member functions ************************/

//
/************************* start of "real" code ************************************/
//

static D_io *D_io_base;
D_byte *dbyte_save;
D_bit *dbit_save;
// int R_select::reg_max = 0;  //
// R_select*  R_select::*reg_select[6];// ??
//
// D_reg *fp_dreg;
// D_base *fp_clear; // clear button object
// D_base *fp_ss1;   // ss1 switch

// static int current_cmd_global;
static lcontrol loop_control;

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

  /***  define top 16 bits */

  dbyte_save = D_io_base->make(&xwire0, 0x26, 0); // make one i/o expander
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

  dbyte_save = D_io_base->make(&xwire0, 0x22, 0); // i2c addr of i/o expander chip
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

  dbyte_save = D_io_base->make(&xwire0, 0x24, 0); // i2c addr of i/o expander chip
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
  dbyte_save = D_io_base->make(&xwire0, 0x20, 0); // i2c addr of i/o expander chip
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
  /*** define 5th byte for CLEAR button ** */
  //
  dbyte_save = D_io_base->make(&xwire0, 0x21, 0); // make one i/o expander (on SDA, SCL

  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_clear = dbit_save->D_reg_link = new D_clear(dbit_save, (char *)"CLR");
  //
  //  define sense switches
  //
  dbyte_save = D_io_base->make(&xwire1, 0x20, 1); // make one i/o expander
                                                  //      (on SDA, SCL = ,5)(20,22,24,26)
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"SS1");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"SS2");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"SS3");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"SS4");

  // define register-select "radio buttons" for A, B, OP, P/Y

  dbyte_save = D_io_base->make(&xwire1, 0x24, 0); // i2c addr of i/o expander chip
                                                  //      (on SDA, SCL = ,5)(20,22,24,26)

  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_rsel_A = dbit_save->D_reg_link = new R_select(dbit_save, (char *)"A");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_rsel_B = dbit_save->D_reg_link = new R_select(dbit_save, (char *)"B");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_rsel_OP = dbit_save->D_reg_link = new R_select(dbit_save, (char *)"OP");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_rsel_PY = dbit_save->D_reg_link = new R_select(dbit_save, (char *)"P/Y");

  dbyte_save = D_io_base->make(&xwire1, 0x22, 0); // i2c addr of i/o expander chip
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_rsel_M = dbit_save->D_reg_link = new R_select(dbit_save, (char *)"M-reg");

  //  define action buttons
  //  define Master Clear, Fetch, P+1
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"M-clear");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"Fetch");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"P+1");

  //  define MA/Run/ Start ..
  dbyte_save = D_io_base->make(&xwire1, 0x26, 0); // i2c addr of i/o expander chip
  dbit_save = dbyte_save->mbit(dbyte_save, 0x01);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"MA/Fetch/");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x02);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"MA/SI/RUN");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x04);
  fp_clear = dbit_save->D_reg_link = new D_demo(dbit_save, (char *)"Start");
  dbit_save = dbyte_save->mbit(dbyte_save, 0x08);
  fp_clear = dbit_save->D_reg_link = new D_spare(dbit_save, (char *)"Spare");
  //      (on SDA, SCL = ,5)

  fp_dreg->D_reg_end_bits(); // update bit number after last bit added

  if (DEBUGX)
  {
    while (Serial.available() <= 0) // wait - enter c/r to start test
      delay((unsigned long)10);
    Serial.println("\nPanel Test - enter anything 2 start");
    Serial.read();
  }
  // test json
  // char xstringL = '"{"name":"H316 Front Panel Status","A":668}"';
  // char *xstring = &xstringL;
  // printf(xstring);
  // int itemp = status_A(xstring);
  // printf("\nreturn A = %o\n", itemp);
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
  D_io_base->R_scan();

  delay(delay_time); // wait for a second
}

/********************************* rotate display pattern *************************/
static unsigned long pattern = 072727;
static unsigned long pattern_reset = 0x8000;

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

int step_cmd()
{ // advance to the next command
  loop_control.current_cmd_global++;
  if (loop_control.current_cmd_global > LOOP_2_display)
    loop_control.current_cmd_global = LOOP_0_count;
  Serial.print("Changed demo command mode: ");
  Serial.println(loop_control.current_cmd_global);
  fp_dreg->D_reg_write_word(0xffff);
  D_io_base->W_wordf(); // flash all ones
  delay(100);
  fp_dreg->D_reg_write_word(0);
  D_io_base->W_wordf();
  return (0);
}

//  decode commands from console (for test)
int command(int i)
{
  int return_value;
  // if (1)
  //   Serial.print("command: from USB - ");
  // if (1)
  //   Serial.println(i, HEX);
  return_value = (i - 'a');
  return (return_value);
}

void reboot()
{
  Serial.println("Reboot now:");
  watchdog_reboot((uint32_t)0, (uint32_t)0, (uint32_t)0x7fffff);
};

void loop()
{
  // unsigned long mytime;
  // mytime = millis();

  // put your main code here, to run repeatedly:
  //
  // the loop function runs over and over again forever
  // the loop is entered many times - not just once
  //

  loop_control.inChar = usbio2.in_char();
  if (loop_control.inChar != -1)
  {
    loop_control.current_cmd_global = command(loop_control.inChar);
    // Serial.println(current_cmd_global,HEX);
    loop_control.delay_time = 1000;
  }
  switch (loop_control.current_cmd_global) // add Loop Control Symbole BJD
  {
  case LOOP_0_count: // a
    count(loop_control.delay_time);
    if (usbio2.in_available() > 0)
      loop_control.delay_time = 50;
    else
      loop_control.delay_time = 1000;
    break;
  case LOOP_1_rotate: // b
    rotate(loop_control.delay_time);
    // if (usbio2.in_available() > 0) delay_time = 50; else delay_time = 1000;
    loop_control.delay_time = 100;
    break;
  case LOOP_2_display: // c
    display_register(loop_control.delay_time);
    break;
  case LOOP_3_clear: // d
    clear_register();
    loop_control.current_cmd_global = LOOP_2_display; // back to display registeer
    break;
  case LOOP_4_save: // e
    save_register();
    loop_control.current_cmd_global = LOOP_2_display; // back to display register
    break;
  case LOOP_5_reboot: // f
    reboot();         // force reboot via wdt
    delay(9000);
    break;
  case LOOP_6_json: // g implies <>
    process_json();
    loop_control.current_cmd_global = LOOP_2_display;
    break;
  default:
    count(50);
    loop_control.current_cmd_global = LOOP_1_rotate; // default to rotate
    break;
  }

  // D_io_base->W_wordf(); // write and read all bytes ---BJD DEBUG - experimental
  // D_io_base->R_scan();

  // /Users/bdietz/Library/Arduino15/packages/arduino/hardware/mbed_rp2040/4.2.4
  //  /cores/arduino/mbed/targets/TARGET_RASPBERRYPI/TARGET_RP2040/pico-sdk/rp2_common/hardware_watchdog/include/hardware
}
