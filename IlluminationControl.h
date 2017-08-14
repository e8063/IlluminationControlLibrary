#ifndef IlluminationControl_h
#define IlluminationControl_h
#include "arduino.h"

struct AddressList{
public:
    char* menber;
    short id;
};

struct Pattern_List{
public:
    void (*func_address)();
    char* controlword;
};

struct Change_list{
public:
    int pattern_number;
    short id;
    bool (*trigger_func)();
    char* comp_sentence;
    char* patternName;
    byte trigger_mode;
};

class IlluminationControl{
public:
    IlluminationControl();
    static bool systemWait;
    static bool change_mode;
    static char* mode;
    static int running_func_number;
    static byte nowModePriority;
    static char Mode;
    static int RunningFunctionNumber;
    //XBee section
    static bool setAddress(long,long,char*);
    static short sarchId(char*);
    static bool SendCommandString(short,char*);
    static bool SendCommand(char*,char);
    static char* getData(char*);
    //Lib section
    static void setup();
    static void loopEnd();
    static void Wait();
    static void delay(unsigned long);
    static bool setPattern(void (*)(),char*);
    static bool setChangeXBee(char*,char*,char*,byte);
    static bool setChangeXBee(char*,char*,byte);
    static bool setChangeFunction(char*,bool (*)(),byte);
    static void XbeeLoop();
    static bool changeMode(char*);
    static bool setDefault(char*);
    //string_sprit
    static int split(String, char, String*);
    //TERMINAL
    static const String commmand[];
    static const String setModeOption[];
    static void terminalSetup(int);
    static void terminal();
    static void set_prompt();
    static void wait_input();
    static int comp_commmand(String);
    static void Info();
    static bool Send(int,String);
    static void terminal_print(String);
    static void terminal_println(String);
    static void terminal_begin(int);
    static void Help();
    static int comp_setmode(String);
    //内部関数
    static void Run();
    static bool SerectMode();
    static int sarchPatternNumber(char*);
    //構造体
    static Pattern_List list[];
    static Change_list trigger_list[];
    static AddressList Address_list[];
private:
};

#endif