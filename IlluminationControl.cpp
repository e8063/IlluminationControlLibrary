#include "arduino.h"
#include "IlluminationControl.h"
#include<XBeeLibrary.h>
#include <Scheduler.h>
XBeeLibrary XBee;

#define func_max 50//登録可能なパターン数
#define ALLWORD "all"

jmp_buf buf_data;

bool IlluminationControl::systemWait = false;//コマンドラインからの制御文字
bool IlluminationControl::change_mode=false;
char* IlluminationControl::mode;
int IlluminationControl::running_func_number = 0;
byte IlluminationControl::nowModePriority = func_max;//現在のMode判定における優先順位を格納する変数　※fanc_maxではすべてのMode判定を処理

AddressList IlluminationControl::Address_list[63];
Pattern_List IlluminationControl::list[func_max];
Change_list IlluminationControl::trigger_list[func_max];


IlluminationControl::IlluminationControl(){
    ;
}

//XBee section
bool IlluminationControl::setAddress(long pHiAddress,long pLowAddress,char* menberName){
    for(int i=0;i<63;i++){
        if(Address_list[i].id == -1){
            Address_list[i].id = XBee.setAddress(pHiAddress, pLowAddress);
            if(Address_list[i].id == -1)
                return false;
            Address_list[i].menber = menberName;
            return true;
        }
    }
    return false;
}

short IlluminationControl::sarchId(char *menberName){//menberからidを検索する関数
    if(XBee.compSentence(menberName,ALLWORD))
        return 0;
    for(int i=0;i<63;i++){
        if(Address_list[i].id == -1)
            return -1;
        if(XBee.compSentence(Address_list[i].menber ,menberName))
            return Address_list[i].id;
    }
    return -1;
}

bool IlluminationControl::SendCommand(char* menberName,char data){//CW送信専用の関数
    char senddata[2];
    senddata[0] = data;
    senddata[1] = "\n";
    return XBee.sendMesseage(sarchId(menberName),senddata);
}

bool IlluminationControl::SendCommandString(short menberName,char* data){//データ送信専用の関数
    return XBee.sendMesseage(sarchId(menberName),data);
}

char* IlluminationControl::getData(char *menberName){//センサーからのデータを読み取る関数(同期ずれの可能性あり....)
    return XBee.recieveRXData(sarchId(menberName));
}

//Lib section
void IlluminationControl::setup(){
    XBee.setup(true);
    Scheduler.startLoop(XbeeLoop);
    Scheduler.startLoop(Run);
}

void IlluminationControl::loopEnd(){
    Scheduler.yield();
}

void IlluminationControl::Wait(){//待機用関数
    while(true){
        if(change_mode)
            break;
        Scheduler.yield();
    }
}

void IlluminationControl::delay(unsigned long ms){//ライブラリ対応版delay関数
    unsigned long start_time = millis();
    while(true){
        if(change_mode){
            change_mode = false;
            longjmp(buf_data,1);
        }
        if(systemWait){
            while(true){
                if(!systemWait){
                    longjmp(buf_data,1);
                }
                delay(100);
            }
        }
        if(start_time + ms <= millis())
            break;
        Scheduler.yield();
    }
}

bool IlluminationControl::setPattern(void (*pattern_func)(),char* select_word){
    for(int i=0;i<func_max;i++){
        if(list[i].func_address == NULL){
            list[i].func_address = pattern_func;
            list[i].controlword = select_word;
            return true;
        }
    }
    return false;
}

bool IlluminationControl::setChangeXBee(char* patternName,char* menberName,char* comp_sentence,byte priority){//ある文字列が送られてきたタイミングでモード変更判定
    trigger_list[priority-1].id = sarchId(menberName);
    if(trigger_list[priority-1].id == -1 && (priority >= 100||priority == 0))
        return false;
    trigger_list[priority-1].pattern_number = sarchPatternNumber(patternName);
    trigger_list[priority-1].comp_sentence = comp_sentence;
    trigger_list[priority-1].trigger_mode = 0;
    trigger_list[priority-1].patternName = patternName;
    return true;
}

bool IlluminationControl::setChangeXBee(char* patternName,char* menberName,byte priority){//XBeeからデータが送られてきた段階でモード変更判定
    trigger_list[priority-1].id = sarchId(menberName);
    if(trigger_list[priority-1].id == -1 && (priority >= 100||priority == 0))
        return false;
    trigger_list[priority-1].pattern_number = sarchPatternNumber(patternName);
    trigger_list[priority-1].trigger_mode = 1;
    trigger_list[priority-1].patternName = patternName;
    return true;
}

//モードが切り替わったらそのモードの条件判別は行わない!!!!!
//(モード内でXBeeからの信号を扱う時のため)
bool IlluminationControl::setChangeFunction(char* patternName,bool (*trigger_func)(),byte priority){//関数の返り値でモード変更判定
    if(priority >= 100||priority == 0)
        return false;
    trigger_list[priority-1].pattern_number = sarchPatternNumber(patternName);
    trigger_list[priority-1].trigger_func = trigger_func;
    trigger_list[priority-1].trigger_mode = 2;
    trigger_list[priority-1].patternName = patternName;
    return true;
}

void IlluminationControl::Run(){//パターン関数を実行するSchedulerループ
    //Serial.println("Run");
    setjmp(buf_data);
    if(list[running_func_number].func_address != NULL)
        list[running_func_number].func_address();
    nowModePriority = func_max;//判定条件をリセット
    Scheduler.yield();
    change_mode = false;//切替後の誤動作防止の為にchagnge_modeをリセットする
}


void IlluminationControl::XbeeLoop(){//送信と受信の処理を行う関数(loopActionはこの関数内だけ!!!)
    //Serial.println("XBee");
    XBee.loopAction();
    if(SerectMode())
        change_mode=true;
    Scheduler.yield();
}

bool IlluminationControl::SerectMode(){//モードの変更を検出する関数 返り値は変更の可否
    for(int i=0;i<func_max;i++){
        if(trigger_list[i].pattern_number == -1)
            return false;//リストが未登録領域に到達した場合、中止する
        if(i == nowModePriority)
            return false;//優先順位が現在よりも低いものは除外
        char* recieve;
        switch(trigger_list[i].trigger_mode){
            case 0:
                recieve = XBee.recieveRXData(trigger_list[i].id);
                if(recieve){
                    if(XBee.compSentence(trigger_list[i].comp_sentence,recieve)){
                        running_func_number = trigger_list[i].pattern_number;
                        mode = trigger_list[i].patternName;
                        nowModePriority = i;//add
                        return true;
                    }
                }
                break;
                
            case 1:
                recieve = XBee.recieveRXData(trigger_list[i].id);
                if(recieve){
                    running_func_number = trigger_list[i].pattern_number;
                    mode = trigger_list[i].patternName;
                    nowModePriority = i;//add
                    return true;
                }
                break;
                
            case 2:
                if(trigger_list[i].trigger_func()){
                    running_func_number = trigger_list[i].pattern_number;
                    mode = trigger_list[i].patternName;
                    nowModePriority = i;//add
                    return true;
                }
                break;
                
            default:
                break;
        }
    }
    return false;
}

bool IlluminationControl::changeMode(char* AfterMode){//モードを変更する関数(フロントエンド)　戻り値は変更の可否
    for(int i=0;i<func_max;i++){
        if(XBee.compSentence(list[i].controlword , AfterMode)){
            change_mode = true;
            running_func_number = i;
            mode = AfterMode;
            return true;
        }
        if(list[i].func_address == NULL){
            return false;
        }
    }
    return false;
}

int IlluminationControl::sarchPatternNumber(char* patternName){//パターンの格納されている番号(funcNumber)を探す関数
    for(int i=0;i<func_max;i++){
        if(XBee.compSentence(list[i].controlword , patternName)){
            return i;
        }
        if(list[i].func_address == NULL){
            return -1;
        }
    }
    return -1;
}

bool IlluminationControl::setDefault(char* AfterMode){//changeModeのchange_modeに対するフラグ立てがないバージョン
    for(int i=0;i<func_max;i++){
        if(XBee.compSentence(list[i].controlword , AfterMode)){
            running_func_number = i;
            mode = AfterMode;
            return true;
        }
        if(list[i].func_address == NULL){
            return false;
        }
    }
    return false;
}

//string section
int IlluminationControl::split(String data, char delimiter, String *dst){
    int index = 0;
    int arraySize = (sizeof(data)/sizeof((data)[0]));
    int datalength = data.length();
    for (int i = 0; i < datalength; i++) {
        char tmp = data.charAt(i);
        if ( tmp == delimiter ) {
            index++;
            if ( index > (arraySize - 1)) return -1;
        }
        else dst[index] += tmp;
    }
    return (index + 1);
}

//terminal section
#define command_num 6
#define command_end '\n'
#define TERMINAL Serial2
#define setmode_num 2

const String IlluminationControl::commmand[command_num]={
    "",//return prompt command
    "Info",
    "Send",
    "Help",
    "Status",
    "Setmode"
};

const String IlluminationControl::setModeOption[setmode_num]={
    "Auto",
    "Manual"
};

void IlluminationControl::terminalSetup(int bps){
    terminal_begin(bps);
    Scheduler.startLoop(terminal);
}

void IlluminationControl::terminal(){
    static int command_select;
    set_prompt();
    wait_input();
    
    String recieve = TERMINAL.readStringUntil(command_end);
    // 分割数 = 分割処理(文字列, 区切り文字, 配列)
    String split_command[4] = {"\0"}; // 分割された文字列を格納する配列
    int index = split(recieve,' ',split_command);
    
    command_select = comp_commmand(split_command[0]);
    
    switch(command_select){
        case 0:
            //no command
            break;
            
        case 1://Info
            Info();
            break;
            
        case 2://Send
            if(index > 3)
                terminal_println("Send:Too many arguments.");
            else if(index < 3){
                terminal_println("Send:Prease type arguments.");
                terminal_println("     Send command have 2 arguments.");
                terminal_println("     Refarence:Send [XBeeNumber] [SendMessage]");
            }
            else{
                if(Send(split_command[1].toInt(),split_command[2]))
                    terminal_println("Send:succsessfully!");
                else
                    terminal_println("Send:Can't send message!");
                terminal_println("     Prease check Message and System.");
            }
            break;
            
        case 3:
            Help();
            break;
            
        case 4:
            
            break;
            
        case 5:
            if(index > 2)
                terminal_println("Setmode:Too many arguments.");
            else if(index < 2){
                terminal_println("Setmode:Prease type arguments.");
                terminal_println("        Setmode command have 1 arguments.");
                terminal_println("        Refarence:Setmode [Mode]");
                terminal_println("        [Auto] is default mode.(Auto running)");
                terminal_println("        [Manual] is test mode.(Manual Control by terminal)");
            }
            else{
                switch(comp_setmode(split_command[1])){
                    case 0:
                        if(systemWait){
                            systemWait = false;
                            terminal_println("Setmode:Running mode changed Auto Mode.");
                        }
                        else
                            terminal_println("Setmode:Running mode is Already Auto Mode.");
                        
                        break;
                    case 1:
                        if(systemWait)
                            terminal_println("Setmode:Running mode changed Already Manual Mode.");
                        else{
                            systemWait = true;
                            terminal_println("Setmode:Running mode is Manual Mode.");
                        }
                        break;
                    default:
                        terminal_println("Setmode:Undefined arguments.");
                        terminal_println("        Defined arguments refarence:");
                        terminal_println("        [Auto] is default mode.(Auto running)");
                        terminal_println("        [Manual] is test mode.(Manual Control by terminal)");
                        break;
                }
            }
            
            break;
            
        case -1:
            terminal_println("Terminal:command not found.");
            break;
            
        case -2:
            terminal_println("Too many space.");
            break;
            
            
        default:
            break;
    }
}

void IlluminationControl::set_prompt(){
    terminal_println("Arduino >");
}

void IlluminationControl::wait_input(){
    while(true){
        if(TERMINAL.available() > 0)
            break;
        delay(100);
    }
}

int IlluminationControl::comp_commmand(String data){
    for(int i=0;i<command_num;i++)
        if(data.equalsIgnoreCase(commmand[i]))
            return i;
    return -1;
}

void IlluminationControl::Info(){
    terminal_println("--NIT,Maizuru Illminatuion Control System--");
    terminal_println("Version 1.0  for Arduino Mega");
    terminal_println("");
    terminal_println("Developer:Kenta  Yamamoto e8063");
    terminal_println("Supporter:Shinji Takuma   e7704");
    terminal_println("");
    terminal_println("This system included XBee Library.");
    terminal_println("XBee Library is created by Shinji Takuma.");
    terminal_println("");
    terminal_println("Developer's Email:e8063@g.maizuru-ct.ac.jp");
}

bool IlluminationControl::Send(int xbee_num,String send_message){
    /*if(xbee_num == 0)
     terminal_print("Do you want to send message to \"all\" XBee member?");
     terminal_print("[Yes:Y/No:N] >");*/
    char buf[10];
    send_message.toCharArray(buf,send_message.length());
    return  XBee.sendMesseage(xbee_num,buf);
}

void IlluminationControl::terminal_print(String data){
    TERMINAL.print(data);
}
void IlluminationControl::terminal_println(String data){
    TERMINAL.println(data);
}

void IlluminationControl::terminal_begin(int bps){
    TERMINAL.begin(bps);
}

void IlluminationControl::Help(){
    terminal_println("Help:-Command list&function-");
    terminal_println("     info    :System infomation");
    terminal_println("     Send    :Send Message(String) to XBee Rooter");
    terminal_println("     Status  :Print Screen Now Status");
    terminal_println("     Setmode :Set or Change Running Mode");
    terminal_println("     ");
}

int IlluminationControl::comp_setmode(String data){
    for(int i=0;i<setmode_num;i++)
        if(data.equalsIgnoreCase(setModeOption[i]))
            return i;
    return -1;
}


