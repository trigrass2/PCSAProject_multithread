

//define 
#define STA_TIMEOUT  5
#define ADDERR       -100


class MDCE;

// call this beore any function in case application is not MFC
void setMFCmode();


// manage com ( if pcom =NULL ) get the default pcom as created from the last openPcom
char * ver(int print);  // if print=1 put message box with MPI dll version, else only return str message with version information
MDCE * openDCE( char *pwdin, char *cnfname);

int  setComPar(int port, 
			   int baudrate, 
			   int mode, 
			   int trid, 
			   int rcid, 
			   int cancanbaudrate, 
			   int msgStand, 
			   int canpipelevel,  
			   int timeout, 
			   int locktime, 
			   int iternum, 
			   MDCE *pcom=NULL);

int  getComPar(int *pport,
			   int *pbaudrate,
			   int *pmode, 
			   int *ptrid, 
			   int *prcid, 
			   int *pcancanbaudrate, 
			   int *pmsgStand, 
			   int *pcanpipelevel, 
			   int *ptimeout, 
			   int *plocktime, 
			   int *piternum, 
			   MDCE *pcom=NULL);

// Similar to setComPar but include also ethernet setting option
int  setComParEx(int port, 
			   int baudrate, 
			   int mode, 
			   int trid, 
			   int rcid, 
			   int cancanbaudrate, 
			   int msgStand, 
			   int canpipelevel,  
			   int timeout, 
			   int locktime, 
			   int iternum, 
			   char *ipstr,     // string with thwe IP address
			   int ipport,      // IP port number
			   int nodelay,    // TRUE =>Disables the Nagle algorithm for send coalescing
			   MDCE *pcom=NULL);

int  getComParEx(int *pport,
			   int *pbaudrate,
			   int *pmode, 
			   int *ptrid, 
			   int *prcid, 
			   int *pcancanbaudrate, 
			   int *pmsgStand, 
			   int *pcanpipelevel, 
			   int *ptimeout, 
			   int *plocktime, 
			   int *piternum, 
			   char *ipstr,     
			   int *pipport,    
			   int *pnodelay,   
			   MDCE *pcom=NULL);

// open Ethernet connection use this instead of setComPar/setComParEx to open communication via Ethernet
int ConnectEthernet(char *ipstr, 
					int ipport, 
					int flag, 
					int showMsgWin, 
					int timeout, 
					int locktime, 
					int iternum, 
					MDCE *pcom=NULL);


// disable all message box  but version message  
void disErrMsgBox(int dis);
// disable all message box  including version message  
void disAllErrMsgBox(int dis);
// return non zero if any version compare error, version compare is done after communication first set up or called to compareFw
int getFwErr(MDCE *pcom=NULL);

// return version error for specific slave, each 2 bits are for each data base file 
unsigned int gerVerErrCodeSl(int slave=0, MDCE *pcom=NULL);


// perform firmwares programs version compare between database in PC and controller contain
void compareFw(int show=0, MDCE *pcom=NULL);
// reset controller
void resetController(int slave=0 , MDCE *pcom=NULL);

int  getDceCnfFname(char *str, MDCE *pcom=NULL);            // get configuration file full name
int  getSlaveFname(char *str, int slave=0, MDCE *pcom=NULL);// get controller ( direcory ) name
void closePort(MDCE *pcom=NULL); 
int  openPort(MDCE *pcom=NULL);
int openRemote(MDCE *pcom=NULL);
void showcomstatus(MDCE *pcom=NULL);
void RunKmi(MDCE *pcom=NULL);
void KillKmi(MDCE *pcom=NULL, int closenet=0);

// load all software, if pdlComp!=0 then also do PDL compilation if needed
int loadDceSw(int pdlComp, MDCE *pcom);
// load firmware name, inf name = NULL then load the current application name
int loadFirmName(char *name, int slave, MDCE *pcom);


// run application in remote mode
int  RunAppl(char *papplname, MDCE *pcom=NULL );
// kill application, applid is the value return from RunAppl
void KillAppl(int applid, MDCE *pcom=NULL, int closenet=0);

// update  database
void updateDataBase(MDCE *pcom);

// call in application end
void deleteDCE(MDCE *pcom=NULL, int closeMFC=FALSE); 

//==============================================================================
// wait on any printl message come from DCE
// This function return any printl PDL command message 
// parameters  :
//   *pcode - return message code, The application may filter this messages by decoding the code
//   *pmsg - return the string of the message as defined by the 'printl' command in the PDL
//   *pcolor - return the color  of the message as defined by the 'printl' command in the PDL
//   *pparam - pparam should point to array of at list 5 int. first item will be written with the number of parameters (0...4)
//   *pnumovrflw - return number of overflow accord ( massage lost ) may used for testing
//   *timeput - time out to wait for message, default is INFINITE as the applicatin should always wait for any message
//   *pcom - point to communication object
// return value: 0 - ok, -1 no communication driver, -2 no message available, 5 time out
// If  return that massage were lost, still return ok the last current message


int waitOnMsgP( int *pcode, char *pmsg=NULL, int *pcolor=NULL, int *pparam=NULL, int *pnumovrflw=NULL, int timeout=INFINITE, MDCE *pcom=NULL);

// if thread wait on waitOnMsgP, this will release it  with return value -2, call it before call deleteCom or delete the thread listen
void   releaseWaitMessage(MDCE *pcom);




// insert event HANDLE to tha event table, it will be signaled when recivied printl with the specofied code (by setEventCode )
// and the first parameter equal to the id in insertEvent.
// code should be 0...199
// return 0 ok, -1 no called openDCE yet  -2 code out of range, 
int insertEvent(int id, HANDLE h, MDCE *pcom=NULL);

// close handle and remove it from table
// return 0 ok, -1 no called openDCE yet  -2 code out of range, -3 fail to close event handle
int closeEvent(int id, MDCE* pcom);

// return the HANDLE of the event located in table entry id
int getEvent(int id, MDCE* pcom);

//setEventCode may call only once to define all printl code that alow to send event , the event id is the first parameter of the printl
//if setEventCode not called then the default is code=1
// example of such printl, code=0x00001234,  id = axis_id
//printl/103/00001234("axis id=%ld finish with error code=%ld",axis_id,axis_err_status);
int setEventCode(int code, MDCE *pcom);

// retrived the lasd data recieved from printl with the specified code, it may be called after signaled the corespund event
// which registerd by insertEvent
int getLastEventData(int id, char *pmsg,  int *pparam, MDCE *pcom=NULL);



//==============================================================================
// record data

void OpenRecord(int show=1, MDCE *pcom=NULL);  //open record window

// start record parameters:
// *p[] -      array of pointers to variable name
// numVar -    number of variable to be record ( 1...maximum of avaliable record variables )
// rate -      define sample rate T = rate *0.05 msec
// numSamples- Maximum samples to be record
// *pdata-     result is written to arrayes of doble , pdata point to 2 dim array of numVar * numSamples
// errstr -    optional string that return error description
// return value 0 -ok, 1 Fail

// mode each bit in mode define if the correspond recorded variable will clipped to 16 bit value
void setRecClip16(int mode, MDCE *pcom);
// mode=1 set record to be start acording to hardwere trigger ( if support)
void setTrigger(int mode, MDCE *pcom);


int StartRecordData (  char *p[], int numVar, int rate, int numSamples, double *pdata, char *errstr, MDCE *pcom=NULL);

//Here the variable names are enter by one string 'varnames' seperated by spaces or comma
int StartRecordDataN(  char *varnames,        int rate, int numSamples, double *pdata, char *errstr, MDCE *pcom=NULL);

// similar to StartRecordDataN but result save to file instead to array
int StartRecordFileN( char *varnames, int rate, int numSamples, char *fileName, char *errstr,  int appened, MDCE *pcom);

void StopRecord(MDCE *pcom=NULL);
int GetRecdordStatus(int *pnumSampColect, MDCE *pcom); // return 0 when ended, 2 still in process, 3 fail to write to file, -1 not available

//==============================================================================
// Frequency analyzer
void FreqAnalOpen(int show=1, MDCE *pcom=NULL); // open frequency analyzer window

//return -1, not valid, 0 not in proccess, >0 in proccess,
// pfreq store current frequncy anlyzied,  proccess return number 0...100 according to how mach alreay done
// plasterror store last error code, errstr printed with last error description
int FreqAnalStatusGet( double *pfreq, double *process, int *plasterror, char *errstr, MDCE *pcom=NULL );

int FreqAnalStart(double startfr,  double stopfr, double stepfr,  double amplitude, 
                         char *varsignal, char *varinput, char *varoutput,  int slave, 
                         int mincyc, double idletime, double datatime,
                        char *filename, char *comment=NULL,  MDCE *pcom=NULL
                         );
// similar to FreqAnalStart but instead of startfr, stopfr, stepfr the frequency points are defined form frp_filename
int FreqAnalStartP(char *frp_filename,  double amplitude, 
                         char *varsignal, char *varinput, char *varoutput,  int slave, 
                         int mincyc, double idletime, double datatime,
                        char *filename, char *comment, MDCE *pcom
                         );


int FreqAnalStop(MDCE *pcom=NULL);


//--------- FFT ---------------
// return 0 ok, -1 memory falt, 1 user stop, -2 in proccess
int fftrun(double *pin, int num, double *pabs,  double *parg, int showWnd);


//===================================Axeess variables/arrayes ===========================================

// get error string  parameters:
// errcode - return code from state/variable/arrayes access functions
// str  - name of the relevant variable/state/ 
// slave slave number
// errstr - string to fill with the erro message. SHould point to string of length minimum 200 characters
// Note: dont use for error return from run PDL func commands 

void GetErrorStr(int errcode, char * str, int slave, char * errstr, MDCE *pcom=NULL ); 

// return variable address and size, if variable not exist return ADDERR
int GetVarAddr(char *varName,   int slave, int *psize, MDCE *pcom=NULL);

// return variable address, size and type, if variable not exist return ADDERR
int GetVarAddrType(char *varName,   int slave, int *psize, int *ptype, MDCE *pcom=NULL);

// read/write single variable
// The variable name may be indexed by [] , for example varName= "rec_buf[2]" ( relvant to access single item in array )
int  SetVarN(char *varName, double data,   int slave=0, MDCE *pcom=NULL);
int  GetVarN(char *varName, double *pdata, int slave=0, MDCE *pcom=NULL);

int  SetVarN64(char *varName, _int64 data,   int slave=0, MDCE *pcom=NULL);
int  GetVarN64(char *varName, _int64 *pdata, int slave=0, MDCE *pcom=NULL);


// read/write array
int setArrayDN(char *varName,  double *pdata, int num, int slave=0, MDCE *pcom=NULL); 
int getArrayDN(char *varName,  double *pdata, int num, int slave=0, MDCE *pcom=NULL); 
int setArrayN (char *varName,  int *pdata,    int num, int slave=0, MDCE *pcom=NULL);  
int getArrayN (char *varName,  int *pdata,    int num, int slave=0, MDCE *pcom=NULL);
int setArraySN (char *varName, short *pdata,  int num, int slave=0, MDCE *pcom=NULL);  
int getArraySN (char *varName, short *pdata,  int num, int slave=0, MDCE *pcom=NULL);

// read varuiable group (packet)
int getPac(char *varnames, char *l, void *data, int slave=0, MDCE *pcom=NULL); 


// read array and write array in one command
// array name to read  = varrep, num items=numrep, data will read into parrrep
// array name to write = varset, num items=numset, data will read into parrset
// parrrep,parrset sould point to the correct type (long ,short ,float) acording to variable type 
int getAndSetArrayN(char *varrep,  int numrep, void *parrrep,
			        char *varset,  int numset, void *parrset,  int slave, MDCE *pcom);

// similar as getAndSetArrayN, but array are double type abd variable in contr must be float
int getAndSetArrayDN(char *varrep,  int numrep, double *parrrep,
  			         char *varset,  int numset, double *parrset,  int slave, MDCE *pcom);

// read several variables +fclk(16bit) ( now support only 2) in the same DSP phase
// varnames=2 variable names are enter by one string 'varnames' seperated by spaces or comma, for example "X_enc_pos Y_enc_pos"
int GetScopeData(char *varnames, double *pdata, short *pfclk, int slave=0, MDCE *pcom=NULL); 


// states 
int setStateN(int slave ,char *stateName, int mode,    MDCE *pcom=NULL);
int getStateN(int slave ,char *stateName, int * state, MDCE *pcom=NULL);

//------------------ PDL run functions----------------------------------------------------

//..... run function in PDL 
// return -1 if fail other wise return task number that run this function (0..39)
// error description is written to errstr (if it is not NULL)
int RunFuncPdlN(char *pdlName, char *errstr, int slave, MDCE *pcom=NULL);

//===== set array anf run PDL function
// combine set array and run PDL function in one function. it is faster to run this function 
// instead of set array ( by setArray)  and after it run PDL function (by 'runFuncPdl')
// addr - variable/buffer address in DCE ( retrived by  getVarAddr )
// arr - pointer to data to set. if addr is to short type DC variable then arr must be pointer to short type array
//       else arr can can be pointer mixed long/float (32bit) array
// num - number of variable to write. if type is long/float then maximum is 13 if type is short then 26
// PDLaddr-PDL label address to run in DCE after data is set ( retrived by  GetFuncAddr )
// return - if success return task number  else negative and errstr contain error description
int SetArrayRunFuncN( char *varName , void * arr,int num,char *pdlName, char *errstr, int slave, MDCE *pcom=NULL);


// kill/stop/continue tasks from st to en, to kill only one task n set st=n en=n+1
// what: 0 kill task(s), 1 stop task(s), 2 continue tasks(s)
// return 0 success, else  call GetErrorStr with the error code return to get error description
int killTask(int st,int en, int what, int slave, MDCE *pcom=NULL);

//=======================call back function ===============================================

//set function to be call when state change the prototype function is: void func(BYTE type, int id)
void setCallBackStEvnt( void (*func)( BYTE ,int),  MDCE *pcom);

//set function to be call when puupd called, the prototype function is: void func(BYTE *rcbuf)
void setCallBackPuUpd( void (*func)( BYTE* ),  MDCE *pcom );


//===================== history loger operation =============================
//... start history loger, size define the maximum last events to store, 
// stepToSave relevant in file mode and define each how many events to write to disk file 
// fileMode if 1 then store to disk on line ( every stepToSave events), if 0 save to memory cyclic and only when stop save to disk
int HisStart(char *logFileName, int size, int stepToSave, int fileMode, MDCE *pcom);

//... stop history loger, result will saved to dce.log file
void HisStop( MDCE *pcom);

//.. save to dce.log and continue
void HisSave( MDCE *pcom);

// open the history view tool to show the last dce.log 
void HisView( MDCE *pcom);
