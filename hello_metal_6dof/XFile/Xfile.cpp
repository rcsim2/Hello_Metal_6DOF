//-----------------------------------------------------------------------------
// File: XFile.cpp
//
// Desc: R/C Sim Sikorsky: an R/C heli simulator
//		 This also shows how to load .X files in D3DIM.
//
// Note: This code uses the D3D Framework helper library.
//
//
// Copyright (c) 2000 Black Sphere Corp.
//-----------------------------------------------------------------------------
#include "precompiled.h"
#include "resource.h"
#include "xfile.h"


//#define STRICT
//#define D3D_OVERLOADS

//#include "xfile.h"
#include "dynamics.h"
#include "proxy.h"
//#include "search.h"
//#include "loadpic.h"
//#include "loadxrm.h"
#include "shell.h"
#include "cs_physics.h"
//#include "fswindow.h"




////////////////////////////////////////////////////////////////////////////////
// NOTE1: from Docs Online
// As you can see, if you have STRICT type checking enabled in one file, but not 
// in another, the C++ compiler will generate different external link symbols for 
// a single function. This will result in link-time errors.
// Oplossing: in elke file STRICT gebruiken, of extern "C" voor de onvindbare
// functies plaatsen
//
// NOTE2: from DirectX docs
// These extensions must be defined with C++ linkage. If D3D_OVERLOADS is defined 
// and the inclusion of D3dtypes.h or D3d.h is surrounded by extern "C", link errors 
// result. For example, the following syntax would generate link errors because of 
// C linkage of D3D_OVERLOADS functionality:
// 
// #define D3D_OVERLOADS
// extern "C" {
// #include <d3d.h>
// };
//
// NOTE3: every header file is made idempotent, therefore make sure to do these:
// #define STRICT
// #define D3D_OVERLOADS
// in every source file, not in the headers
//
// NOTE4a: make sure MSDEV finds the correct include and library files:
// Go to Tools|Options|Directories and add:
// Include files:
//  E:\MSSDK\INCLUDE
//  E:\MSSDK\SAMPLES\MULTIMEDIA\D3DIM\SRC\XFILE\HTMLHELP\INCLUDE AND LIB
//	E:\NTDDK\INC
//	D:\MISC\PHYSICS\ODE\ODE-0.035\ODE-0.035\INCLUDE
// Library files:
//  E:\MSSDK\LIB
//  E:\MSSDK\SAMPLES\MULTIMEDIA\D3DIM\SRC\XFILE\HTMLHELP\INCLUDE AND LIB
//	D:\MISC\PHYSICS\ODE\ODE-0.035\ODE-0.035\LIB
// Make sure to shift these directories to the top so they will be searched
// *before* the standard directories!!!
//
// NOTE4b: Include directories can also be specified in: Project|Settings|C/C++|
//	Category: Preprocessor|Additional include directories
//	Library directories can also be specified in: Project|Settings|Link|
//	Category: Input|Additional library path
//
// NOTE5: we have included the D3D Framework helper library (D3DFrameNew) in the overall
// project folder (XFile). This way we have all our stuff in one folder.
//
// NOTE6: Overview of libraries we link to in Debug configuration:
//  dsound.lib d3dx.lib dinput.lib dxguid.lib winmm.lib ddraw.lib d3dim.lib d3dxof.lib
//  .\d3dframenew\debug\d3dframe.lib kernel32.lib user32.lib gdi32.lib winspool.lib
//  comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib comctl32.lib
//  vfw32.lib htmlhelp.lib ode.lib
//
// NOTE7: We keep D3DFrame and HTMLHelp lib's in our project folder.
//	The DirectX and ODE libs are in other locations.
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
// Matrix layout:
// NOTE: m_matFileObjectMatrix stores the 6DOF's (position and orientation) of the heli.
// 6DOF: 6 Degrees of Freedom (3 linear + 3 angular):
// In a left-handed coordinate system:
// | Right.x Up.x  Forward.x 0 |
// | Right.y Up.y  Forward.y 0 |
// | Right.z Up.z  Forward.z 0 |
// | Pos.x   Pos.x Pos.z     1 |
//
//	m_matFileObjectMatrix._11 m_matFileObjectMatrix._12 m_matFileObjectMatrix._13 m_matFileObjectMatrix._14
//	m_matFileObjectMatrix._21 m_matFileObjectMatrix._22 m_matFileObjectMatrix._23 m_matFileObjectMatrix._24
//	m_matFileObjectMatrix._31 m_matFileObjectMatrix._32 m_matFileObjectMatrix._33 m_matFileObjectMatrix._34
//	m_matFileObjectMatrix._41 m_matFileObjectMatrix._42 m_matFileObjectMatrix._43 m_matFileObjectMatrix._44 
///////////////////////////////////////////////////////////////////////////////







//-----------------------------------------------------------------------------
// Global variables and functions
//-----------------------------------------------------------------------------

#ifdef DEMO
char g_szRCSIMRegistryKey[512] = "Software\\Blacksphere\\R/C Sim Sikorsky Demo";
#else
char g_szRCSIMRegistryKey[512] = "Software\\Blacksphere\\R/C Sim Sikorsky";
#endif

// R/C Sim Sikorsky program directory (set by InstallShield)
//char g_szRCSIMProgramPath[MAX_PATH] = ""; 
char g_szRCSIMProgramPath[512] = "";

// our media path
char g_szRCSIMMediaPath[512] = "..\\media\\";


// initial model position/orientation
float INIT_X = 0.0f;
float INIT_Y = -8.5f; // grounded == -8.5f;
float INIT_Z = 20.0f;
float INIT_RADS_Y = g_PI+0.5f; // nose-in==0.0f nose-out==g_PI

// initial collective and weight, etc.
float INIT_COLLECTIVE = 0.0f; //0.15f;
float INIT_WEIGHT = 0.30f;
float INIT_TORQUE = 0.18f;



// pilot positions
// add 8.5f to get correct Y values
// NONO: add 10.0f: we're in a hover pit of -10.0f

#ifdef _DEBUG
	float PILOTPOSITION1_X = 0.0f;
	float PILOTPOSITION1_Y = 0.0f;
	float PILOTPOSITION1_Z = 0.0f;
#else
	float PILOTPOSITION1_X = 0.0f;
	float PILOTPOSITION1_Y = 0.0f;
	float PILOTPOSITION1_Z = -5.0f;
#endif //_DEBUG

float PILOTPOSITION2_X = -52.0f;
float PILOTPOSITION2_Y = -4.0f;
float PILOTPOSITION2_Z = 115.0f;

float PILOTPOSITION3_X = 50.0f;
float PILOTPOSITION3_Y = -4.0;
float PILOTPOSITION3_Z = -5.0f;

float PILOTPOSITION4_X = 0.0f;
float PILOTPOSITION4_Y = -4.0f;
float PILOTPOSITION4_Z = 40.0f;

int g_iPPMarkSize = 2;

bool g_bShowPP1Mark = true;
bool g_bShowPP2Mark = true;
bool g_bShowPP3Mark = true;
bool g_bShowPP4Mark = true;

bool g_bDrawPPMarkShadows = false;




// use nifty helis
bool g_bBell = false;
bool g_bCobra = false;
bool g_bCougar = false;

// for direct input
#define WM_SYNCACQUIRE (WM_USER+1)


// for lens flare ////////////////////////////////////////////////////////////
// NOTE: on the Voodoo5 5500 PCI lensflare causes a crash
// On the SiS 6326 AGP lensflare works fine, also on the S3 Virge PCI
// So it is not a programming error by us
// Probably, the bug is in D3DIM700.DLL
// Hoewel...: alle lensflare Samples werken OK onder de Voodoo...

//LPDIRECT3D7       g_pD3D         = NULL;
LPDIRECT3DDEVICE7 g_pD3DDevice   = NULL;

D3DDEVICEDESC7 g_D3DDeviceDesc;

//
// Textures
//

char g_szPath[512];

//enum {
//    TEX_FLARE0, TEX_FLARE1, TEX_FLARE2, TEX_FLARE3,
//    NUM_TEXTURES
//};

char *g_szTexName[NUM_TEXTURES] = {
    "flare0.bmp", "flare1.bmp", "flare2.bmp", "flare3.bmp"
};

LPDIRECTDRAWSURFACE7 g_ppTex[NUM_TEXTURES];


//
// Light source
//

D3DXCOLOR g_colorLight(1.0f, 0.95f, 0.8f, 1.0f);
D3DXVECTOR4 g_vecLight(0.1f, -1.0f, -1.5f, 0.0f);

//
// Lens Flare
//

BOOL g_bCapsLensFlare = FALSE;
BOOL g_bDrawLensFlare = FALSE;
CLensFlare *g_pLensFlare = NULL;

D3DXMATRIX g_matPosition;
D3DXMATRIX g_matIdentity
    (1.0f, 0.0f, 0.0f, 0.0f,
     0.0f, 1.0f, 0.0f, 0.0f,
     0.0f, 0.0f, 1.0f, 0.0f,
     0.0f, 0.0f, 0.0f, 1.0f);

BOOL g_bSun = TRUE;
BOOL g_bFlare = TRUE;

///////////////////////////////////////////////////////////////////////////////


// for a wonderful kludge
char msg1[512];
char msg2[512];
char msg3[512];

char msg4[512];
char msg5[512];
char msg6[512];

char msg7[512];
char msg8[512];
char msg9[512];

char msg10[512];
char msg11[512];
char msg12[512];


bool g_bShowValues = false;


// for about dialog box
BOOL CALLBACK AboutProc( HWND, UINT, WPARAM, LPARAM );
void CenterDialog(HWND hdlg);


// for the spiffy about dialog box 
BOOL CALLBACK AboutProc2( HWND, UINT, WPARAM, LPARAM );

HDC hdcMemory, hdcBuffer;
HDC hdcDest, hdcSrc, hdcMask;
HWND hBscWnd;

HBITMAP hbmBack1;
HBITMAP hbmBack2;

HBITMAP hbmSrc, hbmMask;

int x, y;
int count;

HWND g_hwndMCIWnd;
HWND g_hwndMCIWnd2;

bool bShowingAVI = false;
WNDPROC wpOrigBscWndProc;
LRESULT APIENTRY BscWndSubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);


// for an even spiffier 3D about dialog box
BOOL CALLBACK AboutProc3( HWND, UINT, WPARAM, LPARAM );

LPDIRECTDRAWSURFACE7 g_pddsBscFrontBuffer;
LPDIRECTDRAWSURFACE7 g_pddsBscBackBuffer;
LPDIRECTDRAWSURFACE7 g_pddsBscZBuffer;
LPDIRECTDRAW7 g_pDDBsc;

RECT g_rcScreenRect;
RECT g_rcViewportRect;

static HRESULT WINAPI EnumZBufferFormatsCallback( DDPIXELFORMAT* pddpf,
                                                  VOID* pContext );



// for god mode dialog box
BOOL CALLBACK GodModeProc( HWND, UINT, WPARAM, LPARAM );

bool g_bGodModeOn = false;

// for console dialog box
BOOL CALLBACK ConsoleProc( HWND, UINT, WPARAM, LPARAM );

// for preferences dialog box
BOOL CALLBACK PreferencesProc( HWND, UINT, WPARAM, LPARAM );


// for flight info
COLORREF g_crTextColor = RGB(0,0,0);
int g_iAirSpeedUnit = 0;
int g_iAltitudeUnit = 0;



// VxD
#define VMYXD_APIFUNC_1 1
#define VMYXD_APIFUNC_2 2

HANDLE  g_hVxD = 0;

// use int instead of DWORD: DWORD is unsigned causing signed/unsigned probs
// or use __int32 (synonymous with int) to make it clear it is 32 bits wide
DWORD	cbBytesReturned;
int		dwErrorCode;
//DWORD   RetInfo[13];	// 1 Sync Pulse + 10 Channels + 1 IntCount + 1 PulseLength
int		RetInfo[88]; 	// big sucker to test

// TODO: calibrate
int		RetInfoMax[11];
int		RetInfoMin[11];

int		IntCount = 0;
int		ChannelTotal = 0;

CHAR*   strVxDFileName = "Vmyxd.vxd";
CHAR    lpBuffer[MAX_PATH];
CHAR    strVxDFilePath[MAX_PATH];

bool LoadVxD();
bool UnLoadVxD();
// NOTE: don't give the user the option to load/unload the VxD during app session.
// Only load at app start and unload at app exit. It may be a dynaload VxD but
// too much loading/unloading will only introduce potential instability.
// We can give the user the option to load or not to load the VxD during app start.
// (On the basis of that option, which must be a registry value, we should
// enable/disable the Control->Transmitter menu item}.


// WDM
HANDLE  g_hWDM = 0;

bool LoadWDM();
bool UnLoadWDM();

bool g_bShowDriverVersionInfo = false;

	

// smooth controls
//D3DXVECTOR3 g_vecVelocity(0.0f, -0.1f, 0.0f); // we vallen in beeld: Cool!
D3DXVECTOR3 g_vecVelocity(0.0f, 0.1f, 0.0f); 
D3DXVECTOR3 g_vecAngularVelocity(0.0f, 0.0f, 0.0f);

float g_fSpeed        = 2.0f;
float g_fAngularSpeed = 1.0f;

bool g_bFirstFrame = true;


// smooth cam controls
D3DXVECTOR3 g_vecCamVelocity(0.0f, -0.1f, 0.0f);
D3DXVECTOR3 g_vecAngularCamVelocity(0.0f, 0.0f, 0.0f);

float g_fCamSpeed        = 2.0f;
float g_fAngularCamSpeed = 1.0f;



// always handy
FLOAT GetFPS();
float g_fFPS;
float g_fFPSLimit = 60.0f;
float g_fFrameDelay = 0.0f;
bool g_bAutoFrameRate = false;
bool g_bNewFPSLimitSet = false;


// motion
float g_fMotionRate = 1.0f;
float g_fVibration  = 0.003f;
float g_fTurbulence = 10.0f;


// reset latency arrays
bool g_bResetLatency = true;

// for R/C view latency
int g_iLat = 10;
D3DMATRIX g_matOld[MAXLATENCY+3];

// for Chase view latency
int g_iLat2 = 10;
D3DMATRIX g_matOld2[MAXLATENCY+3];

// for Follow view latency
int g_iLat3 = 10;
D3DMATRIX g_matOld3[MAXLATENCY+3];



// for registry window sizing
// TODO: must make this a reg value when this becomes a user option
// DONE
// NOTE: this is a registry value!!!
// Let's make it a DWORD instead of bool to prevent C4800 warnings
// NOTE: C4800 warnings can be prevented by using BOOL (i.e. int) instead of bool
// But not here because the Framework will give linker errors (unresolved external)
// for g_bRegWindowSize if we change its type to BOOL. It's because it's declared as
// an extern BOOL in d3dapp.cpp but assigned a DWORD value later on in that file.
// So keep using DWORD's
DWORD g_bRegWindowSize   = false;
DWORD g_bRegFullScreen   = false;
DWORD g_bRegSplashScreen = true;

DWORD SPLASHSCREEN = true;

// For the Preferences Dialog Box
VOID UpdateDialogControls( HWND hDlg, D3DEnum_DeviceInfo* pCurrentDevice,
                                  DWORD dwCurrentMode );

// for registry initial fullscreen mode
DWORD g_dwRegModeWidth = 640;
DWORD g_dwRegModeHeight = 480;
DWORD g_dwRegModeRGBBitCount = 16;


// for the configuration property sheet
HINSTANCE g_hinst;
VOID DoPropertySheet(HWND hwndOwner);
int CALLBACK ConfigurationProc( HWND hwndDlg, UINT uMsg, LPARAM lParam );

// property sheets won't center by themselves so we subclass
#define WM_PROPSHEETCENTER (WM_USER+2)
WNDPROC wpOrigPropSheetProc;
LRESULT APIENTRY PropSheetSubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );


// VxD
// reg vars for the VxD
DWORD g_bRegLoadVxD = false;
//DWORD g_dwRegWelcomeVxD = 0;
//DWORD g_dwRegGoodbyeVxD = 2;

bool g_bLoadedVxD = false;

// WDM
DWORD g_bRegLoadWDM = false;

bool g_bLoadedWDM = false;
	

// DialogProc for prop pages
BOOL CALLBACK  PropPageWindowProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageVxDProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPagePositionProc ( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageAltitudeProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageControlsProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageCameraProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageZoomProc		( HWND, UINT, WPARAM, LPARAM );




#define COLOR_BLUE 	RGB(0,0,255)
#define COLOR_BLACK	RGB(0,0,0)
#define COLOR_RED	RGB(255,0,0)
#define COLOR_WHITE	RGB(255,255,255)


// need this one in Window prop page
D3DEnum_DeviceInfo*  g_pDeviceInfo;


// TODO: fix bugs (they are not mine but the framework's)
// 1. Load xfile second time in opendialog will show no textures
// 2. When mouse control in fullscreen and going back to windowed gives
//    no mouse cursor when going to keyboard or joystick control

// TODO: implement zoom a la Mortyr
// DONE


// for zoom
bool g_bZoomIn = false;
bool g_bZooming = false;
float g_fZoomValue = 0.0f;


// for landing
bool g_bLanded = true;
bool g_bInverted = false;



// all this crap to get spiffy sliders
WNDPROC wpOrigTrackBar1Proc;
WNDPROC wpOrigTrackBar2Proc;
WNDPROC wpOrigTrackBar3Proc;
WNDPROC wpOrigTrackBar4Proc;
WNDPROC wpOrigTrackBar5Proc;
WNDPROC wpOrigTrackBar6Proc;

LRESULT APIENTRY TrackBar1SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar2SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar3SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar4SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar5SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar6SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

int g_iDelta = 0;

int iMargin1 = 0;
int iMargin2 = 0;
int iMargin3 = 0;
int iMargin4 = 0;
int iMargin5 = 0;
int iMargin6 = 0;

int iSliderPos1 = 0;
int iSliderPos2 = 0;
int iSliderPos3 = 0;
int iSliderPos4 = 0;
int iSliderPos5 = 0;
int iSliderPos6 = 0;

int iOldSliderPos1 = 0;
int iOldSliderPos2 = 0;
int iOldSliderPos3 = 0;
int iOldSliderPos4 = 0;
int iOldSliderPos5 = 0;
int iOldSliderPos6 = 0;

int iPreviousSliderPos1 = 0;
int iPreviousSliderPos2 = 0;
int iPreviousSliderPos3 = 0;
int iPreviousSliderPos4 = 0;
int iPreviousSliderPos5 = 0;
int iPreviousSliderPos6 = 0;

bool bCtrlSliders123Locked = true;
bool bCtrlSliders456Locked = true;
bool bCamSliders123Locked = true;
bool bCamSliders456Locked = true;
bool bZoomSliders12Locked = true;

// controls sensitivity
float CTRL_X_SENS = 1.0f;
float CTRL_Y_SENS = 1.0f;
float CTRL_Z_SENS = 1.0f;
float CTRL_RADS_X_SENS = 1.0f;
float CTRL_RADS_Y_SENS = 1.0f;
float CTRL_RADS_Z_SENS = 1.0f;


// camera sensitivity
float CAM_X_SENS = 1.0f;
float CAM_Y_SENS = 1.0f;
float CAM_Z_SENS = 1.0f;
float CAM_RADS_X_SENS = 1.0f;
float CAM_RADS_Y_SENS = 1.0f;
float CAM_RADS_Z_SENS = 1.0f;


// zoom sensitivity
float ZOOM_IN_SENS  = 1.0f;
float ZOOM_OUT_SENS = 1.0f;



// altitude warning
bool g_bAltitudeWarning = true;
bool g_bAltitudeWarningSound = false;
bool g_bAltitudeWarningSpeaker = false;
float ALTITUDE_WARNING_Y = 5.00f;

TCHAR g_strSoundFileName[512] = "warning.wav";
TCHAR g_strSoundFilePath[512] = "";
HRESULT OpenSoundFileDialog(HWND hWnd);


// sky texture
TCHAR g_strTextureFileName[512] = "sky1.bmp";
TCHAR g_strTextureFilePath[512] = "";
HRESULT OpenTextureFileDialog(HWND hWnd);

bool IsPower2(int x);


// heli file
TCHAR g_strHeliFileName[512] = "heli.x";
TCHAR g_strHeliFilePath[512] = "";
TCHAR g_strHeliFileOld[512] = "";
TCHAR g_strHeliFileTextrName[512] = "heli.bmp";


// sky file
TCHAR g_strSkyFileName[512] = "sky1.x";
TCHAR g_strSkyFilePath[512] = "";
TCHAR g_strSkyFileOld[512] = "";
TCHAR g_strSkyFileTextrName[512] = "sky1.bmp";

HRESULT OpenSkyFileDialog(HWND hWnd);

BOOL g_bShowSkyFile = true;


// terrain file
TCHAR g_strTerrainFileName[512] = "terrain1.x";
TCHAR g_strTerrainFilePath[512] = "";
TCHAR g_strTerrainFileOld[512] = "";
TCHAR g_strTerrainFileTextrName[512] = "terrain1.bmp";

HRESULT OpenTerrainFileDialog(HWND hWnd);


// flight file
TCHAR g_strFlightFileName[512] = "flight1.fld";
TCHAR g_strFlightFilePath[512] = "";
TCHAR g_strFlightFileOld[512] = "";

HRESULT OpenFlightFileDialog(HWND hWnd);
HRESULT SaveFlightFileDialog(HWND hWnd);

// calibration file
TCHAR g_strCalibrationFileName[512] = "calibration1.cal";
TCHAR g_strCalibrationFilePath[512] = "";
TCHAR g_strCalibrationFileOld[512] = "";

HRESULT OpenCalibrationFileDialog(HWND hWnd);
HRESULT SaveCalibrationFileDialog(HWND hWnd);


// scenery file
TCHAR g_strSceneryFileName[512] = "scenery1.sce";
TCHAR g_strSceneryFilePath[512] = "";
TCHAR g_strSceneryFileOld[512] = "";

HRESULT OpenSceneryFileDialog(HWND hWnd);


// flight rec skin
bool g_bFlightRecShowSkin = true;
TCHAR g_strFlightRecSkinDir[MAX_PATH];
int CALLBACK BrowseCallbackProc( HWND hwnd, UINT uMsg, LPARAM lParam, LPARAM lpData );

// flight rec tooltips
bool g_bFlightRecShowTooltips = true;



// say cheese
bool Screenshot (LPCTSTR FileName, LPDIRECTDRAWSURFACE7 lpDDS);
bool Screenshot2(LPCTSTR FileName, HDC hDC);
bool Screenshot3(LPCTSTR FileName, HWND hWnd);

bool g_bScreenshotIncludeWindowBorder = false;


// RT Config Toolwindow
BOOL CALLBACK RTConfigurationProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM );
void CenterDownDialog(HWND hdlg);
HWND g_hWndTools = NULL;
bool bShowingTools = false;
HINSTANCE g_hInst = NULL;


DLGTEMPLATE* pTabPage[5];
HWND g_hTabControl = NULL;
HWND g_hTabCurrent = NULL;


void CreateTabControl(HWND hWnd);
void TabChange(HWND hWnd);
void TabCenter(HWND hWnd);


BOOL CALLBACK Tab1DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab2DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab3DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab4DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab5DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab6DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab7DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab8DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab9DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
BOOL CALLBACK Tab10DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);

int g_iRT1 = 0;
int g_iRT2 = 0;
int g_iRT3 = 0;


HHOOK hHook;
LRESULT CALLBACK MouseHookProc( int code, WPARAM wParam, LPARAM lParam );
bool g_bMouseDown = false;
bool g_bOverButton = false;
//bool g_bCaptured = false;


// tooltips for collapse/restore button
HWND g_hwndTT;		// handle of the ToolTip control 
HWND g_hwndDlg;		// handle of the dialog box 
HHOOK g_hhk;		// handle of the hook procedure 

// hook
LRESULT CALLBACK GetMsgProc(int nCode, WPARAM wParam, LPARAM lParam);




// reset values after new file load
bool g_bResetValues = true;


// Sorry Mr. Strouptrup (or is this OK?)
CMyD3DApplication* g_pd3dApp;




// TODO: channel mapping
// We'll read this in from the registry at start-up or after having received
// a message from Bigpush.exe that a new mapping was set.
int g_iMapPitchCh = 1;
int g_iMapRollCh  = 2;
int g_iMapNickCh  = 3;
int g_iMapYawCh   = 4;

int g_iMapThrottleCh = 0;

int g_iMapForwardCh = 7;
int g_iMapBankCh    = 0;



// TODO: channel inversion, etc.
// Let's make it a DWORD instead of bool to prevent C4800 warnings
// when reading from registry
// NOTE: DWORD is unsigned!!!
DWORD g_bChMute[11] = {
	false,
	false,false,false,false,false,
	false,false,false,false,false,
};


DWORD g_bChInv[11] = {
	false,
	false,false,false,false,false,
	false,false,false,false,false,
};

DWORD g_bChExp[11] = {
	false,
	false,false,false,false,false,
	false,false,false,false,false,
};


// ???
int g_iChSens[11] = {
	100,
	100,100,100,100,100,
	100,100,100,100,100,
};



// registry g_hWndCalibration
void RegistryRead3();
void RegistryWrite3();

// Registry value allowing R/C Sim to check whether Calibration is already running
// and then Restore/SetForeground the window or Launch the app
DWORD g_hWndCalibration = NULL;



// The one and only CLimitSingleInstance object
// Change what is passed to constructor. GUIDGEN Tool may be of help.
CLimitSingleInstance g_SingleInstanceObj( TEXT("{8CAE76C1-11BE-11d5-BB3F-444553540001}") );


// registry g_hWndSikorsky
void RegistryRead4();
void RegistryWrite4();

// Registry value allowing Calibration (or R/C Sim Sikorsky itself) to check whether Sikorsky
// is already running and then Restore/SetForeground the window or Launch the app
DWORD g_hWndSikorsky = NULL;


// light
D3DLIGHT7 g_Light1;
D3DLIGHT7 g_Light2;


// DirectSound
unsigned int g_uCountBeforeMaxVolume = 0;


// Flight Record and Playback
int g_iFrameCount = 1;
int g_iFrameTotal = 1;

HANDLE g_hFileFlightRec = NULL;
//void ResetFilePointer();
bool g_bCanEnter = true;
CRITICAL_SECTION GlobalCriticalSection;


// toolbar
bool g_bToolBar = false;
HWND g_hwndToolBar = NULL;
UINT IDC_TOOLBAR = 7776;
HWND g_hwndDockableToolwindow = NULL;
BOOL CALLBACK ToolwindowProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );

// rebar
HWND g_hwndReBar = NULL;


// status bar
bool g_bStatusBar = true;
HWND g_hwndStatus = NULL;
UINT IDC_STATUSBAR = 7777;

// progress bar on status bar
HWND g_hwndPB = NULL;
void DrawBarEx(HWND hwnd, int percent, bool horz, COLORREF crColorBar, COLORREF crColorText);
bool g_bSmoothProgressBar = false;
COLORREF g_crProgressBarColor = COLOR_DARKGRAY;


// client window
bool g_bClientWindow = true;
HWND g_hwndClient = NULL;


// status bar menu help
struct MENUPAIRS {
   	HMENU hMenuItem;
   	UINT uID;
};

MENUPAIRS g_aMenuHelpIDs[] = 
{
	{ NULL, 0 },

	// File
	{ NULL, IDM_LOADFILE },
    { NULL, IDM_TOGGLESTART },
    { NULL, IDM_SINGLESTEP },
	{ NULL, IDM_EXIT },	

	// Options
	{ NULL, ID_OPTIONS_SPECULAR },
    { NULL, ID_OPTIONS_FOG },
    { NULL, ID_OPTIONS_CULLING },
	{ NULL, ID_OPTIONS_SHADING },
    { NULL, ID_OPTIONS_TEXTURE },
	{ NULL, ID_OPTIONS_LENSFLARE },
    { NULL, ID_OPTIONS_SUN },
    { NULL, ID_OPTIONS_FLARE },
	{ NULL, ID_OPTIONS_SHADOW },
    { NULL, ID_OPTIONS_SOLID },
	{ NULL, ID_OPTIONS_WIREFRAME },
    { NULL, ID_OPTIONS_POINT },
    { NULL, ID_OPTIONS_SOUND },

	// Control
	{ NULL, IDC_KEYBOARD },
    { NULL, IDC_MOUSE },
    { NULL, IDC_JOYSTICK },
	{ NULL, IDC_TRANSMITTER },
    { NULL, IDM_CALIBRATION },
	{ NULL, IDC_ROTARYWING },
    { NULL, IDC_6DOF },
    { NULL, IDC_SMOOTHCONTROLS },
	{ NULL, IDC_SMOOTHCAMCONTROLS },
	{ NULL, IDM_VIBRATION },
    { NULL, IDM_TURBULENCE },
	{ NULL, IDM_FLIGHT_RECORDER },
	{ NULL, IDM_RECORD },
    { NULL, IDM_PLAYBACK },
    { NULL, IDM_RT_CONFIGURATION },
	{ NULL, IDM_CONFIGURATION },

	// View
	{ NULL, IDM_TOGGLEFULLSCREEN },
	{ NULL, IDM_CHANGEDEVICE },
    { NULL, IDM_ZOOM },
    { NULL, ID_SCREENSHOT },
	{ NULL, IDM_STATUSBAR },
	{ NULL, ID_PILOTPOSITION1 },
    { NULL, ID_PILOTPOSITION2 },
    { NULL, ID_PILOTPOSITION3 },
	{ NULL, ID_PILOTPOSITION4 },
    { NULL, ID_SHOWPPMARKS },
	{ NULL, ID_VIEW_RCVIEW },
    { NULL, ID_VIEW_INMODEL },
    { NULL, ID_VIEW_CHASE },
	{ NULL, ID_VIEW_FOLLOW },
    { NULL, ID_SHOWFLIGHTINFO },
	{ NULL, ID_SHOWCHANNELS },
    { NULL, ID_VIEW_CONSOLE },

	// Help
	{ NULL, IDM_HELPTOPICS },
    { NULL, IDM_ABOUT },

	{ NULL, 0 }	
};


// shareware
bool g_bShareWareCheck = true;
int g_iShareWareDaysUsed = 0;

DWORD g_dwShareWareRegisterCode = 0000000000;
DWORD g_dwShareWareUnlockCode = -1;
DWORD g_dwShareWareEncryptionKey = 7777;

TCHAR g_szShareWareUserName[512] = "";
TCHAR g_szShareWareRegistrationCode[512] = ""; // MD5 hash
//BYTE g_arrShareWareRegistrationCode[16];

TCHAR g_szShareWareKey[512] = ""; // AES/SHA-256

bool g_bShareWareRegistered = false;

wchar_t wszDecrypted[512] = L"";


// particle engine
TSnowfall* g_pSnowfall = NULL;   // Snow particle system
TSmoke* g_pSmoke = NULL;         // Smoke particle system

int g_iExhaustSmokeDensity = 0;
float g_fExhaustSmokeVolume = 0.1f;

bool g_bExhaustSmokeBlack = false;

D3DVECTOR g_vExhaustSmokeOutput = D3DVECTOR(0.0f,0.0f,0.0f);


// authentic heli sound
BOOL g_bUseAuthenticHeliSound = true;


// Initialize dialog box
HWND g_hWndInit = NULL;


// Windows 95/98/Me - NT/2K/XP
OSVERSIONINFO osvinfo;
bool g_bWindowsNT;


// Get System Information directory
CHAR lpBuffer2[MAX_PATH];
CHAR strSysInfoPath[MAX_PATH];

// Get DirectX Information directory
CHAR lpBuffer3[MAX_PATH];
CHAR strDirectXInfoPath[MAX_PATH];


// channel assignment
int assignthrottle = 0;
int assignroll	   = 0;
int assignnick     = 0;
int assignyaw      = 0;
int assignpitch    = 0;
int assigngyro     = 0;

int g_iChAssignedThrottle = 1;
int g_iChAssignedRoll     = 2;
int g_iChAssignedNick     = 3;
int g_iChAssignedYaw      = 4;
int g_iChAssignedPitch    = 6;
int g_iChAssignedGyro     = 7;

int g_iChMaxThrottle = 0;
int g_iChMaxRoll     = 0;
int g_iChMaxNick     = 0;
int g_iChMaxYaw      = 0;
int g_iChMaxPitch    = 0;
int g_iChMaxGyro     = 0;

BOOL g_bUpdateChannelAssignment = false;

void RegistryRead5();
void RegistryWrite5();


// flight and calibration
void RegistryRead7();
void RegistryWrite7();


// error checking
void ErrMsgBox();



// small rec/play toolwindow
BOOL CALLBACK FlightRecProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );

HWND g_hWndFlightRec = NULL;
bool g_bShowingFlightRec = false;
//bool g_bSkinFlightRec = true;


// boxes (heli pads)
float BOXPOSITION1_X = 15.0f;
float BOXPOSITION1_Y = -8.0;
float BOXPOSITION1_Z = 44.0f;

float BOXPOSITION2_X = -15.0f;
float BOXPOSITION2_Y = -8.0f;
float BOXPOSITION2_Z = 44.0f;

float BOXPOSITION3_X = 15.0f;
float BOXPOSITION3_Y = -8.0f;
float BOXPOSITION3_Z = 4.0f;

float BOXPOSITION4_X = -15.0f;
float BOXPOSITION4_Y = -8.0f;
float BOXPOSITION4_Z = 4.0f;

int g_iBoxSize = 2;

bool g_bShowBox1 = true;
bool g_bShowBox2 = true;
bool g_bShowBox3 = true;
bool g_bShowBox4 = true;

BOOL g_bShowBoxes = true;


// heli pad (boxes) textures
char *g_szTexNameHeliPad1 = "helipad1.bmp";
char *g_szTexNameHeliPad2 = "helipad2.bmp";
//LPDIRECTDRAWSURFACE7 g_pTexHeliPad1 = NULL;
//LPDIRECTDRAWSURFACE7 g_pTexHeliPad2 = NULL;
int g_iTexHeliPad = 1;


// FMS
float g_fScaleVal = 1.0;

// mrotor axis
D3DVECTOR g_vAxisMRotor = D3DVECTOR(0.0f,0.0f,0.0f);

// trotor axis
D3DVECTOR g_vAxisTRotor = D3DVECTOR(0.0f,0.0f,0.0f);


// used for saving skid vertices
//D3DVERTEX g_pVerticesOrigSkid[6666];

//// used for saving mesh vertices
//// Dynamically allocated vector begins with 0 elements.
//D3DVERTEXVECTOR theVectorSkid;
//D3DVERTEXVECTOR theVectorMRotor;
//D3DVERTEXVECTOR theVectorTRotor;
//
//// Iterator is used to loop through the vector.
//D3DVERTEXVECTOR::iterator theIteratorSkid;
//D3DVERTEXVECTOR::iterator theIteratorMRotor;
//D3DVERTEXVECTOR::iterator theIteratorTRotor;

// used for saving mesh vertices
D3DVERTEX* g_pVerticesOrigSkid = NULL;
D3DVERTEX* g_pVerticesOrigMRotor = NULL;
D3DVERTEX* g_pVerticesOrigTRotor = NULL;
D3DVERTEX* g_pVerticesOrigShaft = NULL;


// variation on Follow mode
bool g_bFollowTail = false;


// ClothSim
DWORD		OldTime, NewTime;
float		dt;
//BOOL		Initialized = false;
float		TotalTime = 0;

// wind
bool g_bWind = true;
float g_fWindSpeed = 2.0f;
float g_fWindDirection = 0;
float g_fWindSpeedTolerance = 0.0f;
float g_fWindDirectionTolerance = 0.0f;
float g_fWindSpeedVariation = 0.0f;
float g_fWindDirectionVariation = 0.0f;


// windsock
BOOL g_bWindsock = true;
bool g_bFlag = false;

#ifdef _DEBUG
float WINDSOCK_X = 0.0f;
float WINDSOCK_Y = -10.0f;
float WINDSOCK_Z = 50.0f;
#else
float WINDSOCK_X = 0.0f;
float WINDSOCK_Y = -10.0f;
float WINDSOCK_Z = 50.0f;
#endif //_DEBUG


// helipad
BOOL g_bHelipad = true;

#ifdef _DEBUG
float HELIPAD_X = 0.0f;
float HELIPAD_Y = -10.1f;
float HELIPAD_Z = 25.0f;
#else
float HELIPAD_X = 0.0f;
float HELIPAD_Y = -10.1f;
float HELIPAD_Z = 25.0f;
#endif //_DEBUG


// runway
BOOL g_bRunway = true;

#ifdef _DEBUG
float RUNWAY_X = -45.0f;
float RUNWAY_Y = -10.1f;
float RUNWAY_Z = -20.0f;
#else
float RUNWAY_X = -45.0f;
float RUNWAY_Y = -10.1f;
float RUNWAY_Z = -20.0f;
#endif //_DEBUG


// field
BOOL g_bField = true;

#ifdef _DEBUG
float FIELD_X = 90.0f;
float FIELD_Y = -10.1f;
float FIELD_Z = -20.0f;
#else
float FIELD_X = 90.0f;
float FIELD_Y = -10.1f;
float FIELD_Z = -20.0f;
#endif //_DEBUG


// trees
BOOL g_bTrees = true;
int g_iNumTrees = 100;

char *g_szTexNameTree = "tree1.bmp";
char *g_szTexNameTreeShadow = "shadow1.bmp";

//float TREE_X[NUM_TREES];
//float TREE_Y[NUM_TREES];
//float TREE_Z[NUM_TREES];


// for m_fSpeedFactor
float g_acc = 0.0f;
unsigned long g_count = 0;


// clean control input values for RTConfig
float g_fX2 = 0.0f;
float g_fY2 = 0.0f;
float g_fZ2 = 0.0f;
float g_fRadsX2 = 0.0f;
float g_fRadsY2 = 0.0f;
float g_fRadsZ2 = 0.0f;


// kludge for exhaust smoke
bool g_bResetExhaustSmoke = true;


// skid spring/suspension
float g_fSkidCFM = 0.5f;
float g_fSkidERP = 0.4f;
bool g_bSkidSpring = true;


// PANORAMA
void DrawSquare( LPDIRECT3DDEVICE7 pdev, LPDIRECTDRAWSURFACE7 pTex, 
					WORD x, WORD y, WORD x2, WORD y2, BYTE factor, bool clear );

IDirectDrawSurface7 *g_ptexPanorama1;
IDirectDrawSurface7 *g_ptexPanorama2;

D3DVALUE g_fCosAngle = 0.0f;
D3DVALUE g_fSinAngle = 0.0f;

D3DVALUE g_fCamPanAngle = 0.0f;
D3DVALUE g_fCamTiltAngle = 0.0f;

BOOL g_bPanorama = false;
bool g_bRecreatePanoramaBitmaps = false;

BOOL g_bShowProgressDialog = true;
bool g_bLoadScenery = false;
bool g_bCancelLoadScenery = false;


//#define NUM_PAN 24

char *g_szPanName[NUM_PAN] = {
    "pan0.bmp", "pan1.bmp", "pan2.bmp", "pan3.bmp", "pan4.bmp", "pan5.bmp", "pan6.bmp", "pan7.bmp",
	"pan8.bmp", "pan9.bmp", "pan10.bmp", "pan11.bmp", "pan12.bmp", "pan13.bmp", "pan14.bmp", "pan15.bmp",
	"pan16.bmp", "pan17.bmp", "pan18.bmp", "pan19.bmp", "pan20.bmp", "pan21.bmp", "pan22.bmp", "pan23.bmp"
};

LPDIRECTDRAWSURFACE7 g_ptexPan[NUM_PAN];

BOOL CALLBACK ProgressProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );
HWND g_hwndProgressDialog = NULL;









///////////////////////////////////////////////////////////////////////////////
// This is the flow of control:
// * WinMain() calls:
//    * CD3DApplication::Create() calls:
//       * CD3DApplication::OneTimeSceneInit()
//		 * RegisterClass() and CreateWindow() (we have m_hWnd)
//       * CD3DApplication::Initialize3DEnvironment() calls:
//          * CD3DFramework7::Initialize()
//          * CD3DApplication::InitDeviceObjects()
//    * CD3DApplication::Run() calls:
//       * CD3DApplication::Render3DEnvironment() calls:
//          * CD3DApplication::FrameMove()
//          * CD3DApplication::Render()
///////////////////////////////////////////////////////////////////////////////


//-----------------------------------------------------------------------------
// Name: WinMain()
// Desc: Entry point to the program. Initializes everything, and goes into a
//       message-processing loop. Idle time is used to render the scene.
//-----------------------------------------------------------------------------
INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
{
	
	// allow only one instance
	if ( g_SingleInstanceObj.IsAnotherInstanceRunning() ) {

		// get HWND of running instance
		RegistryRead4();

		// Restore/SetForeground the window
		if ( IsMinimized((HWND)g_hWndSikorsky) )
			ShowWindow( (HWND)g_hWndSikorsky, SW_RESTORE );
		else
			ShowWindow( (HWND)g_hWndSikorsky, SW_SHOW );
		SetForegroundWindow( (HWND)g_hWndSikorsky );


		// If user double-clicked on registered file while an instance is running:
		// send WM_COPYDATA message to copy strCmdLine to the running instance for
		// processing.
		// TEST:
		//MessageBox(NULL,strCmdLine,"QQQ!",MB_OK);
		//ProcessCommandLine( strCmdLine );
		// CRASH CRASH CRASH
		// We crash here, probably because we send data and then immediately the app
		// ends so that COPYDATASTRUCT is long destroyed before it reaches the other
		// instance? No: SendMessage() does not return until the window procedure has
		// processed the message. We crash because in HandleDragDrop() we cock up with
		// C string functions.
		// NOTE: From the docs on WM_COPYDATA:
		// The data being passed must not contain pointers or other references to objects
		// not accessible to the application receiving the data.
		COPYDATASTRUCT cds;
		cds.dwData = 0; 
		cds.cbData = strlen(strCmdLine)+1; 
		cds.lpData = strCmdLine; 
		SendMessage((HWND)g_hWndSikorsky, WM_COPYDATA, (WPARAM)NULL, (LPARAM)&cds);


		// get outta here
		return FALSE;
	}
	 


	// what OS are we running on
	osvinfo.dwOSVersionInfoSize = sizeof(OSVERSIONINFO);

    if( GetVersionEx(&osvinfo) ) {
		if (osvinfo.dwPlatformId == VER_PLATFORM_WIN32_NT) {
			g_bWindowsNT = true;
		} else {
			g_bWindowsNT = false;
		}
	} else {
		//MessageBox(NULL,"GetVersionEx() failed","QQQ",MB_OK);
	}


	// Show Initialize dialog
	g_hWndInit = CreateDialog( (HINSTANCE)hInst, MAKEINTRESOURCE(IDD_INITIALIZE), NULL,
                               (DLGPROC)NULL );
	
	
    // the one and only
	CMyD3DApplication d3dApp;


	// ugly bugger??? 
	// Need global pointer to allow friends to access members
	g_pd3dApp = &d3dApp;

	// always handy
	g_hinst = hInst;
	g_hInst = hInst;


	// process command line
	ProcessCommandLine( strCmdLine );


    // start creation
	if( FAILED( d3dApp.Create( hInst, strCmdLine ) ) )
        return 0;

	
    return d3dApp.Run();
}




//-----------------------------------------------------------------------------
// Name: CMyD3DApplication()
// Desc: Application constructor. Sets attributes for the app.
//-----------------------------------------------------------------------------
CMyD3DApplication::CMyD3DApplication()
                  :CD3DApplication()
{
    m_strWindowTitle  = TEXT( "R/C Sim Sikorsky" );
	m_bShowStats      = FALSE;

    // SHADOW VOLUME:
	m_bAppUseZBuffer  = TRUE; // FALSE; // we are creating our own stencil/z buffer    
    m_fnConfirmDevice = NULL; // check stencil buffer caps

	m_bRCView		= TRUE;
	m_bInModelView	= FALSE;
	m_bChaseView	= FALSE;
	m_bFollowView	= FALSE;

	m_uPilotPosition = 1;

	m_bRotaryWing	= TRUE;
	m_bFixedWing	= FALSE;
	m_b6DOF			= FALSE;

	m_bText		= TRUE;
	m_bFog		= FALSE;
	m_bSpec		= TRUE;
    m_bCull     = TRUE;
    m_bFlat     = FALSE;
	m_bSolid	= TRUE;
    m_bWire     = FALSE;
	m_bPoint    = FALSE;

	m_iTextureFilter = 1;

	m_bExhaustSmoke = TRUE;

	m_bSound = TRUE;
	m_bActiveSound = TRUE;

	m_bActive = FALSE;
	
	m_bShowPPMarks = FALSE;

	m_bFirstTime = TRUE;
	m_bReAcquire = FALSE;
	m_bFrameMoving = FALSE;
	m_bFrameStepping = FALSE;

	m_bShowFlightInfo = TRUE;
	m_bShowChannels = TRUE;

	m_bDrawShadow = TRUE;
	
	m_bCapsShadow = FALSE;
	m_bDrawShadowVolume = FALSE;

	m_bSplashScreenShowing = TRUE;

	m_bSmoothControls = TRUE;
	m_bSmoothCamControls = FALSE;

	m_bLeftRotating = TRUE;

	m_bVibration = FALSE;
	m_bTurbulence = FALSE;

	m_bRecord = FALSE;
	m_bPlayBack = FALSE;

	m_bEscExits = TRUE;


	m_iSunElevation = 45;
	m_iSunDirection = 315;
	m_iSunIntensity = 0xCC;
	m_fShadowAlpha = 0.8f;

	m_pFileObject    = NULL;
	m_pFileObject2	 = NULL;
    m_pTerrainObject = NULL;
	m_pSkyObject	 = NULL;
	m_pWindsockObject = NULL;
	m_pHelipadObject  = NULL;
	m_pRunwayObject	  = NULL;
	m_pFieldObject    = NULL;


	m_pPP1Object = NULL;
	m_pPP2Object = NULL;
	m_pPP3Object = NULL;
	m_pPP4Object = NULL;

	m_pSphere1Object = NULL;

	m_pFileObjectVertices     = NULL;
	m_dwNumFileObjectVertices = 0;

	m_pFileObjectIndices     = NULL;;
    m_dwNumFileObjectIndices = 0;

	m_pFileObjectVertices2     = NULL;
	m_dwNumFileObjectVertices2 = 0;

	m_pddsDepthBuffer = NULL;

	m_pddsBackBufferShadow = NULL;
	m_pddsZBufferShadow    = NULL;
	
	//m_pTexTree       = NULL; 
	//m_pTexTreeShadow = NULL;

	//m_fRadsY = INIT_RADS_Y; //g_PI; //-g_PI_DIV_2;
	//m_fRadsX = 0.0f;
	//m_fRadsZ = 0.0f;

	//m_fX = INIT_X;
	//m_fY = INIT_Y;
	//m_fZ = INIT_Z;

	m_fCamRadsY = 0.0f;
	m_fCamRadsX = 0.0f;
	m_fCamRadsZ = 0.0f;

	m_fCamX = PILOTPOSITION1_X;
	m_fCamY = PILOTPOSITION1_Y;
	m_fCamZ = PILOTPOSITION1_Z;


	m_fThrottle = 7.35f; //0.20f;
	m_fRevs     = 0.0f;
	m_fValueKey = 0.0f;
	m_fSpeed    = 0.0f;
	m_fAirSpeed = 0.0f;
	m_fSpeedDescent = 0.0f;
	m_fAltitude = 0.0f;
	m_fDistance = 0.0f;

	m_fSpeedFactor = 1.0f;
	m_bResetSpeedFactor = false;

	m_fSpeedX = 0.0f;
	m_fSpeedY = 0.0f;
	m_fSpeedZ = 0.0f;

	m_fCollective = INIT_COLLECTIVE;
	m_fWeight = INIT_WEIGHT;
	m_fTorque = INIT_TORQUE;

	m_fFogStart  = 5.0f;
	m_fFogEnd    = 700.0f;
	m_dwFogColor = 0x00ffffff; // 0xaarrggbb

	m_dwBackGroundColor = D3DRGBA(0.3f, 0.5f, 0.8f, 1.0f); // 0x000000ff;

	m_vRight   = D3DVECTOR( 1.0f, 0.0f, 0.0f );
    m_vUp      = D3DVECTOR( 0.0f, 1.0f, 0.0f );
    m_vForward = D3DVECTOR( 0.0f, 0.0f, 1.0f );


	// read reg values
	RegistryRead();

	// read filepaths and filenames from reg
	RegistryRead7();


	// NOTE: we have read-in the following values from the reg
	// Yet we are gonna set 'em to these default values at start-up as it might
	// be very confusing to start with g_bUseJoystick == TRUE and the Z-axis being
	// full up or down...
	g_bUseKeyboard = TRUE;
	g_bUseMouse    = FALSE;
	g_bUseJoystick = FALSE;
	//g_bUseTransmitter = TRUE;

	// get from registry
	m_fRadsY = INIT_RADS_Y; //g_PI; //-g_PI_DIV_2;
	m_fRadsX = 0.0f;
	m_fRadsZ = 0.0f;

	m_fX = INIT_X;
	m_fY = INIT_Y;
	m_fZ = INIT_Z;
	
}


//-----------------------------------------------------------------------------
// Name: OneTimeSceneInit()
// Desc: Called during initial app startup, this function performs all the
//       permanent initialization.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::OneTimeSceneInit()
{

	// need this in Window prop page
	g_pDeviceInfo = m_pDeviceInfo;


	// WDM
	if (g_bRegLoadWDM) {
		 if ( LoadWDM() )
			 g_bLoadedWDM = true;
		 else
			 g_bLoadedWDM = false;
	} else {
		g_bLoadedWDM = false;
	}

	// VxD
	if (g_bRegLoadVxD) {
		 if ( LoadVxD() )
			 g_bLoadedVxD = true;
		 else
			 g_bLoadedVxD = false;
	} else {
		g_bLoadedVxD = false;
	}


	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Setting matrices...");
	

	// Set matrices
    D3DUtil_SetIdentityMatrix( m_matSave );
	D3DUtil_SetIdentityMatrix( m_matSave2 );
    D3DUtil_SetIdentityMatrix( m_matRotate );
	D3DUtil_SetIdentityMatrix( m_matIdentityMatrix );


	D3DUtil_SetIdentityMatrix(m_matFileObjectMatrix);
	D3DUtil_SetIdentityMatrix(m_matFileObjectMatrix2);
	D3DUtil_SetIdentityMatrix(m_matTerrainMatrix);
	D3DUtil_SetIdentityMatrix(m_matSkyMatrix);
	D3DUtil_SetIdentityMatrix(m_matWindsockMatrix);
	D3DUtil_SetIdentityMatrix(m_matHelipadMatrix);
	D3DUtil_SetIdentityMatrix(m_matRunwayMatrix);
	D3DUtil_SetIdentityMatrix(m_matFieldMatrix);

	for (int i=0; i<NUM_TREES; i++) {
		D3DUtil_SetIdentityMatrix(m_matTreeMatrix[i]);
	}


	// Position and scale the terrain
	D3DMATRIX matTrans;
	D3DMATRIX matScale;
	D3DMATRIX matAll;
	D3DUtil_SetTranslateMatrix( matTrans, 0.0f, 0.0f, 0.0f );
    D3DUtil_SetScaleMatrix( matScale, 4.0f, 1.0f, 4.0f );
	D3DMath_MatrixMultiply(matAll, matTrans, matScale);
	D3DMath_MatrixMultiply(m_matTerrainMatrix, matAll, m_matTerrainMatrix);


	// Position the sky
    D3DUtil_SetTranslateMatrix( m_matSkyMatrix, 0.0f, -20.0f, 0.0f );

	// Scale the sky
	//D3DMATRIX matScale;
	D3DUtil_SetScaleMatrix( matScale, 60.0f, 30.0f, 60.0f );
	// Order!!!
	D3DMath_MatrixMultiply(m_matSkyMatrix, matScale, m_matSkyMatrix);

	// Rotate the sky
	D3DMATRIX matRotX, matRotY, matRotZ;
	//D3DMATRIX matAll;
	D3DUtil_SetRotateYMatrix( matRotY, g_PI_DIV_2/2 );
	D3DUtil_SetRotateZMatrix( matRotZ, g_PI_DIV_2 );
	// Order!!!
	D3DMath_MatrixMultiply(matAll,matRotZ,matRotY);
	D3DMath_MatrixMultiply(m_matSkyMatrix, matAll, m_matSkyMatrix);


	// Position the windsock
	// NOTE: y = -10.0f because terrain has matrix in x file with y = -10.0f 
	// (We're in a hover pit...)
	D3DUtil_SetTranslateMatrix( m_matWindsockMatrix, WINDSOCK_X, WINDSOCK_Y, WINDSOCK_Z );


	// Position the helipad
	// NOTE: y = -10.0f because terrain has matrix in x file with y = -10.0f 
	// (We're in a hover pit...)
	D3DUtil_SetTranslateMatrix( m_matHelipadMatrix, HELIPAD_X, HELIPAD_Y, HELIPAD_Z );


	// Position the runway
	// NOTE: y = -10.0f because terrain has matrix in x file with y = -10.0f 
	// (We're in a hover pit...)
	D3DUtil_SetTranslateMatrix( m_matRunwayMatrix, RUNWAY_X, RUNWAY_Y, RUNWAY_Z );


	// Position the field
	// NOTE: y = -10.0f because terrain has matrix in x file with y = -10.0f 
	// (We're in a hover pit...)
	D3DUtil_SetTranslateMatrix( m_matFieldMatrix, FIELD_X, FIELD_Y, RUNWAY_Z );


	// Position the trees
	// NOTE: y = -10.0f because terrain has matrix in x file with y = -10.0f 
	// (We're in a hover pit...)
//	for (i=0; i<NUM_TREES; i++) {
//		TREE_X[i] = -100.0f + i*20.0f;
//		TREE_Y[i] = -10.0f;
//		TREE_Z[i] = 60.0f;
//		D3DUtil_SetTranslateMatrix( m_matTreeMatrix[i], TREE_X[i], TREE_Y[i], TREE_Z[i] );
//	}

	for( i=0; i<NUM_TREES; i++ ) {
		//m_TreePositions[i] = D3DVECTOR( RandomPos(), 0.0f, RandomPos() );

		float dpos = (30.0f*(FLOAT)(rand()-rand()))/RAND_MAX;
		
		float xpos = RandomPos();
		float zpos = RandomPos();

		// no trees on airfield
		static int count = 1;
		count++;
		if (count%2 == 0) {
			if (xpos>=0.0f && xpos<200.0f)  xpos = 200.0f + dpos;
			if (xpos<0.0f  && xpos>-150.0f) xpos = -150.0f + dpos;
		} else {
			if (zpos>=0.0f && zpos<200.0f)  zpos = 200.0f + dpos;
			if (zpos<0.0f  && zpos>-200.0f) zpos = -200.0f + dpos;
		}

		// keep all trees in world
		if (Magnitude(D3DVECTOR(xpos,0.0f,zpos)) > 300.0f) {
			xpos *= 0.8f;
			zpos *= 0.8f;
		}

		m_TreePositions[i].x = xpos; //-90.0f + i*20.0f;
		m_TreePositions[i].y = -10.0f;
		m_TreePositions[i].z = zpos; //60.0f;

		D3DUtil_SetTranslateMatrix(m_matTreeMatrix[i], m_TreePositions[i].x, m_TreePositions[i].y, m_TreePositions[i].z);
	}
   
	

	// Position the Pilot Position Marks
	D3DUtil_SetTranslateMatrix( m_matPP1Matrix, PILOTPOSITION1_X, PILOTPOSITION1_Y, PILOTPOSITION1_Z );
	D3DUtil_SetTranslateMatrix( m_matPP2Matrix, PILOTPOSITION2_X, PILOTPOSITION2_Y, PILOTPOSITION2_Z );
	D3DUtil_SetTranslateMatrix( m_matPP3Matrix, PILOTPOSITION3_X, PILOTPOSITION3_Y, PILOTPOSITION3_Z );
	D3DUtil_SetTranslateMatrix( m_matPP4Matrix, PILOTPOSITION4_X, PILOTPOSITION4_Y, PILOTPOSITION4_Z );


    // Scale the Pilot Position Marks
	// set to default medium size
	//D3DMATRIX matScale;
	D3DUtil_SetScaleMatrix( matScale, 0.5f, 0.5f, 0.5f );
	// Order!!!
	D3DMath_MatrixMultiply(m_matPP1Matrix, matScale, m_matPP1Matrix);
	D3DMath_MatrixMultiply(m_matPP2Matrix, matScale, m_matPP2Matrix);
	D3DMath_MatrixMultiply(m_matPP3Matrix, matScale, m_matPP3Matrix);
	D3DMath_MatrixMultiply(m_matPP4Matrix, matScale, m_matPP4Matrix);




	// NOTE: got to set the current directory right
	//
	// CD3DFile::Load() searches as follows:
	// 1. D3DUtil_GetDXSDKMediaPath()
	// 2. current directory
	//
	// D3DTextr_CreateTextureFromFile() searches are done by:
	// TextureContainer::LoadImageData() as follows:
	// 1. executable's resource (so it must be possible to put all the bmp's in the .exe)
	// 2. current directory/global texture path (can be set by D3DTextr_SetTexturePath(),
	//		initially set to current directory)
	// 3. D3DUtil_GetDXSDKMediaPath()


	// Set Init dialog text
	//SendDlgItemMessage(g_hWndInit,IDC_STATIC1,WM_SETTEXT,0,(LPARAM)(LPCTSTR)"Loading X File Objects...");
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading file objects...");


	// Load X file objects ////////////////////////////////////////////////////////
	
	// kludge to make sure that when executing from DevStudio the ..\media path is found.
	// DevStudio has ugly habit to execute from project directory
#ifdef _DEBUG
	SetCurrentDirectory( "Debug" );
#else
	SetCurrentDirectory( "Release" );
#endif //_DEBUG
	
	// NOTE: ProcessCommandLine() sets the current dir to g_szRCSIMProgramPath
	// so if user has started the app via double-click on X file, the media path
	// will be set relative to that dir

	// Note: Load() will first try to load from current dir, then from IM media dir
	// Note2: the R/C Sim Sikorsky .exe is in the \bin folder
	SetCurrentDirectory( g_szRCSIMMediaPath );
	//MessageBox(NULL,g_szRCSIMMediaPath,"g_szRCSIMMediaPath",MB_OK);

	// Make g_szRCSIMMediaPath an absolute path
	GetCurrentDirectory( sizeof(g_szRCSIMMediaPath), g_szRCSIMMediaPath );
	strcat(g_szRCSIMMediaPath, "\\");
	//MessageBox(NULL,g_szRCSIMMediaPath,"g_szRCSIMMediaPath",MB_OK);

	
	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading main file object...");

	// Load X files by using the library's Load() directly

#ifdef _DEBUG
	// Load the heli /////////////////////////////////////////////////////////////
    m_pFileObject = new CD3DFile();
	if (g_bCommandLineModel) {
		SetCurrentDirectory( g_strHeliFilePath );
		if( FAILED( m_pFileObject->Load( g_strHeliFileName ) ) ) {
			//MessageBox(NULL,"qqq","QQQ",MB_OK);
			MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
			return E_FAIL;
		}

		// used in open file dialog
		// NOTE NOTE NOTE:
		// g_strHeliFilePath is expected by many functions to be the heli path+name
		GetCurrentDirectory( sizeof(g_strHeliFilePath), g_strHeliFilePath );
		strcat( g_strHeliFilePath, "\\" );
		strcat( g_strHeliFilePath, g_strHeliFileName );
		//
		//strcpy( g_strHeliFileName, g_strHeliFileName );
		//
		// save for kludge
		strcpy( g_strSkyFileOld, g_strHeliFilePath );
	} 
	else 
	{
		if( FAILED( m_pFileObject->Load( "gala1.x" ) ) ) {
			//MessageBox(NULL,"qqq","QQQ",MB_OK);
			MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
			return E_FAIL;
		}

		// used in open file dialog
		GetCurrentDirectory( sizeof(g_strHeliFilePath), g_strHeliFilePath );
		strcat( g_strHeliFilePath, "\\gala1.x" );
		//
		strcpy( g_strHeliFileName, "gala1.x" );
		//
		// save for kludge
		strcpy( g_strSkyFileOld, g_strHeliFilePath );
	}
	///////////////////////////////////////////////////////////////////////////////
#else
	// Load the heli /////////////////////////////////////////////////////////////
    m_pFileObject = new CD3DFile();
	if (g_bCommandLineModel) {
		SetCurrentDirectory( g_strHeliFilePath );
		if( FAILED( m_pFileObject->Load( g_strHeliFileName ) ) ) {
			//MessageBox(NULL,"qqq","QQQ",MB_OK);
			MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
			return E_FAIL;
		}

		// used in open file dialog
		// NOTE NOTE NOTE:
		// g_strHeliFilePath is expected by many functions to be the heli path+name
		GetCurrentDirectory( sizeof(g_strHeliFilePath), g_strHeliFilePath );
		strcat( g_strHeliFilePath, "\\" );
		strcat( g_strHeliFilePath, g_strHeliFileName );
		//
		//strcpy( g_strHeliFileName, g_strHeliFileName );
		//
		// save for kludge
		strcpy( g_strSkyFileOld, g_strHeliFilePath );
	} 
	else
	{		
		if( FAILED( m_pFileObject->Load( "gala1.x" ) ) ) {
			//MessageBox(NULL,"qqq","QQQ",MB_OK);
			MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
			return E_FAIL;
		}
		//
		// used in open file dialog
		GetCurrentDirectory( sizeof(g_strHeliFilePath), g_strHeliFilePath );
		strcat( g_strHeliFilePath, "\\gala1.x" );
		//
		strcpy( g_strHeliFileName, "gala1.x" );
		//
		// save for kludge
		strcpy( g_strSkyFileOld, g_strHeliFilePath );
		//
		//g_bBell = true;
	}
	///////////////////////////////////////////////////////////////////////////////
#endif //_DEBUG


	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Storing mesh vertices...");


	// Save original mesh vertices for mesh morphing
	SaveFileObjectOrigMeshVertices();
	///////////////



	// back to \media dir
	SetCurrentDirectory( g_szRCSIMMediaPath );
	//MessageBox(NULL,g_szRCSIMMediaPath,"QQQ",MB_OK);


	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading pilot position marks...");


	// Load the pilot position marks
    m_pPP1Object = new CD3DFile();
    if( FAILED( m_pPP1Object->Load( "pp1.x" ) ) ) {
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}

	m_pPP2Object = new CD3DFile();
    if( FAILED( m_pPP2Object->Load( "pp2.x" ) ) ) {
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}

	m_pPP3Object = new CD3DFile();
    if( FAILED( m_pPP3Object->Load( "pp3.x" ) ) ) {
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}

	m_pPP4Object = new CD3DFile();
    if( FAILED( m_pPP4Object->Load( "pp4.x" ) ) ) {
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}


	// load the sphere
	//m_pSphere1Object = new CD3DFile();
    //if( FAILED( m_pSphere1Object->Load( "sphere1.x" ) ) ) {
	//	MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
    //    return E_FAIL;
	//}

	
	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading sky...");


	// Load the sky /////////////////////////////////////////////////////////////
	SetCurrentDirectory("sky");
	//
	m_pSkyObject = new CD3DFile();
	if( FAILED( m_pSkyObject->Load( "sky1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	// used in sky prop page
	GetCurrentDirectory( sizeof(g_strSkyFilePath), g_strSkyFilePath );
	strcat( g_strSkyFilePath, "\\sky1.x" );
	//
	// save for kludge
	strcpy( g_strSkyFileOld, g_strSkyFilePath );
	//
	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////////////////

	
	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading terrain...");


	// Load the terrain ///////////////////////////////////////////////////////////
	SetCurrentDirectory("terrain");
	//
	m_pTerrainObject = new CD3DFile();
    if( FAILED( m_pTerrainObject->Load( "terrain1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	//
	// used in terrain prop page
	GetCurrentDirectory( sizeof(g_strTerrainFilePath), g_strTerrainFilePath );
	strcat( g_strTerrainFilePath, "\\terrain1.x" );
	//
	// save for kludge
	strcpy( g_strTerrainFileOld, g_strTerrainFilePath );
	//
	SetCurrentDirectory("..");
	///////////////////////////////////////////////////////////////////////////////
	// Flatten terrain
	// Let's flatten the grass a bit to make shadow OK
	// NOTE: grass.x has a FrameTransformMatrix in the file commanding it to be 
	// placed -10.0 lower.
	// So when projecting the heli shadow we must account for that
	//
	// Get the vertices, so we can tweak 'em
	// Note: Initialize pointer to avoid wild pointer!!!!!!!!!!!
    D3DVERTEX* pVertices = NULL;
    DWORD      dwNumVertices = 0;
    if( FAILED( m_pTerrainObject->GetMeshVertices( "mesh-grass", &pVertices, &dwNumVertices ) ) )
    {
		MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		return E_FAIL;
	} else {
		for( DWORD i=0; i<dwNumVertices; i++ )
		{
			// flatten them all
			//pVertices[i].y = 0.0f;

			// flat gully
			//if ( pVertices[i].x > -30.0f && pVertices[i].x < 30.0f )

			// flat rectangle
			if ( pVertices[i].x > -50.0f && pVertices[i].x < 50.0f &&
				 pVertices[i].z > -70.0f && pVertices[i].z < 70.0f ) 
			{
				pVertices[i].y = 0.0f;
			} 
			else
			{
				// more accidental round the rectangle
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
				pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);				
			}

			//pVertices[i].tu *= 3.1f;
			//pVertices[i].tv *= 3.1f;
		}
	}	
	////////////////////////////////////////////////////////////////////////////



	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading windsock...");


	// Load the windsock ////////////////////////////////////////////////////////
	// Hell: after implementing this, when double-clicking an x file we get the report
	// that the windsock file can't be found. Reason: double-clicking starts from
	// C:\Program Files\RC Sim Sikorsky and there we had not yet added the windsock
	// dir with all the files...
	SetCurrentDirectory("windsock");
	//
	m_pWindsockObject = new CD3DFile();
	if( FAILED( m_pWindsockObject->Load( "windsock1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	// used in windsock prop page
	//GetCurrentDirectory( sizeof(g_strWindsockFilePath), g_strWindsockFilePath );
	//strcat( g_strWindsockFilePath, "\\windsock1.x" );
	//
	// save for kludge
	//strcpy( g_strWindsockFileOld, g_strWindsockFilePath );
	//
	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////////////////



	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Initializing cloth simulation...");


	// TEST: ///////////////////////////////
	// ClothSim
	Initialize();
	////////////////////////////////////////



	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading helipad...");


	// Load the helipad ////////////////////////////////////////////////////////
	SetCurrentDirectory("helipad");
	//
	m_pHelipadObject = new CD3DFile();
	if( FAILED( m_pHelipadObject->Load( "helipad1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	// used in helipad prop page
	//GetCurrentDirectory( sizeof(g_strHelipadFilePath), g_strHelipadFilePath );
	//strcat( g_strHelipadFilePath, "\\helipad1.x" );
	//
	// save for kludge
	//strcpy( g_strHelipadFileOld, g_strHelipadFilePath );
	//
	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////////////////


	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading runway...");


	// Load the runway ////////////////////////////////////////////////////////
	SetCurrentDirectory("runway");
	//
	m_pRunwayObject = new CD3DFile();
	if( FAILED( m_pRunwayObject->Load( "runway1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	// used in runway prop page
	//GetCurrentDirectory( sizeof(g_strRunwayFilePath), g_strRunwayFilePath );
	//strcat( g_strRunwayFilePath, "\\runway1.x" );
	//
	// save for kludge
	//strcpy( g_strRunwayFileOld, g_strRunwayFilePath );
	//
	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////////////////


	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading field...");


	// Load the field ////////////////////////////////////////////////////////
	SetCurrentDirectory("field");
	//
	m_pFieldObject = new CD3DFile();
	if( FAILED( m_pFieldObject->Load( "field1.x" ) ) ) {
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
		MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
        return E_FAIL;
	}
	//
	// used in field prop page
	//GetCurrentDirectory( sizeof(g_strFieldFilePath), g_strFieldFilePath );
	//strcat( g_strFieldFilePath, "\\field1.x" );
	//
	// save for kludge
	//strcpy( g_strFieldFileOld, g_strFieldFilePath );
	//
	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////////////////



	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Loading trees...");


	// Load the trees ////////////////////////////////////////////////////////
//	SetCurrentDirectory("tree");
//	//
//	for (i=0; i<NUM_TREES; i++) {
//		m_pTreeObject[i] = new CD3DFile();
//		if( FAILED( m_pTreeObject[i]->Load( "tree1.x" ) ) ) {
//			//MessageBox(NULL,"qqq","QQQ",MB_OK);
//			MessageBox(NULL, "Can't find X file.", "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
//			return E_FAIL;
//		}
//	}
//	//
//	// used in tree prop page
//	//GetCurrentDirectory( sizeof(g_strTreeFilePath), g_strTreeFilePath );
//	//strcat( g_strTreeFilePath, "\\tree1.x" );
//	//
//	// save for kludge
//	//strcpy( g_strTreeFileOld, g_strTreeFilePath );
//	//
//	SetCurrentDirectory("..");
														//diffuse	//specular
	// Initialize the tree mesh							//aarrggbb  //aarrggbb
    m_TreeMesh[0] = D3DLVERTEX( D3DVECTOR(-10, 0, 0),	0xffffffff, 0xff080808, 0.0f, 1.0f );
    m_TreeMesh[1] = D3DLVERTEX( D3DVECTOR(-10, 20, 0),	0xffffffff, 0xff080808, 0.0f, 0.0f );
    m_TreeMesh[2] = D3DLVERTEX( D3DVECTOR( 10, 0, 0),	0xffffffff, 0xff080808, 1.0f, 1.0f );
    m_TreeMesh[3] = D3DLVERTEX( D3DVECTOR( 10, 20, 0),	0xffffffff, 0xff080808, 1.0f, 0.0f );

	// Create tree textures
//  D3DTextr_CreateTextureFromFile( "Tree.bmp", 0, D3DTEXTR_TRANSPARENTBLACK );
//	D3DTextr_Restore( "Tree.bmp", m_pd3dDevice );	
//	m_pTexTree = D3DTextr_GetSurface( "Tree.bmp" ); // Nono
	/////////////////////////////////////////////////////////////////////////////



	// Flight Rec skin /////////////////////////////////////////////////////////
	// used in flight rec prop page
	GetCurrentDirectory(sizeof(g_strFlightRecSkinDir), g_strFlightRecSkinDir);
	strcat(g_strFlightRecSkinDir, "\\skin\\skin1");
	////////////////////////////////////////////////////////////////////////////


	
	// Seed the random-number generator with current time so that
    // the numbers will be different every time we run.
    srand( (unsigned)time( NULL ) );



	// SHADOW VOLUME:	
	// Setup vertices for the shadow caster ///////////////////////////////////////
	// This will be all the vertices in m_pFileObject
	// And all the vertices of the heli projected under the terrain.
	// volgende is OK voor kleine files, niet voor grote
	// TODO: get pointer to all vertices in file
	// not separate pointers to all the meshes
	// get all vertices
	m_pFileObject->EnumObjects( GetNumFileObjectVerticesCB, NULL,
		(VOID*)&m_dwNumFileObjectVertices );
	
	m_pFileObjectVertices = new D3DVERTEX[m_dwNumFileObjectVertices];
	
	m_pFileObject->EnumObjects( GetFileObjectVerticesCB, NULL,
		(VOID*)m_pFileObjectVertices );
	
	// get all indices
	m_pFileObject->EnumObjects( GetNumFileObjectIndicesCB, NULL,
		(VOID*)&m_dwNumFileObjectIndices );
	
	m_pFileObjectIndices = new WORD[m_dwNumFileObjectIndices];
	
	m_pFileObject->EnumObjects( GetFileObjectIndicesCB, NULL,
		(VOID*)m_pFileObjectIndices );

	

	// TEST: we testen eerst met alleen de main rotor als shadow caster.
	// probeer het eerst met een mesh
//	if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-mrotor", &m_pFileObjectVertices,
//														&m_dwNumFileObjectVertices ) ) ) {
//		MessageBox(NULL,"Can't get mesh","QQQ",MB_OK);
//        return E_FAIL;
//	}

	// shadow: not necessary
	//m_pFileObject2->GetMeshVertices( "mesh-mrotor", &m_pFileObjectVertices2,
	//													&m_dwNumFileObjectVertices2 );

	// de manier is als volgt: laadt twee fileobjects waarvan je er een onder
	// het terrain mee laat vliegen. zo kun je een shadowvolume maken
	// de stand van het schaduw object is afhankelijk van de stand van de zon


	char msg[512];
	sprintf(msg, "Vertices#: %d", m_dwNumFileObjectVertices);
	//MessageBox(NULL,msg,"QQQ!",MB_OK);
	///////////////////////////////////////////////////////////////////////////////


/*
	///////////////////////////////////////////////////////////////////////////////
	// Voor bellrang.x
	// Here we are translating the X file mesh in order to get the origin
	// at the model's centre. This is needed for the controls.
	// Get the vertices, so we can tweak 'em
	// Note: Initialize pointer to avoid wild pointer!!!!!!!!!!!
    D3DVERTEX* pVertices = NULL;
    DWORD      dwNumVertices = 0;
    if( FAILED( m_pFileObject->GetMeshVertices( "rikmesh", &pVertices, &dwNumVertices ) ) )
    {
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	} else {
		// Plaats het model naar voren
		for( DWORD i=0; i<dwNumVertices; i++ )
		{
			pVertices[i].z += 4.5f;
			
			//pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
			//pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
			//pVertices[i].y  += (rand()/(FLOAT)RAND_MAX);
			//pVertices[i].tu *= 10;
			//pVertices[i].tv *= 10;
		}
	}	
	////////////////////////////////////////////////////////////////////////////
*/

	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Initializing dynamics engine...");

	// ODE: init
	InitDynamics();

	
	// Set Init dialog text
	SetWindowText(GetDlgItem(g_hWndInit,IDC_STATIC1), "Initialization finished.");

	// Destroy Init dialog
	DestroyWindow( g_hWndInit );


    return S_OK;
}




//-----------------------------------------------------------------------------
// Name: FrameMove()
// Desc: Called once per frame, the call is the entry point for animating
//       the scene.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::FrameMove( FLOAT fTimeKey )
{

	// frame rate control //////////////////////////////////////////////////////
	// Add a vacuous loop or Sleep() which controls animation speed (cf. CSM)
	// Add an item in the Configuration prop sheet to set speed
	// Sleep() heeft een te lage resolutie: e.g. 50fps == 20ms per frame
	// 1ms (=5%)increase is veel te veel
	// We'll use QueryPerformanceFrequency()
	// Shift: increased values
	// Control: reversed values
	if ( ( GetKeyState('8') & 0x80 && !(GetKeyState(VK_CONTROL) & 0x80) ) || g_bRTSpinnerFPSDelayUp ) {	
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFPSDelay == 1 )	{			
			g_fFrameDelay += 10;
		} else {
			g_fFrameDelay += 1;
		}
	}

	if ( ( GetKeyState('8') & 0x80 && GetKeyState(VK_CONTROL) & 0x80 ) || g_bRTSpinnerFPSDelayDown ) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFPSDelay == 1 )	{			
			g_fFrameDelay -= 10;
		} else {
			g_fFrameDelay -= 1;
		}
	}

	// reset
	if ( GetKeyState('8') & 0x80 && GetKeyState('0') & 0x80 )
		g_fFrameDelay = 0;

	if (g_fFrameDelay < 0)
		g_fFrameDelay = 0;
	
	//Sleep( g_fFrameDelay );

/*
	// automatic frame rate control
	// We should check the FPS every frame and limit to 60 FPS
	// NONO: we should *not* lock framerate. Instead, we should scale our controls
	// and object motion on the basis of the FPS we get: low FPS, bigger motion - 
	// high FPS, smaller motion. You're better off scaling motion to time in order
	// to allow for varying frame rates. Don't lock the frame rate.
	// TODO: Time-based motion control. Don't lock frame rate!
	if (g_bAutoFrameRate) {
		//float g_fFPSLimit = 60.0f; //10.0f; // test 10 FPS limit
		
		//g_fFPS = GetFPS();
		// This does not work
		//if (g_fFPS > g_fFPSLimit)
		//	g_fFrameDelay = ((1/g_fFPSLimit)-(1/g_fFPS))*10000;

		// This does not work: probably because we are not getting the right g_fFPS
		// just before or after Config dialog open. Erratic behaviour. FPS is influenced
		// by opening of dialog. 
		//if (g_bNewFPSLimitSet) {
		//	g_bNewFPSLimitSet = false;
		//	g_fFrameDelay += ((1/g_fFPSLimit)-(1/g_fFPS))*10000;
		//	MessageBeep(-1);
		//	
		//	TCHAR msg[80];
		//	sprintf(msg, "g_fFPSLimit: %f\ng_fFPS: %f\n", g_fFPSLimit, g_fFPS);
		//	MessageBox(NULL,msg,"QQQ!",MB_OK);
		//	
		//}


		
		// This works OK!!! (simple algorithm)
		// Probleem: als de user van 60 naar 20 FPS limiet
		// wil dan duurt het eeuwen voordat deze 20 FPS wordt bereikt.
		// Oplossing: wanneer nieuwe framerate is gekozen moeten we 1 frame de juiste
		// framedelay berekenen, daarna slechts kleine stapjes.
		if (g_fFPS > g_fFPSLimit)
			g_fFrameDelay+=0.1f;
		if (g_fFPS < g_fFPSLimit)
			g_fFrameDelay-=0.1f;
		
		// This works even better!!! (keeps frame rate more stable)
		// NO: at higher framerates there will also be quick addition/subtraction
		// which causes instable frame rates
		// We need another algorithm to lock the frame rate
		// Probleem: bij hogere framerates moeten we kleine stappen nemen om gelijkmatig
		// de framerate te kunnen controleren. Als de user dan echter 20 FPS als limiet
		// wil dan duurt het eeuwen voordat deze 20 FPS wordt bereikt.
		// We moeten de hoogte van de op te tellen/af te trekken waarde dus af laten
		// hangen van de huidige FPS: hoge FPS kleine stappen, lage FPS
		// grote stappen.
//		if ( (g_fFPS-g_fFPSLimit) > 10.0f || (g_fFPS-g_fFPSLimit) < -10.0f ) {		
//			if (g_fFPS > g_fFPSLimit)
//				g_fFrameDelay+=100;
//			if (g_fFPS < g_fFPSLimit)
//				g_fFrameDelay-=100;
//		} else
//		if ( (g_fFPS-g_fFPSLimit) > 1.0f || (g_fFPS-g_fFPSLimit) < -1.0f ) {		
//			if (g_fFPS > g_fFPSLimit)
//				g_fFrameDelay+=10;
//			if (g_fFPS < g_fFPSLimit)
//				g_fFrameDelay-=10;
//		} else
//		if ( (g_fFPS-g_fFPSLimit) > 0.1f || (g_fFPS-g_fFPSLimit) < -0.1f ) {		
//			if (g_fFPS > g_fFPSLimit)
//				g_fFrameDelay+=1;
//			if (g_fFPS < g_fFPSLimit)
//				g_fFrameDelay-=1;
//		} else
//		// We can only use small steps to get stable frame rate control above 60 FPS
//		if ( (g_fFPS-g_fFPSLimit) > 0.01f || (g_fFPS-g_fFPSLimit) < -0.01f ) {		
//			if (g_fFPS > g_fFPSLimit)
//				g_fFrameDelay+=0.1f;
//			if (g_fFPS < g_fFPSLimit)
//				g_fFrameDelay-=0.1f;
//		}

		if (g_fFrameDelay < 0)
			g_fFrameDelay = 0;	
	}
*/

	LARGE_INTEGER liFrequency;
	QueryPerformanceFrequency( &liFrequency ); 
 
	LARGE_INTEGER liCount1;
	LARGE_INTEGER liCount2;
	QueryPerformanceCounter( &liCount1 );
	QueryPerformanceCounter( &liCount2 );

	// liFrequency.QuadPart == number of counts per second
	// liFrequency.QuadPart			-> 1.0f g_fFrameDelay == 1s
	// liFrequency.QuadPart/1000	-> 1.0f g_fFrameDelay == 1ms
	// liFrequency.QuadPart/10000	-> 1.0f g_fFrameDelay == 0.1ms
	// liFrequency.QuadPart/100000	-> 1.0f g_fFrameDelay == 0.01ms
	// liFrequency.QuadPart/1000000 -> 1.0f g_fFrameDelay == 0.001ms == 1uS
	// etc.
	while( liCount2.QuadPart - liCount1.QuadPart < g_fFrameDelay*(liFrequency.QuadPart/10000) ) {
		QueryPerformanceCounter( &liCount2 );
	}

	// TODO: omrekenen naar percentage Frame Rate
	// NOTE: dit gaat zo niet: je wilt het percentage van de initial framerate maar
	// je neemt elke keer het percentage van de nieuwe framerate
	// Note: 1/GetFPS() == Seconds Per Frame
	// 100% = number of 0.1ms per frame
	//   ?% = number of 0.1ms per frame - number of 0.1ms sleep per frame
	// 100% = 1/GetFPS()*10000
	//   ?% = 1/GetFPS()*10000 - g_fFrameDelay
	//float fPercentage = ((1/GetFPS()*10000 - g_fFrameDelay) * 100) / (1/GetFPS()*10000);
	g_fFPS = GetFPS();	

	//sprintf( msg11, "Sleep per Frame (0.1ms): %d, %d%%", g_fFrameDelay, (int)fPercentage );
	//sprintf( msg11, "Sleep per Frame (0.1ms): %d", g_fFrameDelay );
	sprintf( msg11, "Frame Delay (ms): %.1f", g_fFrameDelay*0.1f );
	//////////////////////////////////////////////////////////////////////////



	// Set speed factor ///////////////////////////////////////////////////////
	// For frame rate independent motion control
	// This code was inspired by the GameDev.net article "Frame Rate Independent Movement"
	// by Ben Dilts. It is actually very simple. We check what FPS we currently have and 
	// based on that and a target FPS we calculate m_fSpeedFactor. 
	// m_fSpeedFactor is used in all our control and motion code like this:
	// e.g. m_fX = +0.25f*CTRL_X_SENS*m_fSpeedFactor;
	// So if, for instance, we have initially coded the right motion for a fTargetFPS of 60,
	// a system that can only do some 50 FPS will get a speedfactor of 60/50 == 1.2
	// Therefore al its motion will be sped up by a factor 1.2 resulting in the same motion
	// as if it were running on a 60 FPS system.
	// On faster systems m_fSpeedFactor will slow motions down, again resulting in
	// proper motion.
	// Also, m_fSpeedFactor compensates motion for FPS fluctuations on the same system.
	// This means you'll always get even control/motion behaviour independent of the 
	// current FPS.
	//
	// NOTE NOTE NOTE: this code is too sensitive: m_fSpeedFactor should simply be set
	// by 25/g_fFPS. Our present code is way too sensitive for Windows "burps" because
	// it determines m_fSpeedFactor over a one frame interval: one burp or pause and it 
	// is way off. GetFPS() determines FPS over a one second interval which is much more
	// stable. It could even be made more stable by letting it determine the FPS over 
	// longer intervals
	//
	// Well,well: met g_fFPS zit het ook niet snor: op snellere machines krijgen we
	// 1-seconde-getrapte versnellingen en vertragingen. Niet OK. Beter is om de
	// sensitive timing te houden maar een manier te zoeken om fluctuaties op te vangen.
	// Dus tijdens een burp moet de speed factor niet sterk veranderen. Waarschijnlijk
	// moeten we het gemiddelde nemen over een aantal frames. Of zoiets. Fixit!!!
	// Done!!!
	//
	// Using m_fSpeedFactor to get frame-rate independent dynamics (in addition to frame-rate
	// independent motion/controls) looks quite OK. The only things is that we get some jerky
	// behaviour. What's causing this?
	// It is Windows that is causing this: in full-screen exclusive mode we get a totally
	// stable m_fSpeedFactor, and thus no jerky movements. Windows seems to be doing all sorts
	// of small background tasks which cause slight variations in frame-rate, and thus a variable
	// m_fSpeedFactor, and thus jerky movement. We must find a way to dampen these variations.


	// Since we have initially coded all controls and motions for a system capable of
	// some 25 FPS, we set fTargetFPS to 25. Or a bit higher.
	float fTargetFPS = 30; //60;

	// We don't need all this since we already have code to get the current FPS
	// Well, let's use it anyway because it is more reliable this way
	static LARGE_INTEGER liCount;
	static LARGE_INTEGER liCountOld;
	static bool first = true;
	if (first) {
		first = false;
		QueryPerformanceCounter(&liCountOld);
	}

	QueryPerformanceCounter(&liCount);
	//This frame's length out of desired length
	m_fSpeedFactor = (float)(liCount.QuadPart-liCountOld.QuadPart)/((float)liFrequency.QuadPart/fTargetFPS);

	// limit
	if (g_fFPS<=0.01f) g_fFPS = fTargetFPS;

	// This is much more stable. Well...is it?
	//m_fSpeedFactor = fTargetFPS/g_fFPS;


	// TODO: after a crash we get f*cky behaviour. Fix it!
	// Also after startup. Cause: we get wrong FPS the first few frames.
	// DONE: don't use this code but the original one. 
	//m_fSpeedFactor = fTargetFPS/g_fFPS;
	// TODO: we should set m_fSpeedFactor to 1.0 after user closes a menu. When user
	// opens menu, waits, then closes menu, m_fSpeedFactor will become very big.
	// Use a global var to signal that user has opened a menu.
	// Another option would be to simply check here whether m_fSpeedFactor does not
	// exceed some sensible value.
	// TODO: in fact this should be done after every pause of the app which causes
	// m_fSpeedFactor to increase inordinately
	// NOTE: we cannot use CD3DApplication::m_bFrameMoving or CD3MyDApplication::m_bFrameMoving
	// for this since these are only set by Pause. But we have to cover
	// every pause of the app (Pause, move, resize, menu open, etc.).
	// This is gonna be a pain so let's use some sensible top limit. If m_fSpeedFactor
	// comes above that limit we can assume that it is caused by an app pause event.

	// limit speed factor
	if (m_fSpeedFactor <= 0)
		m_fSpeedFactor = 1;

	if (m_fSpeedFactor > 10)
		m_fSpeedFactor = 1;

//	if (m_bResetSpeedFactor) {
//		m_fSpeedFactor = 1;
//		m_bResetSpeedFactor = false;
//	}

	
	// dampen speed factor
	// Yyyeeesss!!! Simple but brilliant.
	// This will make m_fSpeedFactor stable: Use an accumulator and then calculate the 
	// mean m_fSpeedFactor over a number of frames.
	// NOTE: have to take overflow into account (which will make count 0)
	// Or better: calculate over a fixed number of frames and then reset, otherwise
	// m_fSpeedFactor will become too stable. Let's calculate the mean m_fSpeedFactor over
	// a 1000 frames.
	// It is not good to let m_fSpeedFactor become too stable: when the user does a mode change
	// or resizes the window, the framerate will often change drastically. m_fSpeedFactor
	// should reflect this in a fairly short amount of time. But if we let it accumulate
	// without a reset it will not update quickly. So best would be to use a global acc and count
	// and do a reset every time there is a mode change or when the user resizes the window.
	// Well, the suggested solution (reset on size and mode changes does not work because
	// during those operations the framerate will go way up due to the pause, and thus we
	// get a very low m_fSpeedFactor, and thus the heli will shoot into infinity.
	// Well, no, not when we intelligently limit m_fSpeedFactor
	// A simpler option is to reset e.g. every 1000 frames, but the problem is that during
	// that reset we often get the same jerky behaviour we tried to avoid.
	// So, we'll reset to recalculate m_fSpeedFactor upon every window-sizing and mode change.
	// And other events where a drastic frame-rate change is expected.
	// Well, it is still a good idea to automatically recalculate m_fSpeedFactor every 1000
	// frames or so (even though we get the occasional jerkiness). Otherwise we have to use
	// resets for a zillion events that affect the frame-rate (adding more trees, exhaust
	// volume increase, etc.).
	// Well, we still don't like the minor jerkiness upon every recalculation (every ...
	// frames or every ... sec) that we still get in windowed mode. So let's recalculate
	// only upon events where a frame rate change is expected.
	// Recalculate speed factor upon:
	// * WM_SIZE
	// * IDM_TOGGLEFULLSCREEN
	// * IDM_RESET_SIMULATION
	// * OpenFileDialog()
	// That's also better because when the user does something that lowers the framerate
	// he'll also see the effect of it. With an automatic reset every ... sec, he won't.
	// Then he only has to do a IDM_RESET_SIMULATION to start the simulation afresh.
	// TEST:
	//if (g_count==0) { 
	//	//MessageBox(NULL,"Recalculate speed factor.","QQQ",MB_ICONEXCLAMATION);
	//	//MessageBeep(MB_ICONEXCLAMATION);
	//}
	
	g_count++;
	if (/*g_count<=1000*/ g_count != 0) {
		g_acc+=m_fSpeedFactor;
		m_fSpeedFactor = g_acc/g_count;
	} else {
		// reset
		g_acc = 0.0f;
		g_count = 0;		
		//MessageBeep(MB_ICONEXCLAMATION);
		//MessageBeep(0xFFFFFFFF);
		//Beep(440, 1000);
	}

	// We can also reset and recalculate speed factor every ... sec instead of
	// every ... frames. Which is much better, for if the user has a fast system
	// we'll get too many resets when doing a reset every 1000 frames.
//	static FLOAT fLastTime = 0.0f;
//	FLOAT fTime = timeGetTime() * 0.001f; // Get current time in seconds
//	if( fTime - fLastTime > 8.0f ) {
//		// reset
//		g_acc = 0.0f;
//		g_count = 0;
//		fLastTime = fTime;
//		//MessageBeep(MB_ICONEXCLAMATION);
//	}
	///////////////////////////////////
	
	// save current count in old count for next frame
	liCountOld = liCount;
	///////////////////////////////////////////////////////////////////////////////////////


	// Command Line //////////////////////////////////////////////////////////////////
	if (g_bCommandLineFlight) {
		g_bCommandLineFlight = false;
		SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_FLIGHT_RECORDER,0), 0L);
	}

	if (g_bCommandLineCalibration) {
		g_bCommandLineCalibration = false;
		SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_CALIBRATION,0), 0L);
	}
	//////////////////////////////////////////////////////////////////////////////////



	// Sound //////////////////////////////////////////////////////////////////
	// Note: we are killing the sound in all cases that FrameMove() freezes.
	// Do a string search for "PlaySound( NULL, NULL, SND_PURGE );" to find
	// all these cases.
	// Oddly, the framework keeps spinning the app when the window has no focus.
	// We are implementing in line by not killing the sound in that case.
	// Or are we: it's such a pain hearing a heli sound when an other app is active
	// Let's use m_bActiveSound to detect whether the window is the active one
	// (i.e. is the top-window, has a blue caption bar).
    //if (m_bSound && m_bActiveSound && !m_bSplashScreenShowing && m_fRevs!=0.0f) {
	//	if (g_bBell || g_bCougar) {
	//		if (!m_bInModelView)
	//			PlaySound( "bell.wav", NULL, SND_FILENAME|SND_ASYNC|SND_LOOP|SND_NOSTOP );
	//		else
	//			PlaySound( "bell_i.wav", NULL, SND_FILENAME|SND_ASYNC|SND_LOOP|SND_NOSTOP );
	//	}
	//	if (g_bCobra) {
	//		if (!m_bInModelView)
	//			PlaySound( "cobra.wav", NULL, SND_FILENAME|SND_ASYNC|SND_LOOP|SND_NOSTOP );
	//		else
	//			PlaySound( "cobra_i.wav", NULL, SND_FILENAME|SND_ASYNC|SND_LOOP|SND_NOSTOP );
	//	}
	//	else
	//		PlaySound( "heli.wav", NULL, SND_FILENAME|SND_ASYNC|SND_LOOP|SND_NOSTOP );
	//} else {
	//	PlaySound( NULL, NULL, SND_PURGE );
	//	SetVolume( DSBVOLUME_MIN );
	//	SetVolumeAllBuffers( DSBVOLUME_MIN );
	//}
	///////////////////////////////////////////////////////////////////////////



	// DirectSound ////////////////////////////////////////////////////////////
	// NOTE: m_matView._41, m_matView._42, m_matView._43 does not correspond to 
	// the x,y,z position of the camera. Huh??? Well, m_fCamX, m_fCamY, m_fCamZ do.
	// At least in RC View.
	// TODO: let g_fZoomValue alter sound too.
	// NOTE: g_fZoomValue is negative but RTConfig and Ctrl+R show it as positive.
	// NOTE: position is absolute value: 100 and -100 give the same sound volume.
	// NOTE: Problem is that we can get g_fZoomValue sound alteration to work for
	// x or y or z but when we apply it to all they cancel each other out.
	// Solution: use zoomvalueX, zoomvalueY, zoomvalueZ each of which stands for
	// the amount of zoom per axis.
	// TEST:
	//	static float fzoom = 0.0f;
	//if (GetKeyState('G') & 0x80) fzoom-=10.0f;
	//if (GetKeyState('H') & 0x80) fzoom+=10.0f;

	// TEST:
	//sprintf( msg11, "m_matView._41: %.2f, m_matView._42: %.2f, m_matView._43: %.2f", m_matView._41, m_matView._42, m_matView._43 );
	//sprintf( msg11, "m_fCamX: %.2f, m_fCamY: %.2f, m_fCamZ: %.2f", m_fCamX, m_fCamY, m_fCamZ );
	//sprintf( msg11, "m_fCamRadsX: %.2f, m_fCamRadsY: %.2f, m_fCamRadsZ: %.2f", m_fCamRadsX, m_fCamRadsY, m_fCamRadsZ );


	D3DVECTOR vPosition;
    vPosition.x = (m_matFileObjectMatrix._41 - m_matView._41);
    vPosition.y = (m_matFileObjectMatrix._42 - m_matView._42);
    vPosition.z = (m_matFileObjectMatrix._43 - m_matView._43);


    D3DVECTOR vVelocity;
    vVelocity.x = (m_matFileObjectMatrix._41 - m_matView._41);
    vVelocity.y = (m_matFileObjectMatrix._42 - m_matView._42);
    vVelocity.z = (m_matFileObjectMatrix._43 - m_matView._43);



	// Set the heli sound buffer velocity and position
	//if (!m_bInModelView) {
		SetObjectProperties( &vPosition, &vVelocity );

		// Also set crash sound buffer velocity and position
		// (A distant crash should sound as a distant crash)
		SetObjectProperties2( soundCrash, &vPosition, &vVelocity );
		SetObjectProperties2( soundCrash2, &vPosition, &vVelocity );
	//}


	// Wait a bit before turning the volume up (the matrix values will be fiddled
	// with in the first few frames, so wait for them to set)
	//static unsigned int g_uCountBeforeMaxVolume = 0;
	//if (g_uCountBeforeMaxVolume++ > 3) {
	//	//g_pDSBuffer->SetVolume( DSBVOLUME_MAX );
	//	SetVolume( DSBVOLUME_MAX );
	//}

	// Set frequency
	// DSBFREQUENCY_MIN == 100
	// DSBFREQUENCY_MAX == 100000
	// DSBFREQUENCY_ORIGINAL == 0
	// normal frequency == 7.35f * 2000
	DWORD dwFrequency;

	dwFrequency = DWORD(m_fThrottle*2000) + DWORD(m_fCollective*6000);

	if (g_bUseTransmitter) dwFrequency+=8000;
	
	//dwFrequency += DWORD(m_fCollective*2000);
	sprintf( msg7, "Collective: %.4f", m_fCollective );
	
	// limit
	if (dwFrequency < DSBFREQUENCY_MIN) dwFrequency = DSBFREQUENCY_MIN;
	if (dwFrequency > DSBFREQUENCY_MAX) dwFrequency = DSBFREQUENCY_MAX;

	SetFrequency( dwFrequency );
	SetFrequency2( soundBell_i, dwFrequency );
	SetFrequency2( soundCobra_i, dwFrequency );



	// sensible sound during playback
	bool bSoundPause = false;
	if (g_bRTButtonPauseChecked || g_bFRButtonPauseChecked) {
		bSoundPause = true;

		if (m_bPlayBack) {
			if (g_bRTButtonFastForwardDown || g_bRTButtonRewindDown || g_bRTGrabFlightRecSlider ||
				g_bFRButtonFastForwardDown || g_bFRButtonRewindDown || g_bFRGrabFlightRecSlider)
				bSoundPause = false;
		}
	}


	// Set volume
	if (m_bSound && m_bActiveSound && !m_bSplashScreenShowing && m_fThrottle>0.0f &&
		!bSoundPause && g_uCountBeforeMaxVolume++ > 3)
	{
		if (g_bUseAuthenticHeliSound) {
			if (m_bInModelView) {
				// Set volume of heli internal sound to max
				if (g_bBell || g_bCougar) {
					SetVolume2( soundBell_i, -500/*DSBVOLUME_MAX*/ );
				} else if (g_bCobra) {
					SetVolume2( soundCobra_i, -500/*DSBVOLUME_MAX*/ );
				} else {
					SetVolume( DSBVOLUME_MAX );
				}				
			} else {
				SetVolume( DSBVOLUME_MAX );
			}
		} else {
			SetVolume( DSBVOLUME_MAX );
		}
	} else {
		//PlaySound( NULL, NULL, SND_PURGE );
		SetVolume( DSBVOLUME_MIN );
		SetVolumeAllBuffers( DSBVOLUME_MIN );
	}
	///////////////////////////////////////////////////////////////////////////



	// Start up full screen by faking a menu command ////////////////////////////
	// if reg read g_bRegFullScreen == 1
	// MUST DO IT HERE: CD3DApplication::m_bActive and CD3DApplication::m_bReady
	// are true at this point (See: CD3DApplication::MsgProc())
	static bool firsttime = true;
	if (firsttime) {
		firsttime = false;

		// fake it
		if (g_bRegFullScreen)
			SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_TOGGLEFULLSCREEN,0), 0L);
	}
	//////////////////////////////////////////////////////////////////////////////




	// move and resize window: hier kan het wel
	// Maar het geeft een lullig effect: je ziet hoe de window verplaatst wordt
	// Je kunt beter de initial window size op basis van een registry key bepalen
	//static bool m=1;
	//if (m) {
	//	m=0;
	//	MoveWindow(m_hWnd, 100, 100, 800, 600, TRUE);
	//}




	// do shareware notice //////////////////////
	// NOTE: mind overflow and limits: 
	// UINT_MAX, INT_MAX, INT_MIN
	//static bool firsttime3 = true;
	//if (firsttime3) {
	//	firsttime3 = false;
	static unsigned int count3 = 0;
	count3++;
	if (count3 == 3) {
		//SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_TOGGLEFULLSCREEN,0), 0L);
		Pause(true);
		if (!g_bShareWareRegistered) {
// demo
#ifdef DEMO
			DoShareWareNotice2( m_hWnd );
#else
			//DoShareWareNotice3( m_hWnd );
			DoShareWareNotice4( m_hWnd );
#endif
			//SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_REGISTER,0), 0L);
		}
		Pause(false);
	}
	

	// check time
	static DWORD dwStart = GetTickCount();
	DWORD dwCurrent = GetTickCount();
	DWORD dwTimeElapsed = (dwCurrent-dwStart)/1000;

// demo
#ifdef DEMO
	// TEST
	if (!g_bShareWareRegistered)
		sprintf( msg7, "Time Elapsed: %u sec.", dwTimeElapsed );
	// show shareware notice every 5 min. (300 sec)
//	if (dwTimeElapsed%300 == 0) {
//		if (!g_bShareWareRegistered) {
//			DoShareWareNotice2( m_hWnd );
//		}
//	}

	// time out after 5 min. (300 sec)
	if (dwTimeElapsed>300) {
		// no way if you won't pay
		//SendMessage( m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_EXIT,0), 0L );
		//SendMessage( m_hWnd, WM_CLOSE, 0, 0 );
		if (!g_bShareWareRegistered) {
			DoShareWareNotice2( m_hWnd );
			PostQuitMessage(0);
		}
	}
#endif

	// set window title
	if (count3 > 1) {
		//if (g_bShareWareRegistered) {
		//	SetWindowText( m_hWnd, "R/C Sim Sikorsky" );
		//} else {
		//	//SetWindowText( m_hWnd, "R/C Sim Sikorsky - Unregistered Demo Version" );
// demo
#ifdef DEMO
			SetWindowText( m_hWnd, "R/C Sim Sikorsky - Demo" );
#endif
		//}
	}
	/////////////////////////////////////////////




	// no frame move when splash screen is showing
	if (SPLASHSCREEN) {
		if (m_bSplashScreenShowing)
				return 0;
	}

	

	// Get all user Control inputs ////////////////////////////////////////////
	// NOTE: We should actually not get any input from whatever source when doing 
	// playback: Use if ( !m_bPlayBack ) for every input source.
	// It does not seem to be a problem though, but it could possibly cause minor
	// differences in playback when input is given. Be careful!!!
	
	// check if we have to reacquire the input device
	// this happens after WM_SIZE and IDM_TOGGLEFULLSCREEN
	if(m_bReAcquire) {
		m_bReAcquire = FALSE;
		SelectInputDevice(m_hWnd);
		PostMessage( m_hWnd, WM_SYNCACQUIRE, 0, 0 );
	}


	// WDM
	// VxD
	if ( (g_bLoadedVxD || g_bLoadedWDM) && g_bUseTransmitter )
		GetSticks();

	// RT sliders
	// Note: g_bUseRTSliders == true if one of g_bGrab1-10 == true
	if (g_bUseRTSliders)
		GetSliders();

	// RT sticks
	// Note: must be if ( g_bGrabL || g_bGrabR ) otherwise Pitch Hold won't work properly
	if ( g_bGrabL || g_bGrabR )
		GetVirtualSticks();


	// always get keys and input in FrameMove(), *NOT* in MsgProc()!!!!
	if (!g_bUseKeyboard) GetKeys();
	GetInput();
	///////////////////////////////////////////////////////////////////////////


	
	// used for ShowChannels()
	m_bFrameMoving = TRUE;



	// get height
	m_fAltitude = m_matFileObjectMatrix._42 + 8.5f + m_fWeight;

	// get distance
	D3DVECTOR vec = D3DVECTOR (	m_matFileObjectMatrix._41,
								m_matFileObjectMatrix._42,
								m_matFileObjectMatrix._43 );
	m_fDistance = Magnitude(vec);


	
	//	save for RTConfig  /////////////
//  We'll use these clean values in UpdateRTVirtualTx() and UpdateRTChannels() because
//  we don't want turbulence to be displayed on the controls.
//  NOTE: During Playback, however, we'll still get turbulence displayed on the stick movement.
//	That's because the m_fX, etc. values are recorded with turbulence (if it's on, of course)
//  TODO: In order to prevent this we should also record the clean control input values into the
//  flight data file. During Playback we should then get these clean values for our RT
//  Virtual Tx and Channels.
//  DONE
	if (!m_bPlayBack) {
		g_fX2 = m_fX/m_fSpeedFactor;
		g_fY2 = m_fY/m_fSpeedFactor;
		g_fZ2 = m_fZ/m_fSpeedFactor;
		g_fRadsX2 = m_fRadsX/m_fSpeedFactor;
		g_fRadsY2 = m_fRadsY/m_fSpeedFactor;
		g_fRadsZ2 = m_fRadsZ/m_fSpeedFactor;
	}
	///////////////////////////////////



	// RT channels: update trackbars and progressbars ///////////////////////////////
	// Got to do this in gameloop
	if ( (!g_bUseTransmitter || !g_bLoadedVxD || !g_bLoadedWDM) &&
		TabCtrl_GetCurSel(g_hTabControl) == 5 )
		UpdateRTChannels();
	//////////////////////////////////////////////////////////////////////////////



	// RT virtual tx: update tx /////////////////////////////////////////////////
	// Got to do this in gameloop
	if ( (!g_bUseTransmitter || !g_bLoadedVxD || !g_bLoadedWDM) &&
		TabCtrl_GetCurSel(g_hTabControl) == 4)
		UpdateRTVirtualTx();
	//////////////////////////////////////////////////////////////////////////////




//	// save for ODE dynamics //////////
//  No, later: let's use turbulence for ODE as well
//	g_fZ = m_fZ;
//	g_fX = m_fX;
//	g_fY = m_fY;
//	g_fRadsZ = m_fRadsZ;
//	g_fRadsX = m_fRadsX;
//	g_fRadsY = m_fRadsY;
//	///////////////////////////////////


	// Do some turbulence ///////////////////////////////////////////////////
	// NOTE: Got to do turbulence *before* DoSmoothControls()
	if (m_bTurbulence && m_fAltitude > m_fWeight+0.01f && !g_bLanded) {
		
		// limit
		if (g_fTurbulence < 1.0f)
				g_fTurbulence = 1.0f;
		if (g_fTurbulence > 11.0f)
				g_fTurbulence = 11.0f;

		if ( rand() < RAND_MAX/4 ) {
			float fTurbulence = g_fTurbulence*2.0f; //10.0f; // greater values->less turbulence
			if ( rand() > RAND_MAX/2 ) {
				m_fRadsZ += (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
				m_fRadsX += (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
				m_fRadsY += (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
			} else {
				m_fRadsZ -= (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
				m_fRadsX -= (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
				m_fRadsY -= (((float)rand()/RAND_MAX)*m_fSpeedFactor) / fTurbulence;
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////


	// Do some vibration //////////////////////////////////////////////
	// NOTE: Got to do vibration *after* DoSmoothControls()
	// Well, that don't seem necessary
	if (m_bVibration) {

		// limit
		if (g_fVibration < 0.002f)
				g_fVibration = 0.002f;
		if (g_fVibration > 0.012f)
				g_fVibration = 0.012f;

		float fVibration = g_fVibration*20.0f; //0.003f;
		static int vibcount = 0;
		vibcount++;
		if (vibcount%2 == 0) {
			m_fRadsZ += fVibration;
			m_fRadsX += fVibration;
			m_fRadsY += fVibration;
		} else {
			m_fRadsZ -= fVibration;
			m_fRadsX -= fVibration;
			m_fRadsY -= fVibration;
		}
	}
	////////////////////////////////////////////////////////////////////


	// Do some wind /////////////////////////////////////////////////
	// On the basis of g_fWindSpeed and g_fWindDirection
	// NOTE: Like turbulence and vibration we are using m_fX, etc. for this
	// but it is probably better to do this stuff in ::DoDynamics() by applying
	// using separate values. But since we save clean m_fX, etc. control values we don't
	// get any side-effects by doing it like this.
	// NONO: these are of course heli local values, we need world values
	//m_fX += cosf(D3DXToRadian(g_fWindDirection)) * g_fWindSpeed;
	/////////////////////////////////////////////////////////////////


	// save for ODE dynamics //////////
	// After turbulence because we'll let ODE do turbulence as well
//	g_fZ = m_fZ;
//	g_fX = m_fX;
//	g_fY = m_fY;
//	g_fRadsZ = m_fRadsZ;
//	g_fRadsX = m_fRadsX;
//	g_fRadsY = m_fRadsY;
	
	// save and undo the speed factoring (we already speed factor ODE's stepsize)
	// NOTE: we have speed-factored all controls but since we already speed-factor ODE
	// we undo the speed factor here. Leaving in the speed factor for the controls gives
	// us frame-rate independent motion with plain 6DOF flight dynamics.
	g_fX = m_fX/m_fSpeedFactor;
	g_fY = m_fY/m_fSpeedFactor;
	g_fZ = m_fZ/m_fSpeedFactor;
	g_fRadsX = m_fRadsX/m_fSpeedFactor;
	g_fRadsY = m_fRadsY/m_fSpeedFactor;
	g_fRadsZ = m_fRadsZ/m_fSpeedFactor;	
	///////////////////////////////////


	// kludge for skid spring/suspension
	// must get value before DoSmoothControls()
	m_fSkidSpringY = m_fY;


	// Smooth Controls /////////////////////////////////////////////////////////////
	// Note1: can't do it the first frame since we are adjusting m_fZ and m_fRadsY
	// Note2: has to be done after input polling and before setting the
	//        matrices/quaternions
	if (!g_bFirstFrame && m_bSmoothControls)
			DoSmoothControls();
	g_bFirstFrame = false;
	/////////////////////////////////////////////////////////////////////////////////


	// TODO: Smooth Cam Controls
	if (m_bSmoothCamControls)
		DoSmoothCamControls();


	
	// Values for rotor coning en tilting ///////////////////////////
	// kludge to get proper collective rotor coning
	// Note: must be done before DoDynamics() 
	//m_fRotorConeY = m_fY;
	m_fRotorConeY = m_fY + m_fCollective;
	//m_fRotorConeY = m_fY/m_fSpeedFactor + m_fCollective/m_fSpeedFactor;

	// Make m_fRotorConeY not depend on speed factor otherwise we get enormous
	// coning at slow frame rates
	// Well, we can't because we will then get very erratic values
	// But that's because we let m_fRotorConeY depend on two values that are speed factored
	// Not like this:
	//m_fRotorConeY /= m_fSpeedFactor;
	//m_fRotorConeY /= (m_fSpeedFactor*2);

	// kludge to get rid of one frame of enormous rotor coning after startup, crash
	// or reset, caused by m_fY == 8.50f
	if (g_bResetLatency) {
		m_fRotorConeY = 0.0f;
		//m_fRotorTiltX = 0.0f;
		//m_fRotorTiltZ = 0.0f;
	}

	
	// kludge to get cyclic rotor tilt for shadow
	m_fRotorTiltX = m_fRadsX;
	m_fRotorTiltZ = m_fRadsZ;
	//m_fRotorTiltX = m_fRadsX/m_fSpeedFactor;
	//m_fRotorTiltZ = m_fRadsZ/m_fSpeedFactor;

	// coning for trotor
	//m_fRotorConeX = m_fRadsY;
	m_fRotorConeX = m_fRadsY/m_fSpeedFactor;


	// limits
	if (m_fRotorConeY >  1.0f) m_fRotorConeY =  1.0f;
	if (m_fRotorConeY < -1.0f) m_fRotorConeY = -1.0f;

	if (m_fRotorTiltX >  0.2f) m_fRotorTiltX =  0.2f;
	if (m_fRotorTiltX < -0.2f) m_fRotorTiltX = -0.2f;

	if (m_fRotorTiltZ >  0.2f) m_fRotorTiltZ =  0.2f;
	if (m_fRotorTiltZ < -0.2f) m_fRotorTiltZ = -0.2f;

	if (m_fRotorConeX >  0.8f) m_fRotorConeX =  0.8f;
	if (m_fRotorConeX < -0.8f) m_fRotorConeX = -0.8f;
	/////////////////////////////////////////////////////////////////




	// motion rate control //////////////////////////////////////////////////////
	// Shift: increased values
	// Control: reversed values
	// NOTE: RTConfig Motion Rate value is now set in message loop instead
	// of game loop to get discrete values on fast systems. More values will
	// follow this change
//	if ( ( GetKeyState('9') & 0x80 && !(GetKeyState(VK_CONTROL) & 0x80) ) || g_bRTSpinnerMotionRateUp ) {	
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderMotionRate == 1 )	{			
//			g_fMotionRate += 0.10f;
//		} else {
//			g_fMotionRate += 0.01f;
//		}
//	}
//
//	if ( ( GetKeyState('9') & 0x80 && GetKeyState(VK_CONTROL) & 0x80 ) || g_bRTSpinnerMotionRateDown ) {
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderMotionRate == 1 )	{			
//			g_fMotionRate -= 0.10f;
//		} else {
//			g_fMotionRate -= 0.01f;
//		}
//	}

	// reset
	if ( GetKeyState('9') & 0x80 && GetKeyState('0') & 0x80 )
		g_fMotionRate = 1.00f;

	// limit
	if (g_fMotionRate < 0.01f)
		g_fMotionRate = 0.01f;

	if (g_fMotionRate > 2.00f)
		g_fMotionRate = 2.00f;

	// 0.00 to 2.00, 200 steps, center 1.00
	// 50% to 150%, 100 steps, center 100%
	// g_fMotionRate-1.00f to get -1.00 to 1.00
	//float fTemp = ((g_fMotionRate-1.00f)*100.0f)*(100.0f/200.0f)+100.0f;
	//sprintf( msg12, "Motion Rate (%%): %.0f, %f", g_fMotionRate*100, fTemp );
	//sprintf( msg12, "Motion Rate (%%): %.0f", fTemp );
	sprintf( msg12, "Motion Rate: %.2f", g_fMotionRate );
	//////////////////////////////////////////////////////////////////////////////////////



	// Vibration and Turbulence //////////////////////////////////////////////////////////
//	// Set in messageloop
//	// Shift: increased values
//	// Control: reversed values
//	// Vibration
//	if ( g_bRTSpinnerVibrationUp ) {	
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderVibration == 1 )	{			
//			g_fVibration += 0.001f;
//		} else {
//			g_fVibration += 0.0001f;
//		}
//	}
//
//	if ( g_bRTSpinnerVibrationDown ) {
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderVibration == 1 )	{			
//			g_fVibration -= 0.001f;
//		} else {
//			g_fVibration -= 0.0001f;
//		}
//	}
//
//	// Turbulence
//	if ( g_bRTSpinnerTurbulenceUp ) {	
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderTurbulence == 1 )	{			
//			g_fTurbulence -= 1.0f;
//		} else {
//			g_fTurbulence -= 0.1f;
//		}
//	}
//
//	if ( g_bRTSpinnerTurbulenceDown ) {
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderTurbulence == 1 )	{			
//			g_fTurbulence += 1.0f;
//		} else {
//			g_fTurbulence += 0.1f;
//		}
//	}
//	//////////////////////////////////////////////////////////////////////////////////////



		
	// Flight Characteristics ///////////////////////////////////////////////////////
	// Dynamics
	//if ( (m_bRotaryWing || m_bFixedWing) && !g_bLanded )
	// Note: if (!g_bLanded) is an important condition in connection with landing
	// and grounding the heli. If we are landed we should not do DoDynamics()
	// otherwise any values modified here will be present at take-off, resulting in sucky
	// behaviour
	//if (!g_bLanded)
	//	DoDynamics();
	/*static bool firsttime3 = true;
	if (firsttime3) {
		firsttime3 = false;
		InitDynamics();
	}*/
	//::DoDynamics();

	
	// get descent speed
	static bool firsttime2 = true;
	if (!firsttime2) {		
		static float fOldHeight = m_matFileObjectMatrix._42;
		m_fSpeedDescent = fOldHeight - m_matFileObjectMatrix._42;
		fOldHeight = m_matFileObjectMatrix._42;
	}
	firsttime2 = false;
	//} else {
		//m_fSpeedDescent = 0.0f;
	//}
	//char msg[200];
	//sprintf( msg, "m_fSpeedDescent: %.2f", m_fSpeedDescent );
	//MessageBox(NULL,msg,"QQQ",MB_OK);
	/////////////////////////////////////////////////////////////////////////////////
	

	// get god mode
	GetGodMode();


	// are we doing damage
	//if (!g_bGodModeOn)
	//	CollisionDetect(); // must do this after DoDynamics()




	// don't show RT Config toolwindow in fullscreen
	if (m_pDeviceInfo->bWindowed) {
		if (bShowingTools) ShowWindow(g_hWndTools, SW_SHOW);
	} else {
		// TEST:
		// FSWindow
		ShowWindow(g_hWndTools, SW_HIDE);
	}



	//if (g_bLanded) {
	//	m_fX = 0.0f;
	//	//m_fY = 0.0f;
	//	m_fZ = 0.0f;
	//	//m_fRadsX = 0.0f;
	//	m_fRadsY = 0.0f;
	//	//m_fRadsZ = 0.0f;
	//}

		
	




	// Particle Engine //////////////////////////////////////////////////////////
	// Set in messageloop
	// Shift: increased values
	// Control: reversed values
	// Exhaust Density
//	if ( g_bRTSpinnerExhaustDensityUp ) {		
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderSmokeDensity == 1 )	{			
//			g_iExhaustSmokeDensity += 10;
//		} else {
//			g_iExhaustSmokeDensity++;
//		}
//	}
//
//	if ( g_bRTSpinnerExhaustDensityDown ) {		
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderSmokeDensity == 1 )	{			
//			g_iExhaustSmokeDensity -= 10;
//		} else {
//			g_iExhaustSmokeDensity--;
//		}
//	}
//
//	// Exhaust Volume
//	if ( g_bRTSpinnerExhaustVolumeUp ) {		
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderSmokeVolume == 1 )	{			
//			g_fExhaustSmokeVolume += 0.1f;
//		} else {
//			g_fExhaustSmokeVolume += 0.01f;
//		}
//	}
//
//	if ( g_bRTSpinnerExhaustVolumeDown ) {		
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderSmokeVolume == 1 )	{			
//			g_fExhaustSmokeVolume -= 0.1f;
//		} else {
//			g_fExhaustSmokeVolume -= 0.01f;
//		}
//	}
//
//	// limit
//	if (g_iExhaustSmokeDensity < -20) g_iExhaustSmokeDensity = -20;
//	if (g_iExhaustSmokeDensity > 235) g_iExhaustSmokeDensity = 235;
//	if (g_fExhaustSmokeVolume < 0.0f) g_fExhaustSmokeVolume = 0.0f;
//	if (g_fExhaustSmokeVolume > 1.0f) g_fExhaustSmokeVolume = 1.0f;
	
	// update the particle systems:
	g_pSnowfall->UpdateSystem( (DWORD)fTimeKey );    
	g_pSmoke->UpdateSystem( (DWORD)fTimeKey );
	//////////////////////////////////////////////////////////////////////////////////////



	// Trees //////////////////////////////////////////////////////////////////
	if ( g_bRTSpinnerTreesNumberUp ) {
		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderTreesNumber == 1 )	{			
			g_iNumTrees+=10;
		} else {
			g_iNumTrees++;
		}
	}

	if ( g_bRTSpinnerTreesNumberDown ) {		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderTreesNumber == 1 )	{			
			g_iNumTrees-=10;
		} else {
			g_iNumTrees--;
		}
	}

	// limit
	if (g_iNumTrees<0) g_iNumTrees = 0;
	if (g_iNumTrees>NUM_TREES) g_iNumTrees = NUM_TREES;

	// reset
	if (g_bRTButtonTreesNumberReset) {
		g_bRTButtonTreesNumberReset = false;
		g_iNumTrees = 100;
	}
	///////////////////////////////////////////////////////////////////////////





	// Windsock ///////////////////////////////////////////////////////////////
	if ( g_bRTSpinnerWindSockXUp ) {
		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindSockX == 1 )	{			
			WINDSOCK_X-=0.1f * m_fSpeedFactor;
		} else {
			WINDSOCK_X-=0.01f * m_fSpeedFactor;
		}
	}

	if ( g_bRTSpinnerWindSockXDown ) {		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindSockX == 1 )	{			
			WINDSOCK_X+=0.1f * m_fSpeedFactor;
		} else {
			WINDSOCK_X+=0.01f * m_fSpeedFactor;
		}
	}


	if ( g_bRTSpinnerWindSockZUp ) {
		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindSockZ == 1 )	{			
			WINDSOCK_Z+=0.1f * m_fSpeedFactor;
		} else {
			WINDSOCK_Z+=0.01f * m_fSpeedFactor;
		}
	}

	if ( g_bRTSpinnerWindSockZDown ) {		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindSockZ == 1 )	{			
			WINDSOCK_Z-=0.1f * m_fSpeedFactor;
		} else {
			WINDSOCK_Z-=0.01f * m_fSpeedFactor;
		}
	}
	///////////////////////////////////////////////////////////////////////////



	// QQQTEST:
	// landing: put heli flat on the ground	
//	if (g_bLanded) {
//		// reset values
//		D3DUtil_SetIdentityMatrix( m_matFileObjectMatrix );
//		
//		m_fX = m_matSave._41;
//		m_fY = m_matSave._42;
//		m_fZ = m_matSave._43;
//		m_fRadsX = 0.0f;
//		m_fRadsY = 0.0f;
//		m_fRadsZ = 0.0f;
//
//		//m_matFileObjectMatrix = m_matSave;
//			
//		//m_fSpeed = 0.0f;
//		//m_fCollective = INIT_COLLECTIVE;
//		
////		m_vRight   = D3DVECTOR( 1.0f, 0.0f, 0.0f );
////		m_vUp      = D3DVECTOR( 0.0f, 1.0f, 0.0f );
////		m_vForward = D3DVECTOR( 0.0f, 0.0f, 1.0f );
//
//		//MessageBeep(-1);
//	} else {
//		m_matSave = m_matFileObjectMatrix;
//	}
//
//	if (g_bLanded && m_fCollective > 0.08f) {
//		g_bLanded = false;
//		m_fY = INIT_Y;
//	}





		



/*
	// Matrices /////////////////////////////////////////////////////////////////////
	// TODO: solve gimbal lock:
	// in every framemove do:
	// - get m_fRadsX, m_fRadsY, m_fRadsZ (note: they must be small)
	// - apply these small Euler angles to the object's last saved matrix, i.e.
	//   its last orientation
	// - save object's matrix
	// - reset m_fRadsX, m_fRadsY, m_fRadsZ to zero 
	//
	// DONE: solved gimbal lock ////////////////////////////////////////////////
	// rotate round object axes by using Axis/Angle rotation (no danger of gimbal lock)
	// 
	// TODO: solve drift after several violent spins
	// de vectoren verliezen hun alignment met het object. Solve this!!!
	// 
	// From vector.txt:
	// Rotation Axis/Angle pairs must be converted into Quaternions before finally 
	// being converted into a rotation matrix.
	// Seems like it cannot be done with matrices alone?!
	// From matrix.txt:
	// The only solution to this problem is to make use of Quaternions.
	//
	// Nonsense!!!: it works with matrices (using axis/angle and Gram-Schmidt)
	// But note: ORDER OF MULTIPLICATION IS CRUCIAL!!!!!!!!!!!!!
	// Like this:

	// translation
	D3DMATRIX matTrans;
	D3DUtil_SetTranslateMatrix( matTrans, m_fX, m_fY, m_fZ );
	D3DMath_MatrixMultiply(m_matFileObjectMatrix, matTrans, m_matFileObjectMatrix); // order is crucial!!!	
*/

	// rotation
	D3DMATRIX matRot2, matRotX, matRotY, matRotZ;
	D3DUtil_SetRotationMatrix( matRotX, m_vRight, m_fRadsX );
	D3DUtil_SetRotationMatrix( matRotY, m_vUp, m_fRadsY );
	D3DUtil_SetRotationMatrix( matRotZ, m_vForward, m_fRadsZ );

/*
	D3DMath_MatrixMultiply( matRot2, matRotY, matRotX );
	D3DMath_MatrixMultiply( matRot2, matRotZ, matRot2 ); // order is crucial!!!

	// first bring object back to world origin for rotation
	FLOAT x = m_matFileObjectMatrix._41;
	FLOAT y = m_matFileObjectMatrix._42;
	FLOAT z = m_matFileObjectMatrix._43;

	m_matFileObjectMatrix._41 = 0.0f; 
	m_matFileObjectMatrix._42 = 0.0f;
	m_matFileObjectMatrix._43 = 0.0f;

	// then rotate
	// NOTE: m_matFileObjectMatrix always represents the last orientation
	// het is essentieel dat matRot het laatste argument is!!!
	D3DMath_MatrixMultiply( m_matFileObjectMatrix, m_matFileObjectMatrix, matRot ); // order is crucial!!!

	// bring object back to its position
	m_matFileObjectMatrix._41 = x; 
	m_matFileObjectMatrix._42 = y;
	m_matFileObjectMatrix._43 = z;
*/	
	
	// update object axes
	D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotY);
	D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotY);
	D3DMath_VectorMatrixMultiply(m_vForward, m_vForward, matRotX);
	D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotX);
	D3DMath_VectorMatrixMultiply(m_vRight, m_vRight, matRotZ);
	D3DMath_VectorMatrixMultiply(m_vUp, m_vUp, matRotZ);

/*
	// reset values
	m_fX=m_fY=m_fZ=0.0f;
	m_fRadsY=m_fRadsX=m_fRadsZ=0.0f;
*/
	////////////////////////////////////////////////////////////////////////////////


	// Gram-Schmidt orthogonalization algorithm ////////////////////////////////////
	// perform base vector regeneration
	// (needn't do this every framemove)
	// TODO: we are still drifting after several rolls and pitches. Solve it!
	// DONE: it was a matrix multiplication order prob, nothing wrong here
	m_vForward = Normalize(m_vForward); // just normalize the most important vector
	m_vUp = Normalize( m_vUp - ( DotProduct(m_vUp, m_vForward)*m_vForward ) ); // the Gram-Schmidt step
	m_vRight = Normalize( CrossProduct(m_vForward, m_vUp) );

	// or alternatively:
	//m_vRight = Normalize( CrossProduct(m_vUp, m_vForward) );
	//m_vUp = Normalize( CrossProduct(m_vForward, m_vRight) );

	///////////////////////////////////////////////////////////////////////////////////




	// Quaternions //////////////////////////////////////////////////////////////////
	// Yyyyyyyyyyyyoooooooooooo!!! 
	D3DXMATRIX matTrans;
	D3DXMatrixTranslation( &matTrans, m_fX, m_fY, m_fZ );
	//D3DXMatrixMultiply(&m_matFileObjectMatrix, &matTrans, &m_matFileObjectMatrix);
	D3DMath_MatrixMultiply(m_matFileObjectMatrix, matTrans, m_matFileObjectMatrix);	// order is crucial!!!
	
	D3DXMATRIX matRot;
	D3DXQUATERNION qRot;
	D3DXQuaternionRotationYawPitchRoll( &qRot, m_fRadsY, m_fRadsX, m_fRadsZ );	
	D3DXMatrixRotationQuaternion( &matRot, &qRot );
	//D3DXMatrixMultiply(&m_matFileObjectMatrix, &matRot, &m_matFileObjectMatrix);
	D3DMath_MatrixMultiply(m_matFileObjectMatrix, matRot, m_matFileObjectMatrix); // order is crucial!!!

	
	
	// save for dynamics
	m_matSave  = matTrans;
	m_matSave2 = matRot;

	// from dynamics
	//D3DMath_MatrixMultiply(m_matFileObjectMatrix, m_matSave, m_matFileObjectMatrix);


	// We'll do this at the end of FrameMove() since we need these for main rotor
	// cyclic tilting
	// reset values
	//m_fX=m_fY=m_fZ=0.0f;
	//m_fRadsY=m_fRadsX=m_fRadsZ=0.0f;
	//
	//
	////////////////////////////////////////////////////////////////////////////////



	// NOTE: m_matFileObjectMatrix stores the 6DOF's (position and orientation) of the heli.
	// 6DOF: 6 Degrees of Freedom (3 linear + 3 angular):
	// In a left-handed coordinate system:
	// | Right.x Up.x  Forward.x 0 |
	// | Right.y Up.y  Forward.y 0 |
	// | Right.z Up.z  Forward.z 0 |
	// | Pos.x   Pos.x Pos.z     1 |
	//
	//	m_matFileObjectMatrix._11 m_matFileObjectMatrix._12 m_matFileObjectMatrix._13 m_matFileObjectMatrix._14
	//	m_matFileObjectMatrix._21 m_matFileObjectMatrix._22 m_matFileObjectMatrix._23 m_matFileObjectMatrix._24
	//	m_matFileObjectMatrix._31 m_matFileObjectMatrix._32 m_matFileObjectMatrix._33 m_matFileObjectMatrix._34
	//	m_matFileObjectMatrix._41 m_matFileObjectMatrix._42 m_matFileObjectMatrix._43 m_matFileObjectMatrix._44 


	//DoDynamics();

	
	// ODE dynamics ///////////////////////////////////////////////////////////
	// NOTE: This code makes the above original Matrix/Quaternion code more or 
	// less vacuous because all movement is now done by ODE. Nevertheless, it is
	// better to leave that code working because it is still needed to init the sim
	// before ODE has kicked in...
	//
	// This code allows us to do more than one simstep per frame.
	// Code for more than one simstep per frame
	if (m_bRotaryWing || m_bFixedWing) {
		// NOTE: not the first frame because then m_fX (and thus g_fX),
		// etc. will be set to INIT_X, etc. causing great control forces.

		// set number of simulation steps per frame
		//g_iSimStep = 2;

		static bool firsttime3 = true;
		if (!firsttime3) {			
			for (int i=0; i<g_iSimStep; i++)
				::DoDynamics();
		}
		firsttime3 = false;
	}
	//
	// But there is a much more important issue: making frame rate independent motion
	// work for ODE. This means: make sure we get the same motion at 5 FPS as at 50 FPS.
	// Frame rate independent motion works OK for control input, however, it does not work
	// for ODE. At high framerate we get many simsteps and thus different behaviour. We 
	// could do two things: 
	// 1. Limit framerate (but this is dorky)
	// 2. Make simstep depend on real time and have e.g. one simstep per 10 ms.
	//	  (i.e. separate physics loop from render loop, each running in their own thread)
	// 3. Speed factor the stepsize in dWorldStep()
	// With option 2. we divide the ODE physics cycle from the FrameMove()/Render() cycle.
	// Well, although 2. looked promissing it really is not an option: we get all sorts
	// of side effects. Maybe it is better to 3. speed factor dWorldStep() to get frame rate 
	// independent motion for ODE.
	//
	// Code for real time dependent simsteps:
//	if (m_bRotaryWing) {
//		//LARGE_INTEGER liFrequency;
//		//QueryPerformanceFrequency( &liFrequency );
//		
//		static LARGE_INTEGER liCountSimStep;
//		static LARGE_INTEGER liCountOldSimStep;
//		static bool first2 = true;
//		if (first2) {
//			first2 = false;
//			QueryPerformanceCounter(&liCountOldSimStep);
//		}
//		
//		QueryPerformanceCounter(&liCountSimStep);
//		
//		float g_fSimStep = 1.0f;
//		
//		// liFrequency.QuadPart == number of counts per second
//		// liFrequency.QuadPart			-> 1.0f g_fSimStep == 1s
//		// liFrequency.QuadPart/100		-> 1.0f g_fSimStep == 10ms
//		// liFrequency.QuadPart/1000	-> 1.0f g_fSimStep == 1ms
//		// liFrequency.QuadPart/10000	-> 1.0f g_fSimStep == 0.1ms
//		// liFrequency.QuadPart/100000	-> 1.0f g_fSimStep == 0.01ms
//		// liFrequency.QuadPart/1000000 -> 1.0f g_fSimStep == 0.001ms == 1uS
//		// etc.
//		// NOTE: We will never get here more often than the current framerate. This code
//		// only makes sure that there is a top limit to the number of sims steps. For
//		// instance, if we set the limit to 10 ms then if FPS is higher than 100 we will
//		// still get no more then 10 ms sim steps (i.e. 100 per second).
//		// If we code for a good sim at 25 FPS, we should limit the sim steps to 40 ms.
//		// That will make sure that at e.g. 60 FPS we will still get no more than 40 ms
//		// sim steps (i.e. 25 per second).
//		// 10 FPS  -> 100 ms/frame
//		// 25 FPS  -> 40  ms/frame
//		// 60 FPS  -> 16  ms/frame
//		// 100 FPS -> 10  ms/frame
//		// NOTE: We will get probs however: above 25 FPS we are not doing a simstep every
//		// frame, therefore m_matFileObjectMatrix will sometimes not be set by ::DoDynamics()
//		// and we will get fucky movement. Solution is to not let m_matFileObjectMatrix be changed
//		// when ::DoDynamics() is not called.
//		// Another solution is to always do at least one simstep per frame. To achieve that
//		// we must set the simstep low, e.g. at 10 ms. That way there will only be probs
//		// at FPS > 100. However, that's no use because this way we get no limiting
//		// function: simsteps will simply be done every framemove until FPS > 100, at
//		// which points some simsteps will be dropped. This means we still get much more
//		// simsteps at e.g. 80 FPS than at 20 FPS.
//		// The solution:
//		// YYYYyyyyyyyyyyyyyeeessss: the point is not to call ::DoDynamics() in FrameMove().
//		// It should simply be called somewhere else. Thus we will really remove all
//		// dependence on framerate. But where must we call it? From another thread?
//		// We should call it from its own loop: SimLoop() that runs independent of
//		// FrameMove().
//		// Het probleem is dus dit: je kunt de simstep wel op 10ms zetten, maar als je
//		// een lage framerate hebt (< 100 FPS) zal de simstep nooit worden gedaan als je
//		// deze in Framemove() aanroept. Daarom moet je de simstep vanuit een eigen loop
//		// callen: koppel ODE los van FrameMove().
//		// Dus:
//		// * frame rate independent motion door speed factor
//		// * frame rate independent physics door real time simsteps
//		//
//		// Check if it's time for a simstep
//		if ( liCountSimStep.QuadPart - liCountOldSimStep.QuadPart > 10*g_fSimStep*(liFrequency.QuadPart/1000) ) {
//			// NOTE: we were doing 2 simsteps per frame in our original code
//			::DoDynamics();
//			::DoDynamics();
//			
//			static int count = 0;
//			sprintf( msg8, "Sim Step: %i", count++ );
//			
//			// save current count in old count for next frame
//			liCountOldSimStep = liCountSimStep;
//		}
//	}

	
	// Thread: 
	// Code for real-time dependent simsteps in a separate thread:
	// TEST: Try to run ODE simulation in its own thread based on real time,
	// separated from FrameMove()/Render() and independent of frame rate.
	// That is: separate physics and rendering threads.
	// NOTE: Seems quite OK except we get jittering movement: probably 
	// m_matFileObjectMatrix is getting changed by ThreadFuncSimStep()->DoDynamics()
	// while other code in the FrameMove()/Render() thread is accessing it.
	// Therefore: create mutex or critical section object for m_matFileObjectMatrix
	// to synchronize the threads' access to this variable.
	// NOTE: No go. We get too many synchronization problems. Alas.
//	static bool firsttime3 = true;
//	if (firsttime3) {
//		firsttime3 = false;
//
//		DWORD dwThreadId, dwThrdParam = 1; 
//		HANDLE hThread; 
//		hThread = CreateThread( 
//			NULL,                        // no security attributes 
//			0,                           // use default stack size  
//			ThreadFuncSimStep,			 // thread function 
//			&dwThrdParam,                // argument to thread function 
//			0,                           // use default creation flags 
//			&dwThreadId);                // returns the thread identifier 
//		
//		// Check the return value for success.		
//		if (hThread == NULL) 
//			MessageBox(NULL,"CreateThread failed.","QQQ",MB_OK);
//
//		//SetThreadPriority(hThread, THREAD_PRIORITY_BELOW_NORMAL);
//	}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////
// Ways to get frame-rate independent physics:
// 1. Limit the frame rate.
// 2. Call dWorldStep() every frame with a variable stepsize.
// 3. Call dWorldStep() at certain intervals with a fixed stepsize.
//  a. dWorldStep() is called from FrameMove().
//  b. dWorldStep() is called from a separate thread.
// 
// Ad. 1: This is dorky. Users don't buy faster systems to play games that limit their frame rate.
// Ad. 2: The stepsize is based on the current frame rate. You must make sure, however, not to vary
//        the stepsize too much as this will give jerky dynamics. The best way to do this is to 
//        accumulate a speedfactor (which is based on the target frame rate/current frame rate) and
//        divide it by a frame counter (i.e. the mean speedfactor). Whenever a big frame rate change
//        is expected we can reset the accumulator and frame counter to recalculate the speed factor.
// Ad. 3a: The problem with this approach is that it only works if the frame rate is high enough and
//         we have to call dWorldStep() less often than the frame rate frequency. If, for instance,
//         we want to update the dynamics every 10 ms, everything works fine if we get frame rates
//         higher than or equal to 100 FPS. (At some frames there won't be a dynamics update, but 
//         the user won't notice that; that's the essence of this approach). However, at frame rates
//         lower than 100 FPS, the dynamics will not be called often enough and the simulation will
//         slow down. We could deal with this by checking how much time has elapsed between frames
//         and if, for instance, it is more than 20 ms we do two calls to dWorldStep().        
// Ad. 3b: We separate the physics loop (the sim loop) from the graphics loop (the render loop) by
//         calling it from a separate thread at real-time intervals. This is the most sophisticated
//         approach but difficult to implement because of the synchronization problems that have to
//         be avoided. Whenever the sim loop sets m_matFileObjectMatrix, this variable should not
//         be read from the render loop. This can be done by several synchronization methods 
//         (critical sections, mutex, etc.). However, this approach is best avoided when implementing
//         ODE as an afterthought because it has too many side effects.
///////////////////////////////////////////////////////////



	// calculate airspeed /////////////////////////////////
	static D3DVECTOR vecOld = D3DVECTOR( m_matFileObjectMatrix._41,
										 m_matFileObjectMatrix._42,
										 m_matFileObjectMatrix._43 );

	D3DVECTOR vecNew = D3DVECTOR( m_matFileObjectMatrix._41,
								  m_matFileObjectMatrix._42,
								  m_matFileObjectMatrix._43 );

	m_fAirSpeed = Magnitude(vecNew-vecOld);

	vecOld = vecNew;
	///////////////////////////////////////////////////////




	// ClothSim /////////////////////////////////////////////////////
	//static float f = WINDFACTOR;
	// TEST:
	//VK_OEM_MINUS
	//if( IsKeyDown(VkKeyScan('-')) ) {
//	if ( GetKeyState('O') & 0x80 ) {
//		//MessageBeep(MB_ICONEXCLAMATION);
//		//MessageBeep(-1);
//		//MessageBox(NULL,"qqq","QQQ",MB_OK);
//		f-=10.0f;
//		// limit
//		if (f<0.0f) f=0.0f; 
//		SetWindForceFactor(f);
//	}
//	//VK_OEM_PLUS
//	//if( IsKeyDown(VkKeyScan('=')) ) {
//	if ( GetKeyState('P') & 0x80 ) {
//		//MessageBeep(-1);
//		f+=10.0f;
//		SetWindForceFactor(f);
//	}
//	
	// wind direction
	// Set in messageloop
	if ( g_bRTSpinnerWindDirectionUp ) {		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindDirection == 1 )	{			
			g_fWindDirection+=10 * m_fSpeedFactor;
		} else {
			g_fWindDirection+=1 * m_fSpeedFactor;
		}
	}

	if ( g_bRTSpinnerWindDirectionDown ) {		
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindDirection == 1 )	{			
			g_fWindDirection-=10 * m_fSpeedFactor;
		} else {
			g_fWindDirection-=1 * m_fSpeedFactor;
		}
	}

	// reset
	if (g_bRTButtonWindDirectionReset) {
		g_bRTButtonWindDirectionReset = false;
		g_fWindDirection = 0;
	}

//	// Rotate windsock
//	// NOTE: must do this after ::DoDynamics()
//	// Well, let's just do this in ::DoDynamics()
//	D3DMATRIX matRotY2;
//	D3DUtil_SetRotateYMatrix( matRotY2, D3DXToRadian(-g_fWindDirection) );
//	D3DMath_MatrixMultiply( g_pd3dApp->m_matWindsockMatrix, matRotY2, g_pd3dApp->m_matWindsockMatrix ); // order is crucial!!!


	// wind force (Beaufort): 0-16 
	// wind speed (m/s): 0.0-51.0 (0-100 knots) (1 knot == 0.51 m/s)
	// Set in messageloop
//	if ( g_bRTSpinnerWindForceUp ) {	
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindForce == 1 )	{			
//			g_fWindSpeed+=1.0f;
//		} else {
//			g_fWindSpeed+=0.1f;
//		}
//	}
//
//	if ( g_bRTSpinnerWindForceDown ) {		
//		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderWindForce == 1 )	{			
//			g_fWindSpeed-=1.0f;
//		} else {
//			g_fWindSpeed-=0.1f;
//		}
//	}

	// limit
	if (g_fWindSpeed<0.0f) g_fWindSpeed = 0.0f;
	if (g_fWindSpeed>51.0f) g_fWindSpeed = 51.0f;

	// reset
	if (g_bRTButtonWindForceReset) {
		g_bRTButtonWindForceReset = false;
		g_fWindSpeed = 2.0f;
	}

	
	// randomize wind speed ///////////////////////////////
	static float var = 0.0f;
	static bool up = true;
	static float acc = 0.02f;
	if (up) {
		var += acc;
		//acc+=0.00005f;
		if (var > g_fWindSpeedTolerance) {
			up = false;
			//acc = 0.0f;
		}
	} else {
		var -= acc;
		//acc+=0.00005f;
		if (var < -g_fWindSpeedTolerance) {
			up = true;
			//acc = 0.0f;
		}
	}
	if (dRandReal() > 0.997) {
		up = !up;
		//acc = 0.0f;
	}

	// reset
	if (g_bRTButtonWindSpeedToleranceReset) {
		g_bRTButtonWindSpeedToleranceReset = false;
		//acc = 0.0f;
		var = 0.0f;
	}

	g_fWindSpeedVariation = var;
	///////////////////////////////////////////////////////
	
	
	if (g_bWind) {
		if ( (g_fWindSpeed+g_fWindSpeedVariation)*50 >= 0.0f ) // check
			SetWindForceFactor((g_fWindSpeed+g_fWindSpeedVariation)*50);
	} else {
		SetWindForceFactor(0.0f);
	}

	NewTime = timeGetTime();	
	dt = (float) (NewTime - OldTime)/1000;
	OldTime = NewTime;	

	
	//dt = 0.016f;  // fixed time step	
	dt = 0.032f * m_fSpeedFactor;

	// limit
	if (dt > 0.016f) dt = 0.016f;
	if (dt < 0.001f) dt = 0.001f;
	
	TotalTime += dt;

	StepSimulation(dt);
	/////////////////////////////////////////////////////////////////



	// Wind realtime keys ///////////////////////////////////////////
	if ( GetKeyState(VK_CONTROL) & 0x80 ) {
		if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			if ( GetKeyState(VK_OEM_PERIOD) & 0x80 ) g_fWindSpeed-=1.0f * m_fSpeedFactor;
			if ( GetKeyState(VK_OEM_COMMA) & 0x80 ) g_fWindDirection-=10 * m_fSpeedFactor;
		} else {
			if ( GetKeyState(VK_OEM_PERIOD) & 0x80 ) g_fWindSpeed-=0.1f * m_fSpeedFactor;
			if ( GetKeyState(VK_OEM_COMMA) & 0x80 ) g_fWindDirection-=1 * m_fSpeedFactor;
		}
	} else {
		if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			if ( GetKeyState(VK_OEM_PERIOD) & 0x80 ) g_fWindSpeed+=1.0f * m_fSpeedFactor;
			if ( GetKeyState(VK_OEM_COMMA) & 0x80 ) g_fWindDirection+=10 * m_fSpeedFactor;
		} else {
			if ( GetKeyState(VK_OEM_PERIOD) & 0x80 ) g_fWindSpeed+=0.1f * m_fSpeedFactor;
			if ( GetKeyState(VK_OEM_COMMA) & 0x80 ) g_fWindDirection+=1 * m_fSpeedFactor;
		}
	}

	// limit
	if (g_fWindSpeed<0.0f) g_fWindSpeed = 0.0f;
	if (g_fWindSpeed>51.0f) g_fWindSpeed = 51.0f;

	// reset
	if ( GetKeyState('0') & 0x80 ) {
		if ( GetKeyState(VK_OEM_PERIOD) & 0x80 ) g_fWindSpeed = 2.0f;
		if ( GetKeyState(VK_OEM_COMMA) & 0x80 ) g_fWindDirection = 0.0f;
	}
	/////////////////////////////////////////////////////////////////


	// FMS
	// Scaling for FMS models /////////////////////////////////////////////////
	// Scale FMS x files up because they are too small
	// NOTE: we have done the following in d3dfile.h
	// #define MAX_MATERIAL			128 //16
	// #define MAX_TEXTURE_NAME		512 //80
	// This is to make sure the x file loads OK because many x files made for FMS
	// (using Metasequoia) have more than 16 Materials
	// TODO: 
	// * FMS x files also are too high: we must lower all vertices or use a FrameTransformMatrix
	// in the file. The last is not really an option because we want to be able to use 
	// the files without changing them.
	// NONO: file vertices are OK. We should just change the proxy dy positions in
	// the .DAT file
	// * FMS x files use one mesh. This causes wrong transparency rendering because
	// the XFile code only draws transparent meshes last, not faces. We must make the code
	// draw all transparent faces last.
	// WELL: not exactly. See below.
	// * Also, one mesh means we cannot rotate rotors
	// * RM Viewer code is much faster. We must optimize the IM XFile code.
	// * Also shadow does not work as mesh is unnamed in FMS x file
	// DONE: well, we got shadows working
	//
	// Why is RM x file loader faster/better:
	// It optimizes the x file: it sorts vertices with the same material and then
	// renders them. Thus we have minimal Material/Texture resets on the device.
	// Material/Texture resets cause the greatest performance hit, e.g. load
	// Shuttle RG.x with the materials removed from the x file: FPS more than
	// doubles.
	// NONO: it is not the Material/Texture resets which makes XFile slow, it is
	// the fact that for every material it lets DrawIndexedPrimitive() process
	// all vertices in the mesh (with indices indicating which vertices have that
	// material and should be drawn, but that still means all vertices are processed
	// for every material).
	//
	// ...DrawIndexedPrimitive method, which as I've read from the
	// docs, DOES transform ALL the vertices each time that it is called. So if you
	// have a xfile with several materials you are done! All the vertices are
	// processed time and again!!!.
	//
	// That is correct. It will process all the vertices that you pass to the
	// DrawIndexedPrimitive function. You have 2 options to improve this: 
	// 1) Sort the mesh data by material so that only vertices which uses the
	// current material are passed to the function. This would mean creating
	// more mesh-objects - one for each material. 
	// 2) Use a vertex buffer and process the vertices once, then render the
	// primitives using the DrawIndexedPrimitiveVB function.
	//
	// From the docs on DrawIndexedPrimitive():
	// Do not use this method to render very small subsets of vertices from extremely
	// large vertex arrays. This method transforms every vertex in the provided buffer,
	// regardless of the location or number of vertices being rendered. Thus, if you pass
	// an array that contains thousands of vertices, but only intend to render hundreds,
	// your application's performance suffers dramatically. If you need to render a small
	// number of vertices from a large buffer, use the Direct3D vertex buffer rendering
	// methods. For more information, see Vertex Buffers.
	//
	// Mmm, testing in d3dfile.cpp reveals that it uses the following method:
	// Per frame it sets every material only once and then passes *all* vertices to
	// DrawIndexedPrimitive() with indices to vertices that use that material.
	//
	// As to transparency it could be that RM Viewer draws the faces with more 
	// transpareny the latest (and in order) whereas XFile only makes a distinction between
	// transparency and no transparency.
	//
	// XFile heeft een behoorlijk crappy x file parser: slow, transparency wrong,
	// texture mapping wrong. Als alternatief hebben we:
	// * Rewrite XFile x parser (complicated) 
	// * Use DirectX 8 D3DX functions to parse x files and optimize meshes
	// * Use RM x file loader
	//
	// For a file with e.g. 1 mesh and 4 materials in that mesh, *all*
	// vertices will be processed (i.e. transformed by the world matrix) 4 times per
	// frame. In every material slot, indices indicate which vertices use that material
	// and are actually rendered. This is very unoptimal. 
	// XFile also does not take care of drawing material with more transparency later
	// than material with lower transparency...
	//
	// Q. Why are my Indexed Primitive calls so slow?
	//
	// Note: The following information is based on D3DIM 7.  Under DirectX 8, the 
	//	  DrawIndexedPrimitive command allows you to specify a starting vertex and 
	//	  number of vertices used.  By restricting this to the range used in a rendering 
	//	  call, only those vertices get processed.
	//
	// In most postings I see of this nature, there is a common thread : A) There are a 
	// large number of vertexes overall, but B) each call to DrawIndexedPrimitive() is only 
	// drawing a few polygons.
	//
	// The reason that these operations appear to have huge performance penalties is 
	// because each call to DrawIndexedPrimitive() will transform all of the vertices in 
	// the list, regardless of which are actually utilized.  For example, take a moment to 
	// consider the following code :
	//
	// for (int i=0;i<1000;i+=4)
	//        lpDevice->DrawIndexedPrimitive(D3DPT_TRIANGLESTRIP,D3DFVF_VERTEX,
	//                                       verts,1600,index+i,4,0)
	//
	// That code will render a total of  500 polygons, which is quite reasonable... However, 
	// in doing so, it will transform all 1600 vertexes in the "verts" array 250 times - that 
	// means 400,000 transformations per frame!  To be able to sustain 30 FPS at that kind of 
	// load would mean the CPU has to transform 12 Million transformations each second.
	//
	// To avoid this, only pass the vertexes you will be using, and batch your primitives 
	// together if possible - approximately 100 triangles per call is optimal. The benefit 
	// of batching is significant, and is worth breaking your object into a single triangle 
	// list if it consists of many small triangle strips.
	//
	// Well, well, well, if Moses won't come to the mountain... Eigenlijk is het XFile
	// algoritme nog niet zo slecht - als je tenminste rekening houdt met hoe je je x file
	// opbouwt. Alles in 1 mesh met veel materials is slecht. Zoveel mogelijk objecten
	// opsplitsen in verschillende meshes met liefst 1 material per mesh is optimaal.
	// Om allerlei mooie effecten te krijgen:
	// * rotating rotors (also rotating in shadow)
	// * main rotor coning and tilting, tail rotor coning
	// * fast and slow rotors
	// * skid springs
	// * collision/crash damage
	// is het toch nodig om de objecten in een eigen mesh te zetten.
	// Dus als onze x parser de x file niet goed leest, moeten we de x file maar aanpassen...
	//
	// Hints:
	// * Get the Metasequoia .mqo file
	// * Rearrange objects: at least fuselage, mrotor, trotor, and skid in seperate mesh
	// * Keep transparent parts in one object
	// * Try to use one material per object
	// * Save these objects to x files
	// * For full optimization delete materials which an object does not use before saving 
	// * Objects which do not use textures should not save UV coordinates
	// * Construct final x file in UltraEdit
	// * Rearrange Meshes in the .X file if you cannot see one transparent object through 
	//   another transparent object: order determines what can be seen through what
	// * Material Emmision for transparent object must be one or they will blink
	// * Put mrotor last in x file
	// * Rotors: specular and power: 0.0
	//
	// The renderer renders frames separately, then in each frame it renders meshes separately.
	// Per mesh, material slots are made for each material in a mesh.
	// The renderer renders opaque and transparent meshes in separate passes. Transparent
	// meshes are drawn last. In both passes, meshes are rendered in the order they have
	// in the x file.
	//
	// NOTE: X file Material template:
	// RGB (diffuse+ambient) A (transparency)
	// Power (force of specular highlight)
	// Specular (specular hightlight color)
	// Emissive (emissive color)
	//
	// TEST:
	if (m_b6DOF) {
		D3DMATRIX matScale2;
		float fScaleVal = 1.0f;
		if ( GetKeyState(VkKeyScan('=')) & 0x80 ) fScaleVal += 0.01f;
		if ( GetKeyState(VkKeyScan('-')) & 0x80 ) fScaleVal -= 0.01f;

		D3DUtil_SetScaleMatrix( matScale2, fScaleVal, fScaleVal, fScaleVal );
		D3DMath_MatrixMultiply(m_matFileObjectMatrix, matScale2, m_matFileObjectMatrix);
	}

	if (m_bRotaryWing || m_bFixedWing) {
		D3DMATRIX matScale2;
		//static float fScaleVal = 1.0f;
		// VK_OEM_PLUS/VK_OEM_MINUS
		if ( GetKeyState(VkKeyScan('=')) & 0x80 ) g_fScaleVal += 0.1f;
		if ( GetKeyState(VkKeyScan('-')) & 0x80 ) g_fScaleVal -= 0.1f;

		// NOTE: We get probs here now we are doing real time dependent simsteps:
		// Above 25 FPS we are not doing a simstep every frame, therefore m_matFileObjectMatrix
		// will sometimes not be set by ::DoDynamics() and we will get fucky scaling.
		D3DUtil_SetScaleMatrix( matScale2, g_fScaleVal, g_fScaleVal, g_fScaleVal );
		D3DMath_MatrixMultiply(m_matFileObjectMatrix, matScale2, m_matFileObjectMatrix);
	}
	///////////////////////////////////////////////////////////////////////////




	// Set limit /////////////////////////////////
	// Note: must do this after having set m_matFileObjectMatrix
	if (m_matFileObjectMatrix._42 < -8.5f) {
		//m_matFileObjectMatrix._42 = -8.5f;
		g_bLanded = true;
	} else {
		g_bLanded = false;
	}






	// NOTE: if recording and playing m_matFileObjectMatrix we must do it just after
	// the matrix stuff. We can also record and playback controls here.
	// NO: Better is to record and playback at the end of FrameMove(), i.e. as 
	// close to Render() as possible. m_matFileObjectMatrix *SHOULD NOT BE CHANGED* after
	// this point in Framemove(), or we'll get slightly different playbacks every time.
	//if (m_bRecord)
	FlightRecord();

	//if (m_bPlayBack)
	FlightPlayBack();



	// RT/FR Flight Rec //////////////////////////////////////////////////////
	// The *real* Real-Time way to do transport controls
	// TODO: we get stuck (in the while loop). Solve it!!!
	if (g_bRTGrabFlightRecSlider || g_bFRGrabFlightRecSlider) {
		// For ResetFilePointer() test comment out like this:
		///*if (m_bPlayBack)*/ ResetFilePointer();
		if (m_bPlayBack) ResetFilePointer();

		//if ( !(GetKeyState(VK_LBUTTON) & 0x80) ) {
		//	g_bRTGrabFlightRecSlider = false;
		//	g_bFRGrabFlightRecSlider = false;
		//}
	}

	if (g_bRTButtonPauseChecked || g_bFRButtonPauseChecked) {
		if (m_bPlayBack) ResetFilePointer();
	}



	// Must do this in Render()
	//UpdateFlightRec();
	///////////////////////////////////////////////////////////////////////





/*
	// the eagle has landed
	if (g_bLanded) {
		// ground the fucker
		//m_matFileObjectMatrix = m_matSave;

		// put heli flat on its skids
		//if ( m_vUp.y > sinf(D3DXToRadian(75)) ) {
		//	D3DUtil_SetRotateZMatrix( matRot, m_fRadsZ );
		//	D3DMath_MatrixMultiply(m_matSave, matRot, m_matSave); // order is crucial!!!
		//
		//	// tilt on skids
		//	if ( m_vRight.y < 0.0f)
		//		D3DXMatrixTranslation( &matTrans, -1.2f*m_fRadsZ, -m_fRadsZ, 0.0f );
		//	else
		//		D3DXMatrixTranslation( &matTrans, -1.2f*m_fRadsZ, m_fRadsZ, 0.0f );
		//	D3DMath_MatrixMultiply(m_matSave, matTrans, m_matSave);	// order is crucial!!!
		//
		//}


		// ground the fucker
		//m_matFileObjectMatrix = m_matSave;

		if (m_matFileObjectMatrix._42 < -8.5f) {
			m_matFileObjectMatrix._42 = -8.5f;
			g_bLanded = true;
			//m_matSave = m_matFileObjectMatrix;
		} else {
			g_bLanded = false;
		}

	

		// reset torque
		// TODO: build up torque: the longer we are grounded the more torque
		// (i.e. the less the gyro counteracts at lift-off)
		m_fTorque += 0.001f;
		if (m_fTorque > 0.18f) m_fTorque = 0.18f;

		// up again
		if (m_fY > 0.0f+m_fWeight) {							// +m_fWeight
			//m_matFileObjectMatrix._42 = (m_fY-8.5f)-m_fWeight;	// -m_fWeight lift off
			//g_bLanded = false;
		}
	}
	////////////////////////////////////////////////////////////////
*/	




	



/*
	// shadow //////////////////////////////////////////////////////////////////
	// - plat maken
	// - material zwart
	m_matFileObjectMatrix2 = m_matFileObjectMatrix;

	// x/z position is afhankelijk van stand van de zon
	//m_matFileObjectMatrix2._41 += 1.0f;
	m_matFileObjectMatrix2._42 = -5.0f;
	//m_matFileObjectMatrix2._43 += 0.0f;
*/
/*
	D3DXMATRIX matShadow;
	D3DXVECTOR4 Light(0.0f, 1.0f, 0.0f, 1.0f);
	D3DXPLANE Plane(0.0f, 1.0f, 0.0f, 0.0f);

	D3DXMatrixShadow( &matShadow, &Light, &Plane );
	D3DMath_MatrixMultiply(m_matFileObjectMatrix2, matShadow, m_matFileObjectMatrix2);
*/	
/*


	D3DVERTEX* pVertices = NULL;
    DWORD      dwNumVertices = 0;
    if ( FAILED( m_pFileObject2->GetMeshVertices( "mesh-mrotor", &pVertices, &dwNumVertices ) ) )
    {
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	} else {
		// transform en plat maken
		for ( DWORD i=0; i<dwNumVertices; i++ )
		{

			D3DVECTOR vVector( pVertices[i].x,
							   pVertices[i].y,
							   pVertices[i].z );
			D3DMath_VectorMatrixMultiply(vVector, vVector, m_matFileObjectMatrix); 


			//pVertices[i].x = vVector.x;
			pVertices[i].y = 0.0f;
			//pVertices[i].z = vVector.z;


		}
	}

	if ( FAILED( m_pFileObject2->GetMeshVertices( "mesh-tail", &pVertices, &dwNumVertices ) ) )
    {
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	} else {
		// plat maken
		for ( DWORD i=0; i<dwNumVertices; i++ )
		{
			pVertices[i].y = 0.0f;
		}
	}
	/////////////////////////////////////////////////////////////////////////////
*/


	// Spin the rotors ////////////////////////////////////////////////////////////////
	// Also give cyclic tilting to main rotor
	// TODO:
	// Note: for the Bell it is better to split up shaft and main rotor into two
	// meshes and then spin shaft and main rotor, and give cyclic tilting to main
	// rotor only
	// Note2: for cyclic tilting we also have to translate to x file origin and
	// back (via Y axis)
	// Note3: implement collective movement (hairy...)
	// Note4: or is it? Just split up the rotor blades into separate meshes!
	if (m_fThrottle<0.0f) m_fThrottle=0.0f;
	//m_fValueKey+=m_fThrottle;
	//
	// TODO: make real rotor follow spin of proxy rotor
	// Can't use ang vel: ang vel is given in rads/sec. We take the rads round y
	// but we do not take account of time here. Could use FPS for that but that gives
	// uneven rotation. Let's kludge.
	// NOTE: op zich is het niet zo erg dat we hier een beetje faken: we faken toch al
	// door een flat cylinder als proxy te gebruiken voor de rotors (ipv een box ter
	// groote van de rotor). Daardoor zullen we ook niet de juiste physics krijgen als
	// rotor de grond raakt. Later kunnen we dit verbeteren: 
	// * lage rotor RPM: box proxy rotor and normal .X file rotor and stationary sound
	// * hoge rotor RPM: flat cylinder proxy rotor and disc .X file rotor and full throttle
	// sound
	// See also Reflex.
	// Zo gauw we een box proxy gebruiken voor de rotor (maar hoe gaan we dat doen voor een
	// driebladige rotor?) is het natuurlijk wel van belang dat we de rotor exact de proxy
	// laten volgen.
	// ALSO: we kunnen misschien helemaal niet de real rotor de proxy rotor laten
	// volgen omdat de proxy nu eenmaal niet te hoge dParamVel2 mag hebben omdat dan 
	// de simulatie explodeert (see ODE docs).
	// DONE: See below: use matrix vRight and vForward
	// NONO: that does not work either: we get shear.
	// NOTE NOTE NOTE: ang vel vector is relative to the world!!! It is *NOT* body local!!!
	// Dus we kunnen niet de y waarde nemen voor de mrotor. Magnitude of that vector???
	// Nee ook niet.
	// Kludge:
	//m_fValueKey -= g_vecMRotorAngularVelocity.y;
	//m_fValueKey -= (g_vecMRotorAngularVelocity.y / g_fFPS);
	//m_fValueKey -= g_vecMRotorAngularVelocity.y/18;
	//m_fValueKey -= (g_vecMRotorAngularVelocity.y/20)*500;
	//m_fValueKey -= g_vecMRotorAngularVelocity.y*10;
	
	static float ftemp = 0.0f;

	if (g_bBell || g_bCobra || g_bCougar) {
		ftemp+=m_fThrottle*m_fSpeedFactor;
	} else {
		ftemp+=m_fThrottle*m_fSpeedFactor*0.03f;
	}
	
	m_fValueKey = ftemp;

	// nice effect
	//if (m_fThrottle<6.0f && m_fThrottle>=5.0f) m_fValueKey = -ftemp;
	//if (m_fThrottle<5.0f && m_fThrottle>=4.0f) m_fValueKey = ftemp;


	// wonderful kludge IV ////////////////////////////////////////////
	static FLOAT f1, f2, f3 = 0.0f;

	if ( GetKeyState(VK_SHIFT) & 0x80 ) {
		if ( GetKeyState(VK_MENU) & 0x80 ) {
			if ( GetKeyState('1') & 0x80 ) f1-=0.1f;
			if ( GetKeyState('2') & 0x80 ) f2-=0.1f;
			if ( GetKeyState('3') & 0x80 ) f3-=0.1f;
		} else 
		if ( GetKeyState(VK_CONTROL) & 0x80 ) {
			if ( GetKeyState('1') & 0x80 ) f1-=0.001f;
			if ( GetKeyState('2') & 0x80 ) f2-=0.001f;
			if ( GetKeyState('3') & 0x80 ) f3-=0.001f;
		} else {
			if ( GetKeyState('1') & 0x80 ) f1-=0.01f;
			if ( GetKeyState('2') & 0x80 ) f2-=0.01f;
			if ( GetKeyState('3') & 0x80 ) f3-=0.01f;
		}
	} else {
		if ( GetKeyState(VK_MENU) & 0x80 ) {
			if ( GetKeyState('1') & 0x80 ) f1+=0.1f;
			if ( GetKeyState('2') & 0x80 ) f2+=0.1f;
			if ( GetKeyState('3') & 0x80 ) f3+=0.1f;
		} else
		if ( GetKeyState(VK_CONTROL) & 0x80 ) {
			if ( GetKeyState('1') & 0x80 ) f1+=0.001f;
			if ( GetKeyState('2') & 0x80 ) f2+=0.001f;
			if ( GetKeyState('3') & 0x80 ) f3+=0.001f;
		} else {
			if ( GetKeyState('1') & 0x80 ) f1+=0.01f;
			if ( GetKeyState('2') & 0x80 ) f2+=0.01f;
			if ( GetKeyState('3') & 0x80 ) f3+=0.01f;
		}
	}
	
	// can't use VK_MENU (Alt) here because that's for menu access
	if ( GetKeyState('0') & 0x80 ) {
		if ( GetKeyState('1') & 0x80 ) f1=0.0f;
		if ( GetKeyState('2') & 0x80 ) f2=0.0f;
		if ( GetKeyState('3') & 0x80 ) f3=0.0f;
	}

	sprintf(msg1, "Value 1: %f", f1);
	sprintf(msg2, "Value 2: %f", f2);
	sprintf(msg3, "Value 3: %f", f3);
	////////////////////////////////////////////////////////////////////////
	

	// find Frame in X file
	CD3DFileObject* pObject = m_pFileObject->FindObject( "frame-mrotor" );
    if( pObject )
    {
		D3DMATRIX* pmat = pObject->GetMatrix();
		// use pow() to increase speed gradually (exponentially)
		//D3DUtil_SetRotateYMatrix( *pmat, powf(-m_fThrottle,2) ); //left rotating system
		//MessageBox(NULL,"qqq","QQQ",MB_OK);

		// de waarde 0.75f is het verschil tussen de oorsprong van
		// de heli en de main rotor. Hoe bepalen we die???
		// het beste is als je in de x file een heli origin vertex (0,0,0) hebt
		// en een main rotor origin vertex. Dan weet je hoe je de rotor naar
		// de origin kunt transleren.
		// NOTE: we need the () around (g_vAxisMRotor.x+f1) etc.
		// NOTE: het principe is simpel: translate naar file origin, rotate, translate back
		// NOTE: mrotor: rotY, transX and transZ
		//       trotor: rotX, transY and transZ
		D3DMATRIX matTrans, matTrans2;
		D3DUtil_SetTranslateMatrix( matTrans,  -(g_vAxisMRotor.x+f1),  (g_vAxisMRotor.y+f2), -(g_vAxisMRotor.z+f3) );
		D3DUtil_SetTranslateMatrix( matTrans2,  (g_vAxisMRotor.x+f1),  (g_vAxisMRotor.y+f2),  (g_vAxisMRotor.z+f3) );

		//D3DUtil_SetTranslateMatrix( matTrans,  -f1,  f2, -f3 ); // test
		//D3DUtil_SetTranslateMatrix( matTrans2,  f1,  f2,  f3 );	
		
		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.0f+f2,  0.0f+f3 ); // gala1.x
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.0f+f2, -0.0f+f3 );

		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.0f+f2,  0.0f+f3 ); // raptor.x
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.0f+f2, -0.0f+f3 );

		
		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.09f+f2,  1.253f+f3 ); // red heli
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.09f+f2, -0.759f+f3 );

//		if (g_bBell) {
//			D3DUtil_SetTranslateMatrix( matTrans,   0.01f+f1, -2.19f+f2,  0.11f+f3 ); // bell
//			D3DUtil_SetTranslateMatrix( matTrans2,  0.03f+f1,  2.17f+f2, -0.10f+f3 );
//		}
//
//		if (g_bCobra) {
//			D3DUtil_SetTranslateMatrix( matTrans,   0.03f+f1, 0.0f+f2,  0.01f+f3 ); // cobra
//			D3DUtil_SetTranslateMatrix( matTrans2,  0.03f+f1, 0.0f+f2, -0.01f+f3 );
//		}
//
//		if (g_bCougar) {
//			D3DUtil_SetTranslateMatrix( matTrans,   0.08f+f1, 0.0f+f2,  2.53f+f3 ); // cougar
//			D3DUtil_SetTranslateMatrix( matTrans2, -0.08f+f1, 0.0f+f2, -2.60f+f3 );
//		}


		
		D3DMATRIX matRotY, matRotX, matRotZ;
		D3DUtil_SetRotateYMatrix( matRotY, m_fValueKey );	// spin
		
//		// make real rotor follow spin of proxy rotor
//		// NONO: that does not work either: we get shear
//		D3DUtil_SetIdentityMatrix(matRotY);
//		// vRight                      // vForward
//		matRotY._11 = g_matMRotor._11; matRotY._12 = g_matMRotor._12;
//		matRotY._21 = g_matMRotor._21; matRotY._22 = g_matMRotor._22;
//		matRotY._31 = g_matMRotor._31; matRotY._32 = g_matMRotor._32;


//		D3DUtil_SetRotateXMatrix( matRotX, m_fRadsX*1.0f );	// cyclic tilting
//		D3DUtil_SetRotateZMatrix( matRotZ, m_fRadsZ*1.0f );
//
//		if (g_bBell) {
//			D3DUtil_SetRotateXMatrix( matRotX, m_fRadsX*2.0f );	// cyclic tilting
//			D3DUtil_SetRotateZMatrix( matRotZ, m_fRadsZ*2.0f );
//		}
//
//		if (g_bCobra || g_bCougar) {	
//			D3DUtil_SetRotateXMatrix( matRotX, 0.0f );	// no cyclic tilting
//			D3DUtil_SetRotateZMatrix( matRotZ, 0.0f );
//		}

		D3DUtil_SetRotateXMatrix( matRotX, m_fRotorTiltX*1.0f );	// cyclic tilting
		D3DUtil_SetRotateZMatrix( matRotZ, m_fRotorTiltZ*1.0f );

		if (g_bBell) {
			D3DUtil_SetRotateXMatrix( matRotX, m_fRotorTiltX*2.0f );	// cyclic tilting
			D3DUtil_SetRotateZMatrix( matRotZ, m_fRotorTiltZ*2.0f );
		}

		if (g_bCobra || g_bCougar) {	
			D3DUtil_SetRotateXMatrix( matRotX, 0.0f );	// no cyclic tilting
			D3DUtil_SetRotateZMatrix( matRotZ, 0.0f );
		}

		D3DMATRIX matAll;
		//D3DUtil_SetIdentityMatrix(matAll);

		// Order!!!
		D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
		D3DMath_MatrixMultiply(matAll,matAll,matRotX);
		D3DMath_MatrixMultiply(matAll,matAll,matRotZ);
		D3DMath_MatrixMultiply(*pmat,matAll,matTrans2);



//		pmat->_41 = g_matMRotor._41;
//		pmat->_42 = g_matMRotor._42;
//		pmat->_43 = g_matMRotor._43;


		// TODO: make real mrotor follow proxy mrotor
		// Make real rotor have the same angular velocity as the proxy rotor
//		D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
//		D3DMath_MatrixMultiply(matAll,matAll,matRotX);
//		D3DMath_MatrixMultiply(matAll,matAll,matRotZ);
//		//D3DMath_MatrixMultiply(matAll,matAll,matTrans2);

		//D3DMath_MatrixMultiply(matAll,matAll,matTrans);
		//D3DMath_MatrixMultiply(matAll,matAll,matRotX);
		//D3DMath_MatrixMultiply(matAll,matAll,matRotZ);
		//D3DMath_MatrixMultiply(matAll,matAll,matTrans2);
		//D3DMath_MatrixMultiply(g_matMRotor,matAll,g_matMRotor);

		
		// TODO: make real mrotor follow proxy mrotor
		// Make real rotor have the same angular velocity as the proxy rotor
		// We must give it the same rotation along Y as the proxy mrotor. How???
		//*pmat = g_matMRotor;
		//*pmat = m_matIdentityMatrix;
		//*pmat = m_matFileObjectMatrix;
		//D3DMath_MatrixMultiply(matAll,matAll,matTrans2);
		//D3DMath_MatrixMultiply(g_matMRotor,matAll,g_matMRotor);
	}



	// find Frame in X file
	pObject = m_pFileObject->FindObject( "frame-shaft" );
    if( pObject )
    {
		D3DMATRIX* pmat = pObject->GetMatrix();
		// use pow() to increase speed gradually (exponentially)
		//D3DUtil_SetRotateYMatrix( *pmat, powf(-m_fThrottle,2) ); //left rotating system
		//MessageBox(NULL,"qqq","QQQ",MB_OK);

		// de waarde 0.75f is het verschil tussen de oorsprong van
		// de heli en de main rotor. Hoe bepalen we die???
		// het beste is als je in de x file een heli origin vertex (0,0,0) hebt
		// en een main rotor origin vertex. Dan weet je hoe je de rotor naar
		// de origin kunt transleren.
		D3DMATRIX matTrans, matTrans2;
		D3DUtil_SetTranslateMatrix( matTrans,  -(g_vAxisMRotor.x+f1),  (g_vAxisMRotor.y+f2), -(g_vAxisMRotor.z+f3) );
		D3DUtil_SetTranslateMatrix( matTrans2,  (g_vAxisMRotor.x+f1),  (g_vAxisMRotor.y+f2),  (g_vAxisMRotor.z+f3) );

		//D3DUtil_SetTranslateMatrix( matTrans,  -f1,  f2, -f3 ); // test
		//D3DUtil_SetTranslateMatrix( matTrans2,  f1,  f2,  f3 );

		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.0f+f2,  0.0f+f3 ); // gala1.x
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.0f+f2, -0.0f+f3 );

		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.0f+f2,  0.0f+f3 ); // raptor.x
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.0f+f2, -0.0f+f3 );
		
		
		//D3DUtil_SetTranslateMatrix( matTrans,  0.0f+f1, -0.09f+f2,  1.253f+f3 ); // red heli
		//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f+f1, -0.09f+f2, -0.759f+f3 );

		//if ( GetKeyState('B') & 0x80 ) g_bBell = !g_bBell;
		if (g_bBell) {
			D3DUtil_SetTranslateMatrix( matTrans,   0.01f+f1, 0.0f+f2,  0.11f+f3 ); // bell
			D3DUtil_SetTranslateMatrix( matTrans2,  0.03f+f1, 0.0f+f2, -0.10f+f3 );
		}

		if (g_bCobra) {
			//D3DUtil_SetTranslateMatrix( matTrans,   0.03f+f1, 0.0f+f2,  0.01f+f3 ); // cobra
			//D3DUtil_SetTranslateMatrix( matTrans2,  0.03f+f1, 0.0f+f2, -0.01f+f3 );
		}



		//if (g_bBell) {
			D3DMATRIX matRotY, matRotX, matRotZ;
			D3DUtil_SetRotateYMatrix( matRotY, m_fValueKey );	// spin

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
			D3DMath_MatrixMultiply(*pmat,matAll,matTrans2);

		//}

	}



	// find Frame in X file
	pObject = m_pFileObject->FindObject( "frame-trotor" );
    if( pObject )
    {
		// TODO: rotate tail rotor round its own axis
		D3DMATRIX* pmat = pObject->GetMatrix();
		// use pow() to increase speed gradually (exponentially)
		//D3DUtil_SetRotateYMatrix( *pmat, powf(-m_fThrottle,2) ); //left rotating system
		//MessageBox(NULL,"qqq","QQQ",MB_OK);
	
		// move tail rotor back to heli origin, rotate, then back
		// there must be a better way. Quaternions???
		D3DMATRIX matTrans, matTrans2;
		D3DUtil_SetTranslateMatrix( matTrans,  (g_vAxisTRotor.x+f1), -(g_vAxisTRotor.y+f2), -(g_vAxisTRotor.z+f3) );
		D3DUtil_SetTranslateMatrix( matTrans2, (g_vAxisTRotor.x+f1),  (g_vAxisTRotor.y+f2),  (g_vAxisTRotor.z+f3) );

		//D3DUtil_SetTranslateMatrix( matTrans,  f1, -f2, -f3 ); // test
		//D3DUtil_SetTranslateMatrix( matTrans2, f1,  f2,  f3 );	
		
		//D3DUtil_SetTranslateMatrix( matTrans, f1,   0.12f+f2, -3.42f+f3 ); // gala1.x
		//D3DUtil_SetTranslateMatrix( matTrans2, f1, -0.12f+f2,  3.42f+f3 );

		//D3DUtil_SetTranslateMatrix( matTrans,  f1, -0.071f+f2, -1.022f+f3 ); // raptor.x
		//D3DUtil_SetTranslateMatrix( matTrans2, f1,  0.071f+f2,  1.022f+f3 );

		//D3DUtil_SetTranslateMatrix( matTrans, f1, -0.47f+f2, -5.72f+f3 ); // red heli
		//D3DUtil_SetTranslateMatrix( matTrans2, f1, 0.47f+f2,  5.72f+f3 );
		
		if (g_bBell) {
			D3DUtil_SetTranslateMatrix( matTrans,  -0.02f+f1, -1.11f+f2, -8.40f+f3 ); // bell
			D3DUtil_SetTranslateMatrix( matTrans2, -0.02f+f1,  1.11f+f2,  8.40f+f3 );
		}
		
		if (g_bCobra) {
			D3DUtil_SetTranslateMatrix( matTrans,  -0.02f+f1, -1.08f+f2, -7.94f+f3 ); // cobra
			D3DUtil_SetTranslateMatrix( matTrans2, -0.02f+f1,  1.08f+f2,  7.94f+f3 );
		}

		D3DMATRIX matRotX;
		D3DUtil_SetRotateXMatrix( matRotX, 5.0f*m_fValueKey ); // main to tail gear ratio: 5.0
		D3DMATRIX matAll;
		D3DMath_MatrixMultiply(matAll,matTrans,matRotX);
		D3DMath_MatrixMultiply(*pmat,matAll,matTrans2);
		

		//D3DXMATRIX matRot2;
		//D3DXQUATERNION qRot2;
		//D3DXQuaternionRotationYawPitchRoll( &qRot2, 0.0f, fTimeKey*5, 0.0f );	
		//D3DXMatrixRotationQuaternion( &matRot2, &qRot2 );	
		//D3DMath_MatrixMultiply(*pmat, matRot2, *pmat);
		//pObject->SetMatrix( (D3DMATRIX *)&matRot2 );

    }	
	//////////////////////////////////////////////////////////////////////////////////




	// Collective Coning /////////////////////////////////////////////////////////////	
	if ( !g_bBell && !g_bCobra && !g_bCougar ) {
		D3DVERTEX* pVertices;
		DWORD      dwNumVertices;

		// MRotor
		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-mrotor", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {			

			//MessageBeep(-1);			

			// always snap back to original mrotor
			for ( DWORD i=0; i<dwNumVertices; i++ ) {
				//pVertices[i] = g_pVerticesOrigSkid[i];
				//pVertices[i] = *theIterator++;
				pVertices[i] = g_pVerticesOrigMRotor[i];
			}

			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				//if ( pVertices[i].x == -0.003376f && pVertices[i].z == -1.249372f ) {
				//	// do nothing: Jesus nut
				//} else {
				//	if (i<21)
				//		pVertices[i].y = m_fRotorConeY+1.75f+0.2f;
				//	else
				//		pVertices[i].y = m_fRotorConeY+1.63f+0.2f;
				//}

				// Hoe verder vanaf de mrotor axis, hoe sterker y verandert.
				// De afstand van de oorsprong is te berekenen uit de vertex x en z waarden via
				// de formule die geldt voor de zijden van een driehoek:
				// a^2 + b^2 = c^2 => c = sqrt(a^2 + b^2)
				//
				// NOTE: We compensate x and z if the mrotor axis is not at (0,0,0). This is done
				// subtracting g_vAxisMRotor.x and g_vAxisMRotor.z, respectively.
				// NOTE NOTE NOTE!!!!: Major Gotcha: 
				// In Debug coning is perfect when using pow() instead of powf().
				// In Release we must use powf() or coning will be bungled. Release building is very strict
				// when it comes to precision so we must use powf() if we are using floats.
				// Now we're at it: we should use sqrtf() as well.
				// Sucks!!!

				// mrotor collective up
				if (m_fRotorConeY>0.0f) {
					//pVertices[i].y += m_fRotorConeY * ( sqrtf(powf(pVertices[i].x,2.0)+powf(pVertices[i].z,2.0)) ) *0.15f;
					pVertices[i].y += m_fRotorConeY * ( sqrtf(powf(pVertices[i].x-g_vAxisMRotor.x,2.0)+powf(pVertices[i].z-g_vAxisMRotor.z,2.0)) ) *0.15f;
				}

				// mrotor collective down
				if (m_fRotorConeY<0.0f) {
					//pVertices[i].y += m_fRotorConeY * ( sqrtf(powf(pVertices[i].x,2.0)+powf(pVertices[i].z,2.0)) ) *0.15f;
					pVertices[i].y += m_fRotorConeY * ( sqrtf(powf(pVertices[i].x-g_vAxisMRotor.x,2.0)+powf(pVertices[i].z-g_vAxisMRotor.z,2.0)) ) *0.15f;
				}
				
				// NOT IN VERSION 1.5
//				// TEST: fast/slow rotors
//				// make mrotor disappear
//				if (g_pd3dApp->m_fThrottle<=7.00f) {
//					pVertices[i].x = 0.0f;
//					pVertices[i].y = 0.0f;
//					pVertices[i].z = 0.0f;
//				}
				
			}
		}


		// TRotor
		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-trotor", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {			

			//MessageBeep(-1);			

			// always snap back to original trotor
			for ( DWORD i=0; i<dwNumVertices; i++ ) {
				//pVertices[i] = g_pVerticesOrigSkid[i];
				//pVertices[i] = *theIterator++;
				pVertices[i] = g_pVerticesOrigTRotor[i];
			}

			for ( i=0; i<dwNumVertices; i++ )
			{
			
				// Hoe verder vanaf de trotor axis, hoe sterker x verandert.
				// De afstand van de oorsprong is te berekenen uit de vertex y en z waarden via
				// de formule die geldt voor de zijden van een driehoek:
				// a^2 + b^2 = c^2 => c = sqrtf(a^2 + b^2)
				//
				// NOTE: We compensate y and z if the trotor axis is not at (0,0,0). This is done
				// subtracting g_vAxisTRotor.y and g_vAxisTotor.z, respectively.

				// trotor collective right
				if (m_fRotorConeX>0.0f) {
					//pVertices[i].x += m_fRotorConeX * ( sqrtf(powf(pVertices[i].y,2.0)+powf(pVertices[i].z,2.0)) ) *0.4f;
					pVertices[i].x += m_fRotorConeX * ( sqrtf(powf(pVertices[i].y-g_vAxisTRotor.y,2.0)+powf(pVertices[i].z-g_vAxisTRotor.z,2.0)) ) *0.4f;
				}

				// trotor collective left
				if (m_fRotorConeX<0.0f) {
					//pVertices[i].x += m_fRotorConeX * ( sqrtf(powf(pVertices[i].y,2.0)+powf(pVertices[i].z,2.0)) ) *0.4f;
					pVertices[i].x += m_fRotorConeX * ( sqrtf(powf(pVertices[i].y-g_vAxisTRotor.y,2.0)+powf(pVertices[i].z-g_vAxisTRotor.z,2.0)) ) *0.4f;
				}
				
				// NOT IN VERSION 1.5
//				// TEST: fast/slow rotors
//				// make trotor disappear
//				if (g_pd3dApp->m_fThrottle<=7.00f) {
//					pVertices[i].x = 0.0f;
//					pVertices[i].y = 0.0f;
//					pVertices[i].z = 0.0f;
//				}
				
			}
		}


		// Shaft
		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-shaft", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {			

			//MessageBeep(-1);			

			// always snap back to original shaft
			for ( DWORD i=0; i<dwNumVertices; i++ ) {
				//pVertices[i] = g_pVerticesOrigSkid[i];
				//pVertices[i] = *theIterator++;
				pVertices[i] = g_pVerticesOrigShaft[i];
			}

			for ( i=0; i<dwNumVertices; i++ )
			{
				// NOT IN VERSION 1.5
				// TEST: fast/slow rotors
//				// make shaft disappear
//				if (g_pd3dApp->m_fThrottle<=7.00f) {
//					pVertices[i].x = 0.0f;
//					pVertices[i].y = 0.0f;
//					pVertices[i].z = 0.0f;
//				}
				
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////////////



	
	
	// TEST:
	// Skid Spring ///////////////////////////////////////////////////////////////////
	// Let's do some morphing...
	// If there is pressure on skid (from collective down when grounded or when landing)
	// we should compress skid in y and expand in x. If pressure is gone we should do the
	// opposite.
	// Amount of compression/expansion depends on position of y vertex:
	// Skid top y should get no compression/expansion.
	// Skid bottom y should get maximum compression/expansion.
	// Same goes for x vertex:
	// Middle x should get no compression/expansion.
	// Far x should get maximum compression/expansion.
	// It would probable be better to use a dynamics array to store the original vertices
	// in. See: frameindex.cpp for the use of dynamic arrays. Use STL. This should be
	// done once during file load.
	// NOTE: this is only the graphics side of things. For physics spring/suspension
	// use slider joint or hinge-2 joint with dParamSuspensionERP and dParamSuspensionCFM
	// for the skids.
	// NOTE: It is even more complexer: we also have to account for swing caused by
	// rotor cyclic when on the ground. This means we probably have to implement the
	// skids like 4 wheels. Graphically, it means we must compress the front of the skid
	// when the user gives cyclic fore, and compress the left they give cyclic left, etc.
	// So we also have to deal with m_fRotorTiltX and m_fRotorTiltZ.
	// NOTE: we are using rotor cone and tilt values to morph the mesh but this is just
	// for testing. We might use other variables later on.
	// NOTE: Maybe we should introduce the variables:
	// m_fSkidCollectiveY, m_fSkidTiltX, and m_fSkidTiltZ as skid more or less follows
	// same movement as rotor.
	// Values that then e.g cause skid collective compress should be added:
	// m_fSkidCollectiveY = m_fRotorCellectiveY + LandingForce + ...
	// NOTE: How Reflex does it: e.g. nick fore causes:
	// 1. skid to compress fore
	// 2. fuselage to tilt fore a bit
	// 3. rotor to tilt fore a lot
	// NOTE: we should do body tilt with ODE. The other two we do graphically.
	// NOTE: Collision/Crash Damage: Using dynamic arrays to store the original mesh 
	// vertices for more meshes than just the skid means we can do collision/crash damage.
	// Collision/crash damage could be a combination of
	// * Distorting a mesh's frame matrix: Use: m_pFileObject->FindObject()
	// * Distorting a mesh's vertices. Use: m_pFileObject->GetMeshVertices()
	// NOTE: let's also do tail rotor coning
	// NOTE: If we implement suspension in ODE, tail rotor will also have effect when
	// heli is standing on the skids.
	// NOTE: When doing collective down the skid should be compressed more at aft because
	// the main rotor is more above the back of the skid than the front. Also aft x wider.
	//
	// NOTE: 
	// Physics: Body+Geom+Proxy
	// Graphics: Mesh+Material+Texture
	if (!g_bBell && !g_bCobra && !g_bCougar) {
		D3DVERTEX* pVertices;
		DWORD      dwNumVertices;

		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-skid", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			// mmm, still need the smoothness...
			//m_fSkidSpringY = m_fRotorConeY;

			//static int count = 0;
			//count++;

			// get skid top y
			float top_y = pVertices[0].y;
			for ( DWORD i=0; i<dwNumVertices; i++ )	{				
				if (pVertices[i].y > top_y)
					top_y = pVertices[i].y;			
			}
			//sprintf(msg1, "top_y: %f", top_y);
			//MessageBox(NULL,msg1,"QQQ",MB_OK);

			// TODO: skid should snap back into its original shape automatically
			// So we should always have an array with the original vertices ready
			// Should do this once at file load and use a dynamic array.
			// NONO: Hell, can't use dynamic arrays of type D3DVERTEX:
			// typedef vector<D3DVERTEX> D3DVERTEXVECTOR; is not accepted by STL.
			// NOTE: STL for Visual C++ 5.0 has several bugs. A better STL is obtainable
			// at: www.sirius.com/~ouchida/
			// We could create an array on the free store (the "heap"):
			// Use: D3DVERTEX pVerticesOrig = new D3DVERTEX[dwNumVertices];
//			static D3DVERTEX* pVerticesOrigSkid = new D3DVERTEX[dwNumVertices];
//			// TEST:
//			//static D3DVERTEX g_pVerticesOrigSkid[6666];
//			static bool firsttime7 = true;
//			if ( firsttime7/*GetKeyState('Q') & 0x80*/ ) {
//				firsttime7 = false;
//				for ( i=0; i<dwNumVertices; i++ ) {
//					//g_pVerticesOrigSkid[i] = pVertices[i];
//					//theVectorSkid.push_back(pVertices[i]);
//					pVerticesOrigSkid[i] = pVertices[i];
//				}
//			}

			
			// always snap back to original skid
			for ( i=0; i<dwNumVertices; i++ ) {
				//pVertices[i] = g_pVerticesOrigSkid[i];
				//pVertices[i] = *theIterator++;
				pVertices[i] = g_pVerticesOrigSkid[i];
			}

		

			// skid collective compress
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorConeY<0.0f) {
					//MessageBeep(-1);
					// compress y
					pVertices[i].y += m_fRotorConeY*(pVertices[i].y-top_y) *0.4f;

					// compress y aft
					pVertices[i].y += m_fRotorConeY*(pVertices[i].y-top_y)*pVertices[i].z *0.2f;
				
					// x wider
					pVertices[i].x -= m_fRotorConeY*pVertices[i].x *0.2f;

					// x wider aft
					pVertices[i].x -= m_fRotorConeY*pVertices[i].x*pVertices[i].z *0.1f;
				}
			}

			// skid collective expand
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorConeY>0.0f) {
					//MessageBeep(-1);
					// expand y
					pVertices[i].y += m_fRotorConeY*(pVertices[i].y-top_y) *0.2f;

					// expand y aft
					//pVertices[i].y += m_fRotorConeY*(pVertices[i].y-top_y)*pVertices[i].z *0.2f;

					// x narrower
					pVertices[i].x -= m_fRotorConeY*pVertices[i].x *0.2f;

					// x narrower aft
					//pVertices[i].x -= m_fRotorConeY*pVertices[i].x*pVertices[i].z *0.2f;
				}
			}

			
			// skid nick fore compress
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorTiltX<0.0f) {
					//MessageBeep(-1);
					// compress y fore
					pVertices[i].y -= m_fRotorTiltX*(pVertices[i].y-top_y)*pVertices[i].z *4.0f;
					
					// x fore wider
					pVertices[i].x += m_fRotorTiltX*pVertices[i].x*pVertices[i].z *2.0f;
				}
			}

			// skid nick aft compress
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorTiltX>0.0f) {
					//MessageBeep(-1);
					// compress y aft
					pVertices[i].y -= m_fRotorTiltX*(pVertices[i].y-top_y)*pVertices[i].z *4.0f;
					
					// x aft wider
					pVertices[i].x += m_fRotorTiltX*pVertices[i].x*pVertices[i].z *2.0f;
				}
			}


			// skid roll left compress
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorTiltZ<0.0f) {
					//MessageBeep(-1);
					// compress y left
					pVertices[i].y += m_fRotorTiltZ*(pVertices[i].y-top_y)*pVertices[i].x *4.0f;
					
					// x wider but only for left skid
					if (pVertices[i].x > 0.0f)				
						pVertices[i].x -= m_fRotorTiltZ*pVertices[i].x *1.0f;
				}
			}

			// skid roll right compress
			for ( i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				if (m_fRotorTiltZ>0.0f) {
					//MessageBeep(-1);
					// compress y right
					pVertices[i].y += m_fRotorTiltZ*(pVertices[i].y-top_y)*pVertices[i].x *4.0f;
					
					// x wider but only for right skid
					if (pVertices[i].x < 0.0f)		
						pVertices[i].x += m_fRotorTiltZ*pVertices[i].x *1.0f;
				}
			}

		}
	}
	//////////////////////////////////////////////////////////////////////////////////




	// RT Config Position ////////////////////////////////////////////////////////////
	DoRTConfigPosition();
	//////////////////////////////////////////////////////////////////////////////////



	// pilot position scaling ////////////////////////////////////////////////
	D3DMATRIX matScale;
	D3DUtil_SetScaleMatrix( matScale, 1.0f, 1.0f, 1.0f );

	// keyboard pilot position scaling
	// Shift: increased values
	// Control: reversed values
	if ( GetKeyState('7') & 0x80 ) {
		if ( GetKeyState(VK_CONTROL) & 0x80 ) {
			if ( GetKeyState(VK_SHIFT) & 0x80 )	{			
				D3DUtil_SetScaleMatrix( matScale, 0.9f, 0.9f, 0.9f );
				g_fPPSize *= 0.9f;
			} else {
				D3DUtil_SetScaleMatrix( matScale, 0.99f, 0.99f, 0.99f );
				g_fPPSize *= 0.99f;
			}
		} else {
			if ( GetKeyState(VK_SHIFT) & 0x80 )	{			
				D3DUtil_SetScaleMatrix( matScale, 1.1f, 1.1f, 1.1f );
				g_fPPSize *= 1.1f;
			} else {
				D3DUtil_SetScaleMatrix( matScale, 1.01f, 1.01f, 1.01f );
				g_fPPSize *= 1.01f;
			}
		}
	}
	
	// reset size
	if ( GetKeyState('7') & 0x80 && GetKeyState('0') & 0x80 ) {
		// Position the Pilot Position Marks
		D3DUtil_SetTranslateMatrix( m_matPP1Matrix, PILOTPOSITION1_X, PILOTPOSITION1_Y, PILOTPOSITION1_Z );
		D3DUtil_SetTranslateMatrix( m_matPP2Matrix, PILOTPOSITION2_X, PILOTPOSITION2_Y, PILOTPOSITION2_Z );
		D3DUtil_SetTranslateMatrix( m_matPP3Matrix, PILOTPOSITION3_X, PILOTPOSITION3_Y, PILOTPOSITION3_Z );
		D3DUtil_SetTranslateMatrix( m_matPP4Matrix, PILOTPOSITION4_X, PILOTPOSITION4_Y, PILOTPOSITION4_Z );

		D3DUtil_SetScaleMatrix( matScale, 0.5f, 0.5f, 0.5f );

		g_fPPSize = 100.0f;
	}

	// RT pilot position scaling
	// Shift: increased values
	// Control: reversed values
	if (g_bRTSpinnerPositionSizeDown) {
		if (g_iRTSliderPositionSize == 1 || GetKeyState(VK_SHIFT) & 0x80) {				
			D3DUtil_SetScaleMatrix( matScale, 0.9f, 0.9f, 0.9f );
			g_fPPSize *= 0.9f;
		} else {
			D3DUtil_SetScaleMatrix( matScale, 0.99f, 0.99f, 0.99f );
			g_fPPSize *= 0.99f;
		}
	}
	if (g_bRTSpinnerPositionSizeUp) {
		if (g_iRTSliderPositionSize == 1 || GetKeyState(VK_SHIFT) & 0x80) {				
			D3DUtil_SetScaleMatrix( matScale, 1.1f, 1.1f, 1.1f );
			g_fPPSize *= 1.1f;
		} else {			
			D3DUtil_SetScaleMatrix( matScale, 1.01f, 1.01f, 1.01f );
			g_fPPSize *= 1.01f;
		}
	}

	
	// Scale 'em
	// Order!!!
	D3DMath_MatrixMultiply(m_matPP1Matrix, matScale, m_matPP1Matrix);
	D3DMath_MatrixMultiply(m_matPP2Matrix, matScale, m_matPP2Matrix);
	D3DMath_MatrixMultiply(m_matPP3Matrix, matScale, m_matPP3Matrix);
	D3DMath_MatrixMultiply(m_matPP4Matrix, matScale, m_matPP4Matrix);
	/////////////////////////////////////////////////////////////////////////



	// reset all latency arrays  ////////////////////
	if (g_bResetLatency) {
		g_bResetLatency = false;
		for (int i=0; i<=MAXLATENCY; i++) {			

			//if (g_bRTButtonPauseChecked) {
			//	g_matOld[i] = g_matOld[0];				
			//} else {
			//	g_matOld[i]  = m_matFileObjectMatrix;
			//}
			//
			//if (g_bRTGrabFlightRecSlider || g_bRTButtonFastForwardDown || g_bRTButtonRewindDown) {
			//	g_matOld[i]  = m_matFileObjectMatrix;
			//}
			
			g_matOld[i]  = m_matFileObjectMatrix;
			g_matOld2[i] = m_matFileObjectMatrix;
			g_matOld3[i] = m_matFileObjectMatrix;
		}
	} 
	///////////////////////////////////////////////////////



	// Set the R/C View matrix ////////////////////////////////////////////////
	// TODO: solve latency problems:
	// * After crash
	// * After open new X file
	// * After playback
	// DONE: using g_bResetLatency
	// * When heli rolls through 90 degrees it falls down a bit
	//
	// Huh, what about out of bounds checking? We can just write beyond the end of
	// the array. No complaint anywhere. We krijgen natuurlijk pas een fout als we
	// protected geheugen overschrijven.
	// store present matrix at end of array
	g_matOld[g_iLat+1] = m_matFileObjectMatrix;

	// shift matrices
	for (int i=0; i<=g_iLat; i++) {
		g_matOld[i] = g_matOld[i+1];	
	}

	if (m_bRCView) {
		SetRCViewMatrix();
	}
	///////////////////////////////////////////////////////////////////////////


	// Set In-Model View matrix ///////////////////////////////////////////////
	if (m_bInModelView) {
		SetInModelViewMatrix();
	}
	///////////////////////////////////////////////////////////////////////////


	// Set Chase View matrix //////////////////////////////////////////////////
	// Hell, can't get this in the procedure because it needs to be done every
	// FrameMove() to keep the latency array current
	//
	// Apply some latency for chase view. We are feeding MatrixInvert() old file object
	// matrices. Thus the model will move first, and the camera follows later.
	// This looks more natural than a static chase view.
	//
	// we've made these global so CollisionDetect() can access them to reset
	//const int g_iLat = 12;
	//static D3DMATRIX g_matOld[g_iLat];

	// store present matrix at end of array 
	g_matOld2[g_iLat2+1] = m_matFileObjectMatrix;

	// shift matrices
	for (i=0; i<=g_iLat2; i++) {
		g_matOld2[i] = g_matOld2[i+1];	
	}

	if (m_bChaseView) {
		SetChaseViewMatrix();
	}
	///////////////////////////////////////////////////////////////////////////


	// Set Follow View matrix /////////////////////////////////////////////////
	// store present matrix at end of array 
	g_matOld3[g_iLat3+1] = m_matFileObjectMatrix;

	// shift matrices
	for (i=0; i<=g_iLat3; i++) {
		g_matOld3[i] = g_matOld3[i+1];	
	}

	if (m_bFollowView) {
		SetFollowViewMatrix();
	}
	///////////////////////////////////////////////////////////////////////////


	
	////////////////////////////////////////////////////////////////////////////////
	// Reset Control values
	// NOTE: When doing playback we should not reset, of course, because many other things
	// (UpdateRTChannels(), UpdateRTVirtualTx(), Rotor Coning/Tilting, updating m_vRight,
	// m_vUp, m_vForward, etc.) are dependent on the Control values.
	if ( !m_bPlayBack ) {
		m_fX=m_fY=m_fZ=0.0f;
		m_fRadsY=m_fRadsX=m_fRadsZ=0.0f;
	}
	////////////////////////////////////////////////////////////////////////////////
	

    return S_OK;
}




//-----------------------------------------------------------------------------
// Name: Render()
// Desc: Called once per frame, the call is the entry point for 3d
//       rendering. This function sets up render states, clears the
//       viewport, and renders the scene.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::Render()
{
	
	// splash screen //////////////////
	if (SPLASHSCREEN) {
		if (m_bSplashScreenShowing) {
			RenderSplashScreen();
			return S_FALSE;
		}
	}
	///////////////////////////////////


	// Status Bar /////////////////////
	UpdateStatusBar();
	///////////////////////////////////

	
	// RT/FR Flight Rec ///////////////////////////////////////////////////
	UpdateFlightRec();
	///////////////////////////////////////////////////////////////////////


	// fog color
	// TODO: let user choose fog color in Configuration
	//if (g_bFogBlack) {
	//	m_dwFogColor = 0x00000000;
	//} else {
	//	m_dwFogColor = 0x00ffffff;
	//}
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGCOLOR, m_dwFogColor );


	DWORD dwColor;
	if (m_bFog) dwColor = m_dwFogColor;
	else	    dwColor = m_dwBackGroundColor;	

    // Clear the viewport. Sets background colour
    m_pd3dDevice->Clear( 0, NULL, D3DCLEAR_TARGET|D3DCLEAR_ZBUFFER,
                         dwColor, 1.0f, 0L );


    DWORD dwCullMode  = m_bCull ? D3DCULL_CCW   : D3DCULL_NONE;
    DWORD dwShadeMode = m_bFlat ? D3DSHADE_FLAT : D3DSHADE_GOURAUD;
	//DWORD dwTexture   = m_bText ? D3DTA_TEXTURE : D3DTA_DIFFUSE;
	DWORD dwTexture   = m_bText ? D3DTOP_MODULATE : D3DTOP_DISABLE;

    DWORD dwFillMode;
	if (m_bSolid) dwFillMode = D3DFILL_SOLID;
	if (m_bWire)  dwFillMode = D3DFILL_WIREFRAME;
	if (m_bPoint) dwFillMode = D3DFILL_POINT;

	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGENABLE, m_bFog );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SPECULARENABLE, m_bSpec );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE,  dwCullMode );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SHADEMODE, dwShadeMode );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FILLMODE,  dwFillMode );
	
	//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, dwTexture );
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, dwTexture );

	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGSTART, *((LPDWORD) (&m_fFogStart)) );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGEND,   *((LPDWORD) (&m_fFogEnd))  );


	// texture filtering
	// NOTE: lensflare code has already set texture filtering to linear. That's
	// a good default.
	switch(m_iTextureFilter)
	{
		case 0:
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_POINT );
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_POINT );
			break;
		case 1:
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_LINEAR );
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_LINEAR );
			break;
		case 2:
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_ANISOTROPIC );
			m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_ANISOTROPIC );
			break;
	}



	// Hier wordt de frame getekend!!! /////////////////////////////////////////////
    // Begin the scene 
    if( SUCCEEDED( m_pd3dDevice->BeginScene() ) )
    {

		// set the camera
		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_VIEW, &m_matView );


		// PANORAMA ///////////////////
		// NOTE: The windsock's black/white material fucks up because of
		// D3DXDrawSprite3D() in RenderScenery(). It is probably a lighting
		// or material issue and because the default scenery textures are black.
		// But we will not use the windsock with a panoramic scenery anyway.
		if (g_bPanorama)
			RenderScenery();
		///////////////////////////////



		// set the world matrix for the terrain, then render it
		// the terrain
		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTerrainMatrix );
		if ( !g_bPanorama ) {
			m_pTerrainObject->Render( m_pd3dDevice );
		}

		
		// set the world matrix for the sky, then render it
		// the sky
		if (g_bShowSkyFile) {
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matSkyMatrix );

			// need CW cull mode to see inside of sky sphere
			if (!m_bCull)
				m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_NONE );
			else
				m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_CW );

		
			// TEST:
			// How can we make the sky turn dark and the lights out???
			// How do we simulate day and night???
			// Well, one way would be to use black fog. Another to use SetMaterialData()
			// to set all material colors in x files. Another to use lighter textures.
			// Another to increase the monitor's gamma (can be done programmatically).
			//m_pd3dDevice->LightEnable( 0, FALSE );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, FALSE );

			// need to set directional light upward or else sky will be black			
			ZeroMemory( &g_Light2, sizeof(D3DLIGHT7) );
			
			// Set up for a white point light.
			// Why can't we get blackness. Is it because of the materials???
			// Yes, to get a black sky we must set the emissive color in the x file Material.
			// Lighting is determined by the combination of Light, Material, Normals.
			// Use SetMaterialData() for that.
			g_Light2.dltType =  D3DLIGHT_POINT; //D3DLIGHT_DIRECTIONAL;
			g_Light2.dcvDiffuse.r = 1.0f;
			g_Light2.dcvDiffuse.g = 1.0f;
			g_Light2.dcvDiffuse.b = 1.0f;
			g_Light2.dcvAmbient.r = 1.0f;
			g_Light2.dcvAmbient.g = 1.0f;
			g_Light2.dcvAmbient.b = 1.0f;
			g_Light2.dcvSpecular.r = 1.0f;
			g_Light2.dcvSpecular.g = 1.0f;
			g_Light2.dcvSpecular.b = 1.0f;
 
			// Position it high in the scene, and behind the viewer.
			// (Remember, these coordinates are in world space, so
			// the "viewer" could be anywhere in world space, too. 
			// For the purposes of this example, assume the viewer
			// is at the origin of world space.)
			g_Light2.dvPosition.x = 0.0f;
			g_Light2.dvPosition.y = 1000.0f;
			g_Light2.dvPosition.z = -100.0f;
 
			// Don't attenuate.
			g_Light2.dvAttenuation0 = 1.0f; 
			g_Light2.dvRange = D3DLIGHT_RANGE_MAX;


			HRESULT hr = m_pd3dDevice->SetLight( 0, &g_Light2 );
			if (FAILED(hr)) {
				//MessageBeep(-1);
			}

			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, 0xffffffff );

//			// also need more ambient light
//			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, 0x00FFFFFF );
//			//
//			// let's make it depend on Sun Elevation
//			// 0 min, 90 max, 180 min
//			DWORD dwSkyLight = 0x00FFFFFF;
//			//if ( m_iSunElevation > 0 && m_iSunElevation < 180 ) {
//			//	dwSkyLight = D3DRGB( (float)sinf(D3DXToRadian(m_iSunElevation)),
//			//						 (float)sinf(D3DXToRadian(m_iSunElevation)),
//			//						 (float)sinf(D3DXToRadian(m_iSunElevation)) ); 
//			//} else {
//			//	dwSkyLight = 0x000000;
//			//}
//			// nope: let's make it depend on Sun Intensity
//			if ( (m_iSunIntensity+100) < 255 ) {
//				dwSkyLight = RGB_MAKE( m_iSunIntensity+100,
//								       m_iSunIntensity+100,
//								       m_iSunIntensity+100 );
//			}
//
//			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, dwSkyLight );
			
			// NOTE: by setting D3DRENDERSTATE_ZWRITEENABLE to false we will no
			// longer fly through the sky.
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, FALSE );
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZWRITEENABLE, FALSE );

			// render sky
			if ( !g_bPanorama ) {
				m_pSkyObject->Render( m_pd3dDevice );
			}

			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, TRUE );
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZWRITEENABLE, TRUE );
			

			// reset
			m_pd3dDevice->SetLight( 0, &g_Light1 ); // set light back
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, 0x00343434 ); // restore ambient light
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, dwCullMode); // restore old cull mode
		}

		// NOTE: The windsock's black/white material fucks up because of
		// D3DXDrawSprite3D() in RenderScenery(). It is probably a lighting
		// or material issue and because the default scenery textures are black.
		// But we will not use the windsock with a panoramic scenery anyway.
		//
		// set the world matrix for the windsock, then render it
		// the windsock
		if (g_bWindsock) {
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matWindsockMatrix );
			m_pWindsockObject->Render( m_pd3dDevice );
		}

		// set the world matrix for the helipad, then render it
		// the helipad
		if (g_bHelipad) {
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matHelipadMatrix );
			m_pHelipadObject->Render( m_pd3dDevice );
		}

		// set the world matrix for the runway, then render it
		// the runway
		if (g_bRunway) {
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matRunwayMatrix );
			m_pRunwayObject->Render( m_pd3dDevice );
		}

		// set the world matrix for the field, then render it
		// the field
		if (g_bField) {
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matFieldMatrix );
			m_pFieldObject->Render( m_pd3dDevice );
		}


		// set the world matrix for the trees, then render them
		// NOTE NOTE NOTE: DrawTrees() and RenderLensFlare() both cause the heli
		// to go invisible every now and then. When we do DrawTrees() after drawing the
		// heli, DrawTrees() does not cause that behaviour. This is no option however
		// because we must draw trees before heli to see trees through rotor blades.
		// It probably has to do with how the textures for these functions were created
		// using D3DTEXTR_TRANSPARENTBLACK. Or probably with the renderstates they set.
		// FIXIT!!!
		// DONE: it was caused by: SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_SELECTARG1 );
		// in DrawTrees(). We must restore that state to D3DTOP_DISABLE. Same goes for
		// RenderLensFlare().
		// NONO: reset to D3DTOP_MODULATE.
		//
		//for (int i=0; i<NUM_TREES; i++) {
		//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTreeMatrix[i] );
		//	m_pTreeObject[i]->Render( m_pd3dDevice );
		//}
		//
		// the trees
		if (g_bTrees) {
			DrawTrees();
			if (m_bDrawShadow)
				DrawTreeShadows();		
		}



		// pilot position marks
		if (m_bShowPPMarks) {
			if (g_bShowPP1Mark) {
				m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matPP1Matrix );
				m_pPP1Object->Render( m_pd3dDevice );
			}

			if (g_bShowPP2Mark) {
				m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matPP2Matrix );
				m_pPP2Object->Render( m_pd3dDevice );
			}

			if (g_bShowPP3Mark) {
				m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matPP3Matrix );
				m_pPP3Object->Render( m_pd3dDevice );
			}

			if (g_bShowPP4Mark) {
				m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matPP4Matrix );
				m_pPP4Object->Render( m_pd3dDevice );
			}
		}



		// shadow
		// NOTE: do before lensflare
		//if ( m_bDrawShadow )
			RenderPlanarShadow();
		
		// lensflare
		if (g_bCapsLensFlare && g_bDrawLensFlare)
			RenderLensFlare();

		// restore old cull mode (lensflare has changed it)
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, dwCullMode );



		// particle system
		// NOTE: much nicer when done after heli is drawn
		// But then we won't see smoke through red heli's rotor disc
		// TODO: find a kludge
		// DONE: no kludge: set D3DRENDERSTATE_ZENABLE to false before drawing smoke
		// Well, that is only an option with panoramic scenery with no objects in the world.
		// Otherwise, the smoke will be visible even if the heli is behind the object.
		if (m_bExhaustSmoke)
			RenderExhaustSmoke();



		// heli pad (boxes) /////////////////////////////////////////
		if (g_bShowBoxes) {
			D3DMATERIAL7 mtrl;
			D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, 1.0f );
			m_pd3dDevice->SetMaterial( &mtrl );
		
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, 0x00606060 );
			switch(g_iTexHeliPad)
			{
				// WRONG: Access Violation in Debug Mode (F5)	
				//case 1: m_pd3dDevice->SetTexture(0, g_pTexHeliPad1); break;
				//case 2: m_pd3dDevice->SetTexture(0, g_pTexHeliPad2); break;
				case 1: m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface("helipad1.bmp") ); break;
				case 2: m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface("C:\\Windows\\Temp\\mdl.bmp") ); break;
			}

			for (int i=0; i<4; i++) {
				m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyPad[i].mat );	
				g_pPad[i]->Draw();
			}
			//if (g_bShowBox1) {
			//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyPad[0].mat );	
			//	g_pPad[0]->Draw();
			//}
			//if (g_bShowBox2) {
			//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyPad[1].mat );	
			//	g_pPad[1]->Draw();
			//}
			//if (g_bShowBox3) {
			//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyPad[2].mat );	
			//	g_pPad[2]->Draw();
			//}
			//if (g_bShowBox4) {
			//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyPad[3].mat );	
			//	g_pPad[3]->Draw();
			//}


			m_pd3dDevice->SetTexture(0, NULL);
		}
		/////////////////////////////////////////////////////////////











		
		// Draw Heli //////////////////////////////////////////////////////////////////		
		// heli last to see sun through rotor disc
		// TODO: with lensflare we need this to draw copter right
		//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_DIFFUSE );
		// Oorzaak: lensflare doet: g_pD3DDevice->SetTexture(0, m_pSourceTexture);		
		// DONE: Maar dit is beter want bovenstaande schakelt textures van file object uit
		g_pD3DDevice->SetTexture(0, NULL);

		DWORD dwHeliLight = 0x00343434;//0x00404040;
		dwHeliLight = RGB_MAKE( m_iSunIntensity,
						        m_iSunIntensity,
						        m_iSunIntensity );

		m_pd3dDevice->SetRenderState(  D3DRENDERSTATE_AMBIENT, dwHeliLight );

/*
		// we want a Blacksphere Airlines decal on our heli
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ADDRESS, D3DTADDRESS_CLAMP );
		
		// scale decal to make it a bit smaller
		// transleren heeft geen effect: je kunt niet de decal verplaatsen
		D3DMATRIX matScale;
		D3DUtil_SetScaleMatrix(matScale, 2.0f, 2.0f, 0.0f);		
		D3DMATRIX matTrans;
		D3DUtil_SetTranslateMatrix(matTrans, 2.0f, 3.0f, 0.0f);
		D3DMATRIX matAll;
		D3DMath_MatrixMultiply(matAll, matScale, matTrans);
	

		// enable and apply texture coordinate transformation 
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT2);
		m_pd3dDevice->SetTransform(D3DTRANSFORMSTATE_TEXTURE0, &matAll);
*/
		// kludge for the cobra which isn't drawing right in CULL_CCW
		if (g_bCobra)
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_NONE );

		//EnterCriticalSection(&GlobalCriticalSection2);

		// set the world matrix for the object, then render it
		// main object (heli)
		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matFileObjectMatrix );
        m_pFileObject->Render( m_pd3dDevice );
		
		//LeaveCriticalSection(&GlobalCriticalSection2);

		// end cobra kludge
		if (g_bCobra)
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_CCW );


		// restore old texture values
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ADDRESS, D3DTADDRESS_WRAP );
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
		///////////////////////////////////////////////////////////////////////////////////////




		
		// proxy //////////////////////////////////////////////////////////////
		if ( GetKeyState(VK_SCROLL) & 0x01 ) {
			RenderProxy();
		}
		///////////////////////////////////////////////////////////////////////




		
		// particle system
		// NOTE: much nicer when done after heli is drawn
		// But then we won't see smoke through heli's rotor disc
		// TODO: find a kludge
		// DONE: no kludge: set D3DRENDERSTATE_ZENABLE to false before drawing smoke
		// Well, that is only an option with panoramic scenery with no objects in the world.
		// Otherwise, the smoke will be visible even if the heli is behind the object.
		//if (m_bExhaustSmoke)
		//	RenderExhaustSmoke();		


		
		// SHADOW VOLUME:
		// Looks good: we get the dark rectangle over the scene and can see the
		// shadow volume being drawn. It does not work however: in hardware mode we
		// do not have stencil buffer support. In software mode there is something 
		// going wrong with CreateStencilBuffer() which causes an empty scene. Fix it!!!
		// DONE: we were calling the wrong EnumZBufferFormatsCallback().
		// Mmm, we kunnen het een en ander aan stencil mask zien in softwate mode
		// maar niet veel. Gotto check on faster system.
		//
		// Render the shadow volume into the stencil buffer, then add it into
        // the scene
		if ( m_bCapsShadow && m_bDrawShadowVolume ) {
			RenderShadow();
			DrawShadow();
		}



		// TEST:
		// make real mrotor follow proxy mrotor
		// NOTE: this code can also come in handy when we want to draw a disc for high
		// rotor speed and a normal rotor for low rotor speed. These should be different
		// Frames in the .X file.
//		{
//			//D3DXMATRIX matRot;
//			//D3DXMatrixRotationY( &matRot, g_PI_DIV_2  );
//			//D3DMath_MatrixMultiply( g_matMRotor, matRot, g_matMRotor ); // order is crucial!!!
//			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matMRotor );
//			//m_pPP1Object->Render( m_pd3dDevice ); // works OK so we can in the future
//			// always load the rotor as a separate .X file (although that is not a clean
//			// approach: total heli should be in one .X file)
//			
//			CD3DFileObject* pObject = m_pFileObject->FindObject( "frame-mrotor" );
//			if( pObject )
//			{	
//				D3DMATRIX* pmat = pObject->GetMatrix();
//				D3DUtil_SetIdentityMatrix(*pmat);
//				//*pmat = m_matFileObjectMatrix;
//
//				m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, TRUE );
//				//pObject->SetMatrix(&m_matIdentityMatrix);
//				pObject->Render(m_pd3dDevice, true);
//				m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, FALSE );
//			}
//		}


	











		
		// TODO: zet alle info bij elkaar en laat slechts die items zien die de user
		// heeft gecheckt in een prop page
		// show a message, this must be done in Render()
		if (g_bShowValues) {
			OutputText( 10, 0,  msg1);
			OutputText( 10, 12, msg2);
			OutputText( 10, 24, msg3);
			//OutputText( 10, 36, msg4);
			OutputText( 10, 48, msg4);
			OutputText( 10, 60, msg5);
			OutputText( 10, 72, msg6);
			//OutputText( 10, 84, msg2);
			OutputText( 10, 96, msg7);
			OutputText( 10, 108, msg8);
			OutputText( 10, 120, msg9);
			//OutputText( 10, 132, msg6);
			OutputText( 10, 144, msg10);
			OutputText( 10, 156, msg11);
			OutputText( 10, 168, msg12);
		}


		// show flight info, this must be done in Render()
		if (m_bShowFlightInfo)
			ShowFlightInfo();

		// VxD and WDM
		if (m_bShowChannels)
			ShowChannels();


		
		// TEST:
		// FSWindow ///////////////////////////////////////////////////////////
		// Try to draw RTConfig in fullscreen mode:
		// * slow GDI blit to DDraw surface (don't work)
		// * use m_pDD->FlipToGDISurface(); ???
		// * fast D3D blit using the bitmap as a texture on a rectangle drawn by DrawPrimitive()
		// But even then: on XP windows have rounded top corners so that would require
		// additional operations to make these draw right. Let's call it off...
//		if (g_hWndTools) {
//			HDC hdcDest, hdcSource;
//			//hdcSource = GetWindowDC(g_hWndTools);
//			m_pddsRenderTarget->GetDC(&hdcDest);
//			//hdcDest = GetDC(NULL);
//			//hdcSource = CreateCompatibleDC(hdcDest);
//			hdcSource = CreateCompatibleDC(NULL);
//
//			HBITMAP hbm = CreateBMPFromWindow(g_hWndTools);
//			SelectObject(hdcSource, hbm);
//
//			// Hell, why can't we blit the bitmap to the rendertarget (backbuffer)???
//			// We can blit the bitmap to the desktop, and we can blit blackness to the 
//			// backbuffer. But not the bitmap to the backbuffer...
//			//BitBlt(hdcDest, 0, 0, 200, 100, hdcSource, 0, 0, WHITENESS/*BLACKNESS*/);
//			//BitBlt(hdcDest, 0, 0, 600, 400, hdcSource, 0, 0, WHITENESS/*BLACKNESS*/);
//			BitBlt(hdcDest, 0, 0, 600, 400, hdcSource, 0, 0, SRCCOPY/*BLACKNESS*/);
//
//
//			
//			// fast D3D blit //////////////////
//			// create surface
//			DDSURFACEDESC2 ddsd;
//			LPDIRECTDRAWSURFACE7 pddsCoverSurface;
//
//			ZeroMemory( &ddsd, sizeof(DDSURFACEDESC2) );
//			ddsd.dwSize         = sizeof(DDSURFACEDESC2);
//			ddsd.dwFlags        = DDSD_CAPS | DDSD_HEIGHT | DDSD_WIDTH;
//			ddsd.ddsCaps.dwCaps = DDSCAPS_TEXTURE;
//			ddsd.dwHeight = 512;
//			ddsd.dwWidth  = 512;
//
//			HRESULT hr = m_pDD->CreateSurface( &ddsd, &pddsCoverSurface, NULL );
//			DXVERIFY (hr);
//			pddsCoverSurface->Restore();
//
//			// blit window bitmap to pddsCoverSurface
//			// Hell, how can we get a hbm to a texture
//			// We can load a texture from file so why not from a handle
//			// Well, we should first save the bitmap to memory and then use D3DXLoadTextureFromMemory()
//			// But saving a bitmap to mem requires intricate code, cf. taking screenshots.
//			pddsCoverSurface->GetDC(&hdcDest);
//			BitBlt(hdcDest, 0, 0, 200, 100, hdcSource, 0, 0, SRCCOPY);
//			//pddsCoverSurface->Restore();
//
//			//BltAlphaFactor3( m_pd3dDevice, g_pTexHeliPad1, 
//			//	0, 0, 200, 100, 255, false );
//			BltAlphaFactor3( m_pd3dDevice, pddsCoverSurface, 
//				0, 0, 200, 100, 255, false );
//		
//
//			pddsCoverSurface->ReleaseDC(hdcDest);
//			pddsCoverSurface->Release();
//			/////////////////////////////////////
//
//			
//			//ReleaseDC(g_hWndTools, hdcSource);
//			//ReleaseDC(NULL, hdcDest);
//			m_pddsRenderTarget->ReleaseDC(hdcDest);			
//			DeleteDC(hdcSource);
//			//ReleaseDC(NULL, hdcSource);
//
//			DeleteObject(hbm);
//		}
		/////////////////////////////////////////////////////////////



        // End the scene.
        m_pd3dDevice->EndScene();
    }



    return S_OK;
}




//-----------------------------------------------------------------------------
// Name: InitDeviceObjects()
// Desc: Initialize scene objects.
// Note: Only now do we have m_hWnd (See: flow of control)
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::InitDeviceObjects()
{
	
	// Start up full screen by faking a menu command
	// if reg read fullscreen == 1
	// CAN'T DO IT HERE: CD3DApplication::m_bActive or CD3DApplication::m_bReady
	// are still false at this point (See: CD3DApplication::MsgProc())
	//SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_TOGGLEFULLSCREEN,0), 0L);
	

	// NOTE: only now do we have m_hWnd ///////////////////////////////////////
	// This is the flow of control:
	// * WinMain() calls:
	//    * CD3DApplication::Create() calls:
	//       * CD3DApplication::OneTimeSceneInit()
	//		 * RegisterClass() and CreateWindow() (we have m_hWnd)
	//       * CD3DApplication::Initialize3DEnvironment() calls:
	//          * CD3DFramework7::Initialize()
	//          * CD3DApplication::InitDeviceObjects()
	//    * CD3DApplication::Run() calls:
	//       * CD3DApplication::Render3DEnvironment() calls:
	//          * CD3DApplication::FrameMove()
	//          * CD3DApplication::Render()
	///////////////////////////////////////////////////////////////////////////



	// Vanaf hier hebben we een window (handle)
	// BUT NOTE: CD3DApplication::m_bActive or CD3DApplication::m_bReady
	// are still false at this point (See: CD3DApplication::MsgProc())
	HRESULT hr;
	HMENU hmenu = GetMenu(m_hWnd);


	// tell registry we are running
	g_hWndSikorsky = (DWORD)m_hWnd;
	RegistryWrite4();


	// Create Status bar ///////////////////////////////////////////////////////////
	// NOTE: during every WM_SIZE we get here (because a new backbuffer is created)
	// NOTE: ShowWindow() don't cut it: got to destroy and create
	// TODO: must we really create and destroy? Yes, because we still have the main
	// window as the render window. The client window is in fact a dummy window we just
	// need to get the proper client edges.
	if ( g_bStatusBar ) {
		if (g_hwndStatus)
			DestroyWindow( g_hwndStatus );
		if (g_hwndClient)
			DestroyWindow( g_hwndClient );

		DoCreateStatusWindow(m_hWnd, g_hinst);
		DoCreateClientWindow(m_hWnd, g_hinst);
	} else {
		if (g_hwndStatus)
			DestroyWindow( g_hwndStatus );
		if (g_hwndClient)
			DestroyWindow( g_hwndClient );

		DoCreateClientWindow(m_hWnd, g_hinst);
	}
	////////////////////////////////////////////////////////////////////////////////


	// Create Toolbar ///////////////////////////////////////////////////////////
	// NOTE: during every WM_SIZE we get here (because a new backbuffer is created)
	// However, unlike with the status and client window we do not need to create
	// and destroy every time we get here. We create the toolbar only once.
	// If we create and destroy we get an ugly white client rectangle from the
	// main window drawn over the toolbar at WM_SIZE.
	// TODO: We must however make sure the toolbar sizes with the main window.
	// Well it sizes automatically but not always correctly.
	// TODO: Also, when in fullscreen we should hide the toolbar. Or should we? 
	// The buttons do work, however they get blitted over by the renderer.
	// TODO: Don't use Rebar/Toolbar control but a Dockable Toolwindow/Toolbar.
	// Use resource IDD_TOOLWINDOW, make it dockable, put Toolbar in by making
	// it the owner of the Toolbar.
	if ( g_bToolBar ) {
		static bool firsttime = true;
		if (firsttime) {
			firsttime = false;
			//g_hwndReBar = DoCreateRebar(m_hWnd);
			//g_hwndToolBar = DoCreateToolbar(g_hwndReBar, g_hinst);
			
			// TEST: toolbar
			// It looks quite OK what we have got: Toolbar in Toolwindow. Only need an extra gray
			// window under the menu where the Toolbar can dock in. And then of course all the code
			// for docking, dragging, sizing, etc.
			g_hwndDockableToolwindow = CreateDialog( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_TOOLWINDOW), m_hWnd,
								   (DLGPROC)ToolwindowProc );
			
			g_hwndToolBar = DoCreateToolbar(g_hwndDockableToolwindow, g_hinst);
		}
	}
	//MessageBeep(-1);
	/////////////////////////////////////////////////////////////////////////////
	


	

	// do shareware notice ////////////
	// Note: it would be better to do this after splash screen but when we do that
	// we won't be visible: hidden behind the splash screen
	// TODO: find a kludge
	//if (g_bShareWareCheck) {
	//	g_bShareWareCheck = false;

		//Pause(true);
		//DoShareWareNotice( m_hWnd );
		//Pause(false);
	//}
	///////////////////////////////////




	// For Trees //////////////////////
	// Check the alpha test caps of the device. (The alpha test offers a 
    // performance boost to not render pixels less than some alpha threshold.)
    DWORD dwAlphaCaps = m_pDeviceInfo->ddDeviceDesc.dpcTriCaps.dwAlphaCmpCaps;
    m_bUseAlphaTest = ( dwAlphaCaps & D3DPCMPCAPS_GREATEREQUAL );
	///////////////////////////////////




	// do all this stuff in FrameMove()
	// move and resize window: can't use MoveWindow() as that won't change
	// the backbuffer surface's position and dimensions, only the window's
	//MoveWindow(m_hWnd, 100, 100, 800, 600, TRUE);
	// got to use SendMessage()
	// Huh?! Why don't it work?
	// Answer: because CD3DApplication::m_bActive or CD3DApplication::m_bReady
	// are still false at this point (See: CD3DApplication::MsgProc())
	// But why can't we just get the window to move???
	// Because you can't do that with SendMessage(): WM are sent after a size or move
	// They don't DO the size or move
	//SendMessage( m_hWnd, WM_MOVE, 0, 0 );
	//SendMessage( m_hWnd, WM_SIZE, 300, 200 );



	// SHADOW VOLUME:
	// Hebben we ueberhaupt een stencil buffer nodig voor schaduw volume?
	// Zie: RM shadow
	// Ja, want dat sample zet alleen een schaduw op een flat plane
	// niet op objecten in de scene. Het gebruikt dus fake, projected shadows.
	// Yes!!! Stencil buffer is needed for shadow volume mask.
	
	// NOTE: We hebben drie shaduw methodes geprobeerd:
	// 1. Shadow volumes (using the stencil buffer). Developed by Frank Crow
	//    (Shadow Algorithms for Computer Graphics, Proceedings of SIGGRAPH, 1977,
	//    pp. 242-248).
	//    (See: Practical and Robust Stenciled Shadow Volumes for Hardware-Accelerated
	//    Rendering, Cass Everitt and Mark J. Kilgard).
	// 2. Shadow mapping (shadow texture mapping using the depth buffer generated by
	//    rendering the scene from the light's point of view). Developed by Lance
	//    Williams (Casting Curved Shadows on Curved Surfaces, Proceedings of
	//    SIGGRAPH, 1978, pp. 270-274).
	//    (See: Hardware Shadow Mapping, Cass Everitt, Ashu Rege, Cem Cebenoyan).
	// 3. Planar shadows (fake, projected shadows). Developed by Jim Blinn 
	//	  (Jim Blinn’s corner: Me and my (fake) shadow. IEEE Computer Graphics &
	//	  Applications 8, 1 (January, 1988), 82–86.
	//    (See: Shadows by Tom Hammersley).
	//
	// De laatste is simpel en werkt. Kan alleen geen schaduw op objecten casten.
	// De eerste moet ook te doen zijn en schijnt volgens Carmack "the way to go" te
	// zijn (See: CarmackOnShadowVolumes.txt).
	// Toch zal het een pijnlijke excercitie worden. We moeten eerst face connectivity
	// information genereren. Dan moeten we de silhouette edges vinden.
	// (De .X file moet een gesloten structuur hebben.) Dan de shadow volume construeren.
	// Dan nog twee problemen met shadow volumes:
	// 1. The first is that no matter what finite distance we extrude an object's silhouette
	// away from a light source, it is still possible that it is not far enough to cast a
	// shadow on every object in the scene that should intersect the shadow volume.
	// Fortunately, this problem can be elegantly solved by using a special projection
	// matrix and extruding shadow volumes all the way to infinity. 
	// 2. The second problem shows up when the camera lies inside the shadow volume or the
	// shadow volume is clipped by the near plane. Either of these occurrences can leave
	// incorrect values in the stencil buffer causing the wrong surfaces to be illuminated.
	// The solution to this problem is to add caps to the shadow volume geometry, making it
	// a closed surface, and using different stencil operations.
	//
	//
	//
	// Create the stencil buffer ///////////////////////////////////////////////
	// and reset the viewport which gets trashed in the process
	// Helaas: we hebben (nog) geen hardware device capable of stencilbuffering
	// ConfirmDevice() checkt elk enumerated device op stencil buffer capaciteit
	// en vindt dat slechts in RGB emulation devices
	// Oplossing: koop een Voodoo5 5500
	// We kunnen wel alvast werken aan schaduw implementatie
	// We laten m_fnConfirmDevice = NULL; zodat alle devices worden enumerated
	// en we checken hier pas of schaduw mogelijk is
	// als dat niet zo is dan behouden we de hardware device en doen we gewoon
	// geen schaduw
    if ( (m_pDeviceInfo->ddDeviceDesc.dwStencilCaps) &&
		 (m_pDeviceInfo->ddDeviceDesc.dwVertexProcessingCaps & D3DVTXPCAPS_POSITIONALLIGHTS) )
	{
		m_bCapsShadow = TRUE;
		EnableMenuItem (hmenu, ID_OPTIONS_SHADOWVOLUME, MF_ENABLED);
	} else {
		m_bCapsShadow = FALSE;
		EnableMenuItem (hmenu, ID_OPTIONS_SHADOWVOLUME, MF_GRAYED);
	}

	if (m_bCapsShadow)
	{
		// NOTE: if a bit-compatibele stencil buffer can't be found we get an
		// error message saying user should switch to 16 or 32 bit mode, or vice versa.
		// So let's don't do stencil buffer creation as we still don't do shadow volumes
		//if( FAILED( hr = CreateStencilBuffer() ) ) return hr;
	}
	////////////////////////////////////////////////////////////////////////////



/*	// Dit is code for shadow map methode???
	// Create buffers for shadow ///////////////////////////////////////////
	// Ziet er goed uit: Als we extra buffers aanvragen gaan we op
	// 800x600 al naar software vanwege te weinig geheugen: een teken dat we 
	// dus extra buffers hebben die we kunnen gaan gebruiken voor de light view 
	// en shadow map
	if ( m_pDeviceInfo->bWindowed ) {
		//MessageBox(NULL,"win","QQQ",MB_OK);
		if( FAILED( hr = CreateWindowedShadowBuffers() ) )
			return hr;
	} else {
		//MessageBox(NULL,"fs","QQQ",MB_OK);
		if( FAILED( hr = CreateFullscreenShadowBuffers( 
										&m_pDeviceInfo->ddsdFullscreenMode ) ) )
			return hr;
	}

	/////////////////////////////////////////////////////////////////////////
*/




	// Create the DInput object ////////////////////////////////////////////////
    if( FAILED( CreateDInput( m_hWnd ) ) )
    {
        DestroyWindow( m_hWnd );
        return FALSE;
    }

    // Create a Keyboard device (this is our default input device)
    if( FAILED( CreateInputDevice( m_hWnd, GUID_SysKeyboard, &c_dfDIKeyboard,
                                   DISCL_NONEXCLUSIVE | DISCL_FOREGROUND ) ) )
    {
        DestroyWindow( m_hWnd );
        return FALSE;
    }
	////////////////////////////////////////////////////////////////////////////




	// Init DirectSound ////////////////////////////////////////////////////////
    if( FAILED( InitDirectSound( m_hWnd ) ) )
    {
        MessageBox( NULL, "Error initializing DirectSound.", 
                            "QQQ", MB_OK | MB_ICONERROR );
		DestroyWindow( m_hWnd );
        return FALSE;
    }

	//StopBuffer();
	//LoadWaveFile( m_hWnd, "heli.wav" );

	StopBuffer();

	// load .wav file
	static TCHAR g_strWavFileName[512];

	strcpy( g_strWavFileName, g_strHeliFilePath );
//	if ( strstr( g_strWavFileName, ".x" ) )
//		strstr( g_strWavFileName, ".x" )[0] = '\0'; // clip extension
//	if ( strstr( g_strWavFileName, ".X" ) )
//		strstr( g_strWavFileName, ".X" )[0] = '\0'; // clip extension

	// find extension (case-insensitive and from last instance of '.') and clip extension
	if ( strrchr(g_strWavFileName, '.') ) {
		if ( 0 == _stricmp( strrchr(g_strWavFileName, '.'), ".x") )
			strrchr(g_strWavFileName, '.')[0] = '\0';
		else
			MessageBox(NULL, "No .x or .X found in file name", "QQQ", MB_OK);
	} else {
		MessageBox(NULL, "No . found in file name", "QQQ", MB_OK);
	}

	strcat( g_strWavFileName, ".wav" );


	// make sure we are in the \media dir
	// NOTE: g_szRCSIMMediaPath has been made an absolute path in OneTimeSceneInit()
	SetCurrentDirectory( g_szRCSIMMediaPath );
	
	LoadWaveFile( m_hWnd, g_strWavFileName );

//	if (g_bUseAuthenticHeliSound) {
//		if (g_bBell || g_bCougar) {
//			LoadWaveFile( m_hWnd, "bell.wav" );
//		} else if (g_bCobra) {
//			LoadWaveFile( m_hWnd, "cobra.wav" );
//		} else {
//			//LoadWaveFile( m_hWnd, "heli.wav" );
//			LoadWaveFile( m_hWnd, g_strWavFileName );
//		}
//	} else {
//		//LoadWaveFile( m_hWnd, "heli.wav" );
//		LoadWaveFile( m_hWnd, g_strWavFileName );
//	}


	// Set the options in the DirectSound buffer
	// Note: MinDistance is most important factor. See docs.  
    SetParameters( 1.0f, 1.0f, 100.0f, 800.0f );

	//g_pDSBuffer->SetVolume( DSBVOLUME_MIN );
	SetVolume( DSBVOLUME_MIN );
	g_uCountBeforeMaxVolume = 0;

	// Play the buffer looped
    //if( FAILED( hr = PlayBuffer( TRUE ) ) )
    //    return hr;
	PlayBuffer(TRUE);
	////////////////////////////////////////////////////////////////////////////


	// DirectSound2 ///////////////////////
	StopBuffer2( soundCrash );
	StopBuffer2( soundCrash2 );
	StopBuffer2( soundWarning );

	LoadWaveFile2( m_hWnd, "crash.wav", soundCrash );
	LoadWaveFile2( m_hWnd, "crash2.wav", soundCrash2 );
	LoadWaveFile2( m_hWnd, "warning.wav", soundWarning );


	// used in altitude prop page
	GetCurrentDirectory( sizeof(g_strSoundFilePath), g_strSoundFilePath );
	strcat( g_strSoundFilePath, "\\warning.wav" );

	
	// Set the options in the DirectSound buffer 
    SetParameters2( soundCrash, 1.0f, 1.0f, 100.0f, 800.0f );
	SetParameters2( soundCrash2, 1.0f, 1.0f, 100.0f, 800.0f );
	SetParameters2( soundWarning, 1.0f, 1.0f, 100.0f, 800.0f );

	//PlayBuffer2( soundCrash, TRUE );

	SetVolume2( soundWarning, DSBVOLUME_MIN );
	PlayBuffer2( soundWarning, TRUE );

	// internal heli sounds
//	StopBuffer2( soundBell_i );
//	StopBuffer2( soundCobra_i );
//	LoadWaveFile2( m_hWnd, "bell_i.wav", soundBell_i );
//	LoadWaveFile2( m_hWnd, "cobra_i.wav", soundCobra_i );
//	PlayBuffer2( soundBell_i, TRUE );
//	PlayBuffer2( soundCobra_i, TRUE );
//	SetVolume2( soundBell_i, DSBVOLUME_MIN );
//	SetVolume2( soundCobra_i, DSBVOLUME_MIN );
	///////////////////////////////////////




    // Create CLensFlare //////////////////////////////////////////////////////
    if(!(g_pLensFlare = new CLensFlare))
        return E_OUTOFMEMORY;
	
    if(FAILED(hr = g_pLensFlare->Initialize(6, 95.0f)))
        return hr;

    g_pLensFlare->SetLightColor(g_colorLight);
    g_pLensFlare->SetLightPosition(g_vecLight);


	// BLOODY WILD POINTERS
	// DON'T FORGET TO DO THIS
	g_pD3DDevice = m_pd3dDevice;

	// got our own media path
	strcpy( g_szPath, g_szRCSIMMediaPath );



/*
	// Get media path from registry
	HKEY key;
    g_szPath[0] = '\0';

    if(ERROR_SUCCESS == RegOpenKeyEx(HKEY_LOCAL_MACHINE,
        "Software\\Microsoft\\DirectX", 0, KEY_READ, &key))
    {
        DWORD dwType;
        DWORD dwSize = sizeof(g_szPath);

        if(ERROR_SUCCESS == RegQueryValueEx( key, 
            "DXSDK Samples Path", NULL, &dwType, (BYTE*) g_szPath, &dwSize))
        {
            if(REG_SZ == dwType)
                strcat(g_szPath, "\\D3DX\\Media\\");
            else {
                g_szPath[0] = '\0';
				MessageBox(NULL,"No path for lens flare textures!","QQQ!",MB_OK);
			}
        }

        RegCloseKey(key);
    }
*/



/*
    // Initialize D3DX
    //if(FAILED(hr = D3DXInitialize()))
    //   return hr;
	
	// Note: the following will NOT increment the reference count to the COM object
	// But it totally fucks up our program... Well, not if we don't use D3DX
	// Note 2: By COM rules, when an interface pointer is copied by setting it to 
	// another variable or passing to another object, that copy represents another
	// reference to the object, and therefore the IUnknown::AddRef method of the
	// interface must be called to reflect the change. 
	//g_pD3DDevice = m_pd3dDevice;

	// so like this
	// but then before doing a device change we have to Release() again
	//m_pd3dDevice->AddRef();
*/


    // Grok device caps
    memset(&g_D3DDeviceDesc, 0x00, sizeof(D3DDEVICEDESC7));
    g_pD3DDevice->GetCaps(&g_D3DDeviceDesc);

    if ( (g_D3DDeviceDesc.dpcTriCaps.dwSrcBlendCaps & D3DPBLENDCAPS_ONE) &&
         (g_D3DDeviceDesc.dpcTriCaps.dwDestBlendCaps & D3DPBLENDCAPS_INVSRCCOLOR) &&
         (g_D3DDeviceDesc.dpcTriCaps.dwShadeCaps & D3DPSHADECAPS_ALPHAFLATBLEND) )
	{
		g_bCapsLensFlare = TRUE;
		EnableMenuItem (hmenu, ID_OPTIONS_LENSFLARE, MF_ENABLED);
		EnableMenuItem (hmenu, ID_OPTIONS_SUN,       MF_ENABLED);
		EnableMenuItem (hmenu, ID_OPTIONS_FLARE,     MF_ENABLED);
	} else {
		g_bCapsLensFlare = FALSE;
		EnableMenuItem (hmenu, ID_OPTIONS_LENSFLARE, MF_GRAYED);
		EnableMenuItem (hmenu, ID_OPTIONS_SUN,       MF_GRAYED);
		EnableMenuItem (hmenu, ID_OPTIONS_FLARE,     MF_GRAYED);
	}
	///////////////////////////////////////////////////////////////////////////
	
	
	// Create textures ///////////
	CreateTextures();
	//////////////////////////////

	
	//MessageBeep(MB_ICONEXCLAMATION);


	// PANORAMA //////////////////
	if (g_bLoadScenery) {
		// threadless
		if (g_bShowProgressDialog) {
			// TEST: show progress dialog
			g_hwndProgressDialog = CreateDialog( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
				MAKEINTRESOURCE(IDD_PROGRESS2), m_hWnd,
				(DLGPROC)ProgressProc );		
			SetWindowText(g_hwndProgressDialog, "Loading Scenery...");
		} else {
			ShowWindow( g_hwndPB, SW_SHOW );
		}		
		LoadScenery();

		// threaded
//		/////////////////////////////////////////////////////////////////////
//		// TODO: load scenery in separate thread so user can Cancel
//		// Well, actually that does not make much sense in InitDeviceObjects()
//		// because once the panorama textures are loaded we don't want the user
//		// to Cancel when doing reinit Device
//		//DWORD dwThreadId, dwThrdParam = 1; 
//		//HANDLE hThread;		
//		dwThrdParam = 1;
//		hThread = CreateThread( 
//			NULL,                        // no security attributes 
//			0,                           // use default stack size  
//			ThreadFuncLoadScenery,		 // thread function 
//			&dwThrdParam,                // argument to thread function 
//			0,                           // use default creation flags 
//			&dwThreadId);                // returns the thread identifier 
//		
//		// Check the return value for success.		
//		if (hThread == NULL) 
//			MessageBox(m_hWnd,"CreateThread failed.","QQQ",MB_OK);
//		
//		//SetThreadPriority(hThread, THREAD_PRIORITY_BELOW_NORMAL);
//		/////////////////////////////////////////////////////////////////////
//
//		// Dialog
//		if (g_bShowProgressDialog) {
//			DialogBox( g_hInst, MAKEINTRESOURCE(IDD_PROGRESS2), m_hWnd,
//				(DLGPROC)ProgressProc );
//		} else {
//			// TODO: got to wait for thread here!!! DialogBox() causes this function
//			// to wait, of course, but ShowWindow() returns immediately.
//			ShowWindow( g_hwndPB, SW_SHOW );
//			WaitForSingleObject(hThread, INFINITE);
//		}
//
//		// Close thread
//		CloseHandle(hThread);
//		//////////////////////////////////////////////
	} else {
		// NOTE: We will never load the panoramic scenery from scratch as that may be
		// too frustrating for users on slow systems. Only from OpenSceneryFileDialog()
		// will we load the textures. However, we got to create the textures here already
		// otherwise D3DXCreateTextureFromFile() fails in OpenSceneryFileDialog() as textures
		// can only be created in InitDeviceObjects(). We need not create big textures though:
		// 8x8 will be fine. In OpenSceneryFileDialog() we can then load the full 1024x1024
		// textures.
		for (int i = 0; i < NUM_PAN; i++) {

			D3DXInitialize();
			
//			TCHAR msg[512];
//			strcpy(msg, "Loading ");
//			strcat(msg, g_szPanName[i]);
//			strcat(msg, "...");		
//			
//			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 0|SBT_NOBORDERS, 
//				(LPARAM) msg );
//			SendMessage( g_hwndPB, PBM_SETPOS, (WPARAM)iProgress++, (LPARAM)0 );
			DWORD dwWidth = 8;
			DWORD dwHeight = 8;
			D3DX_SURFACEFORMAT sfPixelFormat = D3DX_SF_R8G8B8;
			
			if( FAILED( hr = D3DXCreateTexture(
				g_pd3dApp->m_pd3dDevice,
				NULL,                   // dwFlags
				&dwWidth,               // width
				&dwHeight,              // height
				&sfPixelFormat,         // surface type
				NULL,                   // pointer to Palette
				&g_ptexPan[i],			// returned pointer to texture
				NULL)))                 // returned number of mipmaps				
			{	
				MessageBox(NULL,"Can't load panorama texture!","QQQ!",MB_OK);
				char errStr[256];
				D3DXGetErrorString(hr, 256, errStr);
				MessageBox(NULL, errStr, "D3DX Error", MB_OK);
			}
		}
	}
	//////////////////////////////





	// Create Particle System /////////////////////////////////////////////////
    // Initialize particle engines:
    // Snow: 50 particles, size=20x20 units at y=10, ground at y=0
    g_pSnowfall = new TSnowfall(m_pd3dDevice, 50, D3DVECTOR(0, 10, 0), 20, 20, 0, "snow.bmp");
    g_pSnowfall->ResetSystem();

    // Smoke: 15 particles, position=(-5, 0, 5), height=10 units
    //g_pSmoke = new TSmoke(m_pd3dDevice, 15, D3DVECTOR(-5, 0, 5), 10, "smoke.bmp");
	g_pSmoke = new TSmoke(m_pd3dDevice, 20, D3DVECTOR(INIT_X, INIT_Y, INIT_X), 10, "smoke.bmp");
    g_pSmoke->ResetSystem();
	///////////////////////////////////////////////////////////////////////////




    // Setup a material
    D3DMATERIAL7 mtrl;
    D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f );
    m_pd3dDevice->SetMaterial( &mtrl );

    // Setup the textures
	// NOTE: we can call this here in InitDeviceObjects() without problems without
	// having to call CreateTextures()...
    D3DTextr_RestoreAllTextures( m_pd3dDevice );

    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE );
    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP,   D3DTOP_MODULATE );
    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_LINEAR );
    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_LINEAR );

	// lensflare 
	// NOTE NOTE NOTE: g_pD3DDevice == m_pd3dDevice
	g_pD3DDevice->SetTextureStageState(0, D3DTSS_MIPFILTER, D3DTFP_POINT);
    g_pD3DDevice->SetTextureStageState(0, D3DTSS_ALPHAOP, D3DTOP_MODULATE);

    
	// other stuff
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DITHERENABLE,     TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SPECULARENABLE,   TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE,		   TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_NORMALIZENORMALS, TRUE );
	

	// Brilliant!!: here we are preparing for blending the grey rotor disks
	// with the terrain to give the impression of rotation and transparency.
	// When using real animated rotors we won't need this.
	// But we can use it for the cockpit windows.
	// Note: the blending used here is color blending. Check the X file for the rotor
	// material.
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_COLORKEYENABLE, TRUE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND,  D3DBLEND_ONE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_ZERO );




    // Set the projection matrix
	// And far- and near clipping plane
    D3DVIEWPORT7 vp;
    m_pd3dDevice->GetViewport(&vp);
    FLOAT fAspect = ((FLOAT)vp.dwHeight) / vp.dwWidth;
	
	// set far clippping plane
	// Note: we'll have a W-Friendly Projection Matrix (fog needs it)
    D3DMATRIX matProj;
    D3DUtil_SetProjectionMatrix( matProj, g_PI/4, fAspect, 1.0f, 2000.0f );
    m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_PROJECTION, &matProj );




 	// a bit of fog
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGENABLE, TRUE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGCOLOR,  m_dwFogColor ); //argb
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGVERTEXMODE, D3DFOG_LINEAR );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGSTART, *((LPDWORD) (&m_fFogStart)) );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_FOGEND,   *((LPDWORD) (&m_fFogEnd))  );



    // Set up the light
    if( m_pDeviceInfo->ddDeviceDesc.dwVertexProcessingCaps &
                                                D3DVTXPCAPS_DIRECTIONALLIGHTS )
    {
        D3DLIGHT7 light;

		// note: D3DUtil_InitLight() sets position and direction to the same vector
		// Zomerochtendzonnetje van rechtsachter op vliegveld The Wings
        D3DUtil_InitLight( light, D3DLIGHT_DIRECTIONAL, -1.0f, -1.0f, 1.0f );

        m_pd3dDevice->SetLight( 0, &light );
        m_pd3dDevice->LightEnable( 0, TRUE );
        m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, TRUE );
        m_pd3dDevice->SetRenderState(  D3DRENDERSTATE_AMBIENT, 0x00343434 );
    }
    else
        m_pd3dDevice->SetRenderState(  D3DRENDERSTATE_AMBIENT, 0x00343434 );


	// ODE proxy
	CreateProxy();


    return S_OK;
}




//-----------------------------------------------------------------------------
// Name: CMyD3DApplication::DeleteDeviceObjects()
// Desc: Called when the app is exitting, or the device is being changed,
//       this function deletes any device dependent objects.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::DeleteDeviceObjects()
{

	//CheckDDrawRefCount();
	// this will destroy all textures AND their DDraw surface
	// now that'll clear up the ref count all right!!!
    D3DTextr_InvalidateAllTextures();
	//CheckDDrawRefCount();

	// TEST:
	// NOTE: Well, safe release my arse: this has to be done if we create the textures
	// with D3DX. But if we don't do that and leave this in we crash at device change. So
	// much for a safe release. This is caused of course because we created the textures
	// with D3DTextr_ functions. D3DTextr_InvalidateAllTextures() takes care of releasing.
	// Doing that again like this causes a crash.
	//SAFE_RELEASE( g_pTexHeliPad1 );

	// waarom kun jij dat niet???
	//D3DXUninitialize();

	//MessageBox(NULL,"device change (or app exit)","QQQ!",MB_OK);

	// je kunt SAFE_RELEASE() gebruiken
	//#define RELEASE(obj) if(obj) { obj->Release(); obj = NULL; } else 0	
    //RELEASE(g_pD3DDevice);
    //RELEASE(g_pD3D);
    //RELEASE(g_pDD);
    //RELEASE(g_pD3DX);

	// Free particle system objects:
    if (g_pSnowfall) { delete g_pSnowfall; g_pSnowfall = NULL; }
    if (g_pSmoke)    { delete g_pSmoke;    g_pSmoke    = NULL; }


	SAFE_RELEASE( m_pddsDepthBuffer );

	SAFE_RELEASE( m_pddsBackBufferShadow );
	SAFE_RELEASE( m_pddsZBufferShadow );

	SAFE_RELEASE( g_ptexPanorama1 );
	SAFE_RELEASE( g_ptexPanorama2 );

	for (int i = 0; i < NUM_PAN; i++) {
		SAFE_RELEASE( g_ptexPan[i] );
	}


	// ODE proxy
	DestroyProxy();

	

	return S_OK;
}




//-----------------------------------------------------------------------------
// Name: FinalCleanup()
// Desc: Called before the app exits, this function gives the app the chance
//       to cleanup after itself.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::FinalCleanup()
{
    SAFE_DELETE( m_pFileObject );
	SAFE_DELETE( m_pTerrainObject );
	SAFE_DELETE( m_pSkyObject );
	SAFE_DELETE( m_pWindsockObject );

	SAFE_DELETE( m_pPP1Object );
	SAFE_DELETE( m_pPP2Object );
	SAFE_DELETE( m_pPP3Object );
	SAFE_DELETE( m_pPP4Object );

	// ODE dynamics
	DestroyDynamics();

	// mesh vertex arrays
	delete[] g_pVerticesOrigSkid;
	delete[] g_pVerticesOrigMRotor;
	delete[] g_pVerticesOrigTRotor;
	delete[] g_pVerticesOrigShaft;

    return S_OK;
}









//-----------------------------------------------------------------------------
// Name: OpenFileDialog()
// Desc: Uses Win32 GetOpenFileName() dialog to get the name of an X file to
//       load, then proceeds to load that file.
//
// Note: SSssshhhhiiittt!!!	
//		 Bug in oorspronkelijk XFile programma: als je dezelfde file als de
//		 huidige file opent laden de textures niet.
//		 TODO: fix it! A kludge would be to check whether we have the same file and 
//       then return immediately. Problem is caused by D3DTextr_RestoreAllTextures().
//		 More specifically the bug is: als een .X file dezelfde texture heeft als de
//		 huidige .X file dan laden deze texture niet. De naam van de .X file maakt
//		 niets uit.
//		 Solution is now found: 
//		 SAFE_DELETE( m_pFileObject ); must be done before loading the new file.
// Note: Well, better is to do m_pFileObject = NULL; instead of SAFE_DELETE( m_pFileObject );
//		 That way we can do it after the loading. SAFE_DELETE() is fucky.
// Note: Use OFN_NOCHANGEDIR to make sure current dir does not change
// Note: Another texture bug here is that D3DTextr_RestoreAllTextures() is necessary
//		 to restore textures in an .X file after it is loaded but also has the nasty
//		 habit to wipe out all textures loaded with D3DTextr_CreateTextureFromFile().
//		 We have solved this with calling CreateTextures() to recreate those textures.
// Note: NOTE NOTE NOTE: Read all about Access Violation in Debug Mode (F5) elsewhere
//		 caused by using a LPDIRECTDRAWSURFACE7
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::OpenFileDialog()
{
	static TCHAR strInitialDir[512] = "";
    static TCHAR strFileName[512];
    TCHAR strCurrentName[512] = "*.x";

	TCHAR strCurrentDirSave[512] = "";



	strcpy(strInitialDir, g_strHeliFilePath);
    
    if( '\0' == strInitialDir[0] )
        strcpy( strInitialDir, g_szRCSIMMediaPath );

	GetCurrentDirectory( sizeof(strCurrentDirSave), strCurrentDirSave );

    // NOTE: use OFN_NOCHANGEDIR to make sure current dir does not change
	// Well, let's not do this as we want to be able to load X files from all dirs
//	OPENFILENAME ofn = { sizeof(OPENFILENAME), m_hWnd, g_hInst,
//                       "X Files (*.x)\0*.x\0All Files (*.*)\0*.*\0\0",
//                       NULL, 0, 1, strCurrentName, 512, strFileName, 512,
//                       strInitialDir, "Open Model File", 
//                       OFN_FILEMUSTEXIST|OFN_SHOWHELP|/*OFN_NOCHANGEDIR|*/OFN_ENABLETEMPLATE|OFN_EXPLORER|OFN_ENABLEHOOK,
//                       0, 1, "X", 0, OFNHookProc1, MAKEINTRESOURCE(IDD_OPENFILE) };

	// NOTE:
	// From the MSDN Docs on OPENFILENAME:
	// For compatibility reasons, the Places Bar is hidden if Flags is set to 
	// OFN_ENABLEHOOK *and* lStructSize is OPENFILENAME_SIZE_VERSION_400.
	//
	// NOTE2: The difference is that for the legacy applications, any new features (like
	// resizing) are disabled, if the template or a hook is used. To enable
	// resizing while using a hook, one should set OFN_ENABLESIZING flag. To enable
	// sidebar, you may need to use V5 OPENFILENAME structure.
	// 
	// NOTE3: To get the Places Bar we must do a #define _WIN32_WINNT 0x0500 in
	// precompiled.h *before* #include <commdlg.h>. Note also that the commdlg.h that
	// has the V5 OPENFILENAME comes from the NTDDK, not from MSVC++ 5.0 as that was
	// pre-Win2000.
	//
	// NOTE4: For the code to run on systems that do not support the Places Bar, we
	// must use OPENFILENAME_SIZE_VERSION_400 for the ofn.lStructSize. We must do an OS
	// check at run time. OS's that support the new OPENFILENAME struct, and thus
	// support the Places Bar, are: Win 2000, XP, and Me.
	//
	// NOTE5: Ugly: After a custom Open dialog with the Places Bar is closed, and you 
	// then open a standard Open dialog with the Places Bar, the size will be that of the 
	// previously opened custom Open dialog.


	OPENFILENAME ofn;

	if (osvinfo.dwMajorVersion >= 5) {
		ofn.lStructSize = sizeof(OPENFILENAME);
	} else {
		ofn.lStructSize = OPENFILENAME_SIZE_VERSION_400;
	}
    ofn.hwndOwner = m_hWnd; 
    ofn.hInstance = g_hInst; 
    ofn.lpstrFilter = "X Files (*.x)\0*.x\0All Files (*.*)\0*.*\0\0"; 
    ofn.lpstrCustomFilter = NULL; 
    ofn.nMaxCustFilter = 0; 
    ofn.nFilterIndex = 1; 
    ofn.lpstrFile = strCurrentName; 
    ofn.nMaxFile = 512; 
    ofn.lpstrFileTitle = strFileName; 
    ofn.nMaxFileTitle = 512; 
    ofn.lpstrInitialDir = strInitialDir; 
    ofn.lpstrTitle = "Open Model File"; 
    ofn.Flags = OFN_FILEMUSTEXIST|OFN_SHOWHELP|/*OFN_NOCHANGEDIR|*/OFN_EXPLORER|OFN_ENABLETEMPLATE|OFN_ENABLEHOOK|OFN_ENABLESIZING; 
    ofn.nFileOffset = 0; 
    ofn.nFileExtension = 1; 
    ofn.lpstrDefExt = "X"; 
    ofn.lCustData = 0; 
    ofn.lpfnHook = OFNHookProc1; 
    ofn.lpTemplateName = MAKEINTRESOURCE(IDD_OPENFILE); 

	if (osvinfo.dwMajorVersion >= 5) {
		ofn.pvReserved = NULL;
		ofn.dwReserved = 0;
		ofn.FlagsEx = 0;//OFN_EX_NOPLACESBAR;
	}



    // Run the OpenFileName dialog.
    if( FALSE == GetOpenFileName( &ofn ) )
        return S_OK;

    // Store the initial directory for next time
    strcpy( strInitialDir, strCurrentName );
    strstr( strInitialDir, strFileName )[0] = '\0'; // clip file name from path

    
	// Change X file
	// NOTE: CD3DFile::Load() will load X file and textures from:
	// 1. Current dir, and if this fails:
	// 2. From SDK media dir
	// WARNING: kludge ahead:
	// Kludge: But only change file if it's a new file. This way we'll 
	// circumvent texture probs with D3DTextr_RestoreAllTextures()
	// Echter: als we zo een file loaden die dezelfde texture als de vorige gebruikt
	// zal de texture alsnog worden restored en dus verdwijnen.
	// DONE: solution is to do SAFE_DELETE( m_pFileObject );
	// Kludging like we did was such a bore:
	// 1. We couldn't reload the same x file
	// 2. If x file with different name used the same texture as the previous x file,
	//    the texture would still not be loaded
	// At last we've found the solution: SAFE_DELETE( m_pFileObject ); when attempting
	// another kludge: load a dummy x file first before loading the x file of the user's choice.
	// NOTE NOTE NOTE: Better is to do m_pFileObject = NULL; instead of SAFE_DELETE( m_pFileObject );
	// That way we can do it after the loading. SAFE_DELETE() is fucky!!!
	//
	//if ( 0 != _stricmp(g_strHeliFileOld, strFileName) )	// kludge
	if ( 1 )												// no kludge
	{		
		// kludge /////////////////////
		// Well, this isn't a kludge: it's a solution
		// Let's load a dummy x file, then the x file we want
		//CD3DFile* pFileObject2 = new CD3DFile();
		//if( FAILED( pFileObject2->Load( "cube.x" ) ) )
		//{
		//	MessageBox( NULL, TEXT("Error loading specified X file"),
		//				TEXT("R/C Sim Sikorsky Error Message"), MB_OK|MB_ICONERROR );
		//	return E_FAIL;
		//}
		//
		// This is all we need:
		// NOTE: this however means we cannot set back to the old file in case the new one
		// fails to load.
		//SAFE_DELETE( m_pFileObject );
		//
		//m_pFileObject = pFileObject2;
		//D3DTextr_RestoreAllTextures( m_pd3dDevice );
		//CreateTextures();
		///////////////////////////////

		
		CD3DFile* pFileObject = new CD3DFile();
    
		if( FAILED( pFileObject->Load( strCurrentName ) ) )
		{
			MessageBox( NULL, TEXT("Error loading specified X file"),
						TEXT("R/C Sim Sikorsky Error Message"), MB_OK|MB_ICONERROR );
			return E_FAIL;
		}

		// If the file was successfully loaded, delete the old one and use this one
		// instead.
		// TODO: we have already done SAFE_DELETE, is this OK? Well, in case we cannot
		// load the new file we cannot set back to the old file. That's the only prob.
		// Solve it.
		// DONE: with: m_pFileObject = NULL;
		// NOTE NOTE NOTE: Better is to do m_pFileObject = NULL; instead of SAFE_DELETE( m_pFileObject );
		// That way we can do it after the loading. SAFE_DELETE() is fucky!!!
		
		//SAFE_DELETE( m_pFileObject ); // fucker!!!
		m_pFileObject = NULL;

		m_pFileObject = pFileObject;

		// kludge: to circumvent texture probs with D3DTextr_RestoreAllTextures()
		// when user chooses the same file
		// save present file path
		strcpy( g_strHeliFileOld, strFileName );

		strcpy( g_strHeliFileName, strFileName );
		strcpy( g_strHeliFilePath, strCurrentName );
		
		
		// Restore the textures (if there's a device)
		// NOTE1: als we dit doen crashen we met lens flare
		// NOTE2: Dit is nodig om de textures van het file object te valideren
		// maar heeft tot gevolg dat alle lensflare textures verloren gaan
		// Ook andere textures gaan verloren. Hierdoor CRASHEN we!!!
		// Tenzij we al de verloren textures weer createn en restoren. Sucks!!!
		D3DTextr_RestoreAllTextures( m_pd3dDevice );

		
		// Daarom:

		// Create textures ///////////
		// TODO:
		// NOTE NOTE NOTE: the dialog no longer has OFN_NOCHANGEDIR so dir changes
		// But CreateTextures() still seeks textures via relative path because it
		// assumes we are in \bin
		// Fix it!!!!!!!!!!!!!!!!!!!
		// Well, it does not seem to be a problem...
		CreateTextures();
		//////////////////////////////


	} 
	else
	{
		//MessageBox(NULL, "same file", "QQQ", MB_OK);	
	}


	// Save original mesh vertices for mesh morphing
	SaveFileObjectOrigMeshVertices();
	///////////////


	////////////////////////////////////////////////////////////////
//	// FMS x file shadows
//	// Well, we don't need it
//	delete[] m_pFileObjectVertices;
//	delete[] m_pFileObjectIndices;
//
//	// get all vertices
//	m_pFileObject->EnumObjects( GetNumFileObjectVerticesCB, NULL,
//		(VOID*)&m_dwNumFileObjectVertices );
//	
//	m_pFileObjectVertices = new D3DVERTEX[m_dwNumFileObjectVertices];
//	
//	m_pFileObject->EnumObjects( GetFileObjectVerticesCB, NULL,
//		(VOID*)m_pFileObjectVertices );
//	
//	// get all indices
//	m_pFileObject->EnumObjects( GetNumFileObjectIndicesCB, NULL,
//		(VOID*)&m_dwNumFileObjectIndices );
//	
//	m_pFileObjectIndices = new WORD[m_dwNumFileObjectIndices];
//	
//	m_pFileObject->EnumObjects( GetFileObjectIndicesCB, NULL,
//		(VOID*)m_pFileObjectIndices );
	/////////////////////////////////////////////////////////////////


/*
    // Set the projection matrix
    D3DVIEWPORT7 vp;
    m_pd3dDevice->GetViewport(&vp);
    FLOAT fAspect = ((FLOAT)vp.dwHeight) / vp.dwWidth;

    D3DMATRIX matProj;
    D3DUtil_SetProjectionMatrix( matProj, g_PI/4, fAspect, 1.0f, 1000.0f );
    m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_PROJECTION, &matProj );
*/


	// NOTE: dangerous to do it like this: e.g. Bell.x and g_bBell will *not* be set
	// to true. Windows XP will convert first letters into capitals. Sucks!!!
	// So better use _stricmp() instead of strcmp()
	// check if it's the bell
	if ( 0 == _stricmp(strFileName, "bell.x") )
		g_bBell = true;
	else
		g_bBell = false;

	// check if it's the cobra
	if ( 0 == _stricmp(strFileName, "cobra.x") ) {
		g_bCobra = true;
	} else {
		g_bCobra = false;
	}

	// check if it's the nsscougar
	if ( 0 == _stricmp(strFileName, "nsscougar.x") )
		g_bCougar = true;
	else
		g_bCougar = false;


	// reset values
	D3DUtil_SetIdentityMatrix( m_matFileObjectMatrix );
	//m_fX = INIT_X;
	//m_fY = INIT_Y;
	//m_fZ = INIT_Z;
	//m_fRadsY = INIT_RADS_Y;

	m_fSpeed = 0.0f;
	m_fCollective = INIT_COLLECTIVE;

	//if (g_bBell || g_bCobra)
	//	m_fThrottle = 1.00f;
	//else if (g_bCougar)
	//	m_fThrottle = 0.50f;
	//else
	//	m_fThrottle = 0.20f;
	m_fThrottle = 7.35f;

	g_bFirstFrame = true;
	//g_vecVelocity = D3DXVECTOR3(0.0f, -0.1f, 0.0f); // we vallen in beeld: Cool!
	g_vecVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
	g_vecAngularVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);

	// used in GetSticks()
	g_bResetValues = true;

	g_bLanded = true;

	// reset latency array
	g_bResetLatency = true;

	// reset exhaust smoke array
	g_bResetExhaustSmoke = true;


	// DirectSound ////////////////////
	StopBuffer();

	// load .wav file
	static TCHAR g_strWavFileName[512];

	strcpy( g_strWavFileName, strCurrentName );
//	if ( strstr( g_strWavFileName, ".x" ) )
//		strstr( g_strWavFileName, ".x" )[0] = '\0'; // clip extension
//	if ( strstr( g_strWavFileName, ".X" ) )
//		strstr( g_strWavFileName, ".X" )[0] = '\0'; // clip extension

	// find extension (case-insensitive and from last instance of '.') and clip extension
	if ( strrchr(g_strWavFileName, '.') ) {
		if ( 0 == _stricmp( strrchr(g_strWavFileName, '.'), ".x") )
			strrchr(g_strWavFileName, '.')[0] = '\0';
		else
			MessageBox(NULL, "No .x or .X found in file name", "QQQ", MB_OK);
	} else {
		MessageBox(NULL, "No . found in file name", "QQQ", MB_OK);
	}

	strcat( g_strWavFileName, ".wav" );

	LoadWaveFile( m_hWnd, g_strWavFileName );

//	if (g_bUseAuthenticHeliSound) {
//		if (g_bBell || g_bCougar) {
//			LoadWaveFile( m_hWnd, "bell.wav" );
//		} else if (g_bCobra) {
//			LoadWaveFile( m_hWnd, "cobra.wav" );
//		} else {
//			//LoadWaveFile( m_hWnd, "heli.wav" );
//			LoadWaveFile( m_hWnd, g_strWavFileName );
//		}
//	} else {
//		LoadWaveFile( m_hWnd, "heli.wav" );
//		//LoadWaveFile( m_hWnd, g_strWavFileName );
//	}


    SetParameters( 1.0f, 1.0f, 100.0f, 800.0f );

	SetVolume( DSBVOLUME_MIN );
	g_uCountBeforeMaxVolume = 0;

	PlayBuffer(TRUE);
	///////////////////////////////////
	
	
	// ODE dynamics
	DestroyDynamics();
	InitDynamics();
	DestroyProxy();
	CreateProxy();


	// Recalculate speed factor
	// frame rate change expected on OpenFileDialog() therefore
	// reset values to recalculate m_fSpeedFactor
	g_acc = 0.0f;
	g_count = 0;


	// reset dir
	//SetCurrentDirectory( g_szRCSIMProgramPath );
	//SetCurrentDirectory( "..\\media" );
	SetCurrentDirectory( strCurrentDirSave );
	

    // Return successful
    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: OFNHookProc1()
// Desc: Hook proc for Open File Dialog 
//-----------------------------------------------------------------------------
UINT APIENTRY OFNHookProc1( HWND hdlg, UINT uiMsg, WPARAM wParam, LPARAM lParam )
{
	//OFNOTIFY on;

	RECT rc;
	POINT pt;

	static TCHAR strPrvFileName[512];

	static TCHAR str[512];

	static HANDLE hFile1 = NULL;

	DWORD dwSize;
	//BY_HANDLE_FILE_INFORMATION bhfi;
	
	FILETIME ftCreationTime;
	FILETIME ftLastAccessTime;
	FILETIME ftLastWriteTime;
	
	FILETIME ft;
	SYSTEMTIME st;  
  

	HDC hdcDest, hdcBack;
	HBITMAP hbmBack;

	HFONT hFont, hFontOld;

	PAINTSTRUCT ps;

	static int count;


	switch(uiMsg)
	{
		case WM_INITDIALOG:
			SetWindowText( GetDlgItem(hdlg, IDC_STATIC1), "" );
			SetWindowText( GetDlgItem(hdlg, IDC_STATIC2), "" );
			SetWindowText( GetDlgItem(hdlg, IDC_STATIC3), "" );

			CheckDlgButton(hdlg, IDC_CHECK1, BST_CHECKED);

			count = 0;
			break;

		case WM_PAINT:
			BeginPaint( hdlg, &ps );
			//MessageBeep(-1);

			hdcDest = GetDC( GetDlgItem(hdlg,IDC_STATIC_PREVIEW) );
			GetClientRect( GetDlgItem(hdlg,IDC_STATIC_PREVIEW), &rc );

			hdcBack = CreateCompatibleDC( hdcDest );
			hbmBack = CreateCompatibleBitmap(hdcDest, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2);
			SelectObject(hdcBack, hbmBack);
			DeleteObject(hbmBack);
			
			//BitBlt( hdcDest, 0, 0, rc.right-rc.left, rc.bottom-rc.top, NULL, 0, 0, WHITENESS );
			
			// TEST:					
			pt.x = 1;
			pt.y = 1;
			
			// read _prv.jpg files
			//CommDlg_OpenSave_GetSpec(GetParent(hdlg), strPrvFileName, sizeof(strPrvFileName)); 
			
			
			
			if ( FALSE == DrawImage(hdcDest,
				strPrvFileName,
				pt,
				GetDlgItem(hdlg,IDC_STATIC_PREVIEW),
				rc) )
			{
				//MessageBox(hdlg, "No preview","QQQ", MB_OK);
				//MessageBeep(-1);

				// delete current image
				// and show "No preview available" text
				BitBlt( hdcBack, 0, 0, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2, NULL, 0, 0, WHITENESS );
				
				// Font
				hFont = CreateFont(14,0,0,0,400,0,0,0,
					ANSI_CHARSET,OUT_DEFAULT_PRECIS,
					CLIP_DEFAULT_PRECIS,
					DEFAULT_QUALITY,
					DEFAULT_PITCH|FF_DONTCARE,
					"MS Sans Serif");
				hFontOld = (HFONT)SelectObject(hdcBack, hFont);
				
				if (count > 0) {
					TextOut(hdcBack, 45, 50, "No preview", strlen("No preview"));
					TextOut(hdcBack, 50, 64, "available", strlen("available"));
				}
				
				// Old Font back
				SelectObject(hdcBack, hFontOld);
				DeleteObject(hFont);
				
				// And finally...
				// blt entire memory bitmap to entire window
				BitBlt(hdcDest, 1, 1, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2, hdcBack, 0, 0, SRCCOPY);
				
			}			

			// And release...
			ReleaseDC( GetDlgItem(hdlg,IDC_STATIC_PREVIEW), hdcDest );

			EndPaint( hdlg, &ps );
			break;

		case WM_COMMAND:
			if (HIWORD(wParam) == BN_CLICKED)
			{ 
				switch ( LOWORD(wParam) )
				{
					case IDC_CHECK1:
						//MessageBeep(-1);
						if ( IsDlgButtonChecked(hdlg, IDC_CHECK1) ) {
							ShowWindow( GetDlgItem(hdlg,IDC_STATIC_PREVIEW), SW_SHOW );
						} else {
							ShowWindow( GetDlgItem(hdlg,IDC_STATIC_PREVIEW), SW_HIDE );
						}
						break;

					case IDC_CHECK2:
						break;

					case IDC_CHECK3:
						break;

					case IDC_BUTTON1:
						break;
					
				}
			}
			break;

		case WM_NOTIFY:
			switch( ((NMHDR FAR *)lParam)->code )
			{
				case CDN_SELCHANGE:
					//MessageBeep(-1);

					// info
					if (count > 0) {
						CommDlg_OpenSave_GetSpec(GetParent(hdlg), str, sizeof(str));

						// create file to get info
						hFile1 = CreateFile( str, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, NULL,
							OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );

						if ( hFile1 == INVALID_HANDLE_VALUE ) {
							//MessageBox((HWND)g_hWndSikorsky, "Can't read file.",
							//	"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
							hFile1 = NULL;
							
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC1), "" );
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC2), "" );
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC3), "" );
						} else {
							// name							
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC1), str );

							// size
							dwSize = GetFileSize(hFile1, NULL);
							//itoa((int)dwSize,str,10);
							sprintf(str, "%i bytes", (int)dwSize);
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC2), str );

							// date
							GetFileTime(hFile1, &ftCreationTime, &ftLastAccessTime, &ftLastWriteTime);
							ft = ftCreationTime;
							FileTimeToLocalFileTime( &ft, &ft );
							FileTimeToSystemTime( &ft, &st );
							sprintf(str, "%d-%d-%d %02d:%02d:%02d\n", st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute, st.wSecond);
							SetWindowText( GetDlgItem(hdlg, IDC_STATIC3), str );
						}					

						
						//if ( hFile1 != INVALID_HANDLE_VALUE ) {
						//	//GetFileInformationByHandle(hFile1, &bhfi);
						//	//FILETIME bhfi.ftLastWriteTime
						//}

						if (hFile1) {
							CloseHandle(hFile1);
							hFile1 = NULL;
						}
					}


					// preview
					hdcDest = GetDC(GetDlgItem(hdlg,IDC_STATIC_PREVIEW));
					GetClientRect(GetDlgItem(hdlg,IDC_STATIC_PREVIEW),&rc);

					hdcBack = CreateCompatibleDC( hdcDest );
					hbmBack = CreateCompatibleBitmap(hdcDest, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2);
					SelectObject(hdcBack, hbmBack);
					DeleteObject(hbmBack);

					//BitBlt( hdcDest, 0, 0, rc.right-rc.left, rc.bottom-rc.top, NULL, 0, 0, WHITENESS );
					
					// TEST:					
					pt.x = 1;
					pt.y = 1;

					// read _prv.jpg files
					CommDlg_OpenSave_GetSpec(GetParent(hdlg), strPrvFileName, sizeof(strPrvFileName)); 
					
//					if ( strstr( strPrvFileName, ".x" ) )
//						strstr( strPrvFileName, ".x" )[0] = '\0'; // clip extension
//					if ( strstr( strPrvFileName, ".X" ) )
//						strstr( strPrvFileName, ".X" )[0] = '\0'; // clip extension

					// find extension (case-insensitive and from last instance of '.') and clip extension
					if ( strrchr(strPrvFileName, '.') ) {
						if ( 0 == _stricmp( strrchr(strPrvFileName, '.'), ".x") ) {
							strrchr(strPrvFileName, '.')[0] = '\0';
						} else {
							//MessageBox(NULL, "No .x or .X found in file name", "QQQ", MB_OK);
						}
					} else {
						//MessageBox(NULL, "No . found in file name", "QQQ", MB_OK);
					}

					strcat( strPrvFileName, "_prv.jpg" );

					
					if ( FALSE == DrawImage(hdcDest,
							strPrvFileName,
							pt,
							GetDlgItem(hdlg,IDC_STATIC_PREVIEW),
							rc) )
					{
						// delete current image
						// and show "No preview available" text
						BitBlt( hdcBack, 0, 0, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2, NULL, 0, 0, WHITENESS );

						// Font
						hFont = CreateFont(14,0,0,0,400,0,0,0,
							ANSI_CHARSET,OUT_DEFAULT_PRECIS,
							CLIP_DEFAULT_PRECIS,
							DEFAULT_QUALITY,
							DEFAULT_PITCH|FF_DONTCARE,
							"MS Sans Serif");
						hFontOld = (HFONT)SelectObject(hdcBack, hFont);

						if (count > 0) {
							TextOut(hdcBack, 45, 50, "No preview", strlen("No preview"));
							TextOut(hdcBack, 50, 64, "available", strlen("available"));
						}
						count++;

						// Old Font back
						SelectObject(hdcBack, hFontOld);
						DeleteObject(hFont);

						// And finally...
						// blt entire memory bitmap to entire window
						BitBlt(hdcDest, 1, 1, (rc.right-rc.left)-2, (rc.bottom-rc.top)-2, hdcBack, 0, 0, SRCCOPY);

					}

					ReleaseDC(hdlg, hdcDest);
					break;

				case CDN_HELP:
					HtmlHelp( GetDesktopWindow(), "..\\HTMLHelp\\Rcsim.chm", HH_DISPLAY_TOPIC, NULL );
					break;
			}
			break;

		/*case WM_HELP:
			break;*/
	}

	return 0;
}



//-----------------------------------------------------------------------------
// Name: LoadFile()
// Desc: Laadt een X File zonder Open File Dialoog 
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::LoadFile( TCHAR* strFilename )
{
    CD3DFile* pFileObject = new CD3DFile();
    
    if( FAILED( pFileObject->Load( strFilename ) ) )
    {
        MessageBox( NULL, TEXT("Error loading specified X file"),
                    TEXT("XFile"), MB_OK|MB_ICONERROR );
        return E_FAIL;
    }

    // If the file was successfully loaded, delete the old one and use this one
    // instead.
    SAFE_DELETE( m_pFileObject );
    m_pFileObject = pFileObject;


/*
    // Set the projection matrix
    D3DVIEWPORT7 vp;
    m_pd3dDevice->GetViewport(&vp);
    FLOAT fAspect = ((FLOAT)vp.dwHeight) / vp.dwWidth;

    D3DMATRIX matProj;
    D3DUtil_SetProjectionMatrix( matProj, g_PI/4, fAspect, 1.0f, 1000.0f );
    m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_PROJECTION, &matProj );

    // Restore the textures (if there's a device).
    D3DTextr_RestoreAllTextures( m_pd3dDevice );
*/

    // Return successful
    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: CalcFileObjectSizeCB()
// Desc: Callback used to calculate the radius of a sphere that encloses all
//       the meshes in the file.
//-----------------------------------------------------------------------------
BOOL CMyD3DApplication::CalcFileObjectSizeCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext )
{
    FLOAT*     pfRadius = (FLOAT*)pContext;
    D3DVERTEX* pVertices;
    DWORD      dwNumVertices;

    if( SUCCEEDED( pObject->GetMeshGeometry( &pVertices, &dwNumVertices, 
                                             NULL, NULL ) ) )
    {
        for( DWORD i=0; i<dwNumVertices; i++ )
        {
            FLOAT x = pVertices[i].x;
            FLOAT y = pVertices[i].y;
            FLOAT z = pVertices[i].z;

            FLOAT mx = x*pmat->_11 + y*pmat->_21 + z*pmat->_31 + pmat->_41;
            FLOAT my = x*pmat->_12 + y*pmat->_22 + z*pmat->_32 + pmat->_42;
            FLOAT mz = x*pmat->_13 + y*pmat->_23 + z*pmat->_33 + pmat->_43;

            // Store the largest r (radius) for any point in the mesh
            FLOAT r = sqrtf( mx*mx + my*my + mz*mz );
            if( r > (*pfRadius) )
                (*pfRadius) = r;
        }
    }

    // Keep enumerating file objects
    return FALSE;
}


//-----------------------------------------------------------------------------
// Name: GetNumFileObjectVerticesCB()
// Desc: Callback used to get the number of vertices in the file
// Note: We need this for our shadow calculations
//-----------------------------------------------------------------------------
BOOL CMyD3DApplication::GetNumFileObjectVerticesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext )
{
	DWORD*     pdwTotalNumVertices = (DWORD*)pContext;
	D3DVERTEX* pVertices;
    DWORD      dwNumVertices;

	if( SUCCEEDED( pObject->GetMeshGeometry( &pVertices, &dwNumVertices, 
                                             NULL, NULL ) ) )
    {
		*pdwTotalNumVertices += dwNumVertices;
    }

	// Keep enumerating file objects
    return FALSE;
}



//-----------------------------------------------------------------------------
// Name: GetFileObjectVerticesCB()
// Desc: Callback used to get all vertices in the file into an array
// Note: We need this for our shadow calculations
//-----------------------------------------------------------------------------
BOOL CMyD3DApplication::GetFileObjectVerticesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext )
{
	D3DVERTEX* pFileObjectVertices = (D3DVERTEX*)pContext;
	D3DVERTEX* pVertices;
    DWORD      dwNumVertices;

	if( SUCCEEDED( pObject->GetMeshGeometry( &pVertices, &dwNumVertices, 
                                             NULL, NULL ) ) )
    {
		static DWORD j = 0;
        for( DWORD i=0; i<dwNumVertices; i++ )
        {
            pFileObjectVertices[j] = pVertices[i];
			j++;
        }
		
    }

	// Keep enumerating file objects
    return FALSE;
}


//-----------------------------------------------------------------------------
// Name: GetNumFileObjectIndicesCB()
// Desc: Callback used to get the number of vertices in the file
// Note: We need this for our shadow calculations
//-----------------------------------------------------------------------------
BOOL CMyD3DApplication::GetNumFileObjectIndicesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext )
{
	DWORD* pdwTotalNumIndices = (DWORD*)pContext;
	WORD*  pIndices;
    DWORD  dwNumIndices;

	if( SUCCEEDED( pObject->GetMeshGeometry( NULL, NULL, 
                                             &pIndices, &dwNumIndices ) ) )
    {
		*pdwTotalNumIndices += dwNumIndices;
    }

	// Keep enumerating file objects
    return FALSE;
}



//-----------------------------------------------------------------------------
// Name: GetFileObjectIndicesCB()
// Desc: Callback used to get all vertices in the file into an array
// Note: We need this for our shadow calculations
//-----------------------------------------------------------------------------
BOOL CMyD3DApplication::GetFileObjectIndicesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext )
{
	WORD* pFileObjectIndices = (WORD*)pContext;
	WORD* pIndices;
    DWORD dwNumIndices;

	if( SUCCEEDED( pObject->GetMeshGeometry( NULL, NULL, 
                                             &pIndices, &dwNumIndices ) ) )
    {
		static DWORD j = 0;
        for( DWORD i=0; i<dwNumIndices; i++ )
        {
            pFileObjectIndices[j] = pIndices[i];
			j++;
        }
		
    }

	// Keep enumerating file objects
    return FALSE;
}


//----------------------------------------------------------------------------
// Name: MsgProc()
// Desc: App custom WndProc function for handling mouse and keyboard input.
//----------------------------------------------------------------------------
LRESULT CMyD3DApplication::MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam,
                                    LPARAM lParam )
{
	static bool firsttime = true;
	static bool firsttime2 = true;
	static float fTemp;
	static int i = 0;

	//static char Buffer[512];

	// TEST: Redraw Kludge
	static bool bRedraw = true;
	static bool bMoving = false;
	static bool bMaximized = false;
	static RECT rcOld;
	static RECT rcOld2;


	PAINTSTRUCT ps;
	RECT rc;
	RECT rc2;

	char msg[512];

	
	// get handle to menu
	HMENU hmenu = GetMenu(hWnd);

	//HMENU hmenuSub = GetSubMenu(hmenu,0);
	// do status bar menu help
	//MenuHelp( uMsg ,wParam, lParam, hmenu, g_hinst, g_hwndStatus, (UINT FAR*)g_adwMenuHelpIDs );

	// Get handle to Mode menu
	HMENU hmenuMode = GetSubMenu( GetSubMenu(hmenu,4), 1 );

	TCHAR strMode[80];
	DWORD mode = 0;
	
	static MENUITEMINFO miiMode;

	miiMode.cbSize = sizeof(MENUITEMINFO);
	miiMode.fMask = MIIM_CHECKMARKS | MIIM_DATA | MIIM_ID | MIIM_STATE | MIIM_SUBMENU | MIIM_TYPE;
	//miiMode.fType = MFT_STRING;
	//miiMode.fState = MFS_ENABLED; //MFS_GRAYED | MFS_CHECKED; 
	//miiMode.wID = mode; //0; 
	//miiMode.hSubMenu = NULL; 
	//miiMode.hbmpChecked = NULL; 
	//miiMode.hbmpUnchecked = NULL; 
	//miiMode.dwItemData = 0; 
	miiMode.dwTypeData = strMode; 
	miiMode.cch = sizeof(strMode);


	//// RM
	//// TEST:
	//AppInfo *info;
	//
    //info = (AppInfo *) GetWindowLong(hWnd, 0);
    //active_window = info;


		
	switch( uMsg )
    {
		case WM_CREATE:
			// do shareware notice ////////////
			//DoShareWareNotice( hWnd );
			//Pause(true);
			//DoShareWareNotice2( m_hWnd );
			//Pause(false);
			//SetForegroundWindow( hWnd );
		

			GetWindowRect( hWnd, &rcOld );
			GetWindowRect( hWnd, &rcOld2 );

			// write filenames and filepaths to registry
			// Must write these to the registry when .fld or .cal file is doubleclicked
			// in Explorer
			RegistryWrite7();

			// drag-drop support
			DragAcceptFiles(hWnd, TRUE);
			break;

		case WM_ACTIVATE:
			//if (firsttime2) {
			//	firsttime2 = false;
			//	DoShareWareNotice( hWnd );
			//}

			// TODO: cover all cases of unacquire (such as window resize)
			// Read the docs about acquiring input devices!!!!!!!!!!!!!
			// Pause or unpause the app (and acquire or unacquire the device)
            // based on whether we are losing or gaining activation.
			// Input device must be acquired before the GetDeviceState is called
            m_bActive = TRUE;//( WA_INACTIVE != wParam );
            // Pause( !m_bActive ); // TODO: check wat Pause() doet
			// Dit is de eerste maal dat er een Acquire() zal worden gedaan
            PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 );


			// TODO: Real-time Configuration Toolwindow 
			// It would be nice to have a real-time console (toolwindow). 
			// If so we must not pause the app (or rotor) when out of focus.
			// We moeten dan een aparte thread (of zelfs een apart process?)
			// creeren om te voorkomen dat de app paused
			// En een manier vinden waarop de console kan communiceren met R/C Sim
			// (IPC, DLL ???)
			// Allemaal niet nodig: een modeless dialog is genoeg. Nu nog de blue caption...


			
			// Kill sound when inactive
			// Pause app when inactive
			// Well no: het is misschien logischer maar het heeft tot gevolg dat
			// wanneer een gepausede app sized of moved wordt, de render surface
			// niet mee verandert. Da's lelijk.
			/*switch( LOWORD(wParam) )
			{
				case WA_INACTIVE:
					m_bActiveSound = FALSE;
					//Pause(TRUE); // toch maar niet dus
					// we kunnen wel de rotors stoppen
					// maar dan zakt de heli naar beneden, da's ook lelijk...
					fTemp = m_fThrottle;
					// kludge to keep revs print OK while paused
					m_fRevs = 100.0f*m_fThrottle;

					m_fThrottle = 0.0f;
					break;
				case WA_ACTIVE:
				case WA_CLICKACTIVE:
					m_bActiveSound = TRUE;
					
					
					// don't pause at app start up 
					if (!firsttime) {						
						//Pause(FALSE);
						m_fThrottle = fTemp;
					}
					firsttime = false;
					break;
			}*/
			break;



		/*case WM_NCACTIVATE:
			if ( (BOOL)wParam == FALSE )
				return FALSE;
			break;*/

		/*case WM_NCPAINT:
			RECT rc;
			GetWindowRect(hWnd, &rc);
			// only draws text...
			DrawCaption( hWnd, GetDC(hWnd), &rc, DC_ACTIVE );
			//return 0;
			break;*/

		case WM_COPYDATA:
			// Sent to window when user double-clicks on registered file and app
			// instance is already running. It sends the command line string.
			//MessageBox(hWnd,"qqq","QQQ!",MB_OK);
			//MessageBox(hWnd,(char*)((PCOPYDATASTRUCT)lParam)->lpData,"QQQ!",MB_OK);
			//ProcessCommandLine( (char*)((PCOPYDATASTRUCT)lParam)->lpData );

			// do the same as when file is drag-dropped
			HandleDragDrop( (HWND)g_hWndSikorsky, NULL, (char*)((PCOPYDATASTRUCT)lParam)->lpData );

			return TRUE;
			break;

		case WM_DROPFILES:
			// drag-drop support
			HandleDragDrop(hWnd, (HDROP)wParam, NULL); // application-defined function
			break; 
			
		case WM_INITMENU:		
			// set menu items 
			SetMenuItems(hmenu);

			// Kill sound when menu is open
			//PlaySound( NULL, NULL, SND_PURGE );
			SetVolume( DSBVOLUME_MIN );
			SetVolumeAllBuffers( DSBVOLUME_MIN );
			break;

		case WM_INITMENUPOPUP:		
			//MessageBox(NULL,"qqq","QQQ!",MB_OK);
			//MessageBeep(-1);
			//InitModeMenu( (HMENU)wParam );
			// fill the Mode submenu
			//HMENU hmenuMode; 
			if ( (HMENU)wParam == hmenuMode ) {
				InitModeMenu( hmenuMode );
			}
			break;
 

		case WM_MENUSELECT:
			// NOTE: this message is sent when we are *over* a menu item, that is, when it
			// is blue (== selected). Clicking the item sends a WM_COMMAND.
			MenuHelp( uMsg ,wParam, lParam, hmenu, g_hinst, g_hwndStatus, (UINT FAR*)&g_aMenuHelpIDs );
			
			// Hide progress bar on status bar when menu is open
			ShowWindow( g_hwndPB, SW_HIDE );

			// If we are over a Mode menu item we'll have to draw the status bar text ourselves
			if ( (HMENU)lParam == hmenuMode ) {	
				//MessageBeep(-1);
				//MessageBox(NULL,"qqq","QQQ!",MB_OK);

				// Check which mode we are over
				//if ( (UINT)LOWORD(wParam) == 4 ) {   
				//	SendMessage(g_hwndStatus, SB_SETTEXT, 0|SBT_NOBORDERS, (LPARAM)"QQQ" );
				//}
		
				// NOTE NOTE NOTE: set fByPosition param FALSE because we want info by menu ID !!!!
				// If we get info by position and have separators, things will f*ck up.
				GetMenuItemInfo( hmenuMode, (UINT)LOWORD(wParam), FALSE, &miiMode );


				// Hell, why can't we check for a separator using & ???
				//if ( (miiMode.fType & MF_SEPARATOR) ) {

				if ( LOWORD(wParam) == 65535 ) {
					// No status message for separators	
					//MessageBeep(-1);
				} else {
					//sprintf( msg, "Switch to fullscreen mode %i", LOWORD(wParam) );
					sprintf( msg, "Switch to %s fullscreen mode", miiMode.dwTypeData );
					//sprintf( msg, "Switch to %s fullscreen mode %u", miiMode.dwTypeData, LOWORD(wParam) );
					//sprintf( msg, "dwTypeData %s, dwItemData %i, menuID %u", miiMode.dwTypeData, miiMode.dwItemData, LOWORD(wParam) );

					//GetMenuString( hmenuMode, (UINT)LOWORD(wParam), msg, sizeof(msg), MF_BYPOSITION );

					// Yyyyyyyyeeeessssss!!!
					SendMessage( g_hwndStatus, SB_SIMPLE, TRUE, 0 );
					SendMessage( g_hwndStatus, SB_SETTEXT, 255|SBT_NOBORDERS, (LPARAM)msg );
				}
				
			}

// demo
#ifdef DEMO
			if (IDM_REGISTER == LOWORD(wParam)) {
				SendMessage( g_hwndStatus, SB_SIMPLE, TRUE, 0 );
				SendMessage( g_hwndStatus, SB_SETTEXT, 255|SBT_NOBORDERS, (LPARAM)"Purchase R/C Sim Sikorsky..." );
			}
#endif
			break;


		case WM_SYNCACQUIRE:
            if( g_pdidDevice7 )
            {
                if( m_bActive ) {
                    g_pdidDevice7->Acquire(); // de eerste maal Acquire()
					//MessageBox(NULL,"acquire!","QQQ!",MB_OK);
				} else {
                    g_pdidDevice7->Unacquire();
					MessageBox(NULL,"unacquire!","QQQ!",MB_OK);
				}
            }
            break;

		case WM_SIZE:
			// TODO: heli spins after a resize with mouse or joy input. Solve this!
			// This does not work:
			// This is because we should actually do this after the framework has
			// handled the WM_SIZE, i.e. after Change3DEnvironment()
			//SelectInputDevice(hWnd);
			//PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 ); // reacquire the fucker!!!!
			//MessageBox(NULL,"size!","QQQ!",MB_OK);

			// nice kludge!!! reacquire in framemove
			// NOTE1: we omzeilen de eerste WM_SIZE van window init want dan
			// is er nog geen DInput device
			// NOTE2: sinds we de input control booleans reg variables hebben gemaakt
			// doen we altijd een reacquire...
			/*if (!m_bFirstTime)*/ 
			/*else m_bFirstTime = FALSE;*/
			m_bReAcquire = TRUE;


			// TEST: Redraw Kludge
			// We seem to be getting two WM_SIZE after resizing and the mouse seems
			// to stick to the window edge. This is a bore since these textures 
			// take a long time to load.
			// Could it be we get that extra WM_SIZE from the client window??? No!
			// It is because when resizing, a flood of WM_SIZE messages are sent. And
			// every time we get a WM_SIZE, InitDeviceObjects() is called. Because that function
			// has become slow due to all the texture loading it looks like we are getting
			// two messages and the mouse seems to stick. But this is normal behaviour.
			// Try it: comment out the texture loading to make InitDeviceObjects() fast:
			// now the mouse does not stick and we get many WM_SIZE during resizing.
			// NOTE: this behaviour is deeply rooted in the DX7 Framework. The DX9 Framework
			// does a StretchBlt() at every WM_SIZE (and only when the window size gets bigger)
			// and only re-starts the Device upon WM_EXITSIZEMOVE.
			// DONE: we have implemented the Redraw Kludge to get this behaviour too (except
			// for the StretchBlt(), but we could easily implement that as well).
			//MessageBox(m_hWnd,"WM_SIZE","QQQ!",MB_OK);
			//MessageBeep(MB_ICONEXCLAMATION);


			// If f*cking status bar won't resize by itself
			//SendMessage( g_hwndStatus, WM_SIZE, wParam, lParam );
			//SendMessage( g_hwndStatus, WM_SIZE, 300, 18 );
			//SetWindowPos(g_hwndStatus, HWND_NOTOPMOST, 300, 500, 100, 12, SWP_SHOWWINDOW);
			//ShowWindow( g_hwndStatus, SW_SHOWMAXIMIZED );
			//GetWindowRect(g_hwndStatus, &rc);
			//MoveWindow(g_hwndStatus, 0+55, 455, rc.right-rc.left, (rc.bottom-rc.top)-5, TRUE);
			//SetWindowPos(g_hwndStatus, HWND_NOTOPMOST, 300, 500, 100, 12, SWP_SHOWWINDOW);

			//MessageBeep(MB_ICONEXCLAMATION);

			/////////////////////////////////////////////////////////////////
			// We have to size/move the status bar by ourselves because we have subclassed
			// it to get 18 pixel height. In the subclass proc we return 0 on WM_SIZE so
			// we can do the sizing/moving here. Thus we can get any height we want.
			//
			// Get the coordinates of the parent window's client area. 
			GetClientRect( hWnd, &rc);
			
			// YYYYYEEEESSS!!! It is this simple. MoveWindow() and a return 0 on WM_SIZE
			// in the subclass proc
			// NOTE: let's not do that with the Redraw Kludge
			//MoveWindow(g_hwndStatus, 0, rc.bottom-18, rc.right-rc.left, 18, TRUE);
			//////////////////////////////////////////////////////////////////

			
			// If f*cking client window won't resize
			// Make client window size with main window
			//if ( g_hwndClient ) {
				//SendMessage( g_hwndClient, WM_SIZE, wParam, lParam );
				//GetClientRect(hWnd, &rc);
				//MoveWindow(g_hwndClient, rc.left, rc.top, LOWORD(lParam), HIWORD(lParam), TRUE);
				//MoveWindow(g_hwndClient, rc.left+20, rc.top+20, 40, 40 ,TRUE);
				//ShowWindow( g_hwndClient, SW_SHOWMAXIMIZED );
			//}

			// Recalculate speed factor
			// frame rate change expected on WM_SIZE therefore
			// reset values to recalculate m_fSpeedFactor
			//MessageBeep(MB_ICONEXCLAMATION);
			// NOTE: we also get WM_SIZE when moving. Sucks!
			// Check if we were moving. If so, don't recalculate.
			GetWindowRect( hWnd, &rc2 );
			if (rcOld2.left-rcOld2.right == rc2.left-rc2.right &&
				rcOld2.bottom-rcOld2.top == rc2.bottom-rc2.top) {
				// do nothing
			} else {
				//MessageBeep(MB_ICONEXCLAMATION);
				g_acc = 0.0f;
				g_count = 0;
			}			
			rcOld2 = rc2;
		


			// TEST: Redraw Kludge
			// Let's try to prevent a redraw upon every WM_SIZE
			if (wParam == SIZE_MAXIMIZED) {
				bRedraw = true;
				bMaximized = true;
			}
			if (bMaximized && wParam == SIZE_RESTORED) {
				bRedraw = true;
				bMaximized = false;
			}

			if (bRedraw) {
				bRedraw = false;
			} else {
				return 0;
			}
			break;

		case WM_EXITSIZEMOVE:
			//MessageBeep(MB_ICONEXCLAMATION);
			// TEST: Redraw Kludge
			// Check if we were moving. If so, don't redraw.
			GetWindowRect( hWnd, &rc );
			if (rcOld.left-rcOld.right == rc.left-rc.right &&
				rcOld.bottom-rcOld.top == rc.bottom-rc.top) {
				bRedraw = false;
			} else {
				bRedraw = true;
			}
			rcOld = rc;
			
			// We fake it: send a resize message to reinit D3D.
			SendMessage(m_hWnd, WM_SIZE, SIZE_RESTORED, MAKELPARAM(rc.right-rc.left, rc.bottom-rc.top));
			break;

		case WM_MOVE:
			break;

		case WM_SIZING:
			break;


		case WM_DRAWITEM:			
			// for owner-drawn status bar
			//MessageBox(NULL,"drawitem!","QQQ!",MB_OK);
			//MessageBeep(-1);
			//SelectObject( LPDRAWITEMSTRUCT(lParam)->hDC, HGDIOBJ(LPDRAWITEMSTRUCT(lParam)->itemData) );

			//SetTextColor( LPDRAWITEMSTRUCT(lParam)->hDC, COLOR_BLUE );
			//SetBkColor( LPDRAWITEMSTRUCT(lParam)->hDC, GetSysColor(COLOR_MENU) );
			//TextOut( LPDRAWITEMSTRUCT(lParam)->hDC, LPDRAWITEMSTRUCT(lParam)->rcItem.left+2, 2, "Test", 4 );

			//SendMessage(g_hwndStatus, SB_SETTEXT, 1|SBT_NOBORDERS, (LPARAM)"QQQ" );
			//SendMessage(g_hwndStatus, SB_SETTEXT, 0|0, (LPARAM)"Testing" );
			
			//MoveWindow( g_hwndStatus, 100,100,300,100, TRUE );


			// Playback/Record info
			if ( LPDRAWITEMSTRUCT(lParam)->CtlID == IDC_STATUSBAR ) {
				if ( LPDRAWITEMSTRUCT(lParam)->itemID == 2 )
				{
					if (m_bPlayBack) {
						SetTextColor( LPDRAWITEMSTRUCT(lParam)->hDC, GetSysColor(COLOR_MENUTEXT) );
						SetBkColor( LPDRAWITEMSTRUCT(lParam)->hDC, GetSysColor(COLOR_BTNFACE) );
						TextOut( LPDRAWITEMSTRUCT(lParam)->hDC, LPDRAWITEMSTRUCT(lParam)->rcItem.left+2, 3, "Playback", strlen("Playback") );
					} 

					if (m_bRecord) {
						SetTextColor( LPDRAWITEMSTRUCT(lParam)->hDC, COLOR_RED );
						SetBkColor( LPDRAWITEMSTRUCT(lParam)->hDC, GetSysColor(COLOR_BTNFACE) );
						TextOut( LPDRAWITEMSTRUCT(lParam)->hDC, LPDRAWITEMSTRUCT(lParam)->rcItem.left+2, 3, "Record", strlen("Record") );
					}
				}
			}
			break;

		case WM_PAINT:
			BeginPaint( hWnd, &ps );
			//UpdateWindow(hWnd);
			//InvalidateRect(hWnd, NULL, TRUE);
			EndPaint( hWnd, &ps );
			break;

		case WM_LBUTTONUP:
			if (m_bSplashScreenShowing)
				m_bSplashScreenShowing = FALSE;
			break;

		//case WM_MOVING:
		//case WM_SIZING:
		case WM_ENTERSIZEMOVE: // Kill sound when moving and sizing
		case WM_GETMINMAXINFO: // Kill sound when maximizing and being minimized
			// Kill sound when app is paused
			//PlaySound( NULL, NULL, SND_PURGE );
			SetVolume( DSBVOLUME_MIN );
			SetVolumeAllBuffers( DSBVOLUME_MIN );
			break;

		case WM_NCLBUTTONDOWN: // Kill sound when left mousebutton down in caption bar
			if ( HTCAPTION == (INT)wParam ) {
				// Kill sound when app is paused
				//PlaySound( NULL, NULL, SND_PURGE );
				SetVolume( DSBVOLUME_MIN );
				SetVolumeAllBuffers( DSBVOLUME_MIN );
			}
			break;

		case WM_KEYDOWN:
			if ( VK_TAB == (INT)wParam ) {
				//MessageBox(NULL,"qqq","QQQ!",MB_OK);
				if (!g_bZooming) {
					g_bZooming = true;
					g_bZoomIn = !g_bZoomIn;
				} else {
					g_bZooming = false;
				}
			}

			//if ( 'C' == (INT)wParam ) {
			//	MessageBox(NULL,"qqq","QQQ!",MB_OK);
			//}

			if ( VK_ESCAPE == (INT)wParam ) {
				if (m_pDeviceInfo->bWindowed) {
					if (m_bEscExits)
						SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_EXIT,0), 0L);
				} else {
					SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_TOGGLEFULLSCREEN,0), 0L);
				}
			}
			break;

	
        case WM_COMMAND:
			// Handle menu and key commands
            switch( LOWORD(wParam) )
            {
				// PANORAMA
				case ID_PANORAMA:
					g_bPanorama = !g_bPanorama;
					break;

				case ID_OPTIONS_CULLING:
					m_bCull = !m_bCull;
					if (!m_bCull)
						CheckMenuItem(hmenu,ID_OPTIONS_CULLING,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_CULLING,MF_UNCHECKED);
					break;

                case ID_OPTIONS_SHADING:
					m_bFlat = !m_bFlat;
					CheckMenuItem(hmenu,ID_OPTIONS_SHADING,m_bFlat?MF_CHECKED:MF_UNCHECKED);
					break;
				
				case ID_OPTIONS_SPECULAR:
					m_bSpec = !m_bSpec;
					if (m_bSpec)
						CheckMenuItem(hmenu,ID_OPTIONS_SPECULAR,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_SPECULAR,MF_UNCHECKED);
					break;
				
				case ID_OPTIONS_FOG:
					m_bFog  = !m_bFog;
					if (m_bFog)
						CheckMenuItem(hmenu,ID_OPTIONS_FOG,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_FOG,MF_UNCHECKED);
					break;

				case ID_OPTIONS_TEXTURE:
					m_bText = !m_bText;
					if (m_bText)
						CheckMenuItem(hmenu,ID_OPTIONS_TEXTURE,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_TEXTURE,MF_UNCHECKED);
					break;
					
				case ID_OPTIONS_TEXTUREFILTERING_POINT:
					m_iTextureFilter = 0;
					break;

				case ID_OPTIONS_TEXTUREFILTERING_LINEAR:
					m_iTextureFilter = 1;
					break;

				case ID_OPTIONS_TEXTUREFILTERING_ANISOTROPIC:
					m_iTextureFilter = 2;
					break;

				case ID_OPTIONS_LENSFLARE:
					g_bDrawLensFlare = !g_bDrawLensFlare;
					if (g_bDrawLensFlare) {
						CheckMenuItem(hmenu,ID_OPTIONS_LENSFLARE,MF_CHECKED);
						EnableMenuItem(hmenu,ID_OPTIONS_SUN,MF_ENABLED);
						EnableMenuItem(hmenu,ID_OPTIONS_FLARE,MF_ENABLED);
					} else {
						CheckMenuItem(hmenu,ID_OPTIONS_LENSFLARE,MF_UNCHECKED);
						EnableMenuItem(hmenu,ID_OPTIONS_SUN,MF_GRAYED);
						EnableMenuItem(hmenu,ID_OPTIONS_FLARE,MF_GRAYED);
					}
					break;
				case ID_OPTIONS_SUN:
					g_bSun	= !g_bSun;
					if (g_bSun)
						CheckMenuItem(hmenu,ID_OPTIONS_SUN,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_SUN,MF_UNCHECKED);
					break;
				case ID_OPTIONS_FLARE:
					g_bFlare = !g_bFlare;
					if (g_bFlare)
						CheckMenuItem(hmenu,ID_OPTIONS_FLARE,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_FLARE,MF_UNCHECKED);
					break;

				case ID_OPTIONS_SHADOW:
					m_bDrawShadow = !m_bDrawShadow;
					if (m_bDrawShadow)
						CheckMenuItem(hmenu,ID_OPTIONS_SHADOW,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_SHADOW,MF_UNCHECKED);
					break;

				case ID_OPTIONS_SHADOWVOLUME:
					m_bDrawShadowVolume = !m_bDrawShadowVolume;
					if (m_bDrawShadowVolume)
						CheckMenuItem(hmenu,ID_OPTIONS_SHADOWVOLUME,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_SHADOWVOLUME,MF_UNCHECKED);
					break;

				case ID_OPTIONS_RUNWAY:
					g_bRunway = !g_bRunway;
					if (g_bRunway)
						CheckMenuItem(hmenu,ID_OPTIONS_RUNWAY,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_RUNWAY,MF_UNCHECKED);
					break;

				case ID_OPTIONS_HELIPAD:
					g_bHelipad = !g_bHelipad;
					if (g_bHelipad)
						CheckMenuItem(hmenu,ID_OPTIONS_HELIPAD,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_HELIPAD,MF_UNCHECKED);
					break;

				case ID_OPTIONS_FIELD:
					g_bField = !g_bField;
					if (g_bField)
						CheckMenuItem(hmenu,ID_OPTIONS_FIELD,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_FIELD,MF_UNCHECKED);
					break;

				case ID_OPTIONS_TREES:
					g_bTrees = !g_bTrees;
					if (g_bTrees)
						CheckMenuItem(hmenu,ID_OPTIONS_TREES,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_TREES,MF_UNCHECKED);
					break;

				case ID_OPTIONS_WINDSOCK:
					g_bWindsock = !g_bWindsock;
					if (g_bWindsock)
						CheckMenuItem(hmenu,ID_OPTIONS_WINDSOCK,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_WINDSOCK,MF_UNCHECKED);
					break;

				case ID_OPTIONS_WIND:
					g_bWind = !g_bWind;
					if (g_bWind)
						CheckMenuItem(hmenu,ID_OPTIONS_WIND,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_WIND,MF_UNCHECKED);
					break;
				
				case ID_OPTIONS_SOLID:
					CheckMenuItem(hmenu,ID_OPTIONS_SOLID,MF_CHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_WIREFRAME,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_POINT,MF_UNCHECKED);
					m_bSolid = TRUE;
					m_bWire = FALSE;
					m_bPoint = FALSE;
					break;
                case ID_OPTIONS_WIREFRAME:
					CheckMenuItem(hmenu,ID_OPTIONS_SOLID,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_WIREFRAME,MF_CHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_POINT,MF_UNCHECKED);
					m_bSolid = FALSE;
					m_bWire = TRUE;
					m_bPoint = FALSE;
					break;
				case ID_OPTIONS_POINT:
					CheckMenuItem(hmenu,ID_OPTIONS_SOLID,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_WIREFRAME,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_OPTIONS_POINT,MF_CHECKED);
					m_bSolid = FALSE;
					m_bWire = FALSE;
					m_bPoint = TRUE; 
					break;

				case ID_OPTIONS_EXHAUSTSMOKE:
					m_bExhaustSmoke = !m_bExhaustSmoke;
					if (m_bExhaustSmoke)
						CheckMenuItem(hmenu,ID_OPTIONS_EXHAUSTSMOKE,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_EXHAUSTSMOKE,MF_UNCHECKED);
					break;

				case ID_OPTIONS_SOUND:
					m_bSound = !m_bSound;
					if (m_bSound)
						CheckMenuItem(hmenu,ID_OPTIONS_SOUND,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_OPTIONS_SOUND,MF_UNCHECKED);
					break;

				case IDC_KEYBOARD:
					CheckMenuItem(hmenu,IDC_KEYBOARD,MF_CHECKED);
					CheckMenuItem(hmenu,IDC_MOUSE,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_JOYSTICK,MF_UNCHECKED);
					g_bUseKeyboard = TRUE;
					g_bUseMouse = FALSE;
					g_bUseJoystick = FALSE;
					SelectInputDevice(hWnd);
					PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 ); // reacquire the fucker!!!!
					m_fSpeed = 0.0f;
					// Hell, we don't get back the cursor when coming back from fullscreen
					// after mouse input was selected. This, together with ShowCursor(FALSE);
					// in case IDC_MOUSE, solves it but has the nasty effect that it also hides
					// the cursor in dialog boxes...hell
					// Doing this alone works, but if we select mouse input again the mouse
					// shows up...
					// An even more serious issue is this: RT Config showing, mouse input, switch
					// to fullscreen and back, we automatically get a cursor while still in
					// mouse input, but if we go to a menu with this cursor the menu and app
					// hang. We can only get back by Alt+Tab.
					//if (m_pDeviceInfo->bWindowed){
					//	while(ShowCursor(TRUE) < 0);
					//}
					//
					// TEST:
					// This kludge solves it but shows a nasty flash
					//SendMessage((HWND)g_hWndSikorsky, WM_COMMAND, MAKEWPARAM(IDM_RT_CONFIGURATION,0), 0L);
					//SendMessage((HWND)g_hWndSikorsky, WM_COMMAND, MAKEWPARAM(IDM_RT_CONFIGURATION,0), 0L);
					// TEST:
					//SetForegroundWindow(hWnd);
					//SetFocus(hWnd);
					//SetActiveWindow(hWnd);
					break;
				case IDC_MOUSE:
					CheckMenuItem(hmenu,IDC_KEYBOARD,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_MOUSE,MF_CHECKED);
					CheckMenuItem(hmenu,IDC_JOYSTICK,MF_UNCHECKED);
					g_bUseKeyboard = FALSE;
					g_bUseMouse = TRUE;
					g_bUseJoystick = FALSE;
					SelectInputDevice(hWnd);
					PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 ); // reacquire the fucker!!!!
					m_fSpeed = 0.0f;
					// Hell, we don't get back the cursor when coming back from fullscreen
					// after mouse input was selected. This solves it.
					// But this has the nasty effect that it also hides the cursor in
					// dialog boxes...hell
					//if (m_pDeviceInfo->bWindowed){
					//	ShowCursor(FALSE);
					//	//while(ShowCursor(FALSE) > -1);
					//}									
					break;
				case IDC_JOYSTICK:
					CheckMenuItem(hmenu,IDC_KEYBOARD,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_MOUSE,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_JOYSTICK,MF_CHECKED);
					g_bUseKeyboard = FALSE;
					g_bUseMouse = FALSE;
					g_bUseJoystick = TRUE;
					SelectInputDevice(hWnd);
					PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 ); // reacquire the fucker!!!!
					break;
				case IDC_TRANSMITTER:
					//Pause(TRUE); // always put this before a dialog or messagebox: 
					// it pauses the app and makes sure parent window does not lose "patches"
					// when messagebox or dialog is moved
					//MessageBox(hWnd,"You already got it!","Transmitter control",MB_OK);
					//Pause(FALSE);
					g_bUseTransmitter = !g_bUseTransmitter;
					if (g_bUseTransmitter)
						CheckMenuItem(hmenu,IDC_TRANSMITTER,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDC_TRANSMITTER,MF_UNCHECKED);
					break;

				case IDC_SMOOTHCONTROLS:
					m_bSmoothControls = !m_bSmoothControls;
					if (m_bSmoothControls)
						CheckMenuItem(hmenu,IDC_SMOOTHCONTROLS,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDC_SMOOTHCONTROLS,MF_UNCHECKED);
					g_vecVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
					g_vecAngularVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);
					break;

				case IDC_SMOOTHCAMCONTROLS:
					m_bSmoothCamControls = !m_bSmoothCamControls;
					if (m_bSmoothCamControls)
						CheckMenuItem(hmenu,IDC_SMOOTHCAMCONTROLS,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDC_SMOOTHCAMCONTROLS,MF_UNCHECKED);
					break;
				
				case IDM_VIBRATION:
					m_bVibration = !m_bVibration;
					if (m_bVibration)
						CheckMenuItem(hmenu,IDM_VIBRATION,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDM_VIBRATION,MF_UNCHECKED);
					break;
				case IDM_TURBULENCE:
					m_bTurbulence = !m_bTurbulence;
					if (m_bTurbulence)
						CheckMenuItem(hmenu,IDM_TURBULENCE,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDM_TURBULENCE,MF_UNCHECKED);
					break;

				case IDM_FLIGHT_RECORDER:
//					if (g_hWndTools) {
//						ShowWindow(g_hWndTools, SW_SHOW);
//						SetFocus(g_hWndTools);
//						SendMessage(g_hWndTools, WM_SYSCOMMAND, MAKEWPARAM(0,0), 0L); //restore
//					} else {
//						SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_RT_CONFIGURATION,0), 0L);
//					}
//					bShowingTools = true;
//					TabCtrl_SetCurSel(g_hTabControl, 9);
//					TabChange(g_hWndTools);

					// small rec/play tool window
					Pause(TRUE);
					g_hInst = (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE );
					if (!g_hWndFlightRec) {
						g_bShowingFlightRec = true;
						if (g_bFlightRecShowSkin) {
							g_hWndFlightRec = CreateDialog( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
										MAKEINTRESOURCE(IDD_FLIGHTREC2), hWnd,
										(DLGPROC)FlightRecProc2 );
						} else {
							g_hWndFlightRec = CreateDialog( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
										MAKEINTRESOURCE(IDD_FLIGHTREC), hWnd,
										(DLGPROC)FlightRecProc );
						}
					} else {
						if (!g_bShowingFlightRec) {
							g_bShowingFlightRec = true;
							ShowWindow(g_hWndFlightRec, SW_SHOW);
							SetFocus(g_hWndFlightRec);
						} else {
							g_bShowingFlightRec = false;
							ShowWindow(g_hWndFlightRec, SW_HIDE);
							//SetFocus(g_hWndFlightRec);
						}
					}					
					Pause(FALSE);					
					break;

				case IDM_RECORD:
					m_bRecord = !m_bRecord;
					if (m_bRecord) {
						CheckMenuItem(hmenu,IDM_RECORD,MF_CHECKED);
						
						// show the Flight Recorder and go to beginning of file
						//SendMessage(hWnd, WM_COMMAND, MAKEWPARAM(IDM_FLIGHT_RECORDER,0), 0L);
						//SendMessage(g_hTabCurrent, WM_COMMAND, MAKEWPARAM(IDC_BUTTON_BEGIN,0), 0L);
					} else {
						CheckMenuItem(hmenu,IDM_RECORD,MF_UNCHECKED);
					}
					break;

				case IDM_PLAYBACK:
					m_bPlayBack = !m_bPlayBack;
					if (m_bPlayBack) {
						CheckMenuItem(hmenu,IDM_PLAYBACK,MF_CHECKED);

						// show the Flight Recorder and go to beginning of file
						//SendMessage(hWnd, WM_COMMAND, MAKEWPARAM(IDM_FLIGHT_RECORDER,0), 0L);
						//SendMessage(g_hTabCurrent, WM_COMMAND, MAKEWPARAM(IDC_BUTTON_BEGIN,0), 0L);
					} else {
						CheckMenuItem(hmenu,IDM_PLAYBACK,MF_UNCHECKED);
					}
					break;

				case IDC_ROTARYWING:
					CheckMenuItem(hmenu,IDC_ROTARYWING,MF_CHECKED);
					CheckMenuItem(hmenu,IDC_FIXEDWING,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_6DOF,MF_UNCHECKED);
					m_bRotaryWing = TRUE;
					m_bFixedWing = FALSE;
					m_b6DOF = FALSE;				
					break;
                case IDC_FIXEDWING:
					CheckMenuItem(hmenu,IDC_ROTARYWING,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_FIXEDWING,MF_CHECKED);
					CheckMenuItem(hmenu,IDC_6DOF,MF_UNCHECKED);
					m_bRotaryWing = FALSE;
					m_bFixedWing = TRUE;
					m_b6DOF = FALSE;
					break;
				case IDC_6DOF:
					CheckMenuItem(hmenu,IDC_ROTARYWING,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_FIXEDWING,MF_UNCHECKED);
					CheckMenuItem(hmenu,IDC_6DOF,MF_CHECKED);
					m_bRotaryWing = FALSE;
					m_bFixedWing = FALSE;
					m_b6DOF = TRUE;				
					break;

				case ID_PILOTPOSITION1:
					CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_CHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
					m_fCamX = PILOTPOSITION1_X;
					m_fCamY = PILOTPOSITION1_Y;
					m_fCamZ = PILOTPOSITION1_Z;
					m_uPilotPosition = 1;
					break;
                case ID_PILOTPOSITION2:
					CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_CHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
					m_fCamX = PILOTPOSITION2_X;
					m_fCamY = PILOTPOSITION2_Y;
					m_fCamZ = PILOTPOSITION2_Z;
					m_uPilotPosition = 2;
					break;
				case ID_PILOTPOSITION3:
					CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_CHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
					m_fCamX = PILOTPOSITION3_X;
					m_fCamY = PILOTPOSITION3_Y;
					m_fCamZ = PILOTPOSITION3_Z;
					m_uPilotPosition = 3;
					break;
				case ID_PILOTPOSITION4:
					CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_CHECKED);
					m_fCamX = PILOTPOSITION4_X;
					m_fCamY = PILOTPOSITION4_Y;
					m_fCamZ = PILOTPOSITION4_Z;
					m_uPilotPosition = 4;
					break;			
				case ID_SHOWPPMARKS:
					m_bShowPPMarks = !m_bShowPPMarks;
					if (m_bShowPPMarks)
						CheckMenuItem(hmenu,ID_SHOWPPMARKS,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_SHOWPPMARKS,MF_UNCHECKED);
					break;

				case ID_VIEW_RCVIEW:
					CheckMenuItem(hmenu,ID_VIEW_RCVIEW,MF_CHECKED);
					CheckMenuItem(hmenu,ID_VIEW_INMODEL,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_CHASE,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_FOLLOW,MF_UNCHECKED);
					m_bRCView = TRUE;
					m_bInModelView = FALSE;
					m_bChaseView = FALSE;
					m_bFollowView = FALSE;
					// reset cam values
					m_fCamRadsY = 0.0f;
					m_fCamRadsX = 0.0f;
					m_fCamRadsZ = 0.0f;
					//m_fCamX = 0.0f;
					//m_fCamY = 0.0f;
					//m_fCamZ = 0.0f;
					g_fZoomValue = 0.0f;
					switch(m_uPilotPosition)
					{
						case 1:
							m_fCamX = PILOTPOSITION1_X;
							m_fCamY = PILOTPOSITION1_Y;
							m_fCamZ = PILOTPOSITION1_Z;
							break;
						case 2:
							m_fCamX = PILOTPOSITION2_X;
							m_fCamY = PILOTPOSITION2_Y;
							m_fCamZ = PILOTPOSITION2_Z;
							break;
						case 3:
							m_fCamX = PILOTPOSITION3_X;
							m_fCamY = PILOTPOSITION3_Y;
							m_fCamZ = PILOTPOSITION3_Z;
							break;
						case 4:
							m_fCamX = PILOTPOSITION4_X;
							m_fCamY = PILOTPOSITION4_Y;
							m_fCamZ = PILOTPOSITION4_Z;
							break;
					}
					//PlaySound( NULL, NULL, SND_PURGE );
					SetVolume( DSBVOLUME_MIN );
					SetVolumeAllBuffers( DSBVOLUME_MIN );
					break;
                case ID_VIEW_INMODEL:
					CheckMenuItem(hmenu,ID_VIEW_RCVIEW,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_INMODEL,MF_CHECKED);
					CheckMenuItem(hmenu,ID_VIEW_CHASE,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_FOLLOW,MF_UNCHECKED);
					m_bRCView = FALSE;
					m_bInModelView = TRUE;
					m_bChaseView = FALSE;
					m_bFollowView = FALSE;
					// reset cam values
					m_fCamRadsY = 0.0f;
					m_fCamRadsX = 0.0f;
					m_fCamRadsZ = 0.0f;
					m_fCamX = 0.0f;
					m_fCamY = 0.0f;
					m_fCamZ = 0.0f;
					g_fZoomValue = 0.0f;
					//PlaySound( NULL, NULL, SND_PURGE );
					SetVolume( DSBVOLUME_MIN );
					SetVolumeAllBuffers( DSBVOLUME_MIN );
					break;
				case ID_VIEW_CHASE:
					CheckMenuItem(hmenu,ID_VIEW_RCVIEW,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_INMODEL,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_CHASE,MF_CHECKED);
					CheckMenuItem(hmenu,ID_VIEW_FOLLOW,MF_UNCHECKED);
					m_bRCView = FALSE;
					m_bInModelView = FALSE;
					m_bChaseView = TRUE;
					m_bFollowView = FALSE;
					// reset cam values
					m_fCamRadsY = 0.0f;
					m_fCamRadsX = 0.0f;
					m_fCamRadsZ = 0.0f;
					m_fCamX = 0.0f;
					m_fCamY = 0.0f;
					m_fCamZ = 0.0f;
					g_fZoomValue = 0.0f;
					//PlaySound( NULL, NULL, SND_PURGE );
					SetVolume( DSBVOLUME_MIN );
					SetVolumeAllBuffers( DSBVOLUME_MIN );
					break;
				case ID_VIEW_FOLLOW:
					CheckMenuItem(hmenu,ID_VIEW_RCVIEW,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_INMODEL,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_CHASE,MF_UNCHECKED);
					CheckMenuItem(hmenu,ID_VIEW_FOLLOW,MF_CHECKED);
					m_bRCView = FALSE;
					m_bInModelView = FALSE;
					m_bChaseView = FALSE;
					m_bFollowView = TRUE;
					// reset cam values
					m_fCamRadsY = 0.0f;
					m_fCamRadsX = 0.0f;
					m_fCamRadsZ = 0.0f;
					m_fCamX = 0.0f;
					m_fCamY = 0.0f;
					m_fCamZ = 0.0f;
					g_fZoomValue = 0.0f;
					//PlaySound( NULL, NULL, SND_PURGE );
					SetVolume( DSBVOLUME_MIN );
					SetVolumeAllBuffers( DSBVOLUME_MIN );
					break;
	
					
                case IDM_LOADFILE:
                    Pause( TRUE );
                    OpenFileDialog();

					// RM
					// TEST:
					//CreateRM(info);
					//LoadXFile("Ec-135.x"/*g_strHeliFilePath*/, info);

                    Pause( FALSE );
                    break;

				case IDM_LOADSCENERY:
                    Pause( TRUE );
					// NOT IN VERSION 1.5
                    OpenSceneryFileDialog(hWnd);
                    Pause( FALSE );
                    break;
					
				case IDM_FILE_OPENFLIGHT:
                    Pause( TRUE );
                    OpenFlightFileDialog(hWnd);
                    Pause( FALSE );
                    break;

				case IDM_FILE_SAVEFLIGHT:
                    Pause( TRUE );
// demo
#ifdef DEMO
						//DoShareWareNotice2( m_hWnd );
						MessageBox(hWnd, "This feature is not available in the demo version.",
							"R/C Sim Sikorsky Demo", MB_OK|MB_ICONINFORMATION);
#else
						SaveFlightFileDialog(hWnd);
#endif
                    Pause( FALSE );
                    break;

				case IDM_FILE_OPENCALIBRATION:
                    Pause( TRUE );
                    OpenCalibrationFileDialog(hWnd);
                    Pause( FALSE );
                    break;

				case IDM_FILE_SAVECALIBRATION:
                    Pause( TRUE );
// demo
#ifdef DEMO
						//DoShareWareNotice2( m_hWnd );
						MessageBox(hWnd, "This feature is not available in the demo version.",
							"R/C Sim Sikorsky Demo", MB_OK|MB_ICONINFORMATION);
#else
						SaveCalibrationFileDialog(hWnd);
#endif
                    Pause( FALSE );
                    break;

				case IDM_FILE_OPENSIMULATION:
                    Pause( TRUE );
                    //OpenSimulationFileDialog(hWnd);
                    Pause( FALSE );
                    break;

				case IDM_FILE_SAVESIMULATION:
                    Pause( TRUE );
					if (!g_bShareWareRegistered) {
						DoShareWareNotice2( m_hWnd );
					} else {
						//SaveSimulationFileDialog(hWnd);
					}
                    Pause( FALSE );
                    break;

				case IDM_RESET_SIMULATION:
					ResetHeli();
					//ResetCam();
					g_bResetLatency = true;

					// Recalculate speed factor
					// frame rate change expected on IDM_RESET_SIMULATION therefore
					// reset values to recalculate m_fSpeedFactor
					g_acc = 0.0f;
					g_count = 0;
					break;

				case IDM_RESET_CAMERA:
					ResetCam();
					break;
				
				// TODO: make window redraw properly when moving this dialog box
				// DONE: by using Pause()
				case IDM_SELECTINPUTDEVICE:
					Pause(TRUE);
                    DialogBox( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
                               MAKEINTRESOURCE(IDD_SELECTINPUTDEVICE), hWnd,
                               (DLGPROC)InputDeviceSelectProc );
					Pause(FALSE);
					break;

				// Yo! Let's give the dorks some help...
				case IDM_HELPTOPICS:
					//c:\windows\winhlp32.exe ..\help\rcsim.hlp
					//_spawnl( _P_WAIT, "C:\\Windows\\notepad.exe", "", NULL );
					//_spawnl( _P_WAIT, "C:\\Windows\\Winhlp32.exe", "..\\Help\\Rcsim.hlp", NULL );
					// Huh. Dork yourself...
					Pause(TRUE);
					//WinHelp( hWnd, "..\\Help\\Rcsim.hlp", HELP_FINDER, 0 );
					HtmlHelp( GetDesktopWindow(), "..\\HTMLHelp\\Rcsim.chm", HH_DISPLAY_TOPIC, NULL );
					Pause(FALSE);
					break;				

				case IDM_SYSINFO:
					// Show System Information
					if ( !GetWindowsDirectory(lpBuffer2, MAX_PATH) ) {
						dwErrorCode = GetLastError();
						sprintf(msg, "Could not get Windows Dir, Error code: %ld\n", dwErrorCode);
						MessageBox(NULL,msg,"QQQ",MB_ICONWARNING);
					} else {
						lpBuffer2[3] = '\0'; // get system drive string, e.g.: "C:\"
						strcpy(strSysInfoPath, lpBuffer2);
						strcat(strSysInfoPath, "Program Files\\Common Files\\Microsoft Shared\\MSInfo\\msinfo32.exe");
					}			

					// Note: arg0 should be filename of executable
					if ( -1 == _spawnl( _P_NOWAIT, strSysInfoPath, "msinfo32.exe", NULL ) ) {
						MessageBox(hWnd, "Could not spawn System Information.",
							"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
					}	
					break;

				case IDM_DIRECTXINFO:
					// Show DirectX Information
					if ( !GetSystemDirectory(lpBuffer3, MAX_PATH) ) {
						dwErrorCode = GetLastError();
						sprintf(msg, "Could not get System Dir, Error code: %ld\n", dwErrorCode);
						MessageBox(NULL,msg,"QQQ",MB_ICONWARNING);
					} else {
						strcpy(strDirectXInfoPath, lpBuffer3);
						strcat(strDirectXInfoPath, "\\dxdiag.exe");
					}			

					if ( -1 == _spawnl( _P_NOWAIT, strDirectXInfoPath, "dxdiag.exe", NULL ) ) {
						MessageBox(hWnd, "Could not spawn DirectX Information.",
							"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
					}	
					break;

				case IDM_WEBSITE:
					ShellExecute( NULL, "open", "http://home.zonnet.nl/blacksphere2/", "",
						"c:\\", SW_SHOWNORMAL );
					break;

				case IDM_REGISTER:
					//MessageBox(NULL,"Register","QQQ!",MB_OK);
					Pause(TRUE);
					//if (!g_bShareWareRegistered) {
// demo
#ifdef DEMO
						DoShareWareNotice2( m_hWnd );
#else
						DoShareWareNotice4( m_hWnd );

						if (g_bShareWareRegistered) {
							MessageBox(hWnd,"This copy of R/C Sim Sikorsky is already registered.","Register",MB_OK|MB_ICONINFORMATION);
						}
#endif

//					} else {
//						// Show the dialog box so the user can see that he has registered
//						//DialogBox( g_hinst, MAKEINTRESOURCE(IDD_SHAREWARE3), hWnd,
//						//						   (DLGPROC)ShareWareProc );
//						if ( GetKeyState(VK_CONTROL) & 0x80 ) {
//							// show the register dialog
//							DialogBox( g_hinst, MAKEINTRESOURCE(IDD_REGISTER2), hWnd,
//												   (DLGPROC)RegisterProc2 );
//						} else {
//							MessageBox(hWnd,"This copy of R/C Sim Sikorsky is already registered.","Register",MB_OK|MB_ICONINFORMATION);
//						}
//					}
					Pause(FALSE);
					break;

				case IDM_ABOUT:
                    // Display the About box
					// Catch this message ourselves because framework does sucky centering
                    Pause(TRUE);

					// spiffy 3D about box only when windowed
					if (m_pDeviceInfo->bWindowed) {
						DialogBox( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_ABOUT3), hWnd,
								   (DLGPROC)AboutProc3 );
					} else {
						DialogBox( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_ABOUT2), hWnd,
								   (DLGPROC)AboutProc2 );
					}

                    Pause(FALSE);
                    return 0; // prevent the default about dialog from showing
					break;

				case ID_VIEW_CONSOLE:
                    // Display the Console
					// Could make this RT in windowed mode too!!!
                    Pause(TRUE);
					// NO: we cannot do a modeless dialog because the main app will
					// still have the keyboard focus and the Return key then does
					// work in the edit window.
                    // modeless console when windowed
					//if (m_pDeviceInfo->bWindowed) {
					//	CreateDialog( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
					//			   MAKEINTRESOURCE(IDD_CONSOLE), /*hWnd*/NULL,
					//			   (DLGPROC)ConsoleProc );
					//} else {
						DialogBox( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
                             MAKEINTRESOURCE(IDD_CONSOLE), hWnd,
							   (DLGPROC)ConsoleProc );
					//}
                    Pause(FALSE);
                    break;

				case IDM_PREFERENCES:
					// Display the Preferences Dialog
                    Pause(TRUE);
					// lParam is needed
                    DialogBoxParam( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
                               MAKEINTRESOURCE(IDD_PREFERENCES), hWnd,
							   (DLGPROC)PreferencesProc, (LPARAM)&m_pDeviceInfo );
                    Pause(FALSE);
					break;

				case IDM_CALIBRATION:
					// NOTE1: current dir is \media
					// NOTE2: This menu item will be disabled when vxd is not loaded.
					// We only show Calibration console if vxd is loaded otherwise there is
					// too much risk of crashing
					
					// check is Calibration up
					RegistryRead3();

					if ( (HWND)g_hWndCalibration ) {
						//MessageBeep(-1);
						ShowWindow( (HWND)g_hWndCalibration, SW_RESTORE );
						SetForegroundWindow( (HWND)g_hWndCalibration );
					} 
					else
					{
#ifdef _DEBUG
						//GetCurrentDirectory( sizeof(Buffer), Buffer );
						//SetCurrentDirectory("F:\\interface\\TxintguiSDK\\Bigpush\\Debug");
						if ( -1 == _spawnl( _P_NOWAIT, "E:\\Bigpush\\Debug\\Bigpush.exe", "Bigpush.exe", NULL ) ) {
								MessageBox(m_hWnd, "Could not spawn Calibration application.",
									"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
						}
						//SetCurrentDirectory( Buffer );
#else					
						if ( -1 == _spawnl( _P_NOWAIT, "..\\bin\\Bigpush.exe", "Bigpush.exe", NULL ) ) {
								MessageBox(m_hWnd, "Could not spawn Calibration application.",
									"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
						}					
#endif //_DEBUG				

					}
					break;

				case IDM_CONFIGURATION:
					// Display the Configuration Prop Sheet
					// Or rather, let's do a dialog with a tree view since we can no longer
					// populate all our config options in a two-tabline prop sheet
                    Pause(TRUE);
					g_hinst = (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE );
					//DoPropertySheet(hWnd);
					DialogBox( g_hinst, MAKEINTRESOURCE(IDD_CONFIGURATION), hWnd,
								   (DLGPROC)ConfigurationProc2 );

					Pause(FALSE);
					break;

				case IDM_RT_CONFIGURATION:
					// Real-Time Configuration Toolwindow (works only in windowed mode)
					// TODO: make it work in Fullscreen mode as well. THIS IS POSSIBLE!!!
					// See: DDraw FSWindow Sample
					// Well, we've tried to implement this code but can't get it to work.
					// It should not be too difficult: every frame take a bitmap of the
					// window and blit it into the backbuffer.
					// Maybe we better not do it with the FSWindow sample code as that is
					// based on slow GDI blitting to DirectDraw surfaces. When we want to
					// port this app to DirectX9 or later, we could not use it. Better
					// implement it ourselves using fast D3D blits of the window bitmap
					// into the backbuffer.
					// Another issue is of course that the windows in XP are no longer
					// rectangular. That would require doing transparent blits to get
					// the window's top corners nicely rounded. And transparent blits can
					// only be done in D3D, not in DirectDraw.
					//
					// Cool!!! The Framework automatically detects if it's a modeless dialog
					// and then doesn't pause. We'll never get the "white patches" with 
					// modeless dialogs so Pause isn't necessary
					// NOTE NOTE: modeless dialogs must have WS_VISIBLE
					Pause(TRUE);
					g_hInst = (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE );
					if (!g_hWndTools) {
						bShowingTools = true;
						g_hWndTools = CreateDialog( (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_RT_CONFIGURATION), hWnd,
								   (DLGPROC)RTConfigurationProc );
					} else {
						if (!bShowingTools) {
							bShowingTools = true;
							ShowWindow(g_hWndTools, SW_SHOW);
							SetFocus(g_hWndTools);
						} else {
							bShowingTools = false;
							ShowWindow(g_hWndTools, SW_HIDE);
							//SetFocus(g_hWndTools);
						}
					}
					Pause(FALSE);
					break;

				case IDM_STATUSBAR:
					g_bStatusBar = !g_bStatusBar;
					if (g_bStatusBar)
						CheckMenuItem(hmenu,IDM_STATUSBAR,MF_CHECKED);
					else
						CheckMenuItem(hmenu,IDM_STATUSBAR,MF_UNCHECKED);
					
					// NOTE: ShowWindow() don't cut it: got to destroy and create
					if ( g_bStatusBar ) {
						DestroyWindow( g_hwndStatus );
						DestroyWindow( g_hwndClient );
				
						g_hinst = (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE );
						DoCreateStatusWindow(hWnd, g_hinst);
						DoCreateClientWindow(hWnd, g_hinst);
					} else {
						DestroyWindow( g_hwndStatus );
						DestroyWindow( g_hwndClient );

						g_hinst = (HINSTANCE)GetWindowLong( hWnd, GWL_HINSTANCE );
						DoCreateClientWindow(hWnd, g_hinst);
					}
					
					// Got to reacquire Input!!!
					//m_bReAcquire = TRUE;

					
					GetWindowRect( hWnd, &rc );
					bRedraw = true;

					// We fake it: send a resize message to reinit D3D.
					// This way we don't need IDM_STATUSBAR in CD3DApplication::MsgProc()
					// NOTE: mind the Redraw Kludge here
					if (bMaximized)
						SendMessage(m_hWnd, WM_SIZE, SIZE_MAXIMIZED, MAKELPARAM(rc.right-rc.left, rc.bottom-rc.top));
					else 
						SendMessage(m_hWnd, WM_SIZE, SIZE_RESTORED, MAKELPARAM(rc.right-rc.left, rc.bottom-rc.top));

					//if (g_bStatusBar)
					//	MoveWindow(g_hwndClient, rc.left, rc.top, dwWidth, dwHeight-20 ,TRUE);
					//else
					//	MoveWindow(g_hwndClient, rc.left, rc.top, dwWidth, dwHeight-50 ,TRUE);

					//SendMessage( g_hwndClient, WM_SIZE, wParam, lParam );
					//MoveWindow(g_hwndClient, rc.left, rc.top, LOWORD(lParam), HIWORD(lParam), TRUE);
					//ShowWindow( g_hwndClient, SW_SHOWMAXIMIZED );

					// always relative to client area of parent window
					//SetWindowPos(g_hwndClient, HWND_NOTOPMOST, 0, 0, 50, 50, SWP_SHOWWINDOW);
					//ShowWindow( g_hwndClient, SW_HIDE );
					//MoveWindow(g_hwndClient, 50, 50, 200, 100, TRUE);
					//DestroyWindow(g_hwndClient);
					break;


				case IDM_ZOOM:	
					if (!g_bZooming) {
						g_bZooming = true;
						g_bZoomIn = !g_bZoomIn;
					} else {
						g_bZooming = false;
					}
					break;

				case IDM_TOGGLEFULLSCREEN:
					// this msg is further handled by CD3DApplication::MsgProc()

					// TODO: heli spins after a toggle with mouse or joy input. Solve this!
					// This does not work:
					//PostMessage( hWnd, WM_SYNCACQUIRE, 0, 0 ); // reacquire the fucker!!!!

					//MessageBox(NULL,"qqq","QQQ",MB_OK);

					// nice kludge
					m_bReAcquire = TRUE;

					// Hell, we don't get back the cursor when coming back from fullscreen with
					// keyboard input selected, after mouse input was selected. This solves it.
					if (!m_pDeviceInfo->bWindowed && g_bUseKeyboard){
						//SetFocus(m_hWnd);
						//SetForegroundWindow(m_hWnd);
						//SetCursor( LoadCursor(NULL,IDC_ARROW) );
						//ShowCursor(TRUE);
						while(ShowCursor(TRUE) < 0);
					}

					// Recalculate speed factor
					// frame rate change expected on IDM_TOGGLEFULLSCREEN therefore
					// reset values to recalculate m_fSpeedFactor
					g_acc = 0.0f;
					g_count = 0;
					break;				


				case IDM_ENTER:
					m_bSplashScreenShowing = FALSE;
					break;

				case IDM_EXIT:
					if (SPLASHSCREEN) {
						if (!m_bSplashScreenShowing) {
							m_bSplashScreenShowing = TRUE;
							return 0;
						}
					}
					break;

				case IDM_TOGGLESTART:
					// no pausing during splash screen
					if (m_bSplashScreenShowing)
						return 0;

					// NOTE: There also is a CD3DApplication::m_bFrameMoving but we
					// cannot get to that: it's private
					m_bFrameMoving = !m_bFrameMoving;

					m_bFrameStepping = false;

					// RT Pauze button should follow keyboard Pauze
					// No: during playback we want to be able to pause and then do
					// FF, REW, or Slider. That won't be possible if we Pause the simulation
					// because FrameMove() will then not be called. RT Pauze button will only
					// be used for pauzing playback.
					//

					if (g_bSyncPause) {
						if ( TabCtrl_GetCurSel(g_hTabControl) == 9 ) {
							if (m_bFrameMoving) {
								//MessageBeep(-1);
								CheckDlgButton(g_hTabCurrent, IDC_CHECK_PAUSE, BST_UNCHECKED);
								//g_bRTButtonPauseChecked = false;
							} else {
								CheckDlgButton(g_hTabCurrent, IDC_CHECK_PAUSE, BST_CHECKED);
								//g_bRTButtonPauseChecked = true;
							}
						}

						if ( g_hWndFlightRec ) {
							if (m_bFrameMoving) {
								//MessageBeep(-1);
								//CheckDlgButton(g_hWndFlightRec, IDC_CHECK_PAUSE, BST_UNCHECKED);
								//g_bFRButtonPauseChecked = false;
							} else {
								//CheckDlgButton(g_hWndFlightRec, IDC_CHECK_PAUSE, BST_CHECKED);
								//g_bFRButtonPauseChecked = true;
							}
						}
					}

					// kludge: pressing RT Pauze button won't kill sound, now it will
					if (!m_bFrameMoving) {
						SetVolume( DSBVOLUME_MIN );
						SetVolumeAllBuffers( DSBVOLUME_MIN );
					}
					break;

				case IDM_SINGLESTEP:
					// no frame stepping during splash screen
					if (m_bSplashScreenShowing)
						return 0;

					m_bFrameStepping = true;					
					//CheckDlgButton(g_hTabCurrent, IDC_CHECK_PAUSE, BST_CHECKED);
					break;


				case ID_SETMOTIONRATE_UP: // 9
					// NOTE: RTConfig Motion Rate value is now set in message loop instead
					// of game loop to get discrete values on fast systems. More values will
					// follow this change
					g_fMotionRate += 0.01f;
					// limit
					if (g_fMotionRate > 2.00f)
						g_fMotionRate = 2.00f;
					break;
				case ID_SETMOTIONRATE_UPFAST: // Shift+9
					g_fMotionRate += 0.10f;
					// limit
					if (g_fMotionRate > 2.00f)
						g_fMotionRate = 2.00f;
					break;
				case ID_SETMOTIONRATE_DOWN: // Ctrl+9
					g_fMotionRate -= 0.01f;
					// limit
					if (g_fMotionRate < 0.01f)
						g_fMotionRate = 0.01f;
					break;
				case ID_SETMOTIONRATE_DOWNFAST: // Ctrl+Shift+9
					g_fMotionRate -= 0.10f;
					// limit
					if (g_fMotionRate < 0.01f)
						g_fMotionRate = 0.01f;
					break;
					
				case ID_SHOWVALUES: // R helper values
					g_bShowValues = !g_bShowValues;
					break;

				case ID_SHOWSTATS: // T frame rate
					m_bShowStats = !m_bShowStats;
					break;

				case ID_SHOWFLIGHTINFO: // I flight info
					m_bShowFlightInfo = !m_bShowFlightInfo;
					if (m_bShowFlightInfo)
						CheckMenuItem(hmenu,ID_SHOWFLIGHTINFO,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_SHOWFLIGHTINFO,MF_UNCHECKED);
					break;

				case ID_SHOWCHANNELS: // Y channel meters
					m_bShowChannels = !m_bShowChannels;
					if (m_bShowChannels)
						CheckMenuItem(hmenu,ID_SHOWCHANNELS,MF_CHECKED);
					else
						CheckMenuItem(hmenu,ID_SHOWCHANNELS,MF_UNCHECKED);
					break;

				case ID_TEXTCOLOR:
					/*if (m_bShowFlightInfo)*/ i++; 
					if (i==6) i=0;
					switch(i) {
						case 0: g_crTextColor = RGB(0,0,0);       break;
						case 1: g_crTextColor = RGB(255,255,255); break;
						case 2: g_crTextColor = RGB(255,0,0);     break;
						case 3: g_crTextColor = RGB(0,0,255);     break;
						case 4: g_crTextColor = RGB(255,255,0);   break;
						case 5: g_crTextColor = RGB(0,255,0);     break;
					}
					break;

				case ID_SCREENSHOT:
					DoScreenShot();
					break;

			}			// switch( LOWORD(wParam) )		
			break; // case WM_COMMAND				


		case WM_CLOSE:
			// write the lot to reg
			RegistryWrite();

			// write filepath and filename to reg
			RegistryWrite7();
			break;

		case WM_DESTROY:
			DestroyInputDevice();
			DestroyDInput();

			// tell registry we are down
			g_hWndSikorsky = (DWORD)NULL;
			RegistryWrite4();

			// drag-drop support end
			DragAcceptFiles(hWnd, FALSE); 
			break;

    } // switch( uMsg )



	// handle Mode menu item commands
	if (uMsg == WM_COMMAND) {
		if ( LOWORD(wParam) >= 0 && LOWORD(wParam) <= GetMenuItemCount(hmenuMode) ) {
			
			// NOTE1: LOWORD(wParam) is the menu ID, which we have set with
			// miiMode.wID in InitModeMenu(). This ID corresponds to the mode we
			// want to have. Alternately, it is possible to associate other data with
			// a menu item via miiMode.dwItemData
			// For instance, if we need another dynamically built submenu, we cannot
			// use the same menu IDs. But if we still want to associate a value of
			// e.g. 0 with a menu item, we'd have to do that via via miiMode.dwItemData
			// Hoewel, dit schijnt niet mogelijk te zijn: want hoe checken we hier welke
			// MENUITEMINFO bij het menu ID hoort???
			// Misschien zo: in WM_MENUSELECT een static MENUITEMINFO vullen. Als de user
			// klikt komen we in WM_COMMAND waar we die MENUITEMINFO kunnen uitlezen.

			// NOTE2: from the WM_COMMAND docs: If an application enables a menu separator,
			// the system sends a WM_COMMAND message with the low-word of the wParam 
			// parameter set to zero when the user selects the separator.
			// Echter: als we op separator klikken krijgen we geen WM_COMMAND
			// Wel is het zo dat als we met de muis over de separator zijn (== selected)
			// we in WM_MENUSELECT een LOWORD(wParam) == 0 hebben.
			
			//MessageBox(NULL,"qqq","QQQ!",MB_OK);
			// Switch to the requested fullscreen mode
			//GetMenuItemInfo( hmenuMode, (UINT)LOWORD(wParam), TRUE, &miiMode );
			//sprintf( msg, "Switch to %s fullscreen mode", miiMode.dwTypeData );
			//m_pDeviceInfo->dwCurrentMode = (DWORD)miiMode.dwTypeData; // must have "640 x 480 x 16" string? No!
			//m_pDeviceInfo->dwCurrentMode = LOWORD(wParam);

			//m_pDeviceInfo->bHardware = TRUE;
			//D3DEnum_SelectDefaultDevice( &m_pDeviceInfo, D3DENUM_SOFTWAREONLY );
			//D3DEnum_SelectDefaultDevice( &m_pDeviceInfo, 0L );

			// Always try to get a hardware device.
			// This makes sure that when we are in a software device, we'll try to
			// get back into a hardware device, without using the Device Change dialog.
			D3DEnum_SelectDefaultDevice( &m_pDeviceInfo, 0L );
			
			//DWORD i = LOWORD(wParam);
			DWORD i = miiMode.dwItemData;

			// See: DeviceEnumCallback() /////////////
			// Also see: ChangeDeviceProc() in d3denum.cpp
			//m_pDeviceInfo->pddsdModes[i].dwWidth = 320;
			//m_pDeviceInfo->pddsdModes[i].dwHeight = 200;
			//m_pDeviceInfo->pddsdModes[i].ddpfPixelFormat.dwRGBBitCount = 16;

			m_pDeviceInfo->ddsdFullscreenMode = m_pDeviceInfo->pddsdModes[i];
			m_pDeviceInfo->dwCurrentMode = i;		
			//////////////////////////////////////////
			
		
			//SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_TOGGLEFULLSCREEN,0), 0L);
			SendMessage(m_hWnd, WM_COMMAND, MAKEWPARAM(IDM_CHANGEFULLSCREEN,0), 0L);
		}
	}



    // Fall through to the app's main windows proc function
    return CD3DApplication::MsgProc( hWnd, uMsg, wParam, lParam );
}





// Key polling ////////////////////////////////////////
void CMyD3DApplication::GetKeys()
{
	if ( GetKeyState('A') & 0x80 ) {
		//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
		YawLeft();
	}
	if ( GetKeyState('D') & 0x80 ) {
		YawRight();
	}

	if ( GetKeyState(VK_DOWN) & 0x80 ) {
		//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
		PitchUp();
	}
	if ( GetKeyState(VK_UP) & 0x80 ) {
		PitchDown();
	}

	if ( GetKeyState(VK_RIGHT) & 0x80 ) {
		//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
		RollRight();
	}
	if ( GetKeyState(VK_LEFT) & 0x80 ) {
		RollLeft();
	}


	// collective binary 
	// NOTE: samen met joystick werkt dit niet altijd: de joystick en keys
	// zitten elkaar dan in de weg waarbij de joystick Master is
	if (m_bRotaryWing || m_b6DOF) {
		if ( GetKeyState(VK_HOME) & 0x80 ) YUp();
		if ( GetKeyState(VK_END)  & 0x80 ) YDown();

		if ( GetKeyState('W') & 0x80 ) YUp();
		if ( GetKeyState('S')  & 0x80 ) YDown();
	}
	if (m_bFixedWing) {
		if ( GetKeyState(VK_HOME) & 0x80 ) ZUp();
		if ( GetKeyState(VK_END)  & 0x80 ) ZDown();

		if ( GetKeyState('W') & 0x80 ) ZUp();
		if ( GetKeyState('S')  & 0x80 ) ZDown();
	}

	// collective incremental stepping
	if ( GetKeyState(VK_PRIOR) & 0x80 ) YUpIncremental();
	if ( GetKeyState(VK_NEXT)  & 0x80 ) YDownIncremental();
	// N.B. dit wordt elke frame gedaan (m_fY wordt ook elke frame op 0 gezet)
	//m_fY+=m_fCollective;

	

	// z-value
	// Note: the model is moving, not the camera!!!
	if (m_bRotaryWing || m_b6DOF) {
		if ( GetKeyState('C') & 0x80) {
			//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
			ZUp();
		}
		if ( GetKeyState('V') & 0x80) {
			ZDown();
		}
	}
	if (m_bFixedWing) {
		if ( GetKeyState('C') & 0x80) {
			//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
			YUp();
		}
		if ( GetKeyState('V') & 0x80) {
			YDown();
		}
	}

	// speed incremental
	if ( GetKeyState(VK_ADD)      & 0x80 ) ZUpIncremental();
	if ( GetKeyState(VK_SUBTRACT) & 0x80 ) ZDownIncremental();
	// N.B. dit wordt elke frame gedaan (m_fZ wordt ook elke frame op 0 gezet)
	if (m_bRotaryWing || m_b6DOF) {
		m_fZ-=m_fSpeed;
	}
	if (m_bFixedWing) {
		m_fY+=m_fSpeed*0.2f;
	}


	// x-value
	if ( GetKeyState('X') & 0x80 ) {
		//MessageBox(hWnd,"qqq!","QQQ!",MB_OK);
		XUp();
	}
	if ( GetKeyState('Z') & 0x80 ) {
		XDown();
	}

	// gas manette
	if ( GetKeyState(VK_INSERT) & 0x80 ) {
		ThrottleUp();
	}
	if ( GetKeyState(VK_DELETE) & 0x80 ) {
		ThrottleDown();
	}


	// CamX
	if ( GetKeyState(VK_NUMPAD4) & 0x80 ) {
		CamXDown();
	}
	if ( GetKeyState(VK_NUMPAD6) & 0x80 ) {
		CamXUp();
	}

	// CamY
	if ( GetKeyState(VK_NUMPAD7) & 0x80 ) {
		CamYUp();
	}
	if ( GetKeyState(VK_NUMPAD1) & 0x80 ) {
		CamYDown();
	}

	// CamZ
	if ( GetKeyState(VK_NUMPAD8) & 0x80 ) {
		CamZUp();
	}
	if ( GetKeyState(VK_NUMPAD2) & 0x80 ) {
		CamZDown();
	}


	if (m_bChaseView) {
		// CamRadsY
		if ( GetKeyState(VK_NUMPAD6) & 0x80 ) {
			CamYawLeft();
		}
		if ( GetKeyState(VK_NUMPAD4) & 0x80 ) {
			CamYawRight();
		}

		if ( GetKeyState(VK_NUMPAD2) & 0x80 ) {
			CamPitchUp();
		}
		if ( GetKeyState(VK_NUMPAD8) & 0x80 ) {
			CamPitchDown();
		}
	}

	// Reset Cam
	if ( GetKeyState(VK_NUMPAD5) & 0x80 ) ResetCam();


	// fog
	if ( GetKeyState(VK_SHIFT) & 0x80 ) {
		if ( GetKeyState('P') & 0x80 ) FogEndUp();
		if ( GetKeyState('L') & 0x80 ) FogStartUp();
	} else {
		if ( GetKeyState('P') & 0x80 ) FogEndDown();
		if ( GetKeyState('L') & 0x80 ) FogStartDown();		
	}

	// RT fog
	if ( g_bRTSpinnerFogEndUp     ) FogEndUp();
	if ( g_bRTSpinnerFogStartUp   ) FogStartUp();
	if ( g_bRTSpinnerFogEndDown   ) FogEndDown();
	if ( g_bRTSpinnerFogStartDown ) FogStartDown();

	// reset fog
	if ( GetKeyState('P') & 0x80 && GetKeyState('0') & 0x80 ) m_fFogEnd = 150.0f;
	if ( GetKeyState('L') & 0x80 && GetKeyState('0') & 0x80 ) m_fFogStart = 5.0f;

}


// Direct Input polling ////////////////////////////////////////
void CMyD3DApplication::GetInput()
{
	g_fCurrentTime = timeGetTime() * 0.001f;

    if( g_pdidDevice7 ) 
    {
        HRESULT      hr;
        BYTE         diks[256]; // DInput keyboard state buffer
        DIMOUSESTATE dims;      // DInput mouse state structure
        DIJOYSTATE   dijs;      // DInput joystick state structure

        // Poll the device before reading the current state. This is required
        // for some devices (joysticks) but has no effect for others (keyboard
        // and mice). Note: this uses a DIDevice2 interface for the device.
        if( FAILED( g_pdidDevice7->Poll() ) )
            return;

        if( g_bUseKeyboard ) // Get the keyboard state
            hr = g_pdidDevice7->GetDeviceState( sizeof(diks), &diks );
        
        if( g_bUseMouse )    // Else, get the mouse state
            hr = g_pdidDevice7->GetDeviceState( sizeof(DIMOUSESTATE), &dims );
        
        if( g_bUseJoystick ) // Else, get the joystick state
            hr = g_pdidDevice7->GetDeviceState( sizeof(DIJOYSTATE), &dijs );

        // Check whether the input stream has been interrupted. If so,
        // re-acquire and try again.
        if( hr == DIERR_INPUTLOST )
        {	//DisplayError("QQQ");
            hr = g_pdidDevice7->Acquire();
            if( SUCCEEDED(hr) )
                return;
        }

        // Read keyboard input
        if( g_bUseKeyboard )
        {

			g_pdiks = diks;
            // Update the variables for the player's ship
            //MovePlayer( diks[DIK_LEFT] & 0x80, diks[DIK_RIGHT] & 0x80,
            //            diks[DIK_UP] & 0x80,   diks[DIK_DOWN] & 0x80,
            //           diks[DIK_SPACE] & 0x80 );
			
			// rollen met die hap
			if ( diks[DIK_LEFT]  & 0x80 ) RollLeft();
			if ( diks[DIK_RIGHT] & 0x80 ) RollRight();
			if ( diks[DIK_UP]    & 0x80 ) PitchDown();
			if ( diks[DIK_DOWN]  & 0x80 ) PitchUp();

			// collective incremental stepping
			if ( diks[DIK_PRIOR] & 0x80 )  YUpIncremental();
			if ( diks[DIK_NEXT]  & 0x80 )  YDownIncremental();
			// N.B. dit wordt elke frame gedaan (m_fY wordt ook elke frame op 0 gezet)
			//m_fY+=m_fCollective;

			// collective binary
			if (m_bRotaryWing || m_b6DOF) {
				if ( diks[DIK_HOME] & 0x80 ) YUp();
				if ( diks[DIK_END]  & 0x80 ) YDown();

				if ( diks[DIK_W] & 0x80 ) YUp();
				if ( diks[DIK_S]  & 0x80 ) YDown();
			}
			if (m_bFixedWing) {
				if ( diks[DIK_HOME] & 0x80 ) ZUp();
				if ( diks[DIK_END]  & 0x80 ) ZDown();

				if ( diks[DIK_W] & 0x80 ) ZUp();
				if ( diks[DIK_S]  & 0x80 ) ZDown();
			}


			
			if ( diks[DIK_D]     & 0x80 ) YawRight();
			if ( diks[DIK_A]     & 0x80 ) YawLeft();

			if ( diks[DIK_X]        & 0x80 ) XUp();
			if ( diks[DIK_Z]        & 0x80 ) XDown();
			//if ( diks[DIK_ADD]      & 0x80 ) ZUp();
			//if ( diks[DIK_SUBTRACT] & 0x80 ) ZDown();

			if (m_bRotaryWing || m_b6DOF) {
				if ( diks[DIK_C]        & 0x80 ) ZUp();
				if ( diks[DIK_V]        & 0x80 ) ZDown();
			}
			if (m_bFixedWing) {
				if ( diks[DIK_C]        & 0x80 ) YUp();
				if ( diks[DIK_V]        & 0x80 ) YDown();
			}

			// speed incremental
			if ( diks[DIK_ADD]      & 0x80 ) ZUpIncremental();
			if ( diks[DIK_SUBTRACT] & 0x80 ) ZDownIncremental();
			// N.B. dit wordt elke frame gedaan (m_fZ wordt ook elke frame op 0 gezet)
			if (m_bRotaryWing || m_b6DOF) {
				m_fZ-=m_fSpeed;
			}
			if (m_bFixedWing) {
				m_fY+=m_fSpeed*0.2f;
			}


			if ( diks[DIK_INSERT] & 0x80 ) ThrottleUp();
			if ( diks[DIK_DELETE] & 0x80 ) ThrottleDown();

			// fog
			if ( GetKeyState(VK_SHIFT) & 0x80 ) {
				if ( diks[DIK_P] & 0x80 ) FogEndUp();
				if ( diks[DIK_L] & 0x80 ) FogStartUp();
			} else {
				if ( diks[DIK_P] & 0x80 ) FogEndDown();
				if ( diks[DIK_L] & 0x80 ) FogStartDown();		
			}

			// RT fog
			if ( g_bRTSpinnerFogEndUp     ) FogEndUp();
			if ( g_bRTSpinnerFogStartUp   ) FogStartUp();
			if ( g_bRTSpinnerFogEndDown   ) FogEndDown();
			if ( g_bRTSpinnerFogStartDown ) FogStartDown();

			// reset fog
			if ( diks[DIK_P] & 0x80 && diks[DIK_0] & 0x80 ) m_fFogEnd = 150.0f;
			if ( diks[DIK_L] & 0x80 && diks[DIK_0] & 0x80 ) m_fFogStart = 5.0f;


			//MessageBeep(-1);

			// CamX
			if ( diks[DIK_NUMPAD6] & 0x80 || g_bRTSpinnerCamXUp ) CamXUp();
			if ( diks[DIK_NUMPAD4] & 0x80 || g_bRTSpinnerCamXDown ) CamXDown();

			// CamY
			if ( diks[DIK_NUMPAD7] & 0x80 || g_bRTSpinnerCamYUp ) CamYUp();
			if ( diks[DIK_NUMPAD1] & 0x80 || g_bRTSpinnerCamYDown ) CamYDown();

			// CamZ
			if ( diks[DIK_NUMPAD8] & 0x80 || g_bRTSpinnerCamZUp ) CamZUp();
			if ( diks[DIK_NUMPAD2] & 0x80 || g_bRTSpinnerCamZDown ) CamZDown();


			if (m_bChaseView) {
				// CamRadsY
				if ( diks[DIK_NUMPAD6] & 0x80 || g_bRTSpinnerCamXUp ) CamYawLeft();
				if ( diks[DIK_NUMPAD4] & 0x80 || g_bRTSpinnerCamXDown ) CamYawRight();
				
				// CamRadsX
				if ( diks[DIK_NUMPAD2] & 0x80 || g_bRTSpinnerCamZDown ) CamPitchUp();
				if ( diks[DIK_NUMPAD8] & 0x80 || g_bRTSpinnerCamZUp ) CamPitchDown();
			}

			// Reset Cam
			if ( diks[DIK_NUMPAD5] & 0x80 ) ResetCam();

        }

        // Read mouse input
        if( g_bUseMouse )
        {
            // Note: on really fast computers, the mouse will appear to be 
            // still most of the time, and move in jumps. To combat this, we'll
            // keep 0.1 seconds of persistence for any up/down values we read.
            //static FLOAT fUpTime = 0.0f;
            //static FLOAT fDnTime = 0.0f;
            //if( dims.lY < 0 ) fDnTime = 0.0f, fUpTime = g_fCurrentTime+0.1f;
            //if( dims.lY > 0 ) fUpTime = 0.0f, fDnTime = g_fCurrentTime+0.1f;

            //MovePlayer( dims.lX<0, dims.lX>0, fUpTime-g_fCurrentTime > 0.0f, 
            //            fDnTime-g_fCurrentTime > 0.0f, dims.rgbButtons[0] & 0x80 );

			//if (dims.rgbButtons[1] & 0x80) {
			//	if( dims.lY < 0 ) YUp();
			//	if( dims.lY > 0 ) YDown();
			//} else {
				if( dims.lY < 0 ) PitchDown();
				if( dims.lY > 0 ) PitchUp();
			//}
			
			//if (dims.rgbButtons[0] & 0x80) {
			//	if( dims.lX < 0 ) YawLeft();
			//	if( dims.lX > 0 ) YawRight();
			//} else {
				if( dims.lX < 0 ) RollLeft();
				if( dims.lX > 0 ) RollRight();
			//}

//			if (m_bRotaryWing || m_b6DOF) {
				if ( GetKeyState(VK_CONTROL) & 0x80 ) {
					if( dims.lZ > 0 ) m_fCollective -= (0.02f/2)*CTRL_Y_SENS;
					if( dims.lZ < 0 ) m_fCollective += (0.02f/2)*CTRL_Y_SENS;
				} else if ( GetKeyState(VK_SHIFT) & 0x80 ) {
					if( dims.lZ > 0 ) m_fCollective -= (0.02f*2)*CTRL_Y_SENS;
					if( dims.lZ < 0 ) m_fCollective += (0.02f*2)*CTRL_Y_SENS;
				} else {
					if( dims.lZ > 0 ) m_fCollective -= (0.02f)*CTRL_Y_SENS; //YDownIncremental();
					if( dims.lZ < 0 ) m_fCollective += (0.02f)*CTRL_Y_SENS; //YUpIncremental();
				}
//			}
//			if (m_bFixedWing) {
//				if ( GetKeyState(VK_CONTROL) & 0x80 ) {
//					if( dims.lZ > 0 ) m_fZ -= (0.02f/2)*CTRL_Z_SENS;
//					if( dims.lZ < 0 ) m_fZ += (0.02f/2)*CTRL_Z_SENS;
//				} else if ( GetKeyState(VK_SHIFT) & 0x80 ) {
//					if( dims.lZ > 0 ) m_fZ -= (0.02f*2)*CTRL_Z_SENS;
//					if( dims.lZ < 0 ) m_fZ += (0.02f*2)*CTRL_Z_SENS;
//				} else {
//					if( dims.lZ > 0 ) m_fZ -= (0.02f)*CTRL_Z_SENS; //ZDownIncremental();
//					if( dims.lZ < 0 ) m_fZ += (0.02f)*CTRL_Z_SENS; //ZUpIncremental();
//				}
//			}




			// N.B. dit wordt elke frame gedaan (m_fY wordt ook elke frame op 0 gezet)
			//m_fY+=m_fCollective;


			//if ( dims.lZ<0 ) m_fSpeed+=0.01f;
			//if ( dims.lZ>0 ) m_fSpeed-=0.01f;
			// N.B. dit wordt elke frame gedaan
			//m_fZ-=m_fSpeed;

			if (dims.rgbButtons[0] & 0x80) YawLeft();			
			if (dims.rgbButtons[1] & 0x80) YawRight();


		}

        // Read joystick input
        if( g_bUseJoystick )
        {
            // Update the variables for the player's ship
            //MovePlayer( dijs.lX<0, dijs.lX>0, dijs.lY<0, dijs.lY>0,
            //            dijs.rgbButtons[0] & 0x80 );
			
			// rollen met die hap
			if ( dijs.lX<0 ) Roll(dijs.lX);
			if ( dijs.lX>0 ) Roll(dijs.lX);
			if ( dijs.lY<0 ) Pitch(dijs.lY);
			if ( dijs.lY>0 ) Pitch(dijs.lY);
			//if ( dijs.lZ<0 ) Yaw(dijs.lZ);
			//if ( dijs.lZ>0 ) Yaw(dijs.lZ);
			
			// joy Z axis gets collective
			if ( dijs.lZ<0 ) YUp(dijs.lZ/3);
			if ( dijs.lZ>0 ) YUp(dijs.lZ/3);
			
			// for flight info
			//m_fSpeed = (float)dijs.lZ;

 //m_fCollective+=0.002f;
 //m_fCollective-=0.002f;

			if ( dijs.rgdwPOV[0]==0 )     { m_fCollective+=0.002f; }
			if ( dijs.rgdwPOV[0]==4500 )  { m_fCollective+=0.002f; YawRight(); }
			if ( dijs.rgdwPOV[0]==9000 )  { YawRight(); }
			if ( dijs.rgdwPOV[0]==13500 ) { YawRight(); m_fCollective-=0.002f; }
			if ( dijs.rgdwPOV[0]==18000 ) { m_fCollective-=0.002f; }
			if ( dijs.rgdwPOV[0]==22500 ) { m_fCollective-=0.002f; YawLeft(); }
			if ( dijs.rgdwPOV[0]==27000 ) { YawLeft(); }
			if ( dijs.rgdwPOV[0]==31500 ) { YawLeft(); m_fCollective+=0.002f; }

			// N.B. dit wordt elke frame gedaan (m_fY wordt ook elke frame op 0 gezet)
			//m_fY+=m_fCollective;
			
			if ( dijs.rgbButtons[1] & 0x80 ) ZUp();
			if ( dijs.rgbButtons[3] & 0x80 ) ZDown();

        }
    }

}


//------------------------------------------------------------------------
// Name: GetGodMode()
// Desc: Checks and sets god mode on/off
//------------------------------------------------------------------------
void CMyD3DApplication::GetGodMode()
{
	if (GetKeyState('W') & 0x80 &&
		GetKeyState('O') & 0x80 &&
		GetKeyState('L') & 0x80 &&
		GetKeyState('F') & 0x80)
	{
		if (!g_bGodModeOn) {
			g_bGodModeOn = true;
			Pause(TRUE);
			DialogBox( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_GODMODE_ON), m_hWnd,
								   (DLGPROC)GodModeProc );
			Pause(FALSE);
		} else {
			g_bGodModeOn = false;
			Pause(TRUE);
			DialogBox( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_GODMODE_OFF), m_hWnd,
								   (DLGPROC)GodModeProc );
			Pause(FALSE);
		}
	}


	if (GetKeyState('G') & 0x80 &&
		GetKeyState('O') & 0x80 &&
		GetKeyState('D') & 0x80)
	{
		g_bGodModeOn = true;
		Pause(TRUE);
		DialogBox( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_GODMODE_ON2), m_hWnd,
								   (DLGPROC)GodModeProc );
		Pause(FALSE);
	}


	if (GetKeyState('M') & 0x80 &&
		GetKeyState('A') & 0x80 &&
		GetKeyState('N') & 0x80)
	{
		g_bGodModeOn = false;
		Pause(TRUE);
		DialogBox( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
								   MAKEINTRESOURCE(IDD_GODMODE_OFF2), m_hWnd,
								   (DLGPROC)GodModeProc );
		Pause(FALSE);
	}
}


//------------------------------------------------------------------------
// Name: ShowFlightInfo()
// Desc: Show speed, altitude, distance
// Note: We can only call this from Render()
//------------------------------------------------------------------------
void CMyD3DApplication::ShowFlightInfo()
{
	char info1[128];
	char info2[128];
	char info3[128];
	char info4[128];

//#pragma warning( disable : 4244 )

	if (m_bActiveSound) // kludge to keep revs print OK when paused
		m_fRevs = 100.0f*m_fThrottle;

//#pragma warning( default : 4244 )
 	

	switch (g_iAirSpeedUnit) {
		case 0: sprintf( info1, "Speed (Kph): %.2f", m_fAirSpeed*100 );			break;
		case 1: sprintf( info1, "Speed (kt): %.2f", (m_fAirSpeed*100)/1.852 );  break;
		case 2: sprintf( info1, "Speed (m/s): %.2f", (m_fAirSpeed*100)/3.6 );   break;
	}
	sprintf( info2, "Revs (RPM): %.0f",   m_fRevs     );
	switch (g_iAltitudeUnit) {
		case 0: sprintf( info3, "Altitude (m): %.2f", m_fAltitude ); break;
		case 1: sprintf( info3, "Altitude (ft): %.2f", m_fAltitude/0.3048 ); break;
	}	
	sprintf( info4, "Distance (m): %.2f", m_fDistance );

	// bloody truncation errors
	// if we've done high speed, then go back to zero, we get -0.00
	// let's solve it for all info
	switch (g_iAirSpeedUnit) {
		case 0: if ( strstr(info1, "-0.00") ) sprintf( info1, "Speed (Kph): 0.00" );  break;
		case 1: if ( strstr(info1, "-0.00") ) sprintf( info1, "Speed (kt): 0.00"  );  break;
		case 2: if ( strstr(info1, "-0.00") ) sprintf( info1, "Speed (m/s): 0.00"  ); break;
	}	
	if ( strstr(info2, "-0.00") ) sprintf( info2, "Revs (RPM): 0.00"   );
	switch (g_iAirSpeedUnit) {
		case 0: if ( strstr(info3, "-0.00") ) sprintf( info3, "Altitude (m): 0.00" ); break;
		case 1: if ( strstr(info3, "-0.00") ) sprintf( info3, "Altitude (ft): 0.00" ); break;
	}	
	if ( strstr(info4, "-0.00") ) sprintf( info4, "Distance (m): 0.00" );


	RECT rc;
	GetClientRect(m_hWnd, &rc); 
	COLORREF crTemp;



	if (m_pDeviceInfo->bWindowed)
	{
		// text goes higher when status bar is present
		if (g_bStatusBar) 
			rc.bottom -= 20;

		OutputText( 10, rc.bottom-66, info1 );
		OutputText( 10, rc.bottom-52, info2 );

		// red altitude warning
		if (g_bAltitudeWarning)
		{
			crTemp = g_crTextColor; 
			if (m_fAltitude <= ALTITUDE_WARNING_Y) {
				if (g_crTextColor == RGB(255,0,0))
					g_crTextColor = RGB(0,0,0);
				else
					g_crTextColor = RGB(255,0,0);
			}
			OutputText( 10, rc.bottom-38, info3 );
			g_crTextColor = crTemp;
		} else {
			OutputText( 10, rc.bottom-38, info3 );
		}

		OutputText( 10, rc.bottom-24, info4 );
	}
	else
	{
		OutputText( 10, rc.bottom-48, info1 );
		OutputText( 10, rc.bottom-34, info2 );

		// red altitude warning
		if (g_bAltitudeWarning)
		{
			crTemp = g_crTextColor; 
			if (m_fAltitude <= ALTITUDE_WARNING_Y) {
				if (g_crTextColor == RGB(255,0,0))
					g_crTextColor = RGB(0,0,0);
				else
					g_crTextColor = RGB(255,0,0);
			}
			OutputText( 10, rc.bottom-20, info3 );
			g_crTextColor = crTemp;
		} else {
			OutputText( 10, rc.bottom-20, info3 );
		}

		OutputText( 10, rc.bottom-6, info4 );
	}

	// play altitude warning sound file
	//bool bWarnAgain = false;
	//if ( m_fAltitude > ALTITUDE_WARNING_Y ) bWarnAgain = true;
	//if ( g_bLanded ) bWarnAgain = true;


	if ( g_bAltitudeWarning && m_fAltitude <= ALTITUDE_WARNING_Y && m_bSound )
	{
		if (g_bAltitudeWarningSound) {
			//PlaySound( g_strSoundFileName, NULL, SND_FILENAME|SND_ASYNC );
			//PlayBuffer2( soundWarning, FALSE );
			SetVolume2( soundWarning, DSBVOLUME_MAX );

			// At 5 meters above ground normal frequency, lower increases frequency
			DWORD dwFrequency = DWORD((22050 + 2500) - (m_fAltitude*500));
			if (dwFrequency < 100)
				dwFrequency = 100;
			SetFrequency2( soundWarning, dwFrequency );
		}

		if (g_bAltitudeWarningSpeaker) {
			MessageBeep(0xFFFFFFFF);
		}
	}

	// shut it!
	if ( m_fAltitude > ALTITUDE_WARNING_Y || m_matFileObjectMatrix._42 <= -8.5f ) {
		//StopBuffer2( soundWarning );
		SetVolume2( soundWarning, DSBVOLUME_MIN );	
	}


	// Show flight playback/record status
	if ( m_bPlayBack ) {
		//crTemp = g_crTextColor;
		//g_crTextColor = RGB(255,0,0);
		OutputText( rc.right-55, rc.top, "Playback" );
		//g_crTextColor = crTemp;

		char info5[128];
		sprintf( info5, "Frame %i/%i",  g_iFrameCount, g_iFrameTotal );
		if (m_bShowStats)
			OutputText( 10, rc.top+14, info5 );
		else
			OutputText( 10, rc.top, info5 );
	}

	if ( m_bRecord) {
		crTemp = g_crTextColor;
		g_crTextColor = RGB(255,0,0);
		OutputText( rc.right-55, rc.top, "Record" );
		g_crTextColor = crTemp;
	}
	

}


//------------------------------------------------------------------------
// Name: CollisionDetect()
// Desc: Basic, very basic
// Note: Must do this after DoDynamics()
//------------------------------------------------------------------------
BOOL CMyD3DApplication::CollisionDetect()
{
	//g_bLanded = false;

	//char msg[200];
	//sprintf( msg, "m_fY: %.2f", m_fY );
	//MessageBox(NULL,msg,"QQQ",MB_OK);

	m_fAltitude = m_matFileObjectMatrix._42 + 8.5f + m_fWeight;

	//char msg[200];
	//sprintf( msg, "m_fAltitude: %.6f", m_fAltitude );
	//MessageBox(NULL,msg,"QQQ",MB_OK);
	
	// NOTE: we are limiting m_matFileObjectMatrix._42 to -8.5f in FrameMove()
	// therefore we'll never get an altitude lower than 0.0f
	// NOTE: m_fSpeedDescent will give problems when doing PlayBack while the heli
	// has some height: Playback() does ResetHeli(), m_fSpeedDescent is then very high,
	// CollisionDetect() detects a crash and does ResetHeli() again.
		
	// dorky hit detection, but we'll be back...
	///if ( /*m_fAltitude <= 0.0f &&*/ !g_bLanded )
	//{
	//	if ( !g_bGodModeOn && 
	//	     ( ( m_fSpeedDescent > 0.50f && !g_bInverted ) ||
	//		   //( m_fY < -0.30f && !g_bInverted ) ||
	//		   ( g_bInverted ) /*|| 
	//		   ( m_vUp.y < sinf(D3DXToRadian(75)) )*/ 
	//		 )
	//		  //||  m_fSpeed > 0.05f || m_fSpeed < -0.05f
	//	   )

	
		
		if ( /*( m_fAltitude <= 0.00f && m_fSpeedDescent > 0.30f ) ||*/
			 /*( m_fAltitude <= 0.02f && g_bInverted )*/
			 ( m_fAltitude <= 0.05f && m_vUp.y < sinf(D3DXToRadian(60)) )
		)		
		{
			// Crash!!! //////////////////////
			//
			//Pause(TRUE);
			//PlaySound( NULL, NULL, SND_PURGE );
			if (m_bSound) 
			{
				// Stop heli sound
				StopBuffer();
	
				// Play crash sound
				if (g_bUseAuthenticHeliSound) {
					if (g_bBell || g_bCobra || g_bCougar) {
						//PlaySound( "crash2.wav", NULL, SND_FILENAME );
						SetVolume2( soundCrash2, DSBVOLUME_MAX );
						PlayBuffer2( soundCrash2, FALSE );
						Sleep(2500);
					} else {
						//PlaySound( "crash.wav", NULL, SND_FILENAME );
						SetVolume2( soundCrash, DSBVOLUME_MAX );
						PlayBuffer2( soundCrash, FALSE );
						Sleep(500);
					}
				} else {
					//PlaySound( "crash.wav", NULL, SND_FILENAME );
					SetVolume2( soundCrash, DSBVOLUME_MAX );
					PlayBuffer2( soundCrash, FALSE );
					Sleep(500);
				}

				

				// Resume heli sound
				PlayBuffer( TRUE );
			} else {		
				//MessageBox(m_hWnd, "Ga naar de Firma Kraak.", "CRASH!", MB_OK);
				Sleep(1500);
			}
			
			

			// reset values ////////
			ResetHeli();
			////////////////////////


			// translation
			D3DMATRIX matTrans;
			D3DUtil_SetTranslateMatrix( matTrans, INIT_X, INIT_Y, INIT_Z );

			// rotation
			D3DMATRIX matRotY;
			D3DUtil_SetRotateYMatrix( matRotY, INIT_RADS_Y );

			D3DMath_MatrixMultiply( m_matFileObjectMatrix, matRotY, matTrans ); // order is crucial!!!
	
			m_fX = 0.0;
			m_fY = 0.0;
			m_fZ = 0.0;
			m_fRadsX = 0.0;
			m_fRadsY = 0.0;
			m_fRadsZ = 0.0;


			// reset view latency arrays
			g_bResetLatency = true;



			return TRUE;
		}
	//}
		/*else
		{
			if (!g_bGodModeOn) {
				// the eagle has landed
				g_bLanded = true;

				m_matSave = m_matFileObjectMatrix;

				m_fRadsY = 0.0f;
				//m_fRadsX = 0.0f;
				//m_fRadsZ = 0.0f;

				m_fX = 0.0f;
				m_fZ = 0.0f;
			}
			
			// TODO: zet heli recht als we geheld landen en houdt 'm 
			// op z'n plaats
		}
	}
	else
	{
		g_bLanded = false;
	}*/


	return FALSE;
}




// Helper Functions /////////////////////////////////////////////

void CMyD3DApplication::CheckD3DDeviceRefCount()
{
	//check ref count
	ULONG ref = m_pd3dDevice->AddRef();
	char msg[200];
	sprintf( msg, "D3DDevice references: %ld", --ref );
	MessageBox(NULL,msg,"QQQ",MB_OK);
	m_pd3dDevice->Release();
}

void CMyD3DApplication::CheckDDrawRefCount()
{
	//check ref count
	ULONG ref = m_pDD->AddRef();
	char msg[200];
	sprintf( msg, "DDraw references: %ld", --ref );
	MessageBox(NULL,msg,"QQQ",MB_OK);
	m_pDD->Release();
}

/////////////////////////////////////////////////////////////////









//-----------------------------------------------------------------------------
// Name: CreateWindowedShadowBuffers()
// Desc: Creates a back buffer with attached z-buffer which will be needed for 
//       our shadow map.
//       Windowed mode and fullscreen mode are handled differently.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::CreateWindowedShadowBuffers()
{
    HRESULT hr;

	RECT rcScreenRect;
	DWORD dwRenderWidth;
	DWORD dwRenderHeight;

    // Get the dimensions of the viewport and screen bounds
	// ClientToScreen() zorgt er voor dat de client rect goed wordt geplaatst
	// Je hoeft dit slechts met left en right te doen: width en height zorgen
	// voor de rest
    GetClientRect( m_hWnd, &rcScreenRect );
    ClientToScreen( m_hWnd, (POINT*)&rcScreenRect.left ); // overbodig??? Nee!!!
    ClientToScreen( m_hWnd, (POINT*)&rcScreenRect.right );
    dwRenderWidth  = rcScreenRect.right  - rcScreenRect.left;
    dwRenderHeight = rcScreenRect.bottom - rcScreenRect.top;

	// Create a back buffer for the light view
    DDSURFACEDESC2 ddsd;
    ZeroMemory( &ddsd, sizeof(ddsd) );
    ddsd.dwSize         = sizeof(ddsd);
    ddsd.dwFlags        = DDSD_WIDTH | DDSD_HEIGHT | DDSD_CAPS;
    ddsd.dwWidth        = dwRenderWidth;
    ddsd.dwHeight       = dwRenderHeight;
    ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN | DDSCAPS_3DDEVICE;

    if( FAILED( hr = m_pDD->CreateSurface( &ddsd, &m_pddsBackBufferShadow, NULL ) ) )
    {
        DEBUG_ERR( hr, _T("Error: Couldn't create the backbuffer") );
        if( hr != DDERR_OUTOFVIDEOMEMORY )
            return D3DFWERR_NOBACKBUFFER;
        DEBUG_MSG( _T("Error: Out of video memory") );
        return DDERR_OUTOFVIDEOMEMORY;
    }



    // Get z-buffer dimensions from the render target
    ddsd.dwSize = sizeof(ddsd);
    m_pddsRenderTarget->GetSurfaceDesc( &ddsd );

    // Setup the surface desc for the z-buffer.
    ddsd.dwFlags        = DDSD_WIDTH | DDSD_HEIGHT | DDSD_CAPS | DDSD_PIXELFORMAT;
    if( m_pDeviceInfo->bHardware )
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_VIDEOMEMORY;
    else
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_SYSTEMMEMORY;
    ddsd.ddpfPixelFormat.dwSize = 0;  // Tag the pixel format as unitialized

    // Get an appropiate pixel format from enumeration of the formats. On the
    // first pass, we look for a zbuffer depth which is equal to the frame
    // buffer depth (as some cards unfornately require this).
    m_pD3D->EnumZBufferFormats( (*m_pDeviceInfo->pDeviceGUID), EnumZBufferFormatsCallback,
                                (VOID*)&ddsd.ddpfPixelFormat );
    if( 0 == ddsd.ddpfPixelFormat.dwSize )
    {
        // Try again, just accepting any 16-bit zbuffer
        ddsd.ddpfPixelFormat.dwRGBBitCount = 16;
        m_pD3D->EnumZBufferFormats( (*m_pDeviceInfo->pDeviceGUID), EnumZBufferFormatsCallback,
                                    (VOID*)&ddsd.ddpfPixelFormat );
            
        if( 0 == ddsd.ddpfPixelFormat.dwSize )
        {
            DEBUG_MSG( _T("Device doesn't support requested zbuffer format") );
			return D3DFWERR_NOZBUFFER;
        }
    }


    // Create and attach a z-buffer to the back buffer for the light view
    if( FAILED( hr = m_pDD->CreateSurface( &ddsd, &m_pddsZBufferShadow, NULL ) ) )
    {
        DEBUG_MSG( _T("Error: Couldn't create a ZBuffer surface") );
        if( hr != DDERR_OUTOFVIDEOMEMORY )
            return D3DFWERR_NOZBUFFER;

        DEBUG_MSG( _T("Error: Out of video memory") );
        return DDERR_OUTOFVIDEOMEMORY;
    }


    if( FAILED( hr = m_pddsBackBufferShadow->AddAttachedSurface( m_pddsZBufferShadow ) ) )
    {
        DEBUG_MSG( _T("Error: Couldn't attach zbuffer to render surface") );
		
		char msg[80];
		sprintf( msg, "hr: %d", HRESULT_CODE(hr) );
		MessageBox(NULL,msg,"QQQ",MB_OK);

        return D3DFWERR_NOZBUFFER;
    }


    return S_OK;
}



//-----------------------------------------------------------------------------
// Name: CreateFullscreenShadowBuffers()
// Desc: Creates a back buffer with attached z-buffer which will be needed for 
//       our shadow map.
//       Windowed mode and fullscreen mode are handled differently.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::CreateFullscreenShadowBuffers( DDSURFACEDESC2* pddsd )
{
    HRESULT hr;

	RECT rcScreenRect;
	DWORD dwRenderWidth;
	DWORD dwRenderHeight;

	// Get the dimensions of the screen bounds
    // Store the rectangle which contains the renderer
    SetRect( &rcScreenRect, 0, 0, pddsd->dwWidth, pddsd->dwHeight );
    dwRenderWidth  = rcScreenRect.right  - rcScreenRect.left;
    dwRenderHeight = rcScreenRect.bottom - rcScreenRect.top;


	///////////////////////////////////////////////////////////////////
	// vanaf hier zijn windowed mode and fullscreen mode hetzelfde

	// Create a back buffer for the light view
    DDSURFACEDESC2 ddsd;
    ZeroMemory( &ddsd, sizeof(ddsd) );
    ddsd.dwSize         = sizeof(ddsd);
    ddsd.dwFlags        = DDSD_WIDTH | DDSD_HEIGHT | DDSD_CAPS;
    ddsd.dwWidth        = dwRenderWidth;
    ddsd.dwHeight       = dwRenderHeight;
    ddsd.ddsCaps.dwCaps = DDSCAPS_OFFSCREENPLAIN | DDSCAPS_3DDEVICE;

    if( FAILED( hr = m_pDD->CreateSurface( &ddsd, &m_pddsBackBufferShadow, NULL ) ) )
    {
        DEBUG_ERR( hr, _T("Error: Couldn't create the backbuffer") );
        if( hr != DDERR_OUTOFVIDEOMEMORY )
            return D3DFWERR_NOBACKBUFFER;
        DEBUG_MSG( _T("Error: Out of video memory") );
        return DDERR_OUTOFVIDEOMEMORY;
    }



    // Get z-buffer dimensions from the render target
    ddsd.dwSize = sizeof(ddsd);
    m_pddsRenderTarget->GetSurfaceDesc( &ddsd );

    // Setup the surface desc for the z-buffer.
    ddsd.dwFlags        = DDSD_WIDTH | DDSD_HEIGHT | DDSD_CAPS | DDSD_PIXELFORMAT;
    if( m_pDeviceInfo->bHardware )
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_VIDEOMEMORY;
    else
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_SYSTEMMEMORY;
    ddsd.ddpfPixelFormat.dwSize = 0;  // Tag the pixel format as unitialized

    // Get an appropiate pixel format from enumeration of the formats. On the
    // first pass, we look for a zbuffer depth which is equal to the frame
    // buffer depth (as some cards unfornately require this).
    m_pD3D->EnumZBufferFormats( (*m_pDeviceInfo->pDeviceGUID), EnumZBufferFormatsCallback,
                                (VOID*)&ddsd.ddpfPixelFormat );
    if( 0 == ddsd.ddpfPixelFormat.dwSize )
    {
        // Try again, just accepting any 16-bit zbuffer
        ddsd.ddpfPixelFormat.dwRGBBitCount = 16;
        m_pD3D->EnumZBufferFormats( (*m_pDeviceInfo->pDeviceGUID), EnumZBufferFormatsCallback,
                                    (VOID*)&ddsd.ddpfPixelFormat );
            
        if( 0 == ddsd.ddpfPixelFormat.dwSize )
        {
            DEBUG_MSG( _T("Device doesn't support requested zbuffer format") );
			return D3DFWERR_NOZBUFFER;
        }
    }


    // Create and attach a z-buffer to the back buffer for the light view
    if( FAILED( hr = m_pDD->CreateSurface( &ddsd, &m_pddsZBufferShadow, NULL ) ) )
    {
        DEBUG_MSG( _T("Error: Couldn't create a ZBuffer surface") );
        if( hr != DDERR_OUTOFVIDEOMEMORY )
            return D3DFWERR_NOZBUFFER;

        DEBUG_MSG( _T("Error: Out of video memory") );
        return DDERR_OUTOFVIDEOMEMORY;
    }


    if( FAILED( hr = m_pddsBackBufferShadow->AddAttachedSurface( m_pddsZBufferShadow ) ) )
    {
        DEBUG_MSG( _T("Error: Couldn't attach zbuffer to render surface") );
		
		char msg[80];
		sprintf( msg, "hr: %d", HRESULT_CODE(hr) );
		MessageBox(NULL,msg,"QQQ",MB_OK);

        return D3DFWERR_NOZBUFFER;
    }

    return S_OK;
}


// SHADOW VOLUME:
//-----------------------------------------------------------------------------
// Name: CreateStencilBuffer()
// Desc: Creates a depth buffer capable of z-buffering and stencil-buffering.
// Note: Doing this causes an empty scene in RGB Emulation mode. Why?
//		 Because we were calling the wrong EnumZBufferFormatsCallback()
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::CreateStencilBuffer()
{
	HRESULT hr;
		
    DDSURFACEDESC2 ddsd;
    ddsd.dwSize = sizeof(ddsd);

    // Get depth-buffer dimensions from the render target
    m_pddsRenderTarget->GetSurfaceDesc( &ddsd );
    DWORD dwFrameBufferDepth = ddsd.ddpfPixelFormat.dwRGBBitCount;

    // Setup the depth-buffer surface description
    ddsd.dwFlags = DDSD_WIDTH|DDSD_HEIGHT|DDSD_CAPS|DDSD_PIXELFORMAT;

    if( m_pDeviceInfo->bHardware )
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_VIDEOMEMORY;
    else
        ddsd.ddsCaps.dwCaps = DDSCAPS_ZBUFFER|DDSCAPS_SYSTEMMEMORY;

    // Get an appropiate pixel format from enumeration of the formats.
    ddsd.ddpfPixelFormat.dwFlags = DDPF_ZBUFFER|DDPF_STENCILBUFFER;
    m_pD3D->EnumZBufferFormats( (*m_pDeviceInfo->pDeviceGUID),
                                EnumZBufferFormatsCallback, (VOID*)&ddsd );

    // Release the old depth-buffer, in case there was one
    m_pddsRenderTarget->DeleteAttachedSurface( 0, NULL );

    // Create and attach a depth-buffer. The SetRenderTarget() call is needed
    // to rebuild internal structures for the newly attached depth-buffer.
    if( FAILED( hr = m_pDD->CreateSurface( &ddsd, &m_pddsDepthBuffer, NULL ) ) )
	{
		char msg[80];
		sprintf( msg, "hr: %d", HRESULT_CODE(hr) );
		MessageBox(NULL,msg,"QQQ",MB_OK);
		return hr;
	}
    
	
	if( FAILED( m_pddsRenderTarget->AddAttachedSurface( m_pddsDepthBuffer ) ) )
		return E_FAIL;
    if( FAILED( m_pd3dDevice->SetRenderTarget( m_pddsRenderTarget, 0L ) ) )
    {
        // See if call failed due to invalid zbuffer depth. (Some cards require
        // that zbuffer depth == frame buffer depth).
        if( dwFrameBufferDepth != ddsd.ddpfPixelFormat.dwRGBBitCount )
            return D3DFWERR_INVALIDZBUFFERDEPTH;
        else
            return E_FAIL;
    }

    return S_OK;
}

// SHADOW VOLUME:
//-----------------------------------------------------------------------------
// Name: EnumZBufferFormatsCallback()
// Desc: Enumeration function to report valid pixel formats for z-buffers.
//		 For stencil buffer
//-----------------------------------------------------------------------------
HRESULT WINAPI CMyD3DApplication::EnumZBufferFormatsCallback( DDPIXELFORMAT* pddpf,
                                                              VOID* pContext )
{
    DDSURFACEDESC2* pddsdOut = (DDSURFACEDESC2*)pContext;

    // Looking for a zbuffer with 1 or more stencil bits
    if( pddpf->dwStencilBitDepth >= 1 )
    {
        pddsdOut->ddpfPixelFormat = (*pddpf);
        return D3DENUMRET_CANCEL;
    }

    return D3DENUMRET_OK;
}


/*
//-----------------------------------------------------------------------------
// Name: EnumZBufferFormatsCallback()
// Desc: Enumeration function to report valid pixel formats for z-buffers.
//	     For double z-buffer
//-----------------------------------------------------------------------------
HRESULT WINAPI CMyD3DApplication::EnumZBufferFormatsCallback( DDPIXELFORMAT* pddpf,
                                                              VOID* pContext )
{
    DDPIXELFORMAT* pddpfOut = (DDPIXELFORMAT*)pContext;

    if( pddpfOut->dwRGBBitCount == pddpf->dwRGBBitCount )
    {
        (*pddpfOut) = (*pddpf);
        return D3DENUMRET_CANCEL;
    }

    return D3DENUMRET_OK;
}
*/


//-----------------------------------------------------------------------------
// Name: ConfirmDevice()
// Desc: Called during device intialization, this code checks the device
//       for some minimum set of capabilities
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::ConfirmDevice( DDCAPS* pddDriverCaps,
                                          D3DDEVICEDESC7* pd3dDeviceDesc )
{
    // Get device's stencil caps
    DWORD dwStencilCaps = pd3dDeviceDesc->dwStencilCaps;

    if( 0 == dwStencilCaps )
        return E_FAIL;

    // Make sure device supports point lights
    if( 0 == ( pd3dDeviceDesc->dwVertexProcessingCaps &
                                            D3DVTXPCAPS_POSITIONALLIGHTS ) )
        return E_FAIL;

    return S_OK;
}


// SHADOW VOLUME:
//-----------------------------------------------------------------------------
// Name: RenderShadow()
// Desc: Draw shadow volume in stencil buffer to make mask used by DrawShadow()
//		 First draws front faces, then back faces of shadow volume.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderShadow()
{
    // Turn depth buffer off, and stencil buffer on
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZWRITEENABLE,  FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILENABLE, TRUE );

    // Set up stencil compare fuction, reference value, and masks
    // Stencil test passes if ((ref & mask) cmpfn (stencil & mask)) is true
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILFUNC,     D3DCMP_ALWAYS );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILREF,      0x1 );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILMASK,     0xffffffff );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILWRITEMASK,0xffffffff );

    // If ztest passes, write 1 into stencil buffer
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILZFAIL, D3DSTENCILOP_KEEP );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILFAIL,  D3DSTENCILOP_KEEP );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILPASS,  D3DSTENCILOP_REPLACE );


    // Make sure that no pixels get drawn to the frame buffer
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND,  D3DBLEND_ZERO );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_ONE );



    // Draw front-side of shadow volume in stencil/z only
    //m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
    //                                    m_pShadowVolumeVertices, 8,
    //                                    m_pShadowVolumeIndices, 24, NULL );


	// NOTE: de shadow volume code werkt goed en is getest op een systeem met stencil
	// buffer. Gedeelten van de rectangle die wordt getekend in DrawShadow() zijn te zien.
	// (daar waar de stencil buffer 1 is). Je ziet ze als je de heli op zijn kop
	// legt (met de huidige foute shadow volume die een hoge zon vertex gebruikt om
	// het volume te bouwen).
	// Wat er fout gaat is dat we nog geen manier hebben gevonden om de shadow
	// volume juist te tekenen. Waarschijnlijk is dat ook bijna niet te doen met .X files.
	// De beste methode is om een volume te maken tussen de vertices van de heli zelf en
	// die van zijn schaduw zoals we die krijgen met RenderPlanarShadow(), maar dan wat
	// verder onder het terrain geprojecteerd. Probleem is, dat het bijna niet te doen is om
	// uit te vinden tussen welke vertices van de .X file je de triangles van je shadow
	// volume moet tekenen. De shadow volume samples werken slechts met een eenvoudig
	// object als shadow caster. Je weet dan precies hoe je je shadow volume op moet bouwen.
	// .X files werken met indices wat betekent dat je voor je shadow volume niet zomaar
	// triangles kunt maken tussen opeenvolgende vertices.
	//
	// NOTE: When rendering, you want to create a shadow volume only from the edges of an
	// object:
	// http://www.gamasutra.com/features/19991115/bestimt_freitag_01.htm
	//
	// De bedoeling is dat je toch met een light source gaat werken en dan alle outside 
	// edge van je object gaat zoeken (gezien vanaf de light source). Daar maak je dan
	// je shadow volume van:
	//
	// The 'outside' edge of an object does not necessarily relate to the convex hull
	// of that object. We need to locate all edges of the object that form the object 
	// silhouette from the perspective of the current light source. Once the edges are
	// located, we can create and project the shadow volume.
	//
	// Als je een outside edge hebt gevonden projecteer je de twee vertices van de edge
	// naar de grond volgens de richting van light source. Tussen de vier vertices teken
	// je dan twee triangles. Je hebt dan een face van je shadow volume. Dit doe je met al
	// de gevonden outside edges en dan heb je je totale shadow volume.
	//
	// The first thing you'll need is the 
	// silhouette of the object from the point of view of the 
	// light. This is best achieved by precomputing an edge list, 
	// which at its most basic, stores the two vertex indices of 
	// its start and end, and the face indices on either side of 
	// it. You'll also need to know the face's normal vector. 
	// Again, ideally, you'd want to precompute this, but you 
	// could calculate it on the fly.
	// Then you need to iterate over each edge, and determine if 
	// its faces point away from or towards the light. If and 
	// only if one face points away from the light, and one 
	// points towards, this edge is a silhouette edge, so it 
	// needs to be marked for later use. If your mesh is open, 
	// i.e., you have edges that only have a face to one side, 
	// you can have all sorts of problems determining if it's a 
	// silhouette or not. This also applies to t-junctions. But 
	// then this is an art problem, so you can pass the buck to 
	// your artists if they're naive enough to give you duff 
	// geometry!



	
	// TEST: 
	// create the shadow volume ///////////////////////////
	// NOTE: dit is niet een goede methode. Beter is om volume tussen de vertices van
	// heli en die van zijn geprojecteerde schaduw te tekenen.
	//
	// dit is een grote array dus niet op de stack
	// static of beter: maak het een global
	static D3DVERTEX pShadowVolumeVertices[120];

	// here comes the sun
	D3DVERTEX vSun = D3DVERTEX( D3DVECTOR( 0, 1000.0f ,0 ),
										  D3DVECTOR(0,1,0), 0, 0 );

		
	// WWWWAAAAAAARRRROM???
	// HUH? waarom worden altijd de vertices van m_pFileObjectVertices2 gepakt????
	// zelfs als we een andere heli file laden
	// Hierom?: de vertices zijn gewoon de vertices uit de X file, NIET de
	// getransformeerde vertices
	// Oplossing: transformeer de vertices van je schaduw zelf
	// je hoeft dan niet eens je schaduw object als X file te laden!
	
	// zet world matrix to identity
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matIdentityMatrix );

	for (int i=0; i<20; i++) {
	
		pShadowVolumeVertices[i*3] = vSun;
		// transform shadow vertices by m_matFileObjectMatrix
		D3DMath_VertexMatrixMultiply( pShadowVolumeVertices[i*3+2], m_pFileObjectVertices[i],
                                      m_matFileObjectMatrix );
		D3DMath_VertexMatrixMultiply( pShadowVolumeVertices[i*3+1], m_pFileObjectVertices[i+1],
                                      m_matFileObjectMatrix );
	
		for (int j=1; j<3; j++) { 
			FLOAT dx = vSun.x - pShadowVolumeVertices[i*3+j].x;
			FLOAT dy = vSun.y - pShadowVolumeVertices[i*3+j].y;
			FLOAT dz = vSun.z - pShadowVolumeVertices[i*3+j].z;
			pShadowVolumeVertices[i*3+j].x -= ((25+m_fAltitude)*dx/dy);
			pShadowVolumeVertices[i*3+j].y -= (25+m_fAltitude);
			pShadowVolumeVertices[i*3+j].z -= ((25+m_fAltitude)*dz/dy);
		}

	}


	// fill 'er up, ma!
	pShadowVolumeVertices[60] = vSun;
	D3DMath_VertexMatrixMultiply( pShadowVolumeVertices[61], m_pFileObjectVertices[0],
                                      m_matFileObjectMatrix );
	D3DMath_VertexMatrixMultiply( pShadowVolumeVertices[62], m_pFileObjectVertices[19],
                                      m_matFileObjectMatrix );
	for (int k=0; k<2; k++) { 
		FLOAT dx = vSun.x - pShadowVolumeVertices[61+k].x;
		FLOAT dy = vSun.y - pShadowVolumeVertices[61+k].y;
		FLOAT dz = vSun.z - pShadowVolumeVertices[61+k].z;
		pShadowVolumeVertices[61+k].x -= ((25+m_fAltitude)*dx/dy);
		pShadowVolumeVertices[61+k].y -= (25+m_fAltitude);
		pShadowVolumeVertices[61+k].z -= ((25+m_fAltitude)*dz/dy);
	}
	///////////////////////////////////////////////////////




	// Note: je moet met triangle list werken
	m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
										pShadowVolumeVertices, 63, NULL );

    // Now reverse cull order so back sides of shadow volume are written,
    // writing 0's into stencil. Result will be any pixel which still has a bit
    // set in the stencil buffer, is inside the shadow.
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILREF, 0x0 );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_CW );

    // Draw back-side of shadow volume in stencil/z only
    //m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
    //                                    m_pShadowVolumeVertices, 8,
    //                                    m_pShadowVolumeIndices, 24, NULL );
	m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
										pShadowVolumeVertices, 63, NULL );


    // Restore render states
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_CCW );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZWRITEENABLE,     TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILENABLE,    FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, FALSE );

    return S_OK;
}



// SHADOW VOLUME:
//-----------------------------------------------------------------------------
// Name: DrawShadow()
// Desc: Draws a big gray polygon over scene according to the mask in the
//       stencil buffer. (Any pixel with stencil==1 is in the shadow.)
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::DrawShadow()
{
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE,       FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILENABLE, TRUE );

    // Turn on alphablending
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND, D3DBLEND_SRCALPHA );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA );

    // Only write where the stencil value == 1
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILREF,  0x1 );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILFUNC, D3DCMP_EQUAL );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILPASS, D3DSTENCILOP_KEEP );

    // Get viewport dimensions for big, gray square
    D3DVIEWPORT7 vp;
    m_pd3dDevice->GetViewport(&vp);
    FLOAT sx = (FLOAT)vp.dwWidth;
    FLOAT sy = (FLOAT)vp.dwHeight;

    // Draw a big, gray square (i.e. black with alpha 0x7f) 
    D3DTLVERTEX vBigGraySquare[4];                              //aarrggbb
    vBigGraySquare[0] = D3DTLVERTEX( D3DVECTOR( 0,sy,0.0f),1.0f,0x7f000000,0,0,0 );
    vBigGraySquare[1] = D3DTLVERTEX( D3DVECTOR( 0, 0,0.0f),1.0f,0x7f000000,0,0,0 );
    vBigGraySquare[2] = D3DTLVERTEX( D3DVECTOR(sx,sy,0.0f),1.0f,0x7f000000,0,0,0 );
    vBigGraySquare[3] = D3DTLVERTEX( D3DVECTOR(sx, 0,0.0f),1.0f,0x7f000000,0,0,0 );
	// TEST: fully opaque blue
//	vBigGraySquare[0] = D3DTLVERTEX( D3DVECTOR( 0,sy,0.0f),1.0f,0xff0000ff,0,0,0 );
//  vBigGraySquare[1] = D3DTLVERTEX( D3DVECTOR( 0, 0,0.0f),1.0f,0xff0000ff,0,0,0 );
//  vBigGraySquare[2] = D3DTLVERTEX( D3DVECTOR(sx,sy,0.0f),1.0f,0xff0000ff,0,0,0 );
//  vBigGraySquare[3] = D3DTLVERTEX( D3DVECTOR(sx, 0,0.0f),1.0f,0xff0000ff,0,0,0 );
    m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_TLVERTEX,
                                 vBigGraySquare, 4, NULL );

    // Restore render states
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE,          TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_STENCILENABLE,    FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, FALSE );

    return S_OK;
}



//-----------------------------------------------------------------------------
// Name: RenderPlanarShadow()
// Desc: Simply projects object's vertices on the ground. Can only cast shadow on
//		 flat surfaces, not on objects.
// Note: This function was once called RenderFakeShadows()
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderPlanarShadow()
{
	// Note: grass.x has a FrameTransformMatrix in the file commanding it to be 
	// placed -10.0 lower.
	// So when projecting the heli shadow we must account for that
	// Project to -9.9 (not to -10.0 for that will cause "Y-fighting")


	// Note: identity matrix!!!
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matIdentityMatrix );

	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_COLORKEYENABLE, TRUE );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND,  D3DBLEND_ZERO );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_ONE );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, TRUE );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAREF, 1 );
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAFUNC, D3DCMP_GREATEREQUAL );

    // Turn on alphablending
	// This will give transparent shadows
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND, D3DBLEND_SRCALPHA );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA );


	// zwarte schaduw 
	// TODO: maar mooier is donkergrijs en iets transparant
	// DONE: niet slecht maar werkt nog niet samen met lensflare
	// en eigenlijk geeft het niet helemaal het juiste effect omdat
	// er geen egale transparantie is
	// Dus toch maar niet doen...
	// Toch maar wel, maar...
	// TODO: avoid Y-fighting of the separate shadows
	// DONE: project rotors a bit lower
	// Setup a material
    D3DMATERIAL7 mtrl;
	D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.08f );
	//if (g_bBell || g_bCobra || g_bCougar)
	//	D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.7f );
	//D3DUtil_InitMaterial( mtrl, 0.1f, 0.1f, 0.1f, 0.6f );
	//D3DUtil_InitMaterial( mtrl, 0.2f, 0.2f, 0.2f, 1.0f );
    m_pd3dDevice->SetMaterial( &mtrl );


	// wonderful kludge III ////////////////////////////////////////////
	static FLOAT f4, f5, f6 = 0.0f;

	if ( GetKeyState(VK_CONTROL) & 0x80 ) {
		if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			if ( GetKeyState('4') & 0x80 ) f4-=0.1f * m_fSpeedFactor;
			if ( GetKeyState('5') & 0x80 ) f5-=0.1f * m_fSpeedFactor;
			if ( GetKeyState('6') & 0x80 ) f6-=0.1f * m_fSpeedFactor;
		} else {
			if ( GetKeyState('4') & 0x80 ) f4-=0.01f * m_fSpeedFactor;
			if ( GetKeyState('5') & 0x80 ) f5-=0.01f * m_fSpeedFactor;
			if ( GetKeyState('6') & 0x80 ) f6-=0.01f * m_fSpeedFactor;
		}
	} else {
		if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			if ( GetKeyState('4') & 0x80 ) f4+=0.1f * m_fSpeedFactor;
			if ( GetKeyState('5') & 0x80 ) f5+=0.1f * m_fSpeedFactor;
			if ( GetKeyState('6') & 0x80 ) f6+=0.1f * m_fSpeedFactor;
		} else {
			if ( GetKeyState('4') & 0x80 ) f4+=0.01f * m_fSpeedFactor;
			if ( GetKeyState('5') & 0x80 ) f5+=0.01f * m_fSpeedFactor;
			if ( GetKeyState('6') & 0x80 ) f6+=0.01f * m_fSpeedFactor;
		}
	}
	
	// can't use VK_MENU (Alt) here because that's for menu access
	if ( GetKeyState('0') & 0x80 ) {
		if ( GetKeyState('4') & 0x80 ) f4=0.0f;
		if ( GetKeyState('5') & 0x80 ) f5=0.0f;
		if ( GetKeyState('6') & 0x80 ) f6=0.0f;
	}

	//sprintf(msg1, "Value 4: %f", f4);
	//sprintf(msg2, "Value 5: %f", f5);
	//sprintf(msg3, "Value 6: %f", f6);
	////////////////////////////////////////////////////////////////////////




	// RT Config Sun ///////////////////////////////////////////////////////
	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderElevation == 1 ) {
		if (g_bRTSpinnerElevationDown) f4-=0.1f * m_fSpeedFactor;
		if (g_bRTSpinnerElevationUp)   f4+=0.1f * m_fSpeedFactor;
	} else {
		if (g_bRTSpinnerElevationDown) f4-=0.01f * m_fSpeedFactor;
		if (g_bRTSpinnerElevationUp)   f4+=0.01f * m_fSpeedFactor;
	}

	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderDirection == 1 ) {
		if (g_bRTSpinnerDirectionDown) f5-=0.1f * m_fSpeedFactor;
		if (g_bRTSpinnerDirectionUp)   f5+=0.1f * m_fSpeedFactor;
	} else {
		if (g_bRTSpinnerDirectionDown) f5-=0.01f * m_fSpeedFactor;
		if (g_bRTSpinnerDirectionUp)   f5+=0.01f * m_fSpeedFactor;
	}

	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderIntensity == 1 ) {
		if (g_bRTSpinnerIntensityDown) f6-=0.1f * m_fSpeedFactor;
		if (g_bRTSpinnerIntensityUp)   f6+=0.1f * m_fSpeedFactor;
	} else {
		if (g_bRTSpinnerIntensityDown) f6-=0.01f * m_fSpeedFactor;
		if (g_bRTSpinnerIntensityUp)   f6+=0.01f * m_fSpeedFactor;
	}

	
	if ( g_bRTButtonElevationReset ) f4=0.0f;
	if ( g_bRTButtonDirectionReset ) f5=0.0f;
	if ( g_bRTButtonIntensityReset ) f6=0.0f;
		
	g_bRTButtonElevationReset = false;
	g_bRTButtonDirectionReset = false;
	g_bRTButtonIntensityReset = false;
	////////////////////////////////////////////////////////////////////////


	// limits: m_iSunIntensity (0-255)
	//if (f6 < -0.52f) f6 = -0.52f;
	//if (f6 > 2.04f)  f6 = 2.04f;


//#pragma warning( disable : 4244 )

	m_iSunElevation = 45 - int(f4*100);
	m_iSunDirection = 315 + int(f5*100);
	m_iSunIntensity = 0x40 + int(f6*100);


	// limits: m_iSunIntensity (0-255)
	if (m_iSunIntensity < 0)    m_iSunIntensity = 0;
	if (m_iSunIntensity > 255)  m_iSunIntensity = 255;



	// Real-Time config
	//m_iSunElevation += g_iRT1;
	//m_iSunDirection += g_iRT2;
	//m_iSunIntensity += g_iRT3;

//#pragma warning( default : 4244 )





	sprintf(msg4, "Sun Elevation (deg): %i", m_iSunElevation);
	sprintf(msg5, "Sun Direction (deg): %i", m_iSunDirection);
	sprintf(msg6, "Sun Intensity: %i (0-255), 0x%X, %i%%", 
						m_iSunIntensity, m_iSunIntensity, m_iSunIntensity*100/255 );


	// light position
	// Perfect!!! Bij m_iSunElevation == 0 deg. wordt ly == 0
	// In het shadow direction algorithm deel je dan door 0
	// Dit heeft als gevolg dat de schaduw niet wordt getekend, wat logisch is bij
	// een elevatie van 0 deg.
	// NOTE: divide bij zero kan misschien gevaarlijk zijn
	float lx = cosf(D3DXToRadian(m_iSunDirection)) * cosf(D3DXToRadian(m_iSunElevation));
	float ly = sinf(D3DXToRadian(m_iSunElevation));
	float lz = sinf(D3DXToRadian(m_iSunDirection)) * cosf(D3DXToRadian(m_iSunElevation));


	// TODO: pas D3DLIGHT aan aan Sun Elevation en Direction
	// DONE
	// TODO: pas lensflare aan aan Sun Elevation en Direction
    // Reset the light
    if( m_pDeviceInfo->ddDeviceDesc.dwVertexProcessingCaps &
                                                D3DVTXPCAPS_DIRECTIONALLIGHTS )
    {
        // Let's make this global cause we need different lights for sky and terrain
		//D3DLIGHT7 light;

		// note: D3DUtil_InitLight() sets position and direction to the same vector
		// init sun light
        D3DUtil_InitLight( g_Light1, D3DLIGHT_DIRECTIONAL, -lx, -ly, -lz );

		// init reflection light (used for sky, this light is at an angle to Sun Elevation)
		// note: dit is het zonlicht dat van het terrain reflecteert en de sky verlicht
        D3DUtil_InitLight( g_Light2, D3DLIGHT_DIRECTIONAL, lx, -ly, lz );


        m_pd3dDevice->SetLight( 0, &g_Light1 );
        m_pd3dDevice->LightEnable( 0, TRUE );
        m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, TRUE );
        m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT,
										RGBA_MAKE( m_iSunIntensity, 
												   m_iSunIntensity,
												   m_iSunIntensity, 
												   0x00 ) );
    }
    else
        m_pd3dDevice->SetRenderState( D3DRENDERSTATE_AMBIENT, 0x00343434 );



	// Draw Shadow ////////////////////////////////////////////////////////////
	if ( m_bDrawShadow ) {

		D3DVERTEX* pVertices = NULL;
		DWORD      dwNumVertices = 0;
		WORD*	   pIndices = NULL;
		DWORD	   dwNumIndices = 0;

		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-mrotor", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			// shadow vertices of rotor follow spin and cyclic tilting
			// (and translation compensation to get concentric spinning)
			// NOTE: we are calling RenderPlanarShadow() when m_fRadsX and m_fRadsZ
			// are 0.0f so we got to kludge to get rotor tilting.
			// We'll use: m_fRotorTiltX and m_fRotorTiltZ
			// NOTE: we were getting funny rotor shadow walks because we were copying
			// the code from the rotor spin code with f1, f2, f3. Here, however, f1, f2, f3
			// used to be local vars set by GetKeyState() button 4, 5, 6. When we rotated
			// the sun, f2 would also change the translate matrix. Sucks.
			// We have changed the local vars to f4, f5, f6 to avoid the confusion. And
			// we have removed f1, f2, f3 from the code here. These are only necessary
			// to find the correct concentric spin.
			// It is never wise to use the same identifiers for relatively unique locals!!!
			D3DMATRIX matTrans, matTrans2;
			D3DUtil_SetTranslateMatrix( matTrans,  -(g_vAxisMRotor.x),  (g_vAxisMRotor.y), -(g_vAxisMRotor.z) );
			D3DUtil_SetTranslateMatrix( matTrans2,  (g_vAxisMRotor.x),  (g_vAxisMRotor.y),  (g_vAxisMRotor.z) );

			//D3DUtil_SetTranslateMatrix( matTrans,  0.0f, -0.09f,  1.253f ); // red heli
			//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f, -0.09f, -0.759f );

//			if (g_bBell) {
//				D3DUtil_SetTranslateMatrix( matTrans,  0.01f, -2.19f,  0.11f ); // bell
//				D3DUtil_SetTranslateMatrix( matTrans2, 0.03f,  2.17f, -0.10f );
//			}
//
//			if (g_bCobra) {
//				D3DUtil_SetTranslateMatrix( matTrans,   0.03f, 0.0f,  0.01f ); // cobra
//				D3DUtil_SetTranslateMatrix( matTrans2,  0.03f, 0.0f, -0.01f );
//			}
//
//			if (g_bCougar) {
//				D3DUtil_SetTranslateMatrix( matTrans,   0.08f, 0.0f,  2.53f ); // cougar
//				D3DUtil_SetTranslateMatrix( matTrans2, -0.08f, 0.0f, -2.60f );
//			}

			D3DMATRIX matRotY, matRotX, matRotZ;
			D3DUtil_SetRotateYMatrix( matRotY, m_fValueKey ); // spin

			D3DUtil_SetRotateXMatrix( matRotX, m_fRotorTiltX*1.0f );	// cyclic tilting
			D3DUtil_SetRotateZMatrix( matRotZ, m_fRotorTiltZ*1.0f );

//			if (g_bBell) {
//				D3DUtil_SetRotateXMatrix( matRotX, m_fRotorTiltX*2.0f );	// cyclic tilting
//				D3DUtil_SetRotateZMatrix( matRotZ, m_fRotorTiltZ*2.0f );
//			}
//
//			if (g_bCobra || g_bCougar) {	
//				D3DUtil_SetRotateXMatrix( matRotX, 0.0f );	// no cyclic tilting
//				D3DUtil_SetRotateZMatrix( matRotZ, 0.0f );
//			}

			D3DMATRIX matAll;
			//D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
			//D3DMath_MatrixMultiply(matAll,matAll,matTrans2);
			//D3DMath_MatrixMultiply(matAll,matAll,matRotX);
			//D3DMath_MatrixMultiply(matAll,matAll,matRotZ);

			// Order!!!
			D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
			D3DMath_MatrixMultiply(matAll,matAll,matRotX);
			D3DMath_MatrixMultiply(matAll,matAll,matRotZ);
			D3DMath_MatrixMultiply(matAll,matAll,matTrans2);

			// the higher the heli the smaller the shadow
			// this is fake but looks realistic
			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMath_MatrixMultiply(matAll,matAll,matScale);
			D3DMath_MatrixMultiply(matAll,matAll,m_matFileObjectMatrix);


			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform en plat maken
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				// - transform X file vertices en zet ze in een array
				// - projecteer de vertices in deze array op de grond (y = 0)
				// - drawprimitive met de vertices in deze array

				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.95f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.95f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.95f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-mrotor", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			// clean up the heap (i.e. the free store)
			delete []pShadowVertices;

		}


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-trotor", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matTrans, matTrans2;
			D3DUtil_SetTranslateMatrix( matTrans,  (g_vAxisTRotor.x), -(g_vAxisTRotor.y), -(g_vAxisTRotor.z) );
			D3DUtil_SetTranslateMatrix( matTrans2, (g_vAxisTRotor.x),  (g_vAxisTRotor.y),  (g_vAxisTRotor.z) );

			//D3DUtil_SetTranslateMatrix( matTrans,  0.0f, -0.071f, -1.022f ); // raptor.x
			//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f,  0.071f,  1.022f );

			//D3DUtil_SetTranslateMatrix( matTrans,  0.0f, -0.47f, -5.72f ); // red heli
			//D3DUtil_SetTranslateMatrix( matTrans2, 0.0f, 0.47f,  5.72f );
				
//			if (g_bBell) {
//				D3DUtil_SetTranslateMatrix( matTrans,  -0.02f, -1.11f, -8.40f ); // bell
//				D3DUtil_SetTranslateMatrix( matTrans2, -0.02f,  1.11f,  8.40f );
//			}
//
//			if (g_bCobra) {
//				D3DUtil_SetTranslateMatrix( matTrans,  -0.02f, -1.08f, -7.94f ); // cobra
//				D3DUtil_SetTranslateMatrix( matTrans2, -0.02f,  1.08f,  7.94f );
//			}
				
			D3DMATRIX matRotX;
			D3DUtil_SetRotateXMatrix( matRotX, 5.0f*m_fValueKey ); // main to tail gear ratio: 5.0
			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matTrans,matRotX);
			D3DMath_MatrixMultiply(matAll,matAll,matTrans2);

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMath_MatrixMultiply(matAll,matAll,matScale);
			D3DMath_MatrixMultiply(matAll,matAll,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i],	matAll );

				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.94f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.94f)/ly )*lz;
					
				// project to the floor
				pShadowVertices[i].y = -9.94f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-trotor", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}


		// For the old helis it is better to draw the shaft shadow more opaque
		// The newer ones have the shaft shadow as transparent as mrotor and trotor
		// When we implement fast and slow rotors we will also have to reset shadows:
		// fast rotor: transparent shadow
		// slow rotor: opaque shadow
		//if (g_bBell || g_bCobra || g_bCougar) {
		//	D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.7f );
		//	m_pd3dDevice->SetMaterial( &mtrl );
		//}
		
		
		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-shaft", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matAll;

			// NOTE: the bell has a rotating shaft so shaft shadow should rotate too
			if (1/*g_bBell*/)
			{
				D3DMATRIX matTrans, matTrans2;
				D3DUtil_SetTranslateMatrix( matTrans,  -(g_vAxisMRotor.x),  (g_vAxisMRotor.y), -(g_vAxisMRotor.z) );
				D3DUtil_SetTranslateMatrix( matTrans2,  (g_vAxisMRotor.x),  (g_vAxisMRotor.y),  (g_vAxisMRotor.z) );

				//D3DUtil_SetTranslateMatrix( matTrans,  0.01f, -2.19f,  0.11f ); // bell
				//D3DUtil_SetTranslateMatrix( matTrans2, 0.03f,  2.17f, -0.10f );
		
				D3DMATRIX matRotY;
				D3DUtil_SetRotateYMatrix( matRotY, m_fValueKey ); // spin

				D3DMath_MatrixMultiply(matAll,matTrans,matRotY);
				D3DMath_MatrixMultiply(matAll,matAll,matTrans2);

				// the higher the heli the smaller the shadow
				// this is fake but looks realistic
				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMath_MatrixMultiply(matAll,matAll,matScale);
				D3DMath_MatrixMultiply(matAll,matAll,m_matFileObjectMatrix);
			} 
			else
			{
				// the higher the heli the smaller the shadow
				// this is fake but looks realistic
				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);
			}


			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform en plat maken
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// algoritme
				// - transform X file vertices en zet ze in een array
				// - projecteer de vertices in deze array op de grond (y = 0)
				// - drawprimitive met de vertices in deze array

				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.95f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.95f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.94f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-shaft", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			// clean up the heap (i.e. the free store)
			delete []pShadowVertices;

		}



		// fuselage shadow black and almost non-transparent
		// we krijgen dan egale schaduw
		D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, m_fShadowAlpha );
		m_pd3dDevice->SetMaterial( &mtrl );


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-fuselage", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform en plat maken
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-fuselage", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}



		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-cpit", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-cpit", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}



		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-tail", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform en plat maken
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-tail", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-guard", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-guard", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-stab", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-stab", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-l1grp", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-l1grp", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}


		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-skid", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-skid", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}



/*
		// kludge ///////////////////////////////////////////////////////////////////////////
		// to draw the red heli's guard right
		// It wasn't drawing right in CULL_CCW so we'll draw it in CULL_NONE
		// Better not do this or we'll get Z-fighting
		// Done: manually edited X file
		if ( FAILED( m_pFileObject->GetMeshVertices( "mesh-guard", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], m_matFileObjectMatrix );
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "mesh-guard", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE,  D3DCULL_NONE );
			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE,  D3DCULL_CCW );

			delete []pShadowVertices;

		}
		//////////////////////////////////////////////////////////////////////////////////////
*/



		// Windsock shadows //////////////////////////////////////////////////////////////////
		if (g_bWindsock)
		{
			D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.4f );
			m_pd3dDevice->SetMaterial( &mtrl );
			
			if ( FAILED( m_pWindsockObject->GetMeshVertices( "mesh-windsock", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {
				
				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matWindsockMatrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );
				
				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matWindsockMatrix);
				
				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];
				
				// transform and flatten
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matWindsockMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;
					
					// project to the floor
					pShadowVertices[i].y = -9.9f;
				}
				
				if ( FAILED( m_pWindsockObject->GetMeshIndices( "mesh-windsock", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}
				
				m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
					pShadowVertices, dwNumVertices,
					pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;
				
			}
			
			
			D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.7f );
			m_pd3dDevice->SetMaterial( &mtrl );
			
			if ( FAILED( m_pWindsockObject->GetMeshVertices( "mesh-pole", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {
				
				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matWindsockMatrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );
				
				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matWindsockMatrix);
				
				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];
				
				// transform and flatten
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matWindsockMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;
					
					// project to the floor
					pShadowVertices[i].y = -9.9f;
				}
				
				if ( FAILED( m_pWindsockObject->GetMeshIndices( "mesh-pole", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}
				
				m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
					pShadowVertices, dwNumVertices,
					pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;
				
			}
		}
		//////////////////////////////////////////////////////////////////////////////////////





		// Pilot Position shadows ////////////////////////////////////////////////////////////
		if (g_bDrawPPMarkShadows && m_bShowPPMarks)
		{
			// A touch more transparent than heli fuselage
			D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.6f );
			m_pd3dDevice->SetMaterial( &mtrl );

			if ( FAILED( m_pPP1Object->GetMeshVertices( "mesh-hedra", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {

				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matPP1Matrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matPP1Matrix);

				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];

				// transform en plat maken
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matFileObjectMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

					// project to the floor
					pShadowVertices[i].y = -9.91f; // avoid Z(Y)-fighting
				}

				if ( FAILED( m_pPP1Object->GetMeshIndices( "mesh-hedra", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}

				// only draw shadow if mark is shown
				if (g_bShowPP1Mark)
					m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
													   pShadowVertices, dwNumVertices,
													   pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;

			}


			if ( FAILED( m_pPP2Object->GetMeshVertices( "mesh-hedra", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {

				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matPP2Matrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matPP2Matrix);

				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];

				// transform en plat maken
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matFileObjectMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

					// project to the floor
					pShadowVertices[i].y = -9.92f;
				}

				if ( FAILED( m_pPP2Object->GetMeshIndices( "mesh-hedra", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}

				// only draw shadow if mark is shown
				if (g_bShowPP2Mark)
					m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
													   pShadowVertices, dwNumVertices,
													   pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;

			}


			if ( FAILED( m_pPP3Object->GetMeshVertices( "mesh-hedra", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {

				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matPP3Matrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matPP3Matrix);

				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];

				// transform en plat maken
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matFileObjectMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

					// project to the floor
					pShadowVertices[i].y = -9.93f;
				}

				if ( FAILED( m_pPP3Object->GetMeshIndices( "mesh-hedra", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}

				// only draw shadow if mark is shown
				if (g_bShowPP3Mark)
					m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
													   pShadowVertices, dwNumVertices,
													   pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;

			}


			if ( FAILED( m_pPP4Object->GetMeshVertices( "mesh-hedra", &pVertices, &dwNumVertices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			} else {

				D3DMATRIX matScale;
				float fScaleVal = 1.0f - (m_matPP4Matrix._42+8.0f)/30.0f;
				D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,m_matPP4Matrix);

				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];

				// transform en plat maken
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by m_matFileObjectMatrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

					// project to the floor
					pShadowVertices[i].y = -9.94f;
				}

				if ( FAILED( m_pPP4Object->GetMeshIndices( "mesh-hedra", &pIndices, &dwNumIndices ) ) )
				{
					//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
					//return E_FAIL;
				}

				// only draw shadow if mark is shown
				if (g_bShowPP4Mark)
					m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
													   pShadowVertices, dwNumVertices,
													   pIndices, dwNumIndices, NULL );
				delete []pShadowVertices;

			}
			
		} // end if (g_bDrawPPMarkShadows)
		/////////////////////////////////////////////////////////////////////////////



		// Heli Pad shadows ////////////////////////////////////////////////////////////
		if (g_bShowBoxes) {
			for (int j=0; j<4; j++) {
				// A touch more transparent than heli fuselage
				D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, 0.6f );
				m_pd3dDevice->SetMaterial( &mtrl );

				//if ( FAILED( m_pPP1Object->GetMeshVertices( "mesh-hedra", &pVertices, &dwNumVertices ) ) )
				LPVOID lpData;
				DWORD dwSize;
				D3DVERTEXBUFFERDESC VBDesc;
				LPDIRECT3DVERTEXBUFFER7 lpVertexBuffer = g_pPad[j]->GetVB();
				lpVertexBuffer->Lock( DDLOCK_READONLY, &lpData, &dwSize );
				lpVertexBuffer->GetVertexBufferDesc( &VBDesc );
				dwNumVertices = VBDesc.dwNumVertices;

				//sprintf( msg1, "dwNumVertices: %i", dwNumVertices ); // 36 (6*6)???
				//MessageBox(NULL,msg1,"QQQ!",MB_OK);

				int stride = 0;
				
				stride = dwSize/dwNumVertices; //32
				//sprintf( msg1, "stride: %i", stride );
				//MessageBox(NULL,msg1,"QQQ!",MB_OK);
				

				// This does not work
				// move FVF vertices to clean D3DVERTEX array using proper stride
				//D3DVERTEX* pCleanVertices = new D3DVERTEX[dwNumVertices];
				//for ( DWORD i=0; i<dwNumVertices; i++ ) 
				//{
				//		memmove( pCleanVertices+(i*sizeof(D3DVERTEX)),
				//			(float*)lpData+(i*stride),
				//			sizeof(D3DVERTEX) );
				//		memmove( pCleanVertices+(i*sizeof(D3DVERTEX)),
				//			(float*)lpData+(i*stride),
				//			sizeof(D3DVERTEX) );
				//}


				// This works. Simple!!!
				pVertices = (D3DVERTEX *)lpData;
				
				lpVertexBuffer->Unlock();
			
				// Must call this when finished
				lpVertexBuffer->Release();
				
			
				D3DMATRIX matScale;
				//float fScaleVal = 1.0f - (proxyPad[j].mat._42+8.0f)/30.0f;
				// tweak shadow slightly smaller to avoid black lines on heli pad boxes
				D3DUtil_SetScaleMatrix( matScale, 0.95f, 1.0f, 0.95f );
		
				D3DMATRIX matAll;
				D3DMath_MatrixMultiply(matAll,matScale,proxyPad[j].mat);

				
				D3DVERTEX* pShadowVertices = NULL;
				pShadowVertices = new D3DVERTEX[dwNumVertices];

				// transform en plat maken
				for ( DWORD i=0; i<dwNumVertices; i++ )
				{
					// transform shadow vertices by the (tweaked) proxy's matrix
					D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			

					// shadow direction algorithm
					pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
					pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

					// project to the floor
					pShadowVertices[i].y = -9.9f; // avoid Z(Y)-fighting
				}

				//if (1)
				//	m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
				//									   pShadowVertices, dwNumVertices,
				//									   pIndices, dwNumIndices, NULL );

				m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
													   pShadowVertices, dwNumVertices,
													   NULL );			


				delete []pShadowVertices;			
			}
		}
		///////////////////////////////////////////////////////////////////////



		// FMS x file shadows //////// ////////////////////////////////////////
		// NOTE: All vertices are in a single unnamed mesh
		// Set material
		D3DUtil_InitMaterial( mtrl, 0.0f, 0.0f, 0.0f, m_fShadowAlpha );
		m_pd3dDevice->SetMaterial( &mtrl );

		// NOTE: we do this in OneTimeSceneInit()
		// NOTE: must also do it after every X file load
		// NOTE: it does not work for files that have more meshes than 1
		// shadows get fucked up
		// NOTE: we don't have to do it like this, we can get all vertices of an
		// unnamed mesh like this: 
		// m_pFileObject->GetMeshVertices( "", &pVertices, &dwNumVertices )
		// See code below. It is that simple.


		// Not like this
//		// get all vertices
//		m_pFileObject->EnumObjects( GetNumFileObjectVerticesCB, NULL,
//													(VOID*)&m_dwNumFileObjectVertices );
//
//		m_pFileObjectVertices = new D3DVERTEX[m_dwNumFileObjectVertices];
//
//		m_pFileObject->EnumObjects( GetFileObjectVerticesCB, NULL,
//													(VOID*)m_pFileObjectVertices );
//
//		// get all indices
//		m_pFileObject->EnumObjects( GetNumFileObjectIndicesCB, NULL,
//													(VOID*)&m_dwNumFileObjectIndices );
//		
//		m_pFileObjectIndices = new WORD[m_dwNumFileObjectIndices];
//
//		m_pFileObject->EnumObjects( GetFileObjectIndicesCB, NULL,
//													(VOID*)m_pFileObjectIndices );
//
//
//		pVertices = m_pFileObjectVertices;
//		dwNumVertices = m_dwNumFileObjectVertices;
//		
//
//		if ( m_pFileObjectVertices == NULL )
//		{
//			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
//			//return E_FAIL;
//		} else {
//
//			D3DMATRIX matScale;
//			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
//			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );
//
//			D3DMATRIX matAll;
//			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);
//
//			D3DVERTEX* pShadowVertices = NULL;
//			pShadowVertices = new D3DVERTEX[dwNumVertices];
//
//			// transform en plat maken
//			for ( DWORD i=0; i<dwNumVertices; i++ )
//			{
//				// transform shadow vertices by m_matFileObjectMatrix
//				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );			
//
//				// shadow direction algorithm
//				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
//				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;
//
//				// project to the floor
//				pShadowVertices[i].y = -9.9f;
//			}
//
//			if ( m_pFileObjectIndices == NULL )
//			{
//				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
//				//return E_FAIL;
//			}
//
//			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
//												   pShadowVertices, dwNumVertices,
//												   m_pFileObjectIndices, m_dwNumFileObjectIndices, NULL );
//		
//			
//			delete []pShadowVertices;
//
//		}


		// But like this
		// get all vertices of the unnamed mesh
		if ( FAILED( m_pFileObject->GetMeshVertices( "", &pVertices, &dwNumVertices ) ) )
		{
			//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
			//return E_FAIL;
		} else {

			D3DMATRIX matScale;
			float fScaleVal = 1.0f - (m_matFileObjectMatrix._42+8.0f)/30.0f;
			D3DUtil_SetScaleMatrix( matScale, fScaleVal, fScaleVal, fScaleVal );

			D3DMATRIX matAll;
			D3DMath_MatrixMultiply(matAll,matScale,m_matFileObjectMatrix);

			D3DVERTEX* pShadowVertices = NULL;
			pShadowVertices = new D3DVERTEX[dwNumVertices];

			// transform and flatten
			for ( DWORD i=0; i<dwNumVertices; i++ )
			{
				// transform shadow vertices by m_matFileObjectMatrix
				D3DMath_VertexMatrixMultiply( pShadowVertices[i], pVertices[i], matAll );
					
				// shadow direction algorithm
				pShadowVertices[i].x -= ( (pShadowVertices[i].y+9.9f)/ly )*lx;
				pShadowVertices[i].z -= ( (pShadowVertices[i].y+9.9f)/ly )*lz;

				// project to the floor
				pShadowVertices[i].y = -9.9f;
			}

			if ( FAILED( m_pFileObject->GetMeshIndices( "", &pIndices, &dwNumIndices ) ) )
			{
				//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
				//return E_FAIL;
			}

			m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
												   pShadowVertices, dwNumVertices,
												   pIndices, dwNumIndices, NULL );
			delete []pShadowVertices;

		}
		///////////////////////////////////////////////////////////////////////



	
	} // end if ( m_bDrawShadow )
	///////////////////////////////////////////////////////////////////////////


	// reset render state
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, FALSE );

    return S_OK;
}


//-----------------------------------------------------------------------------
// Name: RenderLensFlare()
// Desc: From D3DX Tentacle Sample
// Note: Make sure to restore D3DTSS_ALPHAOP to D3DTOP_DISABLE otherwise we get
//		 a heli which turns invisible now and then!!! Nono: D3DTOP_MODULATE.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderLensFlare()
{
	
	// Draw lens flare
	// TODO: lens flare zorgt voor een crash tijdens openfiledialog. Fix it!!!!
	// Oorzaak: openiledialog doet restoretextures voor objecten met textures
	// Alle textures voor lensflare zijn dan foetsie!!! Fix???
	// DONE: restore lens flare textures in openfiledialog
    
	//g_pD3DDevice = m_pd3dDevice;
	g_matPosition = g_matIdentity;

	
	// 
	D3DXMATRIX matTrans;
	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, 55.0f );
	D3DXMatrixMultiply(&g_matPosition, &matTrans, &g_matPosition);

	// this will rotate sun round its own axis
	// it will be invisible when yaw = 0
	D3DXMATRIX matRot;
	D3DXMatrixRotationYawPitchRoll( &matRot, g_PI, 0.0f, 0.0f );	
	D3DXMatrixMultiply(&g_matPosition, &matRot, &g_matPosition);

	// this allows us to set the position of the sun in the world
	// by rotating the world. Ha, Copernicus!
	// set this in the same direction as the directional light
	D3DXMATRIX g_matPosition2 = g_matIdentity;;
	//D3DXMatrixRotationYawPitchRoll( &g_matPosition2, 0.75f*g_PI, 0.0f, 0.0f );	          
	g_pD3DDevice->SetTransform(D3DTRANSFORMSTATE_WORLD, g_matPosition2);


	g_pD3DDevice->SetRenderState(D3DRENDERSTATE_LIGHTING, FALSE);
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_VERTEXBLEND, D3DVBLEND_DISABLE);
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_ZWRITEENABLE, FALSE);
	g_pD3DDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, TRUE);
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_SRCBLEND, D3DBLEND_ONE );
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCCOLOR );
	g_pD3DDevice->SetRenderState( D3DRENDERSTATE_FOGENABLE, FALSE );
    

	//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
	//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE );
	//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP,   D3DTOP_MODULATE );
	
	// NOTE: Use this state but remember to restore D3DTSS_ALPHAOP to D3DTOP_MODULATE!!!
	// Otherwise we'll get a heli that goes invisible every now and then.
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE );
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG2, D3DTA_DIFFUSE );
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1 );

	//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, TRUE);
	//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_SRCBLEND, D3DBLEND_SRCALPHA);
	//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA);

	// Yyyyyyyyyyyyyoooooooooooo!!!!!!!!! Transparency!!!!!!!!!!
	// Enable alpha testing: avoids drawing pixels with less than 
	// a certain alpha
	static DWORD i = 1;
	//if ( GetKeyState('5') & 0x80 ) { ++i; if(i==256) i=255; }
	//if ( GetKeyState('4') & 0x80 ) { --i; if(i== -1) i=0; }
	//sprintf(msg1, "Value 1: %i", i);
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, TRUE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAREF, i );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAFUNC, D3DCMP_GREATEREQUAL );


	// TODO: get rid of rectangle round flares
	// Forget it: these rectangles are caused by a poor 3D video card
	// set color space (ie. range of transparent colors)
	// does our device support this? does this work?
	// DONT: don't use color keys to achieve transparency
	// they are NOT supported by many devices
	//g_pD3DDevice->SetRenderState(D3DRENDERSTATE_COLORKEYENABLE, TRUE);
	//g_pD3DDevice->SetRenderState(D3DRENDERSTATE_COLORKEYBLENDENABLE, TRUE);
	//DDCOLORKEY ddck;
	//ddck.dwColorSpaceLowValue  = 0x0000;
	//ddck.dwColorSpaceHighValue = 0x2222;
	//m_pddsRenderTarget->SetColorKey(DDCKEY_SRCBLT, &ddck);

	// cull mode nodig voor lens flare
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_CW);
	

	// Draw lens flare
	if (m_bRCView) g_pLensFlare->Draw(g_matPosition);
	else		   g_pLensFlare->Draw(m_matFileObjectMatrix);
	
	// restore old cull mode
	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, dwCullMode);

    
	// restore render states
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_LIGHTING, TRUE);
    g_pD3DDevice->SetRenderState(D3DRENDERSTATE_ZWRITEENABLE, TRUE);
	
	g_pD3DDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, FALSE);

	// Restore state
	g_pD3DDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, FALSE );
	g_pD3DDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, m_bFog );

	// Restore state
	g_pD3DDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_MODULATE );

  
	return S_OK;
}



//-----------------------------------------------------------------------------
// Name: RenderExhaustSmoke()
// Desc: 
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderExhaustSmoke()
{

	// Render particle systems:
    //g_pSnowfall->RenderSystem(m_pd3dDevice);
    g_pSmoke->RenderSystem(m_pd3dDevice);

	return S_OK;

}


// WDM
// VxD
//-----------------------------------------------------------------------------
// Name: GetSticks()
// Desc: Polls the VxD or WDM for the parallel port transmitter interface
//-----------------------------------------------------------------------------
void CMyD3DApplication::GetSticks()
{
	if (g_bWindowsNT) {
		// Poll the WDM values
		DeviceIoControl(g_hWDM, IOCTL_TXINTPAR_READ,
						(LPVOID)NULL, 0,
						(LPVOID)RetInfo, sizeof(RetInfo),
						&cbBytesReturned, NULL);

	} else {
		// Poll the VxD values
		DeviceIoControl(g_hVxD, VMYXD_APIFUNC_1,
						(LPVOID)NULL, 0,
						(LPVOID)RetInfo, sizeof(RetInfo),
						&cbBytesReturned, NULL);
	}
						
	// RetInfo[0]  == SyncPulse
	// RetInfo[1]  == Ch1
	// RetInfo[2]  == Ch2
	// RetInfo[3]  == Ch3
	// RetInfo[4]  == Ch4
	// RetInfo[5]  == Ch5
	// RetInfo[6]  == Ch6
	// RetInfo[7]  == Ch7			
	// RetInfo[8]  == Ch8
	// RetInfo[9]  == Ch9
	// RetInfo[10] == Ch10
	// RetInfo[11] == IntCount
	// RetInfo[12] == PulseLength
	// RetInfo[13] == ChannelCount

	/*
		// rollen met die hap
		if ( dijs.lX<0 ) Roll(dijs.lX);
		if ( dijs.lX>0 ) Roll(dijs.lX);
		if ( dijs.lY<0 ) Pitch(dijs.lY);
		if ( dijs.lY>0 ) Pitch(dijs.lY);
		//if ( dijs.lZ<0 ) Yaw(dijs.lZ);
		//if ( dijs.lZ>0 ) Yaw(dijs.lZ);
		if ( dijs.lZ<0 ) ZUp(dijs.lZ);
		if ( dijs.lZ>0 ) ZUp(dijs.lZ);
	*/

	//	Roll(RetInfo[2]);
	//	Pitch(RetInfo[3]);
	//	Yaw(RetInfo[4]);
	//	ZUp(RetInfo[1]);


	// maak 1000-2000 uS
	for (int i=0; i<=10; i++) {
		if (RetInfo[i] != 0) RetInfo[i]-=250; // leave unused channels alone
		// RetInfo[i] must be signed here!!!!!!!!!!!!!!!!!!
		// We've changed it from DWORD to int so it's OK now	
		if (RetInfo[i] < 0)  RetInfo[i] =  0; // never have negative values	
	}
		


	// TODO: make proportional controls
	// DONE
	// TODO: mogelijkheid felheid in te stellen
	// DONE
	// TODO: mogelijkheid invert channel
	// DONE

	// channel sensitivity
	// float Ch1Sens  = 0.75f;
	// float Ch2Sens  = 0.200f;
	// float Ch3Sens  = 0.200f;
	// float Ch4Sens  = 0.35f;
	// float Ch5Sens  = 0.200f;
	// float Ch6Sens  = 0.200f;
	// float Ch7Sens  = 0.75f;
	// float Ch8Sens  = 0.200f;
	// float Ch9Sens  = 0.200f;
	// float Ch10Sens = 0.200f;

	// channel step (don't change)
	// float Ch1Step  = Ch1Sens/760.0f;
	// float Ch2Step  = Ch2Sens/760.0f;
	// float Ch3Step  = Ch3Sens/760.0f;
	// float Ch4Step  = Ch4Sens/760.0f;
	// float Ch5Step  = Ch5Sens/760.0f;
	// float Ch6Step  = Ch6Sens/760.0f;
	// float Ch7Step  = Ch7Sens/760.0f;
	// float Ch8Step  = Ch8Sens/760.0f;
	// float Ch9Step  = Ch9Sens/760.0f;
	// float Ch10Step = Ch10Sens/760.0f;

	// channel invert
	// bool Ch1Inv  = true;
	// bool Ch2Inv  = false;
	// bool Ch3Inv  = false;
	// bool Ch4Inv  = false;
	// bool Ch5Inv  = false;
	// bool Ch6Inv  = false;
	// bool Ch7Inv  = false;
	// bool Ch8Inv  = false;
	// bool Ch9Inv  = false;
	// bool Ch10Inv = false;

	// what's in a name: the stick positions in "radians"
	//float rads;

	// Note: in heli mc-20 program zit collective op kanaal 6
	// TODO: don't hard-code this

	// Graupner mc-20
	// kanaal 1: throttle
	// kanaal 2: roll
	// kanaal 3: nick
	// kanaal 4: heck
	// kannal 6: collective (pitch)


	// keyboard
	// m_fY = +0.25f*CTRL_Y_SENS;
	// m_fRadsY = -(0.05f)*CTRL_RADS_Y_SENS;

	// Bij calibreren: hoogste en laagste RetInfo[1] detecteren.
	// Vervolgens: m_fY = ( RetInfo[1]-(laagste+((hoogste-laagste)/2)) ) *
	// ( sensitivity/(hoogste-laagste) ) * CTRL_Y_SENS; 


	
/*
	// 1000 to 2000, 1000 steps
	// -0.25 to 0.25, 0.50 steps
	// RetInfo[1]-1500 to get -500 to 500
	// 0.50f/1000 to get the steps (increasing 0.50f will give bigger steps and
	// thus higher sensitivity)
	// So: m_fY	= (RetInfo[1]-1500)*(0.50f/1000)*CTRL_Y_SENS;
	//m_fY	 =	(RetInfo[1]-1500)*(1.00f/1000)*CTRL_Y_SENS;			// Channel 1	
	m_fRadsZ = -(RetInfo[2]-1500)*(0.40f/1000)*CTRL_RADS_Z_SENS;	// Channel 2
	m_fRadsX = -(RetInfo[3]-1500)*(0.40f/1000)*CTRL_RADS_X_SENS;	// Channel 3
	m_fRadsY = -(RetInfo[4]-1500)*(0.40f/1000)*CTRL_RADS_Y_SENS;	// Channel 4
	
	// mc-20
	m_fThrottle = (RetInfo[1]*10.0f)/3000;						// Channel 1
	m_fY		= (RetInfo[6]-1500)*(1.00f/1000)*CTRL_Y_SENS;	// Channel 6

	//m_fZ	 = -(RetInfo[7]-1500)*(1.00f/1000)*CTRL_Z_SENS;		// Channel 7
*/

	RegistryRead5();


	if (g_iChAssignedThrottle >= 1 && g_iChAssignedThrottle <= 10)
		m_fThrottle = (RetInfo[g_iChAssignedThrottle]*10.0f)/3000;

	if (g_iChAssignedRoll >= 1 && g_iChAssignedRoll <= 10)
		m_fRadsZ = -(RetInfo[g_iChAssignedRoll]-1500)*(0.40f/1000)*CTRL_RADS_Z_SENS*m_fSpeedFactor;

	if (g_iChAssignedNick >= 1 && g_iChAssignedNick <= 10)
		m_fRadsX = -(RetInfo[g_iChAssignedNick]-1500)*(0.40f/1000)*CTRL_RADS_X_SENS*m_fSpeedFactor;

	if (g_iChAssignedYaw >= 1 && g_iChAssignedYaw <= 10)
		m_fRadsY = -(RetInfo[g_iChAssignedYaw]-1500)*(0.80f/1000)*CTRL_RADS_Y_SENS*m_fSpeedFactor;

	if (g_iChAssignedPitch >= 1 && g_iChAssignedPitch <= 10)
		m_fY     = (RetInfo[g_iChAssignedPitch]-1500)*(1.00f/1000)*CTRL_Y_SENS*m_fSpeedFactor;

	//if (g_iChAssignedGyro >= 0 && g_iChAssignedGyro <= 10)
	//	m_fGyro = RetInfo[g_iChAssignedGyro];

	


	// adjust nose-in/nose-out start
	// and m_fZ start
	if (g_bResetValues) {
		g_bResetValues = false;
		m_fRadsY = INIT_RADS_Y;
		m_fX = INIT_X;
		m_fY = INIT_Y;
		m_fZ = INIT_Z;
	}


	//if (RetInfo[8] > 1200) m_bShowChannels = true;
	//else m_bShowChannels = false;

}


// VxD
//------------------------------------------------------------------------
// Name: ShowChannels()
// Desc: Show channel meters
// Note: We can only call this from Render()
//------------------------------------------------------------------------
void CMyD3DApplication::ShowChannels()
{
	char sync[128];
	char ch1[128];
	char ch2[128];
	char ch3[128];
	char ch4[128];
	char ch5[128];
	char ch6[128];
	char ch7[128];
	char ch8[128];
	char ch9[128];
	char ch10[128];
	//char chtot[128];
	
	// RetInfo[0]  == SyncPulse
	// RetInfo[1]  == Ch1
	// RetInfo[2]  == Ch2
	// RetInfo[3]  == Ch3
	// RetInfo[4]  == Ch4
	// RetInfo[5]  == Ch5
	// RetInfo[6]  == Ch6
	// RetInfo[7]  == Ch7			
	// RetInfo[8]  == Ch8
	// RetInfo[9]  == Ch9
	// RetInfo[10] == Ch10
	// RetInfo[11] == IntCount
	// RetInfo[12] == PulseLength
	// RetInfo[13] == ChannelCount

	
	char* ChL[11];	// We don't use ChL[0] to keep channel numbers the same
					// e.g. ChL[1] corresponds to RetInfo[1]
					// Watch out for wild pointers
	// safety first
	// Can't do that like this: we must initialize to same size as L ?????
	ChL[0] = NULL;
	ChL[1] = NULL;
	ChL[2] = NULL;
	ChL[3] = NULL;
	ChL[4] = NULL;
	ChL[5] = NULL;
	ChL[6] = NULL;
	ChL[7] = NULL;
	ChL[8] = NULL;
	ChL[9] = NULL;
	ChL[10]= NULL;


	char* L[21];

	L[0] = "\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1"; 
	L[1] = "\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[2] = "\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[3] = "\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[4] = "\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[5] = "\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[6] = "\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[7] = "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[8] = "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[9] = "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[10]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[11]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[12]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[13]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1\xB1";
	L[14]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1\xB1";
	L[15]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1\xB1";
	L[16]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1\xB1";
	L[17]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1\xB1";
	L[18]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1\xB1";
	L[19]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xB1";
	L[20]= "\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB\xDB";

	

	for (int i=1; i<=10; i++) {
		int j = 1000;
		if		(RetInfo[i]<(j+=50)) ChL[i]= L[0] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[1] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[2] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[3] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[4] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[5] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[6] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[7] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[8] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[9] ;
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[10];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[11];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[12];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[13];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[14];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[15];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[16];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[17];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[18];
		else if (RetInfo[i]<(j+=50)) ChL[i]= L[19];
		else					     ChL[i]= L[20];
	}


	// Note: don't do this here but in GetSticks() !!!!!!!!!!!
	// pas waarde een beetje aan zodat we 1000-2000 uS printen
	//for (i=0; i<=10; i++) {
	//	if (RetInfo[i] != 0) RetInfo[i]-=250; // leave unused channels alone
	//	// RetInfo[i] must be signed here!!!!!!!!!!!!!!!!!!
	//	// We've changed it from DWORD to int so it's OK now
	//	if (RetInfo[i] < 0)  RetInfo[i] =  0; // never print negative values
	//}	


	bool newint = false;
	// get total number of interrupts
	if ( RetInfo[11] > IntCount ) {
		newint = true;
		IntCount = RetInfo[11];
	}

	// get total number of channels
	if ( RetInfo[13] > ChannelTotal )
					ChannelTotal = RetInfo[13];

	// set unused channels at length 0
	for (i=10; i>ChannelTotal; i--) {
		RetInfo[i] = 0;
		ChL[i] = L[0];
	}


	// TODO: catch Frame Data Defective: dat is wanneer sync pulse of channels
	// buiten de te verwachten waarden vallen
	//bool datadef = false;
	if (RetInfo[0] < 5000 /*|| RetInfo[0] > 15000*/) ChannelTotal = 0;	// check sync
	for (i=1; i<=ChannelTotal; i++) {
		if (RetInfo[i] < 600 || RetInfo[i] > 2400) {		// check channels
			//datadef = true;
			ChannelTotal = 0;
			//ChL[i] = L[0];
		}
	}
	
	// Als de zender uit wordt gezet zal in
	// de VxD ChannelCount op blijft lopen (omdat geen Sync Pulse meer zal worden
	// gevonden die ChannelCount op 0 zet (in de VxD zelf))
	
	// We kunnen dus veel beter OOK een hoog oplopende ChannelTotal als indicatie
	// beschouwen van defective data.
	// Zeker omdat de sync en channels soms toevallig toch binnen de waardes
	// vallen, terwijl hoog oplopende ChannelTotal duidelijk betekent: fucky data
	if (ChannelTotal > 10)
			ChannelTotal = 0;


	// values to be printed
	float fsync = (float)RetInfo[0]/1000 ;
	float fch1  = (float)RetInfo[1]/1000 ;
	float fch2  = (float)RetInfo[2]/1000 ;
	float fch3  = (float)RetInfo[3]/1000 ;
	float fch4  = (float)RetInfo[4]/1000 ;
	float fch5  = (float)RetInfo[5]/1000 ;
	float fch6  = (float)RetInfo[6]/1000 ;
	float fch7  = (float)RetInfo[7]/1000 ;
	float fch8  = (float)RetInfo[8]/1000 ;
	float fch9  = (float)RetInfo[9]/1000 ;
	float fch10 = (float)RetInfo[10]/1000;
	

    // Note: OutPutTextEx can't deal with \n
    if (!newint && m_bFrameMoving)
		sprintf( sync, "No Signal               Channel Total: %ld", ChannelTotal);				
	else if (ChannelTotal == 0)
		sprintf( sync, "Signal Defective        Channel Total: %ld", ChannelTotal);
	else
		sprintf( sync, "Sync Pulse: %6.3f ms   Channel Total: %ld", fsync, ChannelTotal);

	// this kludge will prevent printing No Signal when the app is paused
	// (when the app is paused FrameMove() stops and so does GetSticks() which
	// means the VxD is no longer polled and newint becomes false while we still
	// have a signal)
	// m_bFrameMoving is set to true in FrameMove()
	m_bFrameMoving = FALSE;


	sprintf( ch1,  "Channel  1: %s %.3f ms", ChL[1],  fch1 );
	sprintf( ch2,  "Channel  2: %s %.3f ms", ChL[2],  fch2 );
	sprintf( ch3,  "Channel  3: %s %.3f ms", ChL[3],  fch3 );
	sprintf( ch4,  "Channel  4: %s %.3f ms", ChL[4],  fch4 );
	sprintf( ch5,  "Channel  5: %s %.3f ms", ChL[5],  fch5 );
	sprintf( ch6,  "Channel  6: %s %.3f ms", ChL[6],  fch6 );
	sprintf( ch7,  "Channel  7: %s %.3f ms", ChL[7],  fch7 );
	sprintf( ch8,  "Channel  8: %s %.3f ms", ChL[8],  fch8 );
	sprintf( ch9,  "Channel  9: %s %.3f ms", ChL[9],  fch9 );
	sprintf( ch10, "Channel 10: %s %.3f ms", ChL[10], fch10);

	//sprintf( chtot, "Channel Total: %ld", ChannelTotal);

	int y = 10, k = 12;

	OutputTextEx( 10, y,    sync);
	OutputTextEx( 10, y+=k, ch1 );
	OutputTextEx( 10, y+=k, ch2 );
	OutputTextEx( 10, y+=k, ch3 );
	OutputTextEx( 10, y+=k, ch4 );
	OutputTextEx( 10, y+=k, ch5 );
	OutputTextEx( 10, y+=k, ch6 );
	OutputTextEx( 10, y+=k, ch7 );
	OutputTextEx( 10, y+=k, ch8 );
	OutputTextEx( 10, y+=k, ch9 );
	OutputTextEx( 10, y+=k, ch10);
	//OutputTextEx( 170, 0,  chtot);

	

	
}


//-----------------------------------------------------------------------------
// Name: OutputTextEx()
// Desc: Draws text on the window. Used by ShowChannels()
// Note: Uses Terminal font. This font won't show up in Windows\Fonts. 
//		 To get that font file, copy the Windows\Fonts folder to another location.
//		 Then terminal font is in file: 8514oem.fon
// Note: Let InstallShield copy this file to the Windows\Fonts folder because we
//		 absolutely need that font for proper channel drawing.
// Note: Show Channels does not show up because the Terminal font cannot be found. This 
//		 could be related to a font problem on my XP installation because Metasequoia also
//		 has font problems. On Win98 on the same system there is no prob. Solution: use 
//		 OEM_CHARSET instead of DEFAULT_CHARSET for fdwCharSet in CreateFont()
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::OutputTextEx( DWORD x, DWORD y, TCHAR* str )
{
    HDC hDC;

    // Get a DC for the surface. Then, write out the buffer
    if( m_pddsRenderTarget )
    {
        if( SUCCEEDED( m_pddsRenderTarget->GetDC(&hDC) ) )
        {	
			// NEW: use CreateFont() and SelectObject() to get another font face
			// NOTE: On XP we were not getting the Terminal font
			// Solution: use OEM_CHARSET instead of DEFAULT_CHARSET for fdwCharSet
			HFONT hfont, hfontOld; 
			hfont = CreateFont (10, 0, 0, 0, FW_NORMAL, FALSE, FALSE,
                     FALSE, OEM_CHARSET, OUT_DEFAULT_PRECIS,
                     CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY,
                     DEFAULT_PITCH|FF_DONTCARE, "Terminal");
			
			hfontOld = (HFONT)SelectObject(hDC, hfont);

            SetTextColor( hDC, g_crTextColor );
            SetBkMode( hDC, TRANSPARENT );
            ExtTextOut( hDC, x, y, 0, NULL, str, lstrlen(str), NULL );

			// N.B. must select old font back. Must!!!
			SelectObject(hDC, hfontOld);
			DeleteObject(hfont);
            m_pddsRenderTarget->ReleaseDC(hDC);
        }
    }

    // Do the same for the left surface (in case of stereoscopic viewing).
    if( m_pddsRenderTargetLeft )
    {
        if( SUCCEEDED( m_pddsRenderTargetLeft->GetDC( &hDC ) ) )
        {
            // Use a different color to help distinguish left eye view
            SetTextColor( hDC, RGB(255,0,255) );
            SetBkMode( hDC, TRANSPARENT );
            ExtTextOut( hDC, x, y, 0, NULL, str, lstrlen(str), NULL );
            m_pddsRenderTargetLeft->ReleaseDC(hDC);
        }
    }
}


//-----------------------------------------------------------------------------
// Name: RenderSplashScreen()
// Desc: Draws splash screen.
//       Draws a bitmap into a DirectDrawSurface
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderSplashScreen()
{
    // Load bitmap
	HBITMAP hbm = (HBITMAP)LoadImage( NULL, "..\\media\\Splash.bmp", IMAGE_BITMAP,
                                          0, 0, LR_LOADFROMFILE|LR_CREATEDIBSECTION );
	// Get bitmap info 
	BITMAP bm; 
    GetObject( hbm, sizeof(BITMAP), &bm ); 

	// Create a DC and setup the bitmap
    HDC hdcBitmap = CreateCompatibleDC( NULL );
    if( NULL == hdcBitmap )
        return E_FAIL;

    SelectObject( hdcBitmap, hbm );

	// Not like this
	// get window dimensions
	//RECT rc;
	//GetWindowRect( m_hWnd, &rc );

	// nor like this
	//GetClientRect( m_hWnd, &rc );

	// But like this
	// Need to get surface dimensions to get it right in both windowed and full-screen mode
	DDSURFACEDESC2 ddsd;
	ddsd.dwSize = sizeof(ddsd);
    ddsd.dwFlags = DDSD_HEIGHT | DDSD_WIDTH;
    m_pddsRenderTarget->GetSurfaceDesc(&ddsd);


	HDC hDC;

    // Get a DC for the surface. Then, write out the buffer
    if( m_pddsRenderTarget )
    {
        if( SUCCEEDED( m_pddsRenderTarget->GetDC(&hDC) ) )
        {
			// Make sure to use the stretching mode best for color pictures
			SetStretchBltMode( hDC, COLORONCOLOR );

			//BitBlt(hDC, 0, 0, bm.bmWidth, bm.bmHeight, hdcBitmap, 0, 0, SRCCOPY );
			StretchBlt(hDC, 0, 0, ddsd.dwWidth, ddsd.dwHeight,
						hdcBitmap, 0, 0, bm.bmWidth, bm.bmHeight, SRCCOPY );

            m_pddsRenderTarget->ReleaseDC(hDC);
        }
    }
	
	// Clean up
	DeleteDC( hdcBitmap );

	// laat het geheugen niet vollopen...
	DeleteObject(hbm);

	return S_OK;

}



//-----------------------------------------------------------------------------
// Name: DoScreenShot()
// Desc: Say cheese. Snapshots are saved as screenshot1.bmp etc. in \screenshots
//		 folder
// Note: Paint Shop Pro 4.1 won't read these 16 and 32 bpp screenshots. Sucks!!!
//		 Use Paint or Imaging or Corel PhotoPaint, anything.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::DoScreenShot()
{
				
	// kludge: DevStudio executes from Project directory!!!
	// we must execute from \Release, final version executes from \bin
	SetCurrentDirectory( "Release" );

	// note: present working directory is \media
	SetCurrentDirectory( ".." );
	CreateDirectory("screenshots", NULL);
	SetCurrentDirectory( "screenshots" );

	// TODO: read dir to see if previous screenshots are present and
	// adapt file name number
	WIN32_FIND_DATA wfd;
	int i = 1;
	char strFileName[80];
	sprintf(strFileName, "screenshot%i.bmp", i);
	
	while ( INVALID_HANDLE_VALUE != FindFirstFile(strFileName, &wfd) ) {
		i++;
		sprintf(strFileName, "screenshot%i.bmp", i);
	}	


	// say cheese
	if (g_bScreenshotIncludeWindowBorder) {
		Screenshot3(strFileName, m_hWnd);
	} else {
		Screenshot(strFileName, m_pddsRenderTarget);
	}
	

	// back to \media
	SetCurrentDirectory( "..\\media" );


	return S_OK;

}



//-----------------------------------------------------------------------------
// Name: FlightRecord()
// Desc: Record flight data file
// Note1: .fld File Format:
//
//		 [Frame 1]
//		 m_matFileObjectMatrix._11 m_matFileObjectMatrix._12 m_matFileObjectMatrix._13 m_matFileObjectMatrix._14
//		 m_matFileObjectMatrix._21 m_matFileObjectMatrix._22 m_matFileObjectMatrix._23 m_matFileObjectMatrix._24
//		 m_matFileObjectMatrix._31 m_matFileObjectMatrix._32 m_matFileObjectMatrix._33 m_matFileObjectMatrix._34
//		 m_matFileObjectMatrix._41 m_matFileObjectMatrix._42 m_matFileObjectMatrix._43 m_matFileObjectMatrix._44 
//		 m_fX m_fY m_fZ
//		 m_fRadsX m_fRadsY m_fRadsZ
//		 g_fX2 g_fY2 g_fZ2
//		 g_fRadsX2 g_fRadsY2 g_fRadsZ2
//		 m_fCollective
//		 m_fThrottle
//
//		 [Frame 2]
//		 etc.
// Note2: We *always* record to a temp file: flights\flight.temp
// Note3: It is better to create a temp file with GetTempFileName(). Although:
//		  from the Docs: To avoid problems resulting from converting an ANSI character
//		  set string to a Windows string, an application should call the CreateFile function
//		  to create a temporary file.
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::FlightRecord()
{
	
	// Record flight data ////////////////////////////////////////////
	//static TCHAR strTemp[512] = "flight1.fld";
	//static TCHAR strTemp[512] = "..\\flights\\flight1.fld";
	static TCHAR strTemp[512] = "..\\flights\\flight.temp";
	static HANDLE hFile1 = NULL;
	//static HANDLE hFile2 = NULL;
	static TCHAR Buffer[512];
	static DWORD nBytesToWrite;
	static DWORD nBytesWritten;
	//static DWORD nBytesToRead;
	//static DWORD nBytesRead;
	static bool bInitRec = true;
	//static bool bInitPlay = true;
	static int iFrameCount = 0;

	// TODO: when recording it would be best to also save the m_matFileObjectMatrix from
	// which the recording starts
	// TODO: it is probably better to save m_matFileObjectMatrix instead of Controls
	// NO: just make sure Playback starts from the same initial settings as Record: take
	// care to reset values properly!!!
	// But if we want Fast Forward and Reverse we must use m_matFileObjectMatrix (and
	// then do FrameMove() skipping several frames)
	// We probably need both m_matFileObjectMatrix and Controls, the first to get Fast 
	// Forward and Reverse, the second to update the Virtual Tx and Channels
	// Using m_matFileObjectMatrix also makes it possible to record and play from anywhere
	// in a flight, not just from the initial setting.
	// NOTE: the way we record and playback is OK. Every frame we read and write a small
	// buffer from/to the file. An alternative would be to read and write a large buffer, the
	// moment the buffer is spent. This is unacceptable, however, because that would cause
	// uneven FPS. Using small buffers that are written to/read from the file creates a
	// little overhead every frame, making sure we've got even FPS.
	//
	// NOTE: m_matFileObjectMatrix stores the 6DOF's of the heli.
	// 6DOF: 6 Degrees of Freedom (3 linear + 3 angular):
	// In a left-handed coordinate system:
	// | Right.x Up.x  Forward.x 0 |
	// | Right.y Up.y  Forward.y 0 |
	// | Right.z Up.z  Forward.z 0 |
	// | Pos.x   Pos.x Pos.z     1 |
	//
	//	m_matFileObjectMatrix._11 m_matFileObjectMatrix._12 m_matFileObjectMatrix._13 m_matFileObjectMatrix._14
	//	m_matFileObjectMatrix._21 m_matFileObjectMatrix._22 m_matFileObjectMatrix._23 m_matFileObjectMatrix._24
	//	m_matFileObjectMatrix._31 m_matFileObjectMatrix._32 m_matFileObjectMatrix._33 m_matFileObjectMatrix._34
	//	m_matFileObjectMatrix._41 m_matFileObjectMatrix._42 m_matFileObjectMatrix._43 m_matFileObjectMatrix._44 
	//		
	// NOTE: Since we are recording and playing back from m_matFileObjectMatrix, we no longer
	// need to do ResetHeli(). We can rec and play from any position!

	if (m_bRecord) 
	{

		if (bInitRec) {
			bInitRec = false;
		

			hFile1 = CreateFile( strTemp, GENERIC_WRITE | GENERIC_READ, 
								FILE_SHARE_READ, NULL, CREATE_ALWAYS, 
								FILE_ATTRIBUTE_TEMPORARY |FILE_FLAG_RANDOM_ACCESS,
								NULL );

			if ( hFile1 == INVALID_HANDLE_VALUE ) {
				MessageBox(m_hWnd, "Can't create flight data file.",
							"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
				hFile1 = NULL;
				return S_FALSE;
			}

			// reset values //////////
			//ResetHeli();
			//////////////////////////

			iFrameCount = 0;

			// give initial 1000 frames
			g_iFrameTotal = 1000;

			// Write file info
			nBytesToWrite = sprintf(Buffer, "// R/C Sim Sikorsky Flight Data File\n");
			WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );
		}

		// Write frame number
		nBytesToWrite = sprintf(Buffer, "[Frame %i]\n", ++iFrameCount);

		g_iFrameCount = iFrameCount;

		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );


		// Write matrix
		nBytesToWrite = sprintf(Buffer, "%.6f %.6f %.6f %.6f\n"
										"%.6f %.6f %.6f %.6f\n"
										"%.6f %.6f %.6f %.6f\n"
										"%.6f %.6f %.6f %.6f\n",			 
			m_matFileObjectMatrix._11, m_matFileObjectMatrix._12, m_matFileObjectMatrix._13, m_matFileObjectMatrix._14,
			m_matFileObjectMatrix._21, m_matFileObjectMatrix._22, m_matFileObjectMatrix._23, m_matFileObjectMatrix._24,
			m_matFileObjectMatrix._31, m_matFileObjectMatrix._32, m_matFileObjectMatrix._33, m_matFileObjectMatrix._34,
			m_matFileObjectMatrix._41, m_matFileObjectMatrix._42, m_matFileObjectMatrix._43, m_matFileObjectMatrix._44);
		
		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );


		// Write controls
		nBytesToWrite = sprintf(Buffer, "%.6f %.6f %.6f\n"
												"%.6f %.6f %.6f\n",
											m_fX, m_fY, m_fZ,
											m_fRadsX, m_fRadsY, m_fRadsZ);

		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );


		// Write clean controls
		nBytesToWrite = sprintf(Buffer, "%.6f %.6f %.6f\n"
												"%.6f %.6f %.6f\n",
											g_fX2, g_fY2 , g_fZ2,
											g_fRadsX2, g_fRadsY2, g_fRadsZ2);

		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );


		// Write collective
		nBytesToWrite = sprintf(Buffer, "%.6f\n",
											m_fCollective);

		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );


		// Write throttle
		nBytesToWrite = sprintf(Buffer, "%.6f\n\n",
											m_fThrottle);

		WriteFile( hFile1, Buffer, nBytesToWrite, &nBytesWritten, NULL );
		
	}
	else // if (!m_bRecord) 
	{
		if (hFile1) {
			CloseHandle(hFile1);
			hFile1 = NULL;

			// reset values //////////
			//ResetHeli();
			//////////////////////////
		}
			
		bInitRec = true;
	}
	////////////////////////////////////////////////////////////////


	return S_OK;
}



//-----------------------------------------------------------------------------
// Name: FlightPlayBack()
// Desc: Play back flight data file
// Note1: .fld File Format:
//
//		 [Frame 1]
//		 m_matFileObjectMatrix._11 m_matFileObjectMatrix._12 m_matFileObjectMatrix._13 m_matFileObjectMatrix._14
//		 m_matFileObjectMatrix._21 m_matFileObjectMatrix._22 m_matFileObjectMatrix._23 m_matFileObjectMatrix._24
//		 m_matFileObjectMatrix._31 m_matFileObjectMatrix._32 m_matFileObjectMatrix._33 m_matFileObjectMatrix._34
//		 m_matFileObjectMatrix._41 m_matFileObjectMatrix._42 m_matFileObjectMatrix._43 m_matFileObjectMatrix._44 
//		 m_fX m_fY m_fZ
//		 m_fRadsX m_fRadsY m_fRadsZ
//		 g_fX2 g_fY2 g_fZ2
//		 g_fRadsX2 g_fRadsY2 g_fRadsZ2
//		 m_fCollective
//		 m_fThrottle
//
//		 [Frame 2]
//		 etc.
// Note2: We *always* playback from a temp file: flights\flight.temp
// Note3: It is better to create a temp file with GetTempFileName(). Although:
//		  from the Docs: To avoid problems resulting from converting an ANSI character
//		  set string to a Windows string, an application should call the CreateFile function
//		  to create a temporary file.
// Note4: When opening a .FLD file in an editor, the editor might silently convert it from
//		  UNIX to DOS format in which \n (LF, 0xA) becomes (CRLF, 0x0D 0x0A)
// Note5: We were getting a memory leak with Pause during Playback. You can check this with
//		  Performance Monitor. Pause calls ResetFilePointer() in a loop.
//		  ResetFilePointer() calls SearchStringInFile().
//		  In SearchStringInFile() we were doing a malloc() without free().
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::FlightPlayBack()
{
	
	// Playback flight data ////////////////////////////////////////////
	static TCHAR strTemp[512] = "..\\flights\\flight.temp";
	//static HANDLE hFile1 = NULL;
	static HANDLE hFile2 = NULL;
	static TCHAR Buffer[512];
	//static DWORD nBytesToWrite;
	//static DWORD nBytesWritten;
	static DWORD nBytesToRead = 0;
	static DWORD nBytesRead = 0;
	//static bool bInitRec = true;
	static bool bInitPlay = true;
	static int iFrameCount = 0;

	BOOL bResult;

	//static OVERLAPPED o;





	if (m_bPlayBack) 
	{
		if (bInitPlay) {
			bInitPlay = false;
			
			// Find memory leak /////////////////
			//_RPTF0(_CRT_WARN, "InitPlay\n");
			//DEBUG_MSG("InitPlay\n");
			//MessageBeep(MB_ICONEXCLAMATION);
			//MessageBox(NULL, "qqq", "QQQ", MB_OK);
			/////////////////////////////////////

			hFile2 = CreateFile( strTemp, GENERIC_READ, FILE_SHARE_READ, NULL,
									OPEN_ALWAYS, FILE_ATTRIBUTE_TEMPORARY |
									FILE_FLAG_RANDOM_ACCESS, NULL );

		
			if ( hFile2 == INVALID_HANDLE_VALUE ) {
				MessageBox(m_hWnd, "Can't open flight data file.",
							"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
				hFile2 = NULL;
				return S_FALSE;
			}


			// Memory Leak Indicator: commenting this out pointed us to the cause
			// of the memory leak because g_hFileFlightRec is used in ResetFilePointer().
			// Pause calls ResetFilePointer() in a loop. ResetFilePointer() calls 
			// SearchStringInFile(). In SearchStringInFile() we were doing a malloc()
			// without free().
			//
			// the global handle
			g_hFileFlightRec = hFile2;

			
			// Initialize the critical section.
			InitializeCriticalSection(&GlobalCriticalSection);



			// reset values //////////
			//ResetHeli();
			//////////////////////////

			//D3DUtil_SetIdentityMatrix( m_matFileObjectMatrix );

			g_bResetLatency = true;


			// Request ownership of the critical section.
			__try 
			{
				EnterCriticalSection(&GlobalCriticalSection);
				
				// First get Frame Total ////////////////
				// NOTE: format is [Frame #]
				char *pdest2 = NULL;
				int result2 = 0;
				//TCHAR BufferTemp2[64];
				
				SetFilePointer( hFile2, -(LONG)sizeof(Buffer), NULL, FILE_END );
				bResult = ReadFile( hFile2, Buffer, sizeof(Buffer), &nBytesRead, NULL );
				
				pdest2 = strrchr( Buffer, ']' );	// find last ']'
				result2 = pdest2 - Buffer;
				Buffer[result2] = '\0';				// set to '\0'
				
				pdest2 = strrchr( Buffer, '[' );	// find last '['
				pdest2 += strlen("Frame ")+1;
				
				g_iFrameTotal = atoi( pdest2 );
				
				// check
				//MessageBox(NULL, pdest2, "QQQ", MB_OK);				
				

				// reset file pointer to [Frame g_iFrameCount] ////////////
				//MessageBox(NULL,"qqq!","QQQ!",MB_OK);
				ResetFilePointer();

			}
			__finally 
			{
				// Release ownership of the critical section.
				LeaveCriticalSection(&GlobalCriticalSection);
			}
			/////////////////////////////////////////
			
			

			
			//Sleep(3000);

			// Don't read first frame as that will contain large reset values
			// No: this does not matter
			//ReadFile( hFile2, Buffer, sizeof(Buffer), &nBytesRead, NULL );
			//nBytesToRead = ( strstr(Buffer, "\n\n") - Buffer ) + 1;
			//SetFilePointer ( hFile2, (LONG)nBytesRead, NULL, FILE_BEGIN );
		}
		// END: if (bInitPlay) //////////////////////////////////////////////////////




		// Make sure to return if we have no file handle
		if ( hFile2 == NULL ) {
			m_bPlayBack = false;
			return S_FALSE;
		}
		
		// Read in one frame's flight data (frames are delimited by \n\n)
		// NOTE: We read in a buffer, search for \n\n, reset file pointer, then reread
		// exactly one frame.
		// NOTE: The strncpy function copies the initial count characters of strSource to
		// strDest and returns strDest. If count is less than or equal to the length of 
		// strSource, *A NULL CHARACTER IS NOT APPENDED AUTOMATICALLY* to the copied string.
		// So we must do things like BufferTemp[result] = '\0'; ourselves!!!

	
		// Request ownership of the critical section.
		__try 
		{
			EnterCriticalSection(&GlobalCriticalSection);
			
			bResult = ReadFile( hFile2, Buffer, sizeof(Buffer), &nBytesRead, NULL );
		}
		__finally 
		{
			// Release ownership of the critical section.
			LeaveCriticalSection(&GlobalCriticalSection);
		}
		////////////////////////////////////////////////////

		if (bResult && nBytesRead == 0) {

			// ErrMsgBox() heeft geen zin hier omdat bResult == 0 als ReadFile() mislukt.
			// Als we hier komen is ReadFile() *niet mislukt*.
			// We komen hier uitsluitend vanwege dit: 
			// from the Docs: If the return value is nonzero and the number of bytes read
			// is zero, the file pointer was beyond the current end of the file at the time
			// of the read operation...If the function fails, the return value is zero.
			// We mogen hier alleen komen als g_iFrameCount >= g_iFrameTotal zodat we
			// playback kunnen stoppen. Maar we komen hier soms ook als dat niet zo is:
			// Dus: de filepointer is voorbij EOF gezet terwijl dit niet mocht.
			// Hoe komt dit???
			// In ResetFilePointer() gaat het fout: daar wordt door foute parse code
			// de filepointer soms voor FILE_BEGIN gezet en soms na EOF. ReadFile() kan
			// dan niet meer lezen en we behouden de huidige m_matFileObjectMatrix.
			// Dan schiet de heli into space...


			//ErrMsgBox();

			//MessageBeep(-1);
			//ResetFilePointer();

			// Passed EOF (End of File)
			//MessageBox(m_hWnd, "Passed EOF.", "QQQ", MB_OK);
			
			// kludge: when doing transport during playback we sometimes get here.
			// TODO: find the reason why!!!
			// We then drop out of playback unwanted. This kludge solves it.
			// Another option would be to check another way for passing EOF (e.g.
			// by checking if we have passed the last Frame.
			//if (!g_bRTButtonRewindDown && !g_bRTButtonFastForwardDown &&
			//	!g_bRTButtonBeginDown  && !g_bRTButtonEndDown &&
			//	!g_bRTGrabFlightRecSlider)
			if (g_iFrameCount >= g_iFrameTotal)
			{
				//MessageBox(m_hWnd, "qqq", "QQQ", MB_OK);
				m_bPlayBack = false;
			}
			//CloseHandle( hFile );
			
			// reset values //////////
			//ResetHeli();
			//////////////////////////

		} else {
			// NOTE: it is probably better to search for "\n[" as frame delimiter:
			// * If user has opened and saved the file in an editor which silently 
			//   converts the file to DOS format, we still find the next frame
			// * It allows the user to put some comments between the frames without
			//   the need to enter \n\n before a new next frame
			//nBytesToRead = ( strstr(Buffer, "\n[") - Buffer ) + 2;
			// NOTE: it is probably better still to search for "\n[Frame" as frame delimiter:
			//nBytesToRead = ( strstr(Buffer, "\n[Frame") - Buffer ) + 7;
			//
			// But hey, let's be strict
			nBytesToRead = ( strstr(Buffer, "\n\n") - Buffer ) + 2;

			// Request ownership of the critical section.
			__try 
			{
				EnterCriticalSection(&GlobalCriticalSection);
				
				SetFilePointer( hFile2, -(LONG)nBytesRead, NULL, FILE_CURRENT );
				ReadFile( hFile2, Buffer, nBytesToRead, &nBytesRead, NULL );
			}
			__finally 
			{
				// Release ownership of the critical section.
				LeaveCriticalSection(&GlobalCriticalSection);
			}
			////////////////////////////////////////////////////
			
			
			
			// check Frame data
			//Buffer[nBytesToRead] = '\0';
			//MessageBox(NULL, Buffer,"QQQ", MB_OK);
			//if (Buffer[nBytesToRead] == '\n') MessageBox(NULL, "OK","QQQ", MB_OK);



			char *pdest = NULL;
			int result = 0;
			int start = 0;
			TCHAR BufferTemp[64];



			// Get Frame # ////////////////////////////////
			// NOTE: format is [Frame #]
			pdest = strchr( Buffer, '\n' );
			result = pdest - Buffer;
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer, result );
				BufferTemp[result] = '\0';

				// BufferTemp is now: "[Frame #]"
				BufferTemp[result-1] = '\0';						// get rid of ']'
				iFrameCount = atoi( strchr(BufferTemp, ' ')+1 );	// read the number

				if (!g_bRTButtonRewindDown && !g_bRTButtonFastForwardDown &&
					!g_bRTButtonBeginDown && !g_bRTButtonEndDown &&
					!g_bRTGrabFlightRecSlider &&
					!g_bFRButtonRewindDown && !g_bFRButtonFastForwardDown &&
					!g_bFRButtonBeginDown && !g_bFRButtonEndDown &&
					!g_bFRGrabFlightRecSlider &&
					!g_bRTButtonPauseChecked && !g_bFRButtonPauseChecked)
				{
					g_iFrameCount = iFrameCount;
				}
			}

			// check limits
			if (g_iFrameCount < 1) g_iFrameCount = 1;
			if (g_iFrameCount > g_iFrameTotal) g_iFrameCount = g_iFrameTotal;

			// check
			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);			
			//if (Buffer[0] == '\n') MessageBox(NULL, "shit","QQQ", MB_OK);
			///////////////////////////////////////////////


			// Get the Matrix /////////////////////////////
			//m_matFileObjectMatrix._11, m_matFileObjectMatrix._12, m_matFileObjectMatrix._13, m_matFileObjectMatrix._14,
			//m_matFileObjectMatrix._21, m_matFileObjectMatrix._22, m_matFileObjectMatrix._23, m_matFileObjectMatrix._24,
			//m_matFileObjectMatrix._31, m_matFileObjectMatrix._32, m_matFileObjectMatrix._33, m_matFileObjectMatrix._34,
			//m_matFileObjectMatrix._41, m_matFileObjectMatrix._42, m_matFileObjectMatrix._43, m_matFileObjectMatrix._44);


			// Row 1
			// get m_matFileObjectMatrix._11
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._11 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get m_matFileObjectMatrix._12
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._12 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get m_matFileObjectMatrix._13
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._13 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get m_matFileObjectMatrix._14
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._14 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);
			

			// Row 2
			// get m_matFileObjectMatrix._21
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._21 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._22
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._22 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._23
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._23 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._24
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._24 = (float)atof( BufferTemp );
			}


			// Row 3
			// get m_matFileObjectMatrix._31
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._31 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._32
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._32 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._33
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._33 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._34
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._34 = (float)atof( BufferTemp );
			}


			// Row 4
			// get m_matFileObjectMatrix._41
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._41 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._42
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._42 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._43
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._43 = (float)atof( BufferTemp );
			}

			// get m_matFileObjectMatrix._44
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_matFileObjectMatrix._44 = (float)atof( BufferTemp );
			}
			///////////////////////////////////////////////



			// Get the Controls
			// get m_fX
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fX = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get m_fY
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fY = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get m_fZ
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fZ = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);
			

			// get m_fRadsX
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fRadsX = (float)atof( BufferTemp );
			}

			// get m_fRadsY
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fRadsY = (float)atof( BufferTemp );
			}

			// get m_fRadsZ
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fRadsZ = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);
			///////////////////////////////////////////////


			// Get the clean Controls
			// get g_fX2
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fX2 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get g_fY2
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fY2 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);

			// get g_fZ2
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fZ2 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);
			

			// get g_fRadsX2
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fRadsX2 = (float)atof( BufferTemp );
			}

			// get g_fRadsY2
			start += result+1;
			pdest = strchr( Buffer+start, ' ' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fRadsY2 = (float)atof( BufferTemp );
			}

			// get g_fRadsZ2
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				g_fRadsZ2 = (float)atof( BufferTemp );
			}

			//MessageBox(NULL, BufferTemp,"QQQ", MB_OK);
			///////////////////////////////////////////////


			// Get the collective
			// get m_fCollective
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fCollective = (float)atof( BufferTemp );
			}

			// Get the throttle
			// get m_fThrottle
			start += result+1;
			pdest = strchr( Buffer+start, '\n' );
			result = pdest - (Buffer+start);
			if( pdest != NULL ) {
				strncpy( BufferTemp, Buffer+start, result );
				BufferTemp[result] = '\0';
				m_fThrottle = (float)atof( BufferTemp );
			}
			/////////////////////////////////////////////////


	
			// NOTE: We must update some stuff because many things depend on the Control
			// values.
			// NOTE2: It is better not to do this here. Instead we'll make sure that when
			// we are doing playback, the control values are not set to 0.0f at the end of
			// FrameMove(). There are so may things dependent on the Control values that
			// we might forget some.
			//UpdateRTVirtualTx();
			//UpdateRTChannels();

			// kludge to get cyclic rotor tilt for shadow
			//m_fRotorTiltX = m_fRadsX;
			//m_fRotorTiltZ = m_fRadsZ;

			// This one must still be done here...
			// kludge to get proper collective rotor coning
			// Note: must be done before DoDynamics() 
			m_fRotorConeY = m_fY;


		}
		
	}
	else // if (!m_bPlayBack)
	{
		if (hFile2) {
			CloseHandle(hFile2);
			hFile2 = NULL;

			// reset values //////////
			//ResetHeli();
			//////////////////////////
		}

		bInitPlay = true;

	}
	////////////////////////////////////////////////////////////////


	return S_OK;
}



//-----------------------------------------------------------------------------
// Name: DoSmoothControls()
// Desc: Smoothens the controls.
// Note1: can't do it the first frame since we are adjusting m_fZ and m_fRadsY.
// Note2: has to be done after input polling and before setting the
//        matrices/quaternions.
// Note3: in gamedev speak this is called "dampening" 
//-----------------------------------------------------------------------------
void CMyD3DApplication::DoSmoothControls()
{
	D3DXVECTOR3 vecT = D3DXVECTOR3( m_fX, m_fY, m_fZ );
	D3DXVECTOR3 vecR = D3DXVECTOR3( m_fRadsY, m_fRadsX, m_fRadsZ );

	//g_fSpeed        = 2.0f;
	//g_fAngularSpeed = 1.0f;

	g_vecVelocity = g_vecVelocity * 0.9f + vecT * 0.1f;
    g_vecAngularVelocity = g_vecAngularVelocity * 0.9f + vecR * 0.1f;

    //vecT = g_vecVelocity /** g_pFrameTimer->GetSecsPerFrame()*/ * g_fSpeed;
    //vecR = g_vecAngularVelocity /** g_pFrameTimer->GetSecsPerFrame()*/ * g_fAngularSpeed;

	vecT = g_vecVelocity * /*(30.0f/GetFPS()) **/ g_fSpeed;
	vecR = g_vecAngularVelocity * /*(30.0f/GetFPS()) **/ g_fAngularSpeed;

//	vecT = g_vecVelocity * g_fSpeed * 1/m_fSpeedFactor;
//	vecR = g_vecAngularVelocity * g_fAngularSpeed * 1/m_fSpeedFactor;

//	vecT = g_vecVelocity * (1.0f/GetFPS()) * g_fSpeed * 20.0f;
//	vecR = g_vecAngularVelocity * (1.0f/GetFPS()) * g_fAngularSpeed * 20.0f;


	m_fX = vecT.x;
	m_fY = vecT.y;
	m_fZ = vecT.z;

	m_fRadsY = vecR.x;
	m_fRadsX = vecR.y;
	m_fRadsZ = vecR.z;
}



//-----------------------------------------------------------------------------
// Name: DoSmoothCamControls()
// Desc: Smoothens the camera controls.
// Note: has to be done after input polling and before setting the
//       matrices/quaternions.
// Note: the problem here is that m_fCamX etc. are absolute values in the world,
//		 whereas m_fX etc. are just values by which the model's position should be
//		 translated per frame. Remember: m_fX etc. get set to 0.0f at the end of
//		 every FrameMove. So we cannot do smooth cam controls here. We should check
//		 if user pushes cam controls and then accelerate/decelerate the cam movement.
// Note: not bad, but let's skip smooth cam controls. Just because we can, does not
//		 mean we should...
// Note: in gamedev speak this is called "dampening"
//-----------------------------------------------------------------------------
void CMyD3DApplication::DoSmoothCamControls()
{
	// TODO:
	// - Use m_fSpeedFactor DONE
	// - CamRads DONE
	// - Compensate normal control with VK_DECIMAL/VK_NUMPAD0
	

	static float accelX1 = 0.0f;
	static float accelX2 = 0.0f;
	static float accelY1 = 0.0f;
	static float accelY2 = 0.0f;
	static float accelZ1 = 0.0f;
	static float accelZ2 = 0.0f;


	// TEST:
	float accelFactor = 1.0f;
	
	float accelLimit;
	
	if (m_bRCView || m_bFollowView) {
		accelLimit = 0.5f;
		if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 0 ) accelLimit*=2;
		if ( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 2 ) accelLimit/=2;
	}
	if (m_bChaseView) {
		accelLimit = 0.02f;
		if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 0 ) accelLimit*=2;
		if ( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 2 ) accelLimit/=2;
	}


	// We use this variable to compensate for the normal cam control values. Then we can
	// apply the smooth cam control values. Still it would be better not to evaluate the
	// normal cam control values when doing smooth cam. Then we wouldn't have to compensate.
	float fCompensate;
	
	if (m_bRCView || m_bFollowView) {
		fCompensate = 0.5f;
		if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 0 ) fCompensate*=2;
		if ( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 2 ) fCompensate/=2;
	}
	if (m_bChaseView) {
		fCompensate = 0.02f;
		if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 0 ) fCompensate*=2;
		if ( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 2 ) fCompensate/=2;
	}


	

	// TODO: Also compensate normal control with VK_DECIMAL/VK_NUMPAD0
	if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 0 ) accelFactor*=2;
	if ( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 2 ) accelFactor/=2;
	
	accelFactor*=m_fSpeedFactor;

	if (m_bRCView || m_bFollowView) 
	{
		// CamX		
		if ( GetKeyState(VK_NUMPAD6) & 0x80 ) {
			// compensate the normal control
			// Also compensate normal control with VK_DECIMAL/VK_NUMPAD0
			// It is better just to not do the normal cam controls when doing smooth controls
			m_fCamX-=fCompensate*CAM_X_SENS*m_fSpeedFactor;
//			// This is too bloated
//			if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 ) 
//				m_fCamX-=(0.5f/2)*CAM_X_SENS*m_fSpeedFactor;
//			else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 ) 
//				m_fCamX-=(0.5f*2)*CAM_X_SENS*m_fSpeedFactor;
//			else									  
//				m_fCamX-=(0.5f)*CAM_X_SENS*m_fSpeedFactor;

			accelX1+=0.01f*accelFactor;
			if (accelX1>accelLimit) accelX1 = accelLimit;
		} else {
			accelX1-=0.01f*accelFactor;
			if (accelX1<0.0f) accelX1 = 0.0f;
		}
		m_fCamX += accelX1*CAM_X_SENS*m_fSpeedFactor;

		if ( GetKeyState(VK_NUMPAD4) & 0x80 ) {
			m_fCamX+=fCompensate*CAM_X_SENS*m_fSpeedFactor; // compensate
//			if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 ) 
//				m_fCamX+=(0.5f/2)*CAM_X_SENS*m_fSpeedFactor;
//			else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 ) 
//				m_fCamX+=(0.5f*2)*CAM_X_SENS*m_fSpeedFactor;
//			else									  
//				m_fCamX+=(0.5f)*CAM_X_SENS*m_fSpeedFactor;

			accelX2+=0.01f*accelFactor;
			if (accelX2>accelLimit) accelX2 = accelLimit;
		} else {
			accelX2-=0.01f*accelFactor;
			if (accelX2<0.0f) accelX2 = 0.0f;
		}
		m_fCamX -= accelX2*CAM_X_SENS*m_fSpeedFactor;


		// CamY		
		if ( GetKeyState(VK_NUMPAD7) & 0x80 ) {
			m_fCamY-=fCompensate*CAM_Y_SENS*m_fSpeedFactor; // compensate
			accelY1+=0.01f*accelFactor;
			if (accelY1>accelLimit) accelY1 = accelLimit;
		} else {
			accelY1-=0.01f*accelFactor;
			if (accelY1<0.0f) accelY1 = 0.0f;
		}
		m_fCamY += accelY1*CAM_Y_SENS*m_fSpeedFactor;

		if ( GetKeyState(VK_NUMPAD1) & 0x80 ) {
			m_fCamY+=fCompensate*CAM_Y_SENS*m_fSpeedFactor; // compensate
			accelY2+=0.01f*accelFactor;
			if (accelY2>accelLimit) accelY2 = accelLimit;
		} else {
			accelY2-=0.01f*accelFactor;
			if (accelY2<0.0f) accelY2 = 0.0f;
		}
		m_fCamY -= accelY2*CAM_Y_SENS*m_fSpeedFactor;

		
		// CamZ		
		if ( GetKeyState(VK_NUMPAD8) & 0x80 ) {
			m_fCamZ-=fCompensate*CAM_Z_SENS*m_fSpeedFactor;  // compensate
			accelZ1+=0.01f*accelFactor;
			if (accelZ1>accelLimit) accelZ1 = accelLimit;
		} else {
			accelZ1-=0.01f*accelFactor;
			if (accelZ1<0.0f) accelZ1 = 0.0f;
		}
		m_fCamZ += accelZ1*CAM_Z_SENS*m_fSpeedFactor;

		if ( GetKeyState(VK_NUMPAD2) & 0x80 ) {
			m_fCamZ+=fCompensate*CAM_Z_SENS*m_fSpeedFactor; // compensate
			accelZ2+=0.01f*accelFactor;
			if (accelZ2>accelLimit) accelZ2 = accelLimit;
		} else {
			accelZ2-=0.01f*accelFactor;
			if (accelZ2<0.0f) accelZ2 = 0.0f;
		}
		m_fCamZ -= accelZ2*CAM_Z_SENS*m_fSpeedFactor;
	}


	if (m_bChaseView) 
	{
		// CamRadsX
		if ( GetKeyState(VK_NUMPAD2) & 0x80 ) {
			// compensate the normal control
			// Also compensate normal control with VK_DECIMAL/VK_NUMPAD0
			// It is better to not do the normal cam controls when doing smooth controls
			m_fCamRadsX-=fCompensate*CAM_RADS_X_SENS*m_fSpeedFactor;
			// This is too bloated
//			if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 ) 
//				m_fCamRadsX-=(0.02f/2)*CAM_RADS_X_SENS*m_fSpeedFactor;
//			else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 ) 
//				m_fCamRadsX-=(0.02f*2)*CAM_RADS_X_SENS*m_fSpeedFactor;
//			else									  
//				m_fCamRadsX-=(0.02f)*CAM_RADS_X_SENS*m_fSpeedFactor;

			accelX1+=0.0008f*accelFactor;
			if (accelX1>accelLimit) accelX1 = accelLimit;
		} else {
			accelX1-=0.0008f*accelFactor;
			if (accelX1<0.0f) accelX1 = 0.0f;
		}
		m_fCamRadsX += accelX1*CAM_RADS_X_SENS*m_fSpeedFactor;

		if ( GetKeyState(VK_NUMPAD8) & 0x80 ) {
			m_fCamRadsX+=fCompensate*CAM_RADS_X_SENS*m_fSpeedFactor; // compensate
			accelX2+=0.0008f*accelFactor;
			if (accelX2>accelLimit) accelX2 = accelLimit;
		} else {
			accelX2-=0.0008f*accelFactor;
			if (accelX2<0.0f) accelX2 = 0.0f;
		}
		m_fCamRadsX -= accelX2*CAM_RADS_X_SENS*m_fSpeedFactor;


		// CamRadsY
		if ( GetKeyState(VK_NUMPAD6) & 0x80 ) {
			m_fCamRadsY-=fCompensate*CAM_RADS_Y_SENS*m_fSpeedFactor; // compensate
			accelY1+=0.0008f*accelFactor;
			if (accelY1>accelLimit) accelY1 = accelLimit;
		} else {
			accelY1-=0.0008f*accelFactor;
			if (accelY1<0.0f) accelY1 = 0.0f;
		}
		m_fCamRadsY += accelY1*CAM_RADS_Y_SENS*m_fSpeedFactor;

		if ( GetKeyState(VK_NUMPAD4) & 0x80 ) {
			m_fCamRadsY+=fCompensate*CAM_RADS_Y_SENS*m_fSpeedFactor; // compensate
			accelY2+=0.0008f*accelFactor;
			if (accelY2>accelLimit) accelY2 = accelLimit;
		} else {
			accelY2-=0.0008f*accelFactor;
			if (accelY2<0.0f) accelY2 = 0.0f;
		}
		m_fCamRadsY -= accelY2*CAM_RADS_Y_SENS*m_fSpeedFactor;

		
//		// CamRadsZ
//		if ( GetKeyState(VK_NUMPAD8) & 0x80 ) {
//			m_fCamRadsZ-=fCompensate*CAM_RADS_Z_SENS*m_fSpeedFactor; // compensate
//			accelZ1+=0.0008f*accelFactor;
//			if (accelZ1>accelLimit) accelZ1 = accelLimit;
//		} else {
//			accelZ1-=0.0008f*accelFactor;
//			if (accelZ1<0.0f) accelZ1 = 0.0f;
//		}
//		m_fCamRadsZ += accelZ1*CAM_RADS_Z_SENS*m_fSpeedFactor;
//
//		if ( GetKeyState(VK_NUMPAD2) & 0x80 ) {
//			m_fCamRadsZ+=fCompensate*CAM_RADS_Z_SENS*m_fSpeedFactor; // compensate
//			accelZ2+=0.0008f*accelFactor;
//			if (accelZ2>accelLimit) accelZ2 = accelLimit;
//		} else {
//			accelZ2-=0.0008f*accelFactor;
//			if (accelZ2<0.0f) accelZ2 = 0.0f;
//		}
//		m_fCamRadsZ -= accelZ2*CAM_RADS_Z_SENS*m_fSpeedFactor;
	}


	// reset
	if ( GetKeyState(VK_NUMPAD5) & 0x80 ) {
		accelX1 = 0.0f;
		accelX2 = 0.0f;
		accelY1 = 0.0f;
		accelY2 = 0.0f;
		accelZ1 = 0.0f;
		accelZ2 = 0.0f;
	}



//	if ( GetKeyState(VK_NUMPAD4) & 0x80 ) {
//		m_fCamX+=0.5f;
//		accel+=0.02f;
//		if (accel>0.5f) accel = 0.5f;
//	} else {
//		accel-=0.02f;
//		if (accel<0.0f) accel = 0.0f;
//	}
//	m_fCamX -= accel;


//	D3DXVECTOR3 vecT = D3DXVECTOR3( m_fCamX, m_fCamY, m_fCamZ );
//	D3DXVECTOR3 vecR = D3DXVECTOR3( m_fRadsY, m_fRadsX, m_fRadsZ );
//
//	//g_fSpeed        = 2.0f;
//	//g_fAngularSpeed = 1.0f;
//
//	g_vecCamVelocity = g_vecCamVelocity * 0.9f + vecT * 0.1f;
//    g_vecAngularCamVelocity = g_vecAngularCamVelocity * 0.9f + vecR * 0.1f;
//
//    //vecT = g_vecVelocity /** g_pFrameTimer->GetSecsPerFrame()*/ * g_fSpeed;
//    //vecR = g_vecAngularVelocity /** g_pFrameTimer->GetSecsPerFrame()*/ * g_fAngularSpeed;
//
//	vecT = g_vecCamVelocity * /*(30.0f/GetFPS()) **/ g_fCamSpeed;
//	vecR = g_vecAngularCamVelocity * /*(30.0f/GetFPS()) **/ g_fAngularCamSpeed;
//
//	m_fCamX = vecT.x;
//	m_fCamY = vecT.y;
//	m_fCamZ = vecT.z;
//
//	m_fCamRadsY = vecR.x;
//	m_fCamRadsX = vecR.y;
//	m_fCamRadsZ = vecR.z;


}



//-----------------------------------------------------------------------------
// Name: DoDynamics()
// Desc: Does rotary-wing flight characteristics.
//
// Note: Call in FrameMove() before doing normal 6DOF
//       This should become one of the best procedures in the bizz
// Note: Well, ODE does a much better job...
//-----------------------------------------------------------------------------
//void CMyD3DApplication::DoDynamics()
//{
//
//	// Flight Characteristics ///////////////////////////////////////////////////////
//	// YOYOYOYOYOYOYOYOYOYO!!!!!!!!!!!!!!!!!!
//	// Flight char OKOKOK???
//	// Quite OK
//	// TODO: - als de traagheid is overwonnen moet de snelheid groter worden
//	//       - hoe groter de voorwaartse snelheid hoe minder wegbreken bij rollen
//	//		   en hoe meer er automatisch yaw is (windvaan effect)
//	// TODO: get inverted flight OK
//	// DONE
//	// TODO: get rid of rotor snap during normal<->inverted flight transition
//	// DONE: Save clean m_fY for collective coning
//	// We are doing this in FrameMove() because DoDynamics() will only
//	// be called if (!g_bLanded)
//	// m_fRotorConeY = m_fY;
//	// TODO: get rid of view snap during normal<->inverted flight transition
//	// DONE: altered algorithm
//	// TODO: Motion Rate a la CSM. Motion Rate == Heavy 6DOF<->Heavy Rotary Wing
//	// DONE
//	// TODO: fuselage rotation and tilting due to torque
//	// DONE
//	// TODO: mix tail with cyclic on basis of forward speed:
//	// if cyclic roll && nick && fast speed do mix tail
//	// TODO: faster speed, make heli fly more like 6DOF, i.e.
//	// reduce influence of dynamics, and increase m_fZ
//	// Note: het moet mogelijk zijn om een circuit te draaien alleen op roll en nick,
//	// en dit is mogelijk bij 6DOF en m_fZ
//	// En het moet mogelijk zijn een cleane loop te draaien bij hoge snelheid
//
//
//
//
//
//	// check inverted
//	if (m_vUp.y < 0.0f) {
//		g_bInverted = true;
//	} else {
//		g_bInverted = false;
//	}
//
//
//	// Weight
//	if ( m_bRotaryWing ) {
//		//if ( m_fAltitude > 0.30f )
//		//	m_matFileObjectMatrix._42 -= m_fAltitude;
//		//else
//		//if (m_fAltitude > 0.0f)
//			//m_matFileObjectMatrix._42 -= 0.30f;
//			//if (m_matFileObjectMatrix._42 < -(8.5f+0.30f))
//			//	m_matFileObjectMatrix._42 = -(8.5f+0.30f);
//		//m_fWeight = 0.30f;
//		//if (m_matFileObjectMatrix._42 > -8.5f)
//		//if (!g_bLanded)
//			// NOTE: we cannot speedfactor m_fWeight: when user opens menu (or because of
//			// Windows "burps") frame rate will be very low during one frame, thus
//			// m_fSpeedFactor will be very high and the heli will suddenly drop during
//			// one frame.
//			m_matFileObjectMatrix._42 -= m_fWeight/**m_fSpeedFactor*/;
//			
//	}
//	
//	
//	// TODO: better get angles from the matrix
//	// NOTE NOTE NOTE: THESE ANGLES ARE FAULTY!!!!!!!!!!!!!!!!!!!
//
//	// Roll
//	D3DVECTOR vFlatRight(m_vRight.x,
//						 0.0f,
//					     m_vRight.z);
//	vFlatRight = Normalize(vFlatRight);
//
//
//	D3DVALUE fCosAngle = 0.0f;
//	D3DVALUE fRollAngle = 0.0f;
//
//
//	fCosAngle = DotProduct(m_vRight,vFlatRight);
//	if (fCosAngle>=-1.0f && fCosAngle<=1.0f)
//		fRollAngle = acosf(fCosAngle);
//	if (m_vRight.y > 0.0f) fRollAngle = -fRollAngle;
//
//	sprintf( msg7, "Roll (deg): %.2f", (fRollAngle*180.0f)/g_PI );
//
//	// Kluns: m_fY mag je nooit veranderen bij roll of nick: deze blijft
//	// altijd gelijk!!!! m_fY == Pitch (Collective) en die blijft constant.
//	// Wat verandert zijn de x (en z) en y componenten van vector m_fY.
//	// Door m_fX (en m_fZ) te veranderen op basis van de kantelhoek*m_fY
//	// veranderen we zowel de x (en z) als y component van vector m_fY.
//	//
//	// sinf(fAngle) == x/m_fY
//	// cosf(fAngle) == x/m_fX
//	// m_fX = m_fY*sinf(fAngle)/cosf(fAngle)
//	// m_fX = m_fY*tanf(fAngle)
//
//	// CSM: 40-130%, 0.4f-1.3f
//	// g_fMotionRate: 0.0-2.0f
//
//	// NOTE: we cannot use m_fSpeedFactor here. Reason: see above.
//
//	if ( m_bRotaryWing ) {
//		// roll
//		// fAngle: 0-PI/2, 0-1.57
//		// sinf(fAngle): 0-1.0
//		//if (fAngle!=0.0f) m_fX += (0.40f+m_fY)*tanf(fAngle);
//		//if (fAngle!=0.0f) m_fX += (0.40f+m_fY)*fAngle;
//		//if (fAngle!=0.0f) 
//		if (g_bInverted) {
//			m_fX += (0.80f+m_fY)*sinf(fRollAngle)*g_fMotionRate*0.5f;
//			//m_fX += (0.60f+m_fY)*sinf(fAngle)*1.0f;  // no weight
//		} else {
//			m_fX += (0.00f+m_fY)*sinf(fRollAngle)*g_fMotionRate*0.5f;
//			//m_fX += (0.30f+m_fY)*sinf(fAngle)*1.0f; // no weight
//			//m_fX += sinf(fRollAngle)*g_fMotionRate*0.3f*m_fSpeedFactor;
//		}
//
//		// Set the left/right factors
//		// The longer/stronger we go left the higher the fLeftFactor,
//		// the lower the fRightFactor
//		static float fLeftFactor  = 1.0f;
//		static float fRightFactor = 1.0f;
//		
//
//		// roll left
//		if (fRollAngle>0.0f) {
//			fLeftFactor+=0.08f;
//			
//			// TODO: the bigger m_fZ (forward speed), the smaller fLeftFactor
//			// *This* enables flying circuits without using yaw because then
//			// we'll have more fixed-wing style flight characteristics
//			//fLeftFactor-=m_fZ;
//
//			m_fX *= fLeftFactor;
//
//			// reset
//			fRightFactor = 1.0f;
//
//			// TODO: make also depend on nick angle
//			// NO
//			//m_fRadsY += 0.02f*sinf(fRollAngle); // tail falls left
//
//			
//
//			
//			//	//if (g_bInverted) {
//			//	//	m_fY += fAngle*0.25f;
//			//	//} else {
//			//	//	m_fY -= fAngle*0.25f;
//			//	//}
//			//
//			//	//m_fX += fAngle*(0.40f+m_fY*1.0f); // afhankelijk van de grootte van de kantelhoek
//			//	m_fX += (0.40f+m_fY)*tanf(fAngle);
//		}
//
//		// roll right
//		if (fRollAngle<0.0f) {
//			fRightFactor+=0.08f;
//
//			// TODO: the bigger m_fZ, the smaller fRightFactor
//
//			m_fX *= fRightFactor;
//
//			// reset
//			fLeftFactor = 1.0f;
//			
//			//m_fRadsY += 0.02f*sinf(fRollAngle); // tail falls right
//
//			//	//if (g_bInverted) {
//			//	//	m_fY -= fAngle*0.25f;
//			//	//} else {
//			//	//	m_fY += fAngle*0.25f;
//			//	//}
//			//
//			//	//m_fX += fAngle*(0.40f+m_fY*1.0f);
//			//	m_fX += (0.40f+m_fY)*tanf(fAngle);
//		}		
//	}
//
//
//	// TEST: crisp yaw
//	//m_fRadsY *= 2.0f;
//
//
//
//	// Nick
//	D3DVECTOR vFlatForward(m_vForward.x,
//						   0.0f,
//					       m_vForward.z);
//	vFlatForward = Normalize(vFlatForward);
//
//	
//	D3DVALUE fNickAngle = 0.0f;
//
//
//	fCosAngle = DotProduct(m_vForward,vFlatForward);
//	if (fCosAngle>=-1.0f && fCosAngle<=1.0f)
//		fNickAngle = acosf(fCosAngle);
//	if (m_vForward.y > 0.0f) fNickAngle = -fNickAngle;
//
//	sprintf( msg8, "Nick (deg): %.2f", (fNickAngle*180.0f)/g_PI );
//
//	
//	if ( m_bRotaryWing ) {
//		// nick
//		//if (fAngle!=0.0f) m_fZ += (0.40f+m_fZ)*tanf(fAngle);
//		//if (fAngle!=0.0f) m_fZ += (0.40f+m_fZ)*fAngle;
//		//if (fAngle!=0.0f) 
//		if (g_bInverted) {
//			m_fZ += (0.80f+m_fY)*sinf(fNickAngle)*g_fMotionRate*0.5f;
//			//m_fZ += (0.60f+m_fY)*sinf(fAngle)*1.0f; // no weight
//		} else {
//			m_fZ += (0.00f+m_fY)*sinf(fNickAngle)*g_fMotionRate*0.5f;
//			//m_fZ += (0.30f+m_fY)*sinf(fAngle)*1.0f; // no weight
//			//m_fZ += sinf(fNickAngle)*g_fMotionRate*0.3f*m_fSpeedFactor;
//		}
//
//		// Set the forward/backward factors
//		// The longer/stronger we go forward the higher the fForwardFactor,
//		// the lower the fBackwardFactor
//		static float fForwardFactor  = 1.0f;
//		static float fBackwardFactor = 1.0f;
//
//	
//		// nick aft
//		if (fNickAngle>0.0f) {
//			fBackwardFactor+=0.08f;
//			m_fZ *= fBackwardFactor;
//
//			// reset
//			//fForwardFactor-=0.08f;
//			//if (fForwardFactor<1.0f)
//				fForwardFactor = 1.0f;
//
//			//	//if (g_bInverted) {
//			//	//	m_fY += fAngle*0.25f;
//			//	//} else {
//			//	//	m_fY -= fAngle*0.25f;
//			//	//}
//			//
//			//	//m_fZ += fAngle*(0.40f+m_fY*1.0f); // afhankelijk van de grootte van de kantelhoek
//			//	m_fZ += (0.40f+m_fZ)*tanf(fAngle);
//		}
//
//		// nick fore
//		if (fNickAngle<0.0f) {
//			fForwardFactor+=0.08f;
//			m_fZ *= fForwardFactor;
//
//			// reset
//			//fBackwardFactor-=0.08f;
//			//if (fBackwardFactor<1.0f)
//				fBackwardFactor = 1.0f;
//
//			//	//if (g_bInverted) {
//			//	//	m_fY -= fAngle*0.25f;
//			//	//} else {
//			//	//	m_fY += fAngle*0.25f;
//			//	//}
//			//
//			//	//m_fZ += fAngle*(0.40f+m_fY*1.0f);
//			//	m_fZ += (0.40f+m_fZ)*tanf(fAngle);
//		}
//	}
//
//	
//		
//	//sprintf( msg9, "Collective (deg): %.2f", 10.0f*m_fY );
//	sprintf( msg9, "Collective: %.2f", m_fCollective );
//	//sprintf( msg9, "Descent Speed: %.2f", m_fSpeedDescent );
//
//
//
//
//	// TODO:
//	// when m_fZ is big, and we roll left, heli yaws left and nicks aft
//	// This enables flying circuits without using yaw
//	// NO
//	// roll left
//	if (fRollAngle>0.0f) {
//		//m_fRadsY += 0.05*m_fZ;
//		//m_fRadsX -= 0.05*m_fZ;
//	}
//	// roll right
//	if (fRollAngle<0.0f) {
//		//m_fRadsY -= 0.05*m_fZ;
//		//m_fRadsX -= 0.05*m_fZ;
//	}
//
//
//	// Tail falls
//	//m_fRadsY += 0.02f*sinf(fRollAngle)*cosf(fNickAngle); // tail falls left/right
//
//
//
//	// Torque tilting
//	// Note1: CSM: direct torque between main rotor and body tends to tilt body
//	// (opposed by pendulum stability of body)
//	// Doordat de tail rotor de torsie tegengaat ontstaat zijdelingse drift. Deze
//	// drift wordt weer opgeheven door de tilting van de main rotor, welke een
//	// tegengesteld gerichte zijdelingse drift geeft.
//	// Door tilting zal de tail rotor ook een component naar boven hebben. Deze
//	// wordt weer opgeheven door back cyclic van main rotor
//	//
//	// left-rotating rotor:
//	// right-rotating body due to torque
//	// tail rotor counteracts torque and causes drift to right
///*	// left-tilting body (and main rotor) due to torque counteracts drift to right
//	if ( m_bRotaryWing ) {		
//		if (m_bLeftRotating) {
//			// for left-rotating rotor (anti-clockwise)
//			m_fX -= 0.04f*g_fMotionRate;		 // drift right
//			//m_fRadsZ -= 0.0001f*g_fMotionRate; // tilt left (not necessary)
//		} else {
//			// for right-rotating rotor (clockwise)
//			m_fX += 0.04f*g_fMotionRate;		 // drift left
//			//m_fRadsZ += 0.0001f*g_fMotionRate; // tilt right
//		}
//	}
//*/	
//
//	// Torque rotation1
//	// Only do this at lift-off. When airborne, gyro will counteract.
//	if ( m_bRotaryWing && m_fTorque > 0.0f) {
//		m_fTorque -= 0.008f;
//		if (m_bLeftRotating) {
//			// for left-rotating rotor, anti-clockwise
//			//m_fRadsY += m_fTorque;//*g_fMotionRate;	// rotate right
//		} else {
//			// for right-rotating rotor, clockwise
//			//m_fRadsY -= m_fTorque;//*g_fMotionRate;	// rotate left
//		}
//	}
//
//
//	// Torque rotation2
//	// Yawing against torque goes slower
//	if ( m_bRotaryWing ) {
//		if (m_bLeftRotating) {
//			// for left-rotating rotor, anti-clockwise
//			if (m_fRadsY<0.0f) m_fRadsY *= 0.7f;//*g_fMotionRate;	// yaw left slower
//		} else {
//			// for right-rotating rotor, clockwise
//			if (m_fRadsY>0.0f) m_fRadsY *= 0.7f;//*g_fMotionRate;	// yaw right slower
//		}
//	}
//
//
//
//	// calculate airspeed
//	static D3DVECTOR vecOld = D3DVECTOR( m_matFileObjectMatrix._41,
//										 m_matFileObjectMatrix._42,
//										 m_matFileObjectMatrix._43 );
//
//	D3DVECTOR vecNew = D3DVECTOR( m_matFileObjectMatrix._41,
//								  m_matFileObjectMatrix._42,
//								  m_matFileObjectMatrix._43 );
//
//	m_fAirSpeed = Magnitude(vecNew-vecOld);
//
//	vecOld = vecNew;
//
//	
//	// calculate m_fSpeedX, m_fSpeedY, m_fSpeedZ /////////
//	static float PosXOld = m_matFileObjectMatrix._41;
//	static float PosYOld = m_matFileObjectMatrix._42;
//	static float PosZOld = m_matFileObjectMatrix._43;
//
//	float PosXNew = m_matFileObjectMatrix._41;
//	float PosYNew = m_matFileObjectMatrix._42;
//	float PosZNew = m_matFileObjectMatrix._43;
//	
//	m_fSpeedX = PosXNew - PosXOld; 
//	m_fSpeedY = PosYNew - PosYOld;  
//	m_fSpeedZ = PosZNew - PosZOld;
//
//	PosXOld = PosXNew;
//	PosYOld = PosYNew;
//	PosZOld = PosZNew;
//	/////////////////////////////////////////////
//
//}





//-----------------------------------------------------------------------------
// Name: RegistryRead()
// Desc: Read registry values such as window position and dimensions
// TODO: read in ALL reg values
//-----------------------------------------------------------------------------
void CMyD3DApplication::RegistryRead()
{
    HKEY  hKey;
	//BYTE  lpData[512];
	DWORD cbData;
	DWORD dw;
	BOOL b;
	FLOAT f;
	//LONG  lRet;

	// read registry
	if ( RegOpenKeyEx(HKEY_LOCAL_MACHINE, g_szRCSIMRegistryKey,
				0, KEY_QUERY_VALUE, &hKey) == ERROR_SUCCESS )
	{			
		// read g_bRegWindowSize
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRegWindowSize", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRegWindowSize = b;
		}

		// read g_bRegFullScreen
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRegFullScreen", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRegFullScreen = b;
		}

		// read g_bRegSplashScreen
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRegSplashScreen", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRegSplashScreen = b;
			SPLASHSCREEN = g_bRegSplashScreen;
			m_bSplashScreenShowing = g_bRegSplashScreen;
		}


		// read g_dwRegModeWidth
		cbData = sizeof(dw);
		if ( RegQueryValueEx( hKey, "g_dwRegModeWidth", NULL, NULL,
			(LPBYTE)&dw, &cbData ) == ERROR_SUCCESS )
		{
			g_dwRegModeWidth = dw;
		}

		// read g_dwRegModeHeight
		cbData = sizeof(dw);
		if ( RegQueryValueEx( hKey, "g_dwRegModeHeight", NULL, NULL,
			(LPBYTE)&dw, &cbData ) == ERROR_SUCCESS )
		{
			g_dwRegModeHeight = dw;
		}

		// read g_dwRegModeRGBBitCount
		cbData = sizeof(dw);
		if ( RegQueryValueEx( hKey, "g_dwRegModeRGBBitCount", NULL, NULL,
			(LPBYTE)&dw, &cbData ) == ERROR_SUCCESS )
		{
			g_dwRegModeRGBBitCount = dw;
		}


		// read g_bRegLoadWDM
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRegLoadWDM", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRegLoadWDM = b;
		}


		// read g_bRegLoadVxD
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRegLoadVxD", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRegLoadVxD = b;
		}

		// read g_dwRegWelcomeVxD
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_dwRegWelcomeVxD", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_dwRegWelcomeVxD = b;
		}

		// read g_dwRegGoodbyeVxD
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_dwRegGoodbyeVxD", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_dwRegGoodbyeVxD = b;
		}


		// read m_bSound
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bSound", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bSound = b;
		}

		// read m_bShowPPMarks
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bShowPPMarks", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bShowPPMarks = b;
		}

		// read m_bDrawShadow
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bDrawShadow", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bDrawShadow = b;
		}

		// read m_bExhaustSmoke
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bExhaustSmoke", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bExhaustSmoke = b;
		}

		// read m_bVibration
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bVibration", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bVibration = b;
		}

		// read m_bTurbulence
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bTurbulence", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bTurbulence = b;
		}

		// read g_bShowSkyFile
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bShowSkyFile", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bShowSkyFile = b;
		}


		// read g_bUseKeyboard
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bUseKeyboard", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bUseKeyboard = b;
		}

		// read g_bUseMouse
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bUseMouse", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bUseMouse = b;
		}

		// read g_bUseJoystick
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bUseJoystick", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bUseJoystick = b;
		}

		// read g_bUseTransmitter
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bUseTransmitter", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bUseTransmitter = b;
		}


		// read INIT_X
		cbData = sizeof(f);
		if ( RegQueryValueEx( hKey, "INIT_X", NULL, NULL,
			(LPBYTE)&f, &cbData ) == ERROR_SUCCESS )
		{
			INIT_X = f;
		}

		// read INIT_Y
		cbData = sizeof(f);
		if ( RegQueryValueEx( hKey, "INIT_Y", NULL, NULL,
			(LPBYTE)&f, &cbData ) == ERROR_SUCCESS )
		{
			INIT_Y = f;
		}

		// read INIT_Z
		cbData = sizeof(f);
		if ( RegQueryValueEx( hKey, "INIT_Z", NULL, NULL,
			(LPBYTE)&f, &cbData ) == ERROR_SUCCESS )
		{
			INIT_Z = f;
		}

		// read INIT_RADS_Y
		cbData = sizeof(f);
		if ( RegQueryValueEx( hKey, "INIT_RADS_Y", NULL, NULL,
			(LPBYTE)&f, &cbData ) == ERROR_SUCCESS )
		{
			INIT_RADS_Y = f;
		}


		// read m_bShowChannels
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bShowChannels", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bShowChannels = b;
		}

		// read m_bShowFlightInfo
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bShowFlightInfo", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bShowFlightInfo = b;
		}

		// read m_dwBackGroundColor
		cbData = sizeof(dw);
		if ( RegQueryValueEx( hKey, "m_dwBackGroundColor", NULL, NULL,
			(LPBYTE)&dw, &cbData ) == ERROR_SUCCESS )
		{
			m_dwBackGroundColor = dw;
		}

					
		// read g_bUseAuthenticHeliSound
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bUseAuthenticHeliSound", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bUseAuthenticHeliSound = b;
		}


		// read g_bShowBoxes
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bShowBoxes", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bShowBoxes = b;
		}

		// read g_bWindsock
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bWindsock", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bWindsock = b;
		}


		// read g_bHelipad
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bHelipad", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bHelipad = b;
		}

		// read g_bTrees
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bTrees", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bTrees = b;
		}

		// read g_bField
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bField", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bField = b;
		}

		// read g_bRunway
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "g_bRunway", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			g_bRunway = b;
		}


		// read g_bPanorama
//		cbData = sizeof(b);
//		if ( RegQueryValueEx( hKey, "g_bPanorama", NULL, NULL,
//			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
//		{
//			g_bPanorama = b;
//		}


		// read m_bEscExits
		cbData = sizeof(b);
		if ( RegQueryValueEx( hKey, "m_bEscExits", NULL, NULL,
			(LPBYTE)&b, &cbData ) == ERROR_SUCCESS )
		{
			m_bEscExits = b;
		}

		
		RegCloseKey(hKey);
	}
}



//-----------------------------------------------------------------------------
// Name: RegistryWrite()
// Desc: Write registry values such as window position and dimensions
//-----------------------------------------------------------------------------
void CMyD3DApplication::RegistryWrite()
{
    HKEY  hKey;
	//BYTE  lpData[80];
	//DWORD cbData = sizeof(lpData);
	DWORD dwDisposition;
	DWORD dw;
	BOOL b;
	FLOAT f;

	RECT rc;
	GetWindowRect( m_hWnd, &rc );

	int x = rc.left;
	int y = rc.top;
	int w = rc.right-rc.left;
	int h = rc.bottom-rc.top;

	

	if ( RegCreateKeyEx(HKEY_LOCAL_MACHINE, g_szRCSIMRegistryKey,
			0, "Class", REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS, NULL,
			&hKey, &dwDisposition) == ERROR_SUCCESS )
	{
		//MessageBox(NULL,"Registry open","QQQ",MB_OK);
/*
		// write x
		_itoa(x, (char *)lpData, 10);
		if ( RegSetValueEx(hKey, "1_x", 0, REG_SZ, 
				(CONST BYTE*)lpData, sizeof(*lpData)) == ERROR_SUCCESS )
		{
			//MessageBox(NULL,"Registry write x","QQQ",MB_OK);
		}

		// write y
		_itoa(y, (char *)lpData, 10);
		RegSetValueEx(hKey, "1_y", 0, REG_SZ, 
				(CONST BYTE*)lpData, sizeof(*lpData));

		// write w
		_itoa(w, (char *)lpData, 10);
		RegSetValueEx(hKey, "1_w", 0, REG_SZ, 
				(CONST BYTE*)lpData, sizeof(*lpData));

		// write h
		_itoa(h, (char *)lpData, 10);
		RegSetValueEx(hKey, "1_h", 0, REG_SZ, 
				(CONST BYTE*)lpData, sizeof(*lpData) );
*/

	
		// Better use DWORD instead of strings here
		// write x
		dw = x;
		if ( RegSetValueEx(hKey, "1_x", 0, REG_DWORD, 
				(LPBYTE)&dw, sizeof(dw)) == ERROR_SUCCESS )
		{
			//MessageBox(NULL,"Registry write x","QQQ",MB_OK);
		}

		// write y
		dw = y;
		RegSetValueEx(hKey, "1_y", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));

		// write w
		dw = w;
		RegSetValueEx(hKey, "1_w", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));

		// write h
		dw = h;
		RegSetValueEx(hKey, "1_h", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));


		// write g_bRegWindowSize
		b = g_bRegWindowSize;
		RegSetValueEx(hKey, "g_bRegWindowSize", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));
		
		// write g_bRegFullScreen
		b = g_bRegFullScreen;
		RegSetValueEx(hKey, "g_bRegFullScreen", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bRegSplashScreen
		b = g_bRegSplashScreen;
		RegSetValueEx(hKey, "g_bRegSplashScreen", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write g_dwRegModeWidth
		dw = g_dwRegModeWidth;
		RegSetValueEx(hKey, "g_dwRegModeWidth", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));

		// write g_dwRegModeHeight
		dw = g_dwRegModeHeight;
		RegSetValueEx(hKey, "g_dwRegModeHeight", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));

		// write g_dwRegModeRGBBitCount
		dw = g_dwRegModeRGBBitCount;
		RegSetValueEx(hKey, "g_dwRegModeRGBBitCount", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));


		// write g_bRegLoadWDM
		b = g_bRegLoadWDM;
		RegSetValueEx(hKey, "g_bRegLoadWDM", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

 
		// write g_bRegLoadVxD
		b = g_bRegLoadVxD;
		RegSetValueEx(hKey, "g_bRegLoadVxD", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_dwRegWelcomeVxD
		b = g_dwRegWelcomeVxD;
		RegSetValueEx(hKey, "g_dwRegWelcomeVxD", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_dwRegGoodbyeVxD
		b = g_dwRegGoodbyeVxD;
		RegSetValueEx(hKey, "g_dwRegGoodbyeVxD", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write m_bSound
		b = m_bSound;
		RegSetValueEx(hKey, "m_bSound", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bShowPPMarks
		b = m_bShowPPMarks;
		RegSetValueEx(hKey, "m_bShowPPMarks", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bDrawShadow
		b = m_bDrawShadow;
		RegSetValueEx(hKey, "m_bDrawShadow", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bExhaustSmoke
		b = m_bExhaustSmoke;
		RegSetValueEx(hKey, "m_bExhaustSmoke", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bVibration
		b = m_bVibration;
		RegSetValueEx(hKey, "m_bVibration", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bTurbulence
		b = m_bTurbulence;
		RegSetValueEx(hKey, "m_bTurbulence", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bShowSkyFile
		b = g_bShowSkyFile;
		RegSetValueEx(hKey, "g_bShowSkyFile", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));



		// write g_bUseKeyboard
		b = g_bUseKeyboard;
		RegSetValueEx(hKey, "g_bUseKeyboard", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bUseMouse
		b = g_bUseMouse;
		RegSetValueEx(hKey, "g_bUseMouse", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bUseJoystick
		b = g_bUseJoystick;
		RegSetValueEx(hKey, "g_bUseJoystick", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bUseTransmitter
		b = g_bUseTransmitter;
		RegSetValueEx(hKey, "g_bUseTransmitter", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// NOTE: must write REG_BINARY for floats!!!!!!
		// write INIT_X
		f = INIT_X;
		RegSetValueEx(hKey, "INIT_X", 0, REG_BINARY, (LPBYTE)&f, sizeof(f));

		// write INIT_Y
		f = INIT_Y;
		RegSetValueEx(hKey, "INIT_Y", 0, REG_BINARY, (LPBYTE)&f, sizeof(f));

		// write INIT_Z
		f = INIT_Z;
		RegSetValueEx(hKey, "INIT_Z", 0, REG_BINARY, (LPBYTE)&f, sizeof(f));

		// write INIT_RADS_Y
		f = INIT_RADS_Y;
		RegSetValueEx(hKey, "INIT_RADS_Y", 0, REG_BINARY, (LPBYTE)&f, sizeof(f));

		
		// write m_bShowChannels
		b = m_bShowChannels;
		RegSetValueEx(hKey, "m_bShowChannels", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_bShowFlightInfo
		b = m_bShowFlightInfo;
		RegSetValueEx(hKey, "m_bShowFlightInfo", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write m_dwBackGroundColor
		dw = m_dwBackGroundColor;
		RegSetValueEx(hKey, "m_dwBackGroundColor", 0, REG_DWORD, (LPBYTE)&dw, sizeof(dw));


		// write g_bUseAuthenticHeliSound
		b = g_bUseAuthenticHeliSound;
		RegSetValueEx(hKey, "g_bUseAuthenticHeliSound", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write g_bShowBoxes
		b = g_bShowBoxes;
		RegSetValueEx(hKey, "g_bShowBoxes", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bWindsock
		b = g_bWindsock;
		RegSetValueEx(hKey, "g_bWindsock", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write g_bHelipad
		b = g_bHelipad;
		RegSetValueEx(hKey, "g_bHelipad", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bTrees
		b = g_bTrees;
		RegSetValueEx(hKey, "g_bTrees", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bField
		b = g_bField;
		RegSetValueEx(hKey, "g_bField", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));

		// write g_bRunway
		b = g_bRunway;
		RegSetValueEx(hKey, "g_bRunway", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write g_bPanorama
		b = g_bPanorama;
		RegSetValueEx(hKey, "g_bPanorama", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		// write m_bEscExits
		b = m_bEscExits;
		RegSetValueEx(hKey, "m_bEscExits", 0, REG_DWORD, (LPBYTE)&b, sizeof(b));


		RegCloseKey(hKey);
	}
}


//-----------------------------------------------------------------------------
// Name: SetMenuItems()
// Desc: Checks/unchecks, enables/disables menu items
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::SetMenuItems(HMENU hmenu)
{
	// NOT IN VERSION 1.5
	DeleteMenu(hmenu, IDM_LOADSCENERY, MF_BYCOMMAND);
	DeleteMenu(hmenu, IDM_FILE_OPENSIMULATION, MF_BYCOMMAND);
	DeleteMenu(hmenu, IDM_FILE_SAVESIMULATION, MF_BYCOMMAND);
	static bool firsttime = true;
	if (firsttime) {
		firsttime = false;
		DeleteMenu( GetSubMenu(hmenu,0), 8, MF_BYPOSITION ); // delete separator
	}
	
	// register
	//if (g_bShareWareRegistered)
	//	EnableMenuItem(hmenu,IDM_REGISTER,MF_GRAYED);


// demo
#ifdef DEMO
//	MENUITEMINFO mii;
//	mii.cbSize = sizeof(MENUITEMINFO);
//	mii.fMask = MIIM_CHECKMARKS | MIIM_DATA | MIIM_ID | MIIM_STATE | MIIM_SUBMENU | MIIM_TYPE;
//	mii.fType = MFT_STRING;
//	mii.fState = MFS_ENABLED | MFS_UNCHECKED; 
//	mii.wID = IDM_REGISTER; 
//	mii.hSubMenu = NULL; 
//	mii.hbmpChecked = NULL; 
//	mii.hbmpUnchecked = NULL; 
//	mii.dwItemData = 0; 
//	mii.dwTypeData = "&Purchase..."; 
//	mii.cch = sizeof("&Purchase...");
//
//	SetMenuItemInfo(hmenu, IDM_REGISTER, FALSE, &mii);

	ModifyMenu(hmenu, IDM_REGISTER, MF_BYCOMMAND|MF_STRING, IDM_REGISTER, "&Purchase...");
#endif


	// channels
	if (m_bShowChannels)
		CheckMenuItem(hmenu,ID_SHOWCHANNELS,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_SHOWCHANNELS,MF_UNCHECKED);

	// flight info
	if (m_bShowFlightInfo)
		CheckMenuItem(hmenu,ID_SHOWFLIGHTINFO,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_SHOWFLIGHTINFO,MF_UNCHECKED);


	// pilot position
	switch(m_uPilotPosition)
	{
		case 1:
			CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_CHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
			break;
		case 2:
			CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_CHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
			break;
		case 3:
			CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_CHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_UNCHECKED);
			break;
		case 4:
			CheckMenuItem(hmenu,ID_PILOTPOSITION1,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION2,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION3,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_PILOTPOSITION4,MF_CHECKED);
			break;
	}

	// RT Config
	if (m_pDeviceInfo->bWindowed) {
		// FSWindow
		EnableMenuItem(hmenu,IDM_RT_CONFIGURATION,MF_ENABLED);
		EnableMenuItem(hmenu,IDM_FLIGHT_RECORDER,MF_ENABLED);
	} else {
		EnableMenuItem(hmenu,IDM_RT_CONFIGURATION,MF_GRAYED);
		EnableMenuItem(hmenu,IDM_FLIGHT_RECORDER,MF_GRAYED);
	}
	
	// WDM
	// VxD
	if (g_bLoadedVxD || g_bLoadedWDM) {
		EnableMenuItem(hmenu,IDC_TRANSMITTER,MF_ENABLED);
		EnableMenuItem(hmenu,IDM_CALIBRATION,MF_ENABLED);
	} else {
		EnableMenuItem(hmenu,IDC_TRANSMITTER,MF_GRAYED);
		EnableMenuItem(hmenu,IDM_CALIBRATION,MF_GRAYED);
	}

	// sound
	if (m_bSound)
		CheckMenuItem(hmenu,ID_OPTIONS_SOUND,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_SOUND,MF_UNCHECKED);

	// controls
	if (g_bUseKeyboard)
		CheckMenuItem(hmenu,IDC_KEYBOARD,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_KEYBOARD,MF_UNCHECKED);

	if (g_bUseMouse)
		CheckMenuItem(hmenu,IDC_MOUSE,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_MOUSE,MF_UNCHECKED);

	if (g_bUseJoystick)
		CheckMenuItem(hmenu,IDC_JOYSTICK,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_JOYSTICK,MF_UNCHECKED);

	if (g_bUseTransmitter)
		CheckMenuItem(hmenu,IDC_TRANSMITTER,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_TRANSMITTER,MF_UNCHECKED);

	
	// smooth controls
	if (m_bSmoothControls)
		CheckMenuItem(hmenu,IDC_SMOOTHCONTROLS,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_SMOOTHCONTROLS,MF_UNCHECKED);	
	
	// smooth cam controls
	if (m_bSmoothCamControls)
		CheckMenuItem(hmenu,IDC_SMOOTHCAMCONTROLS,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDC_SMOOTHCAMCONTROLS,MF_UNCHECKED);

	
	// pilot position only in R/C and Follow View
	// hell, we can't enable/disable the popup
	if (m_bRCView || m_bFollowView) {
		EnableMenuItem(hmenu,ID_PILOTPOSITION1,MF_ENABLED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION2,MF_ENABLED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION3,MF_ENABLED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION4,MF_ENABLED);
	} else {
		EnableMenuItem(hmenu,ID_PILOTPOSITION1,MF_GRAYED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION2,MF_GRAYED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION3,MF_GRAYED);
		EnableMenuItem(hmenu,ID_PILOTPOSITION4,MF_GRAYED);
	}

	// marks
	if (m_bShowPPMarks)
		CheckMenuItem(hmenu,ID_SHOWPPMARKS,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_SHOWPPMARKS,MF_UNCHECKED);

	
	// fog
	if (m_bFog)
		CheckMenuItem(hmenu,ID_OPTIONS_FOG,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_FOG,MF_UNCHECKED);



	// lensflare
	if (g_bDrawLensFlare) {
		CheckMenuItem(hmenu,ID_OPTIONS_LENSFLARE,MF_CHECKED);
		EnableMenuItem(hmenu,ID_OPTIONS_SUN,MF_ENABLED);
		EnableMenuItem(hmenu,ID_OPTIONS_FLARE,MF_ENABLED);
	} else {
		CheckMenuItem(hmenu,ID_OPTIONS_LENSFLARE,MF_UNCHECKED);
		EnableMenuItem(hmenu,ID_OPTIONS_SUN,MF_GRAYED);
		EnableMenuItem(hmenu,ID_OPTIONS_FLARE,MF_GRAYED);
	}

	// sun
	if (g_bSun)
		CheckMenuItem(hmenu,ID_OPTIONS_SUN,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_SUN,MF_UNCHECKED);

	// flare
	if (g_bFlare)
		CheckMenuItem(hmenu,ID_OPTIONS_FLARE,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_FLARE,MF_UNCHECKED);


	// vibration
	if (m_bVibration)
		CheckMenuItem(hmenu,IDM_VIBRATION,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDM_VIBRATION,MF_UNCHECKED);

	// turbulence
	if (m_bTurbulence)
		CheckMenuItem(hmenu,IDM_TURBULENCE,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDM_TURBULENCE,MF_UNCHECKED);


	// record
	if (m_bRecord) {
		CheckMenuItem(hmenu,IDM_RECORD,MF_CHECKED);
		EnableMenuItem(hmenu,IDM_PLAYBACK,MF_GRAYED);
	} else {
		CheckMenuItem(hmenu,IDM_RECORD,MF_UNCHECKED);
		EnableMenuItem(hmenu,IDM_PLAYBACK,MF_ENABLED);
	}

	// playback
	if (m_bPlayBack) {
		CheckMenuItem(hmenu,IDM_PLAYBACK,MF_CHECKED);
		EnableMenuItem(hmenu,IDM_RECORD,MF_GRAYED);
	} else {
		CheckMenuItem(hmenu,IDM_PLAYBACK,MF_UNCHECKED);
		EnableMenuItem(hmenu,IDM_RECORD,MF_ENABLED);
	}


	// status bar
	if (g_bStatusBar)
		CheckMenuItem(hmenu,IDM_STATUSBAR,MF_CHECKED);
	else
		CheckMenuItem(hmenu,IDM_STATUSBAR,MF_UNCHECKED);

	if (m_pDeviceInfo->bWindowed)
		EnableMenuItem(hmenu,IDM_STATUSBAR,MF_ENABLED);
	else
		EnableMenuItem(hmenu,IDM_STATUSBAR,MF_GRAYED);


	// fullscreen toggle
	if (m_pDeviceInfo->bWindowed)
		CheckMenuItem(hmenu,IDM_TOGGLEFULLSCREEN,MF_UNCHECKED);
	else
		CheckMenuItem(hmenu,IDM_TOGGLEFULLSCREEN,MF_CHECKED);
	

	// shadow
	if (m_bDrawShadow)
		CheckMenuItem(hmenu,ID_OPTIONS_SHADOW,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_SHADOW,MF_UNCHECKED);

	// shadow volume
	if (m_bDrawShadowVolume)
		CheckMenuItem(hmenu,ID_OPTIONS_SHADOWVOLUME,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_SHADOWVOLUME,MF_UNCHECKED);

	
	// exhaust smoke
	if (m_bExhaustSmoke)
		CheckMenuItem(hmenu,ID_OPTIONS_EXHAUSTSMOKE,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_EXHAUSTSMOKE,MF_UNCHECKED);


	// runway
	if (g_bRunway)
		CheckMenuItem(hmenu,ID_OPTIONS_RUNWAY,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_RUNWAY,MF_UNCHECKED);
	
	// helipad
	if (g_bHelipad)
		CheckMenuItem(hmenu,ID_OPTIONS_HELIPAD,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_HELIPAD,MF_UNCHECKED);	
	
	// field
	if (g_bField)
		CheckMenuItem(hmenu,ID_OPTIONS_FIELD,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_FIELD,MF_UNCHECKED);

	
	// trees
	if (g_bTrees)
		CheckMenuItem(hmenu,ID_OPTIONS_TREES,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_TREES,MF_UNCHECKED);
	
	
	// windsock
	if (g_bWindsock)
		CheckMenuItem(hmenu,ID_OPTIONS_WINDSOCK,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_WINDSOCK,MF_UNCHECKED);

	// wind
	if (g_bWind)
		CheckMenuItem(hmenu,ID_OPTIONS_WIND,MF_CHECKED);
	else
		CheckMenuItem(hmenu,ID_OPTIONS_WIND,MF_UNCHECKED);
		

	
	// texture filtering
	switch(m_iTextureFilter)
	{
		case 0:
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_POINT,MF_CHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_LINEAR,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_ANISOTROPIC,MF_UNCHECKED);
			break;
		
		case 1:
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_POINT,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_LINEAR,MF_CHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_ANISOTROPIC,MF_UNCHECKED);
			break;
			
		case 2:
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_POINT,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_LINEAR,MF_UNCHECKED);
			CheckMenuItem(hmenu,ID_OPTIONS_TEXTUREFILTERING_ANISOTROPIC,MF_CHECKED);
			break;
	}


	// dynamics
	if (m_bFixedWing) {
		CheckMenuItem(hmenu,IDC_FIXEDWING,MF_CHECKED);
		CheckMenuItem(hmenu,IDC_ROTARYWING,MF_UNCHECKED);
		CheckMenuItem(hmenu,IDC_6DOF,MF_UNCHECKED);
	}
	if (m_bRotaryWing) {
		CheckMenuItem(hmenu,IDC_FIXEDWING,MF_UNCHECKED);
		CheckMenuItem(hmenu,IDC_ROTARYWING,MF_CHECKED);
		CheckMenuItem(hmenu,IDC_6DOF,MF_UNCHECKED);
	}
	if (m_b6DOF) {
		CheckMenuItem(hmenu,IDC_FIXEDWING,MF_UNCHECKED);
		CheckMenuItem(hmenu,IDC_ROTARYWING,MF_UNCHECKED);
		CheckMenuItem(hmenu,IDC_6DOF,MF_CHECKED);
	}
	
}






//-----------------------------------------------------------------------------
// Name: SetRCViewMatrix()
// Desc: Sets the R/C View matrix
//-----------------------------------------------------------------------------
void CMyD3DApplication::SetRCViewMatrix()
{
	// TODO: make camera follow the object like an R/C pilot follows his model
	// DONE!!!
	// not below decks
	if (m_fCamY<-9.6f) m_fCamY=-9.6f;

	// run rings
	//m_fCamX = m_fCamX - (m_fCamX - cosf(m_fCamRadsY)*m_fDistance);
	//m_fCamZ = m_fCamZ - (m_fCamZ - sinf(m_fCamRadsY)*m_fDistance);


	D3DVECTOR vEyePt    = D3DVECTOR( m_fCamX, m_fCamY, m_fCamZ );
	D3DVECTOR vLookatPt = D3DVECTOR( 0.0f, 0.0f, 1.0f );
	D3DVECTOR vUpVec    = D3DVECTOR( 0.0f, 1.0f, 0.0f );

	// camera looks at object
	//vLookatPt.x = m_matFileObjectMatrix._41;
	//vLookatPt.y = m_matFileObjectMatrix._42/**0.7f*/; // kijk iets onder model bij steil
	//vLookatPt.z = m_matFileObjectMatrix._43;			// omhoog kijken (= natuurlijker)


	// TODO: camera latency. DONE: Use latency array. See: Chase View
	// NOTE: commenting this out keeps the camera motionless
	vLookatPt.x = g_matOld[0]._41;
	vLookatPt.y = g_matOld[0]._42;
	vLookatPt.z = g_matOld[0]._43;

	// NOTE: Reflex uses a camera latency from 0% to 100% with 0% keeping the model
	// always in the centre and 100% keeping the camera totally motionless. The latter
	// means that Reflex uses an other algorithm than latency arrays... But what?
	// When 0% it also follows the heli's bounces.


	// TODO:
	// Camera in R/C View should not follow the bounces. See: Reflex
	// By dampening? Dampening only when on the ground?
	// Well, Reflex does it as well: follow the bounces. It is just that
	// it is more conspicuous now that we have a panoramic scenery. So don't
	// change anything here.
//	static D3DVECTOR vLookatPtOld;
//	float factor = 0.01f;
//
//	if ( vLookatPtOld.x-vLookatPt.x < factor /*|| vLookatPt.x-vLookatPtOld.x < factor*/ )
//		vLookatPt.x = vLookatPtOld.x;
//	if ( vLookatPtOld.y-vLookatPt.y < factor || vLookatPt.y-vLookatPtOld.y < factor )
//		vLookatPt.y = vLookatPtOld.y;
//	if ( vLookatPtOld.z-vLookatPt.z < factor /*|| vLookatPt.z-vLookatPtOld.z < factor*/ )
//		vLookatPt.z = vLookatPtOld.z;
//
//	vLookatPtOld = vLookatPt;

//	static bool fix = false;
//	if (m_fAltitude < 1.0f && fix == false) {
//		fix = true;
//		vLookatPtOld = vLookatPt;
//	} 
//	
//	if (m_fAltitude >= 1.0f) {
//		fix = false;
//	}
//	if (fix) vLookatPt = vLookatPtOld;







	// TODO:
	// Implement pilot view pan/tilt
	// Well, don't: let's keep things simple.





	// PANORAMA
	D3DVECTOR vWorldRight(1.0f, 0.0f, 0.0f);
	D3DVECTOR vCamFlat(vLookatPt.x, 0.0f, vLookatPt.z);

	D3DVECTOR vCamPan(vLookatPt.x, 0.0f, vLookatPt.z);
	D3DVECTOR vCamTilt(vLookatPt.x, vLookatPt.y, vLookatPt.z);

	vCamPan  = Normalize(vCamPan);
	vCamTilt = Normalize(vCamTilt);
	vCamFlat = Normalize(vCamFlat);

	g_fCosAngle = DotProduct(vCamPan, vWorldRight);
	g_fSinAngle = DotProduct(vCamTilt, vCamFlat);

	g_fCamPanAngle = 0.0f;
	g_fCamTiltAngle = 0.0f;

	g_fCamPanAngle = acosf(g_fCosAngle);
	if (vLookatPt.z < 0.0f) g_fCamPanAngle = g_PI + (g_PI-g_fCamPanAngle);

	g_fCamTiltAngle = asinf(g_fSinAngle);
	// Note: we want 0 = full up and 180 = full down
	if (vLookatPt.y < 0.0f) g_fCamTiltAngle = g_PI_DIV_2 + (g_PI_DIV_2-g_fCamTiltAngle);


	sprintf( msg8, "Camera Pan  (deg): %.2f", (g_fCamPanAngle *180.0f)/g_PI );
	//sprintf( msg8, "Camera Tilt (deg): %.2f", (g_fCamTiltAngle*180.0f)/g_PI );



	// zoom
	//g_fZoomValue = 0.0f;
	//m_fDistance += g_fZoomValue;
	m_fDistance = Magnitude(vEyePt-vLookatPt)+g_fZoomValue;
	if (g_bZooming) {
		if (g_bZoomIn) {
			g_fZoomValue -= 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue -= 0.3f;

			// zoom max
			if (m_fDistance<6.0f) {
				g_fZoomValue += 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
				g_bZooming=false;
			}
		} else {
			g_fZoomValue += 0.5f*ZOOM_OUT_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue += 0.3f;
			
			// zoom min
			if (g_fZoomValue>0.0f) {
				g_fZoomValue=0.0f;
				g_bZooming=false;
			}
		}
	}
	   
	// Set the view matrix
	D3DUtil_SetViewMatrix( m_matView, vEyePt, vLookatPt, vUpVec );

	// translation
	//D3DMATRIX matTrans;
	//D3DUtil_SetTranslateMatrix( matTrans, m_fCamX, m_fCamY, m_fCamZ );
	//D3DMath_MatrixMultiply(m_matView, m_matView, matTrans);


	// reset
	//m_fCamX=m_fCamY=m_fCamZ=0.0f;

	// run rings //////////////////////////	
	//D3DXMATRIX matRotY;
	//D3DUtil_SetRotateYMatrix( matRotY, m_fCamX*0.1f);

	
	//D3DXMatrixRotationAxis(&matRotY, (D3DXVECTOR3 *)&vUpVec, m_fCamRadsY);

	// to heli origin
	//FLOAT x = m_matView._41;
	//FLOAT y = m_matView._42;
	//FLOAT z = m_matView._43;

	//m_matView._41 = 0.0f; 
	//m_matView._42 = 0.0f;
	//m_matView._43 -= m_fDistance;

	// then rotate
	//D3DMath_MatrixMultiply( m_matView, m_matView, matRotY ); // order is crucial!!!

	// bring object back to its position
	//m_matView._41 = x; 
	//m_matView._42 = y;
	//m_matView._43 = z;

	////////////////////////////////////////


	// zoom
	m_matView._43 += g_fZoomValue;
	//m_matView._42 += (g_fZoomValue/8.0f);
	sprintf( msg10, "Zoom (m): %.2f", -g_fZoomValue );
	
}


//-----------------------------------------------------------------------------
// Name: SetInModelViewMatrix()
// Desc: Sets the In-Model View matrix
//-----------------------------------------------------------------------------
void CMyD3DApplication::SetInModelViewMatrix()
{
	D3DMath_MatrixInvert(m_matView, m_matFileObjectMatrix);
	D3DMATRIX matRot2;
	D3DUtil_SetRotateYMatrix(matRot2, g_PI);
	D3DMath_MatrixMultiply(m_matView, m_matView, matRot2);


	// zoom
	//g_fZoomValue = 0.0f;
	//m_fDistance += g_fZoomValue;
	if (g_bZooming) {
		if (g_bZoomIn) {
			g_fZoomValue -= 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue -= 0.3f;

			// zoom max
			if (g_fZoomValue<-1000.0f) {
				g_fZoomValue += 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
				g_bZooming=false;
			}
		} else {
			g_fZoomValue += 0.5f*ZOOM_OUT_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue += 0.3f;
			
			// zoom min
			if (g_fZoomValue>0.0f) {
				g_fZoomValue=0.0f;
				g_bZooming=false;
			}
		}
	}

	m_matView._43 -= 2.0f;

	// zoom
	m_matView._43 += g_fZoomValue;
	sprintf( msg10, "Zoom (m): %.2f", -g_fZoomValue );


	if (g_bCobra)
		m_matView._43 -= 1.6f;
	if (g_bBell)
		m_matView._43 -= 1.15f;
	if (g_bCougar)
		m_matView._43 -= 1.7f;
}


//-----------------------------------------------------------------------------
// Name: SetChaseViewMatrix()
// Desc: Sets the Chase View matrix
//-----------------------------------------------------------------------------
void CMyD3DApplication::SetChaseViewMatrix()
{
//	g_matOld2[0]._11 = 0.0f;
//	g_matOld2[0]._21 = 0.0f;
//	g_matOld2[0]._31 = 0.0f;

	// we don't want latency for the distance between heli and camera
	g_matOld2[0]._41 = m_matFileObjectMatrix._41;
	g_matOld2[0]._42 = m_matFileObjectMatrix._42;
	g_matOld2[0]._43 = m_matFileObjectMatrix._43;


	D3DMath_MatrixInvert(m_matView, g_matOld2[0]);



	D3DMATRIX matRot, matRotX, matRotY;
	D3DUtil_SetRotateXMatrix(matRotX, m_fCamRadsX);
	D3DUtil_SetRotateYMatrix(matRotY, g_PI+m_fCamRadsY);

	// order order!!! and use matRot to first combine!!!
	D3DMath_MatrixMultiply(matRot, matRotY, matRotX);
	D3DMath_MatrixMultiply(m_matView, m_matView, matRot);


	// zoom
	//g_fZoomValue = 0.0f;
	//m_fDistance += g_fZoomValue;
	if (g_bZooming) {
		if (g_bZoomIn) {
			g_fZoomValue -= 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue -= 0.3f;

			// zoom max
			// max zoom to origin of heli
			if (g_fZoomValue<-20.0f) { 
				g_fZoomValue += 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
				g_bZooming=false;
			}
		} else {
			g_fZoomValue += 0.5f*ZOOM_OUT_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue += 0.3f;
			
			// zoom min
			if (g_fZoomValue>1000.0f) {
				g_fZoomValue=1000.0f;
				g_bZooming=false;
			}
		}
	}

	// trail
	m_matView._43 += 10.0f;

	// zoom
	m_matView._43 += g_fZoomValue;
	sprintf( msg10, "Zoom (m): %.2f", -g_fZoomValue );

}


//-----------------------------------------------------------------------------
// Name: SetFollowViewMatrix()
// Desc: Sets the Follow matrix
//-----------------------------------------------------------------------------
void CMyD3DApplication::SetFollowViewMatrix()
{
	// not below decks
	//if (m_fCamY<-6.0f) m_fCamY=-6.0f;

	// run rings
	//m_fCamX = m_fCamX - (m_fCamX - cosf(m_fCamRadsY)*m_fDistance);
	//m_fCamZ = m_fCamZ - (m_fCamZ - sinf(m_fCamRadsY)*m_fDistance);

	D3DVECTOR vEyePt    = D3DVECTOR( 0.0f, 0.0f, 0.0f );
	D3DVECTOR vLookatPt = D3DVECTOR( 0.0f, 0.0f, 1.0f );
	D3DVECTOR vUpVec    = D3DVECTOR( 0.0f, 1.0f, 0.0f );

	// camera looks at object
	//vLookatPt.x = m_matFileObjectMatrix._41;
	//vLookatPt.y = m_matFileObjectMatrix._42;
	//vLookatPt.z = m_matFileObjectMatrix._43;

	//vEyePt.x = m_matFileObjectMatrix._41+m_fCamX;
	//vEyePt.y = m_matFileObjectMatrix._42+m_fCamY;
	//vEyePt.z = m_matFileObjectMatrix._43+m_fCamZ+20.0f;


	vLookatPt.x = g_matOld3[0]._41;
	vLookatPt.y = g_matOld3[0]._42;
	vLookatPt.z = g_matOld3[0]._43;

	vEyePt.x = g_matOld3[0]._41 + m_fCamX;
	vEyePt.y = g_matOld3[0]._42 + m_fCamY;
	vEyePt.z = g_matOld3[0]._43 + m_fCamZ-20.0f;

	if (g_bFollowTail) {
		// TEST:
		// Make Follow View always follow the tail
		vEyePt.x = 0.0f  - m_fCamX;
		vEyePt.y = 0.0f  + m_fCamY;
		vEyePt.z = 15.0f - m_fCamZ;
		
		D3DMath_VectorMatrixMultiply(vEyePt, vEyePt, g_matOld3[0]/*g_pd3dApp->m_matFileObjectMatrix*/);

		// camera y stays same height as heli
		vEyePt.y = g_matOld3[0]._42 + m_fCamY;
		

		//if (vEyePt.y < -8.5) vEyePt.y = -8.5f;
	}

	// Set the view matrix
	D3DUtil_SetViewMatrix( m_matView, vEyePt, vLookatPt, vUpVec );


	// zoom
	//g_fZoomValue = 0.0f;
	//m_fDistance += g_fZoomValue;
	if (g_bZooming) {
		if (g_bZoomIn) {
			g_fZoomValue -= 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue -= 0.3f;

			// zoom max
			//if (g_fZoomValue<-10.0f) {
			//	g_fZoomValue += 0.5f*ZOOM_IN_SENS*m_fSpeedFactor;
			//	g_bZooming=false;
			//}
		} else {
			g_fZoomValue += 0.5f*ZOOM_OUT_SENS*m_fSpeedFactor;
			//if (g_bBell || g_bCobra) g_fZoomValue += 0.3f;
			
			// zoom min
			//if (g_fZoomValue>100.0f) {
			//	g_fZoomValue=100.0f;
			//	g_bZooming=false;
			//}
		}
	}

	// zoom
	m_matView._43 += g_fZoomValue;
	sprintf( msg10, "Zoom (m): %.2f", -g_fZoomValue );


}













//-----------------------------------------------------------------------------
// Name: DoRTConfigPosition()
// Desc: Set Pilot Position Marks
//-----------------------------------------------------------------------------
void CMyD3DApplication::DoRTConfigPosition()
{
	// Position 1
	if (g_bRTSpinnerPosition1XUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2)
			PILOTPOSITION1_X += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0)
			PILOTPOSITION1_X += 0.01f;
		else
			PILOTPOSITION1_X += 0.1f;
		m_matPP1Matrix._41 = PILOTPOSITION1_X;
	}
	if (g_bRTSpinnerPosition1XDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2)
			PILOTPOSITION1_X -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION1_X -= 0.01f;
		else
			PILOTPOSITION1_X -= 0.1f;
		m_matPP1Matrix._41 = PILOTPOSITION1_X;
	}

	if (g_bRTSpinnerPosition1YUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2)
			PILOTPOSITION1_Y += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION1_Y += 0.01f;
		else
			PILOTPOSITION1_Y += 0.1f;
		m_matPP1Matrix._42 = PILOTPOSITION1_Y;
	}  
	if (g_bRTSpinnerPosition1YDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION1_Y -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION1_Y -= 0.01f;
		else
			PILOTPOSITION1_Y -= 0.1f;
		m_matPP1Matrix._42 = PILOTPOSITION1_Y;
	}

	if (g_bRTSpinnerPosition1ZUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION1_Z += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION1_Z += 0.01f;
		else
			PILOTPOSITION1_Z += 0.1f;
		m_matPP1Matrix._43 = PILOTPOSITION1_Z;
	}  
	if (g_bRTSpinnerPosition1ZDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION1_Z -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION1_Z -= 0.01f;
		else
			PILOTPOSITION1_Z -= 0.1f;
		m_matPP1Matrix._43 = PILOTPOSITION1_Z;
	}


	// Position 2
	if (g_bRTSpinnerPosition2XUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION2_X += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION2_X += 0.01f;
		else
			PILOTPOSITION2_X += 0.1f;
		m_matPP2Matrix._41 = PILOTPOSITION2_X;
	}  
	if (g_bRTSpinnerPosition2XDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION2_X -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION2_X -= 0.01f;
		else
			PILOTPOSITION2_X -= 0.1f;
		m_matPP2Matrix._41 = PILOTPOSITION2_X;
	}

	if (g_bRTSpinnerPosition2YUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION2_Y += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION2_Y += 0.01f;
		else
			PILOTPOSITION2_Y += 0.1f;
		m_matPP2Matrix._42 = PILOTPOSITION2_Y;
	}  
	if (g_bRTSpinnerPosition2YDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION2_Y -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION2_Y -= 0.01f;
		else
			PILOTPOSITION2_Y -= 0.1f;
		m_matPP2Matrix._42 = PILOTPOSITION2_Y;
	}

	if (g_bRTSpinnerPosition2ZUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION2_Z += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION2_Z += 0.01f;
		else
			PILOTPOSITION2_Z += 0.1f;
		m_matPP2Matrix._43 = PILOTPOSITION2_Z;
	}  
	if (g_bRTSpinnerPosition2ZDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION2_Z -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION2_Z -= 0.01f;
		else
			PILOTPOSITION2_Z -= 0.1f;
		m_matPP2Matrix._43 = PILOTPOSITION2_Z;
	}


	// Position 3
	if (g_bRTSpinnerPosition3XUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION3_X += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION3_X += 0.01f;
		else
			PILOTPOSITION3_X += 0.1f;
		m_matPP3Matrix._41 = PILOTPOSITION3_X;
	}  
	if (g_bRTSpinnerPosition3XDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION3_X -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION3_X -= 0.01f;
		else
			PILOTPOSITION3_X -= 0.1f;
		m_matPP3Matrix._41 = PILOTPOSITION3_X;
	}

	if (g_bRTSpinnerPosition3YUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION3_Y += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION3_Y += 0.01f;
		else
			PILOTPOSITION3_Y += 0.1f;
		m_matPP3Matrix._42 = PILOTPOSITION3_Y;
	}  
	if (g_bRTSpinnerPosition3YDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION3_Y -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION3_Y -= 0.01f;
		else
			PILOTPOSITION3_Y -= 0.1f;
		m_matPP3Matrix._42 = PILOTPOSITION3_Y;
	}

	if (g_bRTSpinnerPosition3ZUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION3_Z += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION3_Z += 0.01f;
		else
			PILOTPOSITION3_Z += 0.1f;
		m_matPP3Matrix._43= PILOTPOSITION3_Z;
	}  
	if (g_bRTSpinnerPosition3ZDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION3_Z -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION3_Z -= 0.01f;
		else
			PILOTPOSITION3_Z -= 0.1f;
		m_matPP3Matrix._43 = PILOTPOSITION3_Z;
	}


	// Position 4
	if (g_bRTSpinnerPosition4XUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION4_X += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION4_X += 0.01f;
		else
			PILOTPOSITION4_X += 0.1f;
		m_matPP4Matrix._41 = PILOTPOSITION4_X;
	}  
	if (g_bRTSpinnerPosition4XDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionX == 2 )
			PILOTPOSITION4_X -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionX == 0 )
			PILOTPOSITION4_X -= 0.01f;
		else
			PILOTPOSITION4_X -= 0.1f;
		m_matPP4Matrix._41 = PILOTPOSITION4_X;
	}

	if (g_bRTSpinnerPosition4YUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION4_Y += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION4_Y += 0.01f;
		else
			PILOTPOSITION4_Y += 0.1f;
		m_matPP4Matrix._42 = PILOTPOSITION4_Y;
	}  
	if (g_bRTSpinnerPosition4YDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionY == 2 )
			PILOTPOSITION4_Y -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionY == 0 )
			PILOTPOSITION4_Y -= 0.01f;
		else
			PILOTPOSITION4_Y -= 0.1f;
		m_matPP4Matrix._42 = PILOTPOSITION4_Y;
	}

	if (g_bRTSpinnerPosition4ZUp) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION4_Z += 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION4_Z += 0.01f;
		else
			PILOTPOSITION4_Z += 0.1f;
		m_matPP4Matrix._43 = PILOTPOSITION4_Z;
	}  
	if (g_bRTSpinnerPosition4ZDown) {
		if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderPositionZ == 2 )
			PILOTPOSITION4_Z -= 1.0f;
		else if ( GetKeyState(VK_CONTROL) & 0x80 || g_iRTSliderPositionZ == 0 )
			PILOTPOSITION4_Z -= 0.01f;
		else
			PILOTPOSITION4_Z -= 0.1f;
		m_matPP4Matrix._43 = PILOTPOSITION4_Z;
	}

}



//-----------------------------------------------------------------------------
// Name: ResetCam()
// Desc: Reset Camera
//-----------------------------------------------------------------------------
void CMyD3DApplication::ResetCam()
{
	m_fCamRadsY = 0.0f;
	m_fCamRadsX = 0.0f;
	m_fCamRadsZ = 0.0f;
	g_fZoomValue = 0.0f;

	//PlaySound( NULL, NULL, SND_PURGE );
	SetVolume( DSBVOLUME_MIN );
	SetVolumeAllBuffers( DSBVOLUME_MIN );

	if (m_bRCView) {		
		switch(m_uPilotPosition)
		{
			case 1:
				m_fCamX = PILOTPOSITION1_X;
				m_fCamY = PILOTPOSITION1_Y;
				m_fCamZ = PILOTPOSITION1_Z;
				break;
			case 2:
				m_fCamX = PILOTPOSITION2_X;
				m_fCamY = PILOTPOSITION2_Y;
				m_fCamZ = PILOTPOSITION2_Z;
				break;
			case 3:
				m_fCamX = PILOTPOSITION3_X;
				m_fCamY = PILOTPOSITION3_Y;
				m_fCamZ = PILOTPOSITION3_Z;
				break;
			case 4:
				m_fCamX = PILOTPOSITION4_X;
				m_fCamY = PILOTPOSITION4_Y;
				m_fCamZ = PILOTPOSITION4_Z;
				break;
		}
		
	} else {
		m_fCamX = 0.0f;
		m_fCamY = 0.0f;
		m_fCamZ = 0.0f;
	}

}



//-----------------------------------------------------------------------------
// Name: ResetHeli()
// Desc: Reset Heli
// Note: Effect depends on where we call this function in FrameMove()
//-----------------------------------------------------------------------------
void CMyD3DApplication::ResetHeli()
{
	// reset values
	D3DUtil_SetIdentityMatrix( m_matFileObjectMatrix );

//	m_fX = INIT_X;
//	m_fY = INIT_Y;
//	m_fZ = INIT_Z;
//	m_fRadsX = 0.0;
//	m_fRadsY = INIT_RADS_Y;
//	m_fRadsZ = 0.0;

	m_fX = 0.0f;
	m_fY = 0.0f;
	m_fZ = 0.0f;
	m_fRadsX = 0.0f;
	m_fRadsY = 0.0f;
	m_fRadsZ = 0.0f;

	m_fSpeed = 0.0f;
	m_fCollective = INIT_COLLECTIVE;

	m_vRight   = D3DVECTOR( 1.0f, 0.0f, 0.0f );
	m_vUp      = D3DVECTOR( 0.0f, 1.0f, 0.0f );
	m_vForward = D3DVECTOR( 0.0f, 0.0f, 1.0f );


	// clean controls
	g_fX2 = 0.0f;
	g_fY2 = 0.0f;
	g_fZ2 = 0.0f;
	g_fRadsX2 = 0.0f;
	g_fRadsY2 = 0.0f;
	g_fRadsZ2 = 0.0f;

	// reset RTConfig Virtual Tx values
	g_lx = 19;
	g_ly = 19;
	g_ry = 19;
	g_ry = 19;

	
	// Note: FrameMove() will also do an extra framemove if we call this function
	// before the matrix/quaternion stuff, therefore this is enough to reset
	// translation
	//D3DXMATRIX matTrans;
	//D3DXMatrixTranslation( &matTrans, m_fX, m_fY, 0.0f );	
	//D3DMath_MatrixMultiply(m_matFileObjectMatrix, matTrans, m_matFileObjectMatrix);	// order is crucial!!!
	
	// rotation
	//D3DXMATRIX matRot;
	//D3DXQUATERNION qRot;
	//D3DXQuaternionRotationYawPitchRoll( &qRot, m_fRadsY, m_fRadsX, m_fRadsZ );	
	//D3DXMatrixRotationQuaternion( &matRot, &qRot );
	//D3DMath_MatrixMultiply(m_matFileObjectMatrix, matRot, m_matFileObjectMatrix); // order is crucial!!!

	// reset some more
	//m_fSpeed = 0.0f;
	//m_fCollective = INIT_COLLECTIVE;
	//m_fTorque = INIT_TORQUE;
	//m_fThrottle = 0.20f;
	//m_fValueKey = 0.0f;
	//g_fZoomValue = 0.0f;

	//if (g_bBell || g_bCobra)
	//	m_fThrottle = 1.00f;
	//else
	//	m_fThrottle = 0.20f;
	m_fThrottle = 7.35f;

	g_bFirstFrame = true;
	//g_vecVelocity = D3DXVECTOR3(0.0f, -0.1f, 0.0f); // we vallen in beeld: Cool!
	//g_vecVelocity = D3DXVECTOR3(0.0f, 0.15f, 0.0f); // heli ongeveer stil
	g_vecVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f); // zachte val in beeld
	g_vecAngularVelocity = D3DXVECTOR3(0.0f, 0.0f, 0.0f);

	// used in GetSticks()
	g_bResetValues = true;

	g_bLanded = true;

	// reset exhaust smoke array
	g_bResetExhaustSmoke = true;

	// ODE dynamics
	DestroyDynamics();
	InitDynamics();
	DestroyProxy();
	CreateProxy();

}



/*
// Draws progress bar
// New param: bool horz
void DrawBarEx(HWND hwnd, int percent, bool horz, COLORREF crColorBar, COLORREF crColorText)
{
//#define COLOUR_TEXT		RGB(255,255,255)
//#define COLOUR_BAR		RGB(0,0,255)

    RECT rect;
    GetClientRect(hwnd,&rect);
	int divider;

	// omit this and big bars might be drawn anywhere...
	if (hwnd == NULL)
		return;

	if (horz) {
		divider=MulDiv(percent,rect.right,100);
	} else {
		divider=MulDiv(percent,rect.bottom,100);
		divider=rect.bottom-divider;
	}
 
    RECT rectfull,rectempty;
	if (horz) {
		SetRect(&rectfull,0,0,divider,rect.bottom);
		SetRect(&rectempty,divider,0,rect.right,rect.bottom);
	} else {
		SetRect(&rectempty,0,0,rect.right,divider);
		SetRect(&rectfull,0,divider,rect.right,rect.bottom);
	}
 
    char buffer[20];
	int x;
	int y;

	if (horz) {
		//wsprintf(buffer,"%d %%",percent);
		wsprintf(buffer,"",percent);
		x=int(rect.right/2);
		y=int(rect.bottom*0.9);
	} else {
		//wsprintf(buffer,"%d %%",percent);
		wsprintf(buffer,"",percent);
		x=int(rect.right/2);
		y=int(rect.bottom/2);
	}
 
    HDC hdc=GetDC(hwnd);
 
    SetTextAlign(hdc,TA_CENTER | TA_BASELINE);
 
    // full rectangle
    SetTextColor(hdc,crColorText);
    SetBkColor  (hdc,crColorBar);
    ExtTextOut  (hdc,x,y,ETO_CLIPPED | ETO_OPAQUE,
            &rectfull, buffer,strlen(buffer),NULL);
 
    // empty rectangle
    SetTextColor(hdc,crColorBar);
    SetBkColor  (hdc,crColorText);
    ExtTextOut  (hdc,x,y,ETO_CLIPPED | ETO_OPAQUE,
            &rectempty,buffer,strlen(buffer),NULL);
 
    ReleaseDC(hwnd,hdc);
}
*/


//-----------------------------------------------------------------------------
// Name: DrawBarEx()
// Desc:
//-----------------------------------------------------------------------------
void DrawBarEx(HWND hwnd, int percent, bool horz, COLORREF crColorBar, COLORREF crColorText)
{
//#define COLOUR_TEXT		RGB(255,255,255)
//#define COLOUR_BAR		RGB(0,0,255)

	RECT rect;
    GetClientRect(hwnd,&rect);
	
	int divider;


	// TEST:
	//percent = g_percent;
	int pinch = 1;

	// limit
	if (percent < 0)
		percent = 0;

/*
	// TEST: check if file exists
	// No good, because if user has Classic Windows set as default
	// style, we'll have classic style even when manifest file exists
	// We must use the Theme API to check for the presence of visual styles
	// But that API is not in VC++ 5.0
	TCHAR strManifestFilePath[512] = "";
	strcpy(strManifestFilePath, g_szRCSIMProgramPath);
	strcat(strManifestFilePath, "\\bin\\");
	strcat(strManifestFilePath, "bigpush.exe.manifest");
	HANDLE hFile = CreateFile( strManifestFilePath, GENERIC_READ, FILE_SHARE_READ, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	
	if ( hFile == INVALID_HANDLE_VALUE ) {
		//MessageBox(hDlg, "Can't find file. Please check file name and path.",
		//	"R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
		//MessageBeep(MB_ICONEXCLAMATION);
		// Classic-style GUI
		pinch = 1;
	} else {
		// XP-style GUI
		pinch = 2;
	}
*/
   
	// TEST
	char buffer2[20];
	sprintf(buffer2, "%i", rect.right);
	//MessageBox(NULL, buffer2, "QQQ", MB_OK|MB_ICONERROR);
	//_RPT1(_CRT_WARN, "Client rect width: %s\n", buffer2);
	_RPT1(_CRT_WARN, "Client rect width: %i\n", rect.right);


	if (horz) {
		if (rect.bottom == 13) pinch = 1; // Classic style	
		if (rect.bottom == 15) pinch = 2; // XP style
	} else {
		if (rect.right == 12) pinch = 1; // Classic style	
		if (rect.right == 14) pinch = 2; // XP style
	}




	// omit this and big bars might be drawn anywhere...
	if (hwnd == NULL)
		return;

	if (horz) {
		divider=MulDiv(percent,(rect.right-2*pinch),100);
		divider+=pinch;
	} else {
		divider=MulDiv(percent,(rect.bottom-2*pinch),100);
		divider=(rect.bottom)-divider;
		divider-=pinch;
	}
	

	
    RECT rectfull,rectempty;
	if (horz) {
		SetRect(&rectfull,0,0,divider,rect.bottom);
		SetRect(&rectempty,divider,0,rect.right,rect.bottom);
	} else {
		SetRect(&rectempty,0,0,rect.right,divider);
		SetRect(&rectfull,0,divider,rect.right,rect.bottom);
	}


	// TEST:
	if (horz) {
		rectempty.left+=0;
		rectempty.right-=pinch;
		rectempty.top+=pinch;
		rectempty.bottom-=pinch;

		rectfull.left+=pinch;
		rectfull.right-=0;
		rectfull.top+=pinch;
		rectfull.bottom-=pinch;
	} else {
		rectempty.left+=pinch;
		rectempty.right-=pinch;
		rectempty.top+=pinch;
		rectempty.bottom-=0;

		rectfull.left+=pinch;
		rectfull.right-=pinch;
		rectfull.top+=0;
		rectfull.bottom-=pinch;
	}


    char buffer[20];
	int x;
	int y;

	if (horz) {
		//wsprintf(buffer,"%d %%",percent); // text
		wsprintf(buffer,"",percent);		// no text
		x=int(rect.right/2);
		y=int(rect.bottom*0.9);
	} else {
		//wsprintf(buffer,"%d %%",percent); // text
		wsprintf(buffer,"",percent);		// no text
		x=int(rect.right/2);
		y=int(rect.bottom/2);
	}
 
    HDC hdc=GetDC(hwnd);
 
    SetTextAlign(hdc,TA_CENTER | TA_BASELINE);
 
    // full rectangle
    SetTextColor(hdc,crColorText);
    SetBkColor  (hdc,crColorBar);
    ExtTextOut  (hdc,x,y,ETO_CLIPPED | ETO_OPAQUE,
            &rectfull, buffer,strlen(buffer),NULL);
 
    // empty rectangle
    SetTextColor(hdc,crColorBar);
    SetBkColor  (hdc,crColorText);
    ExtTextOut  (hdc,x,y,ETO_CLIPPED | ETO_OPAQUE,
            &rectempty,buffer,strlen(buffer),NULL);
 
    ReleaseDC(hwnd,hdc);
}





//-----------------------------------------------------------------------------
// Name: GetSliders()
// Desc: Gets RT slider values
//-----------------------------------------------------------------------------
void CMyD3DApplication::GetSliders()
{
	// Poll the VxD values
	//DeviceIoControl(g_hVxD, VMYXD_APIFUNC_1,
	//				(LPVOID)NULL, 0,
	//				(LPVOID)RetInfo, sizeof(RetInfo),
	//				&cbBytesReturned, NULL);
						
	// RetInfo[0]  == SyncPulse
	// RetInfo[1]  == Ch1
	// RetInfo[2]  == Ch2
	// RetInfo[3]  == Ch3
	// RetInfo[4]  == Ch4
	// RetInfo[5]  == Ch5
	// RetInfo[6]  == Ch6
	// RetInfo[7]  == Ch7			
	// RetInfo[8]  == Ch8
	// RetInfo[9]  == Ch9
	// RetInfo[10] == Ch10
	// RetInfo[11] == IntCount
	// RetInfo[12] == PulseLength
	// RetInfo[13] == ChannelCount

	/*
		// rollen met die hap
		if ( dijs.lX<0 ) Roll(dijs.lX);
		if ( dijs.lX>0 ) Roll(dijs.lX);
		if ( dijs.lY<0 ) Pitch(dijs.lY);
		if ( dijs.lY>0 ) Pitch(dijs.lY);
		//if ( dijs.lZ<0 ) Yaw(dijs.lZ);
		//if ( dijs.lZ>0 ) Yaw(dijs.lZ);
		if ( dijs.lZ<0 ) ZUp(dijs.lZ);
		if ( dijs.lZ>0 ) ZUp(dijs.lZ);
	*/

	//	Roll(RetInfo[2]);
	//	Pitch(RetInfo[3]);
	//	Yaw(RetInfo[4]);
	//	ZUp(RetInfo[1]);


	// maak 1000-2000 uS
	//for (int i=0; i<=10; i++) {
	//	if (RetInfo[i] != 0) RetInfo[i]-=250; // leave unused channels alone
	//	// RetInfo[i] must be signed here!!!!!!!!!!!!!!!!!!
	//	// We've changed it from DWORD to int so it's OK now	
	//	if (RetInfo[i] < 0)  RetInfo[i] =  0; // never have negative values	
	//}
		


	// TODO: make proportional controls
	// DONE
	// TODO: mogelijkheid felheid in te stellen
	// DONE
	// TODO: mogelijkheid invert channel
	// DONE

	// channel sensitivity
	// float Ch1Sens  = 0.75f;
	// float Ch2Sens  = 0.200f;
	// float Ch3Sens  = 0.200f;
	// float Ch4Sens  = 0.35f;
	// float Ch5Sens  = 0.200f;
	// float Ch6Sens  = 0.200f;
	// float Ch7Sens  = 0.75f;
	// float Ch8Sens  = 0.200f;
	// float Ch9Sens  = 0.200f;
	// float Ch10Sens = 0.200f;

	// channel step (don't change)
	// float Ch1Step  = Ch1Sens/760.0f;
	// float Ch2Step  = Ch2Sens/760.0f;
	// float Ch3Step  = Ch3Sens/760.0f;
	// float Ch4Step  = Ch4Sens/760.0f;
	// float Ch5Step  = Ch5Sens/760.0f;
	// float Ch6Step  = Ch6Sens/760.0f;
	// float Ch7Step  = Ch7Sens/760.0f;
	// float Ch8Step  = Ch8Sens/760.0f;
	// float Ch9Step  = Ch9Sens/760.0f;
	// float Ch10Step = Ch10Sens/760.0f;

	// channel invert
	// bool Ch1Inv  = true;
	// bool Ch2Inv  = false;
	// bool Ch3Inv  = false;
	// bool Ch4Inv  = false;
	// bool Ch5Inv  = false;
	// bool Ch6Inv  = false;
	// bool Ch7Inv  = false;
	// bool Ch8Inv  = false;
	// bool Ch9Inv  = false;
	// bool Ch10Inv = false;

	// what's in a name: the stick positions in "radians"
	//float rads;

	// Note: in heli mc-20 program zit collective op kanaal 6
	// TODO: don't hard-code this
	//int i;



	// 1000 to 2000, 1000 steps
	// -0.25 to 0.25, 0.50 steps
	// RetInfo[1]-1500 to get -500 to 500
	// 0.50f/1000 to get the steps (increasing 0.50f will give bigger steps and
	// thus higher sensitivity)
	// So: m_fY	= (RetInfo[1]-1500)*(0.50f/1000)*CTRL_Y_SENS*m_fSpeedFactor;
    if (g_bGrab1) m_fY     =  (g_iRTSliderCh1-1500)*(1.00f/1000)*CTRL_Y_SENS;      // Channel 1   
    if (g_bGrab2) m_fRadsZ = -(g_iRTSliderCh2-1500)*(0.40f/1000)*CTRL_RADS_Z_SENS; // Channel 2
    if (g_bGrab3) m_fRadsX = -(g_iRTSliderCh3-1500)*(0.40f/1000)*CTRL_RADS_X_SENS; // Channel 3
    if (g_bGrab4) m_fRadsY = -(g_iRTSliderCh4-1500)*(0.40f/1000)*CTRL_RADS_Y_SENS; // Channel 4

    if (g_bGrab7) m_fZ     = -(g_iRTSliderCh7-1500)*(1.00f/1000)*CTRL_Z_SENS;      // Channel 7


}




//-----------------------------------------------------------------------------
// Name: GetVirtualSticks()
// Desc: Gets RT stick values
//-----------------------------------------------------------------------------
void CMyD3DApplication::GetVirtualSticks()
{
	// Poll the VxD values
	//DeviceIoControl(g_hVxD, VMYXD_APIFUNC_1,
	//				(LPVOID)NULL, 0,
	//				(LPVOID)RetInfo, sizeof(RetInfo),
	//				&cbBytesReturned, NULL);
						
	// RetInfo[0]  == SyncPulse
	// RetInfo[1]  == Ch1
	// RetInfo[2]  == Ch2
	// RetInfo[3]  == Ch3
	// RetInfo[4]  == Ch4
	// RetInfo[5]  == Ch5
	// RetInfo[6]  == Ch6
	// RetInfo[7]  == Ch7			
	// RetInfo[8]  == Ch8
	// RetInfo[9]  == Ch9
	// RetInfo[10] == Ch10
	// RetInfo[11] == IntCount
	// RetInfo[12] == PulseLength
	// RetInfo[13] == ChannelCount

	/*
		// rollen met die hap
		if ( dijs.lX<0 ) Roll(dijs.lX);
		if ( dijs.lX>0 ) Roll(dijs.lX);
		if ( dijs.lY<0 ) Pitch(dijs.lY);
		if ( dijs.lY>0 ) Pitch(dijs.lY);
		//if ( dijs.lZ<0 ) Yaw(dijs.lZ);
		//if ( dijs.lZ>0 ) Yaw(dijs.lZ);
		if ( dijs.lZ<0 ) ZUp(dijs.lZ);
		if ( dijs.lZ>0 ) ZUp(dijs.lZ);
	*/

	//	Roll(RetInfo[2]);
	//	Pitch(RetInfo[3]);
	//	Yaw(RetInfo[4]);
	//	ZUp(RetInfo[1]);


	// maak 1000-2000 uS
	//for (int i=0; i<=10; i++) {
	//	if (RetInfo[i] != 0) RetInfo[i]-=250; // leave unused channels alone
	//	// RetInfo[i] must be signed here!!!!!!!!!!!!!!!!!!
	//	// We've changed it from DWORD to int so it's OK now	
	//	if (RetInfo[i] < 0)  RetInfo[i] =  0; // never have negative values	
	//}
		


	// TODO: make proportional controls
	// DONE
	// TODO: mogelijkheid felheid in te stellen
	// DONE
	// TODO: mogelijkheid invert channel
	// DONE

	// channel sensitivity
	//float Ch1Sens  = 0.75f;
	//float Ch2Sens  = 0.200f;
	//float Ch3Sens  = 0.200f;
	//float Ch4Sens  = 0.35f;
	//float Ch5Sens  = 0.200f;
	//float Ch6Sens  = 0.200f;
	//float Ch7Sens  = 0.75f;
	//float Ch8Sens  = 0.200f;
	//float Ch9Sens  = 0.200f;
	//float Ch10Sens = 0.200f;

	// channel step (don't change)
	//float Ch1Step  = Ch1Sens/760.0f;
	//float Ch2Step  = Ch2Sens/760.0f;
	//float Ch3Step  = Ch3Sens/760.0f;
	//float Ch4Step  = Ch4Sens/760.0f;
	//float Ch5Step  = Ch5Sens/760.0f;
	//float Ch6Step  = Ch6Sens/760.0f;
	//float Ch7Step  = Ch7Sens/760.0f;
	//float Ch8Step  = Ch8Sens/760.0f;
	//float Ch9Step  = Ch9Sens/760.0f;
	//float Ch10Step = Ch10Sens/760.0f;

	// channel invert
	//bool Ch1Inv  = true;
	//bool Ch2Inv  = true;
	//bool Ch3Inv  = true;
	//bool Ch4Inv  = true;
	//bool Ch5Inv  = false;
	//bool Ch6Inv  = false;
	//bool Ch7Inv  = false;
	//bool Ch8Inv  = false;
	//bool Ch9Inv  = false;
	//bool Ch10Inv = false;

	// what's in a name: the stick positions in "radians"
	//float rads;

	// Note: in heli mc-20 program zit collective op kanaal 6
	// TODO: don't hard-code this

	// increment value
	//int i;




	// mode1		mode2					
	// g_ly: nick   g_ly: pitch
	// g_lx: yaw    g_lx: yaw
	// g_ry: pitch  g_ry: nick
	// g_rx: roll   g_rx: roll

	// mode3        mode4	
	// g_ly: nick   g_ly: pitch
	// g_lx: roll   g_lx: roll
	// g_ry: pitch  g_ry: nick
	// g_rx: yaw    g_rx: yaw


	// Channel 1: pitch
	// Channel 2: roll
	// Channel 3: nick
	// Channel 4: yaw

	
	// m_fY		=  (RetInfo[1]-1500)*(1.00f/1000)*CTRL_Y_SENS;		// Channel 1	
	// m_fRadsZ = -(RetInfo[2]-1500)*(0.40f/1000)*CTRL_RADS_Z_SENS; // Channel 2
	// m_fRadsX = -(RetInfo[3]-1500)*(0.40f/1000)*CTRL_RADS_X_SENS; // Channel 3
	// m_fRadsY = -(RetInfo[4]-1500)*(0.40f/1000)*CTRL_RADS_Y_SENS; // Channel 4
	// 
	// m_fZ		= -(RetInfo[7]-1500)*(1.00f/1000)*CTRL_Z_SENS;		// Channel 7



	// keyboard
	// m_fY = +0.25f*CTRL_Y_SENS;
	// m_fRadsY = -(0.05f)*CTRL_RADS_Y_SENS;

	// Bij calibreren: hoogste en laagste RetInfo[1] detecteren.
	// Vervolgens: m_fY = ( RetInfo[1]-(laagste+((hoogste-laagste)/2)) ) *
	// ( sensitivity/(hoogste-laagste) ) * CTRL_Y_SENS; 

	
	// -13 to 51, 64 steps
	// -0.25 to 0.25, 0.50 steps
	// g_ly-19 to get -32 to 32
	// 0.50f/64 to get the steps (increasing 0.50f will give bigger steps and
	// thus higher sensitivity)
	// So: m_fY = (g_ly-19)*(0.50f/64)*CTRL_Y_SENS;
	switch (g_iRTVirtualTxMode)
	{
		case 1:
			if (g_bGrabL) {
				// nick
				m_fRadsX = (g_ly-19)*(0.40f/64)*CTRL_RADS_X_SENS;
				// yaw
				m_fRadsY = (g_lx-19)*(0.40f/64)*CTRL_RADS_Y_SENS;
			}			
			if (g_bGrabR) {
				// pitch
				if (g_bRTPitchPull) {
					if (g_bRTPitchHold) {
						m_fCollective = (g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = (g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				} else {
					if (g_bRTPitchHold) {
						m_fCollective = -(g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = -(g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				}
				// roll
				m_fRadsZ = (g_rx-19)*(0.40f/64)*CTRL_RADS_Z_SENS;
			}
			break;

		case 2:
			if (g_bGrabL) {
				// pitch
				if (g_bRTPitchPull) {
					if (g_bRTPitchHold) {
						m_fCollective = (g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = (g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				} else {
					if (g_bRTPitchHold) {
						m_fCollective = -(g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = -(g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				}
				// yaw
				m_fRadsY = (g_lx-19)*(0.40f/64)*CTRL_RADS_Y_SENS;
			}
			if (g_bGrabR) {
				// nick
				m_fRadsX = (g_ry-19)*(0.40f/64)*CTRL_RADS_X_SENS;
				// roll
				m_fRadsZ = (g_rx-19)*(0.40f/64)*CTRL_RADS_Z_SENS;
			}
			break;

		case 3:
			if (g_bGrabL) {
				// nick
				m_fRadsX = (g_ly-19)*(0.40f/64)*CTRL_RADS_X_SENS;
				// roll
				m_fRadsZ = (g_lx-19)*(0.40f/64)*CTRL_RADS_Z_SENS;
			}
			if (g_bGrabR) {
				// pitch
				if (g_bRTPitchPull) {
					if (g_bRTPitchHold) {
						m_fCollective = (g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = (g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				} else {
					if (g_bRTPitchHold) {
						m_fCollective = -(g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = -(g_ry-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				}
				// yaw
				m_fRadsY = (g_rx-19)*(0.40f/64)*CTRL_RADS_Y_SENS;
			}
			break;
		
		case 4:			
			if (g_bGrabL) {
				// pitch
				if (g_bRTPitchPull) {
					if (g_bRTPitchHold) {
						m_fCollective = (g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = (g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				} else {
					if (g_bRTPitchHold) {
						m_fCollective = -(g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					} else {
						m_fY = -(g_ly-19)*(1.00f/64)*CTRL_Y_SENS;
					}
				}
				// roll
				m_fRadsZ = (g_lx-19)*(0.40f/64)*CTRL_RADS_Z_SENS;
			}
			if (g_bGrabR) {
				// nick
				m_fRadsX = (g_ry-19)*(0.40f/64)*CTRL_RADS_X_SENS;
				// yaw
				m_fRadsY = (g_rx-19)*(0.40f/64)*CTRL_RADS_Y_SENS;
			}
			break;
	}


}




//-----------------------------------------------------------------------------
// Name: GetVirtualTrims()
// Desc: Gets RT trim values. Must call this after virtual tx stick values are
//	     updated, otherwise the sticks will slide with the trims.
//-----------------------------------------------------------------------------
void CMyD3DApplication::GetVirtualTrims()
{
	// trims //////////////////////////////////////////////////////////////////
	// -20 to 20, 40 steps (actually 40+1...)
	switch (g_iRTVirtualTxMode)
	{
		case 1:
			// left
			// nick
			m_fRadsX += g_lyTrim*0.001f*CTRL_RADS_X_SENS;
			// yaw
			m_fRadsY += g_lxTrim*0.001f*CTRL_RADS_Y_SENS;
					
			// right
			// pitch
			if (g_bRTPitchPull) {
				m_fY += g_ryTrim*0.001f*CTRL_Y_SENS;
			} else {
				m_fY -= g_ryTrim*0.001f*CTRL_Y_SENS;
			}
			// roll
			m_fRadsZ += g_rxTrim*0.001f*CTRL_RADS_Z_SENS;			
			break;

		case 2:
			// left
			// pitch
			if (g_bRTPitchPull) {
				m_fY += g_lyTrim*0.001f*CTRL_Y_SENS;
			} else {
				m_fY -= g_lyTrim*0.001f*CTRL_Y_SENS;
			}
			// yaw
			m_fRadsY += g_lxTrim*0.001f*CTRL_RADS_Y_SENS;
			
			// right
			// nick
			m_fRadsX += g_ryTrim*0.001f*CTRL_RADS_X_SENS;
			// roll
			m_fRadsZ += g_rxTrim*0.001f*CTRL_RADS_Z_SENS;
			break;

		case 3:
			// left
			// nick
			m_fRadsX += g_lyTrim*0.001f*CTRL_RADS_X_SENS;
			// roll
			m_fRadsZ += g_lxTrim*0.001f*CTRL_RADS_Z_SENS;
			
			// right
			// pitch
			if (g_bRTPitchPull) {
				m_fY += g_ryTrim*0.001f*CTRL_Y_SENS;
			} else {
				m_fY -= g_ryTrim*0.001f*CTRL_Y_SENS;
			}
			// yaw
			m_fRadsY += g_rxTrim*0.001f*CTRL_RADS_Y_SENS;
			break;
		
		case 4:			
			// left
			// pitch
			if (g_bRTPitchPull) {
				m_fY += g_lyTrim*0.001f*CTRL_Y_SENS;
			} else {
				m_fY -= g_lyTrim*0.001f*CTRL_Y_SENS;
			}
			// roll
			m_fRadsZ += g_lxTrim*0.001f*CTRL_RADS_Z_SENS;
			
			// right
			// nick
			m_fRadsX += g_ryTrim*0.001f*CTRL_RADS_X_SENS;
			// yaw
			m_fRadsY += g_rxTrim*0.001f*CTRL_RADS_Y_SENS;
			break;
	}


}




//-----------------------------------------------------------------------------
// Name: UpdateRTChannels()
// Desc:
// Note: We are using the clean (no speed-factor, no turbulence) saved control values:
//		 g_fX2, g_fY2, g_fZ2, g_fRadsY2, g_fRadsX2, g_fRads2 
//-----------------------------------------------------------------------------
void CMyD3DApplication::UpdateRTChannels()
{
	// m_fY = +(lJoyAxis*0.001f)*CTRL_Y_SENS*m_fSpeedFactor;
	// m_fRadsY = -(0.20f)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	// m_fRadsX = -(0.05f)*CTRL_RADS_X_SENS*m_fSpeedFactor;		
	if (g_bTrackbar) {
		// set trackbars to m_fY etc.
		// No, set trackbars to g_fY2 etc.
		// 1000-2000
		pos1 = int(g_fY2*500) + int(m_fCollective*500) + 1500;
		pos2 = int(g_fRadsZ2*2500) + 1500;
		pos3 = int(g_fRadsX2*2500) + 1500;
		pos4 = int(g_fRadsY2*625) + 1500;
		pos5 = 1500;
		pos6 = 1500;
		pos7 = int(g_fZ2*500) + 1500;
		pos8 = 1500;
		pos9 = 1500;
		pos10 = 1500;


		// pull pitch
		// invert: 1000->2000 and 2000->1000
		pos1 = (-(pos1-1000))+2000;

		// invert: 1000->2000 and 2000->1000
		if (!g_bVertical) {
			pos1 = (-(pos1-1000))+2000;
			pos2 = (-(pos2-1000))+2000;
			pos3 = (-(pos3-1000))+2000;
			pos4 = (-(pos4-1000))+2000;
			pos5 = (-(pos5-1000))+2000;
			pos6 = (-(pos6-1000))+2000;
			pos7 = (-(pos7-1000))+2000;
			pos8 = (-(pos8-1000))+2000;
			pos9 = (-(pos9-1000))+2000;
			pos10 = (-(pos10-1000))+2000;			
		}

		
		// mute
		if (!g_bVertical) {
			if (g_bChMute[1])  pos1 = 0;
			if (g_bChMute[2])  pos2 = 0;
			if (g_bChMute[3])  pos3 = 0;
			if (g_bChMute[4])  pos4 = 0;
			if (g_bChMute[5])  pos5 = 0;
			if (g_bChMute[6])  pos6 = 0;
			if (g_bChMute[7])  pos7 = 0;
			if (g_bChMute[8])  pos8 = 0;
			if (g_bChMute[9])  pos9 = 0;
			if (g_bChMute[10]) pos10 = 0;
		} else {
			if (g_bChMute[1])  pos1 = 2000;
			if (g_bChMute[2])  pos2 = 2000;
			if (g_bChMute[3])  pos3 = 2000;
			if (g_bChMute[4])  pos4 = 2000;
			if (g_bChMute[5])  pos5 = 2000;
			if (g_bChMute[6])  pos6 = 2000;
			if (g_bChMute[7])  pos7 = 2000;
			if (g_bChMute[8])  pos8 = 2000;
			if (g_bChMute[9])  pos9 = 2000;
			if (g_bChMute[10]) pos10 = 2000;
		}
		


		if (!g_bGrab1)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER1), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos1 ); 
		if (!g_bGrab2)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER2), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos2 ); 
		if (!g_bGrab3)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER3), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos3 ); 
		if (!g_bGrab4)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER4), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos4 ); 
		if (!g_bGrab5)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER5), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos5 ); 
		if (!g_bGrab6)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER6), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos6 ); 
		if (!g_bGrab7)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER7), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos7 );
		if (!g_bGrab8)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER8), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos8 ); 
		if (!g_bGrab9)  SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER9), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos9 ); 
		if (!g_bGrab10) SendMessage( GetDlgItem(g_hTabCurrent, IDC_SLIDER10), TBM_SETPOS, (WPARAM)TRUE, (LPARAM)pos10 );

	} else {
		// set progressbars to m_fY etc.
		// we are setting in WM_TIMER
		// 0-100
		pos1 = int(g_fY2*50) + int(m_fCollective*50) + 50;
		pos2 = int(g_fRadsZ2*250) + 50;
		pos3 = int(g_fRadsX2*250) + 50;
		pos4 = int(g_fRadsY2*62.5f) + 50;
		pos5 = 50;
		pos6 = 50;
		pos7 = int(g_fZ2*50) + 50;
		pos8 = 50;
		pos9 = 50;
		pos10 = 50;

		// pull pitch
		// invert: 0->100 and 100->0
		pos1 = (-(pos1))+100;

		// invert: 0->100 and 100->0
		pos1 = (-(pos1))+100;
		pos2 = (-(pos2))+100;
		pos3 = (-(pos3))+100;
		pos4 = (-(pos4))+100;
		pos5 = (-(pos5))+100;
		pos6 = (-(pos6))+100;
		pos7 = (-(pos7))+100;
		pos8 = (-(pos8))+100;
		pos9 = (-(pos9))+100;
		pos10 = (-(pos10))+100;

		// mute
		if (g_bChMute[1])  pos1 = 0;
		if (g_bChMute[2])  pos2 = 0;
		if (g_bChMute[3])  pos3 = 0;
		if (g_bChMute[4])  pos4 = 0;
		if (g_bChMute[5])  pos5 = 0;
		if (g_bChMute[6])  pos6 = 0;
		if (g_bChMute[7])  pos7 = 0;
		if (g_bChMute[8])  pos8 = 0;
		if (g_bChMute[9])  pos9 = 0;
		if (g_bChMute[10]) pos10 = 0;
		
	}			

}




//-----------------------------------------------------------------------------
// Name: UpdateRTVirtualTx()
// Desc: 
// Note: We are using the clean (no speed-factor, no turbulence) saved control values:
//		 g_fX2, g_fY2, g_fZ2, g_fRadsY2, g_fRadsX2, g_fRads2
//-----------------------------------------------------------------------------
void CMyD3DApplication::UpdateRTVirtualTx()
{
	
	// 65x65 and 27x27
	// total left == 0-13 == -13
	// total right == 65-13 == 52
	// center == (65+27)/2 == 19
	// -13 to 52
	// m_fY = +0.25f*CTRL_Y_SENS;
	// m_fRadsY = -(0.05f)*CTRL_RADS_Y_SENS;
	// 0.25	== 1/4
	// Shift+0.25 == 1/2
	// 0.25*2.0 == 3/4
	// Shift+0.25*2.0 == 4/4	
	switch (g_iRTVirtualTxMode)
	{
		case 1:
			if (!g_bGrabL) {
				// yaw
				g_lx = int(g_fRadsY2*40) + 19;
				// nick
				g_ly = int(g_fRadsX2*160) + 19;
			}
			if (!g_bGrabR) {
				// roll
				g_rx = int(g_fRadsZ2*160) + 19;
				// pitch
				if (g_bRTPitchHold) {
					// do nothing
				} else {						
					if (g_bRTPitchPull) {
						g_ry = int(g_fY2*32) + 19;
					} else {
						g_ry = int(-g_fY2*32) + 19;
					}
				}					
			}
			break;

		case 2:
			if (!g_bGrabL) {
				// yaw
				g_lx = int(g_fRadsY2*40) + 19;
				// TODO: pitch hold
				// pitch hold
				// pitch stick centers with g_fY2	
				// pitch stick holds with g_fCollective_2
				// Als we voor g_ly geen update doen werkt pitch hold OK
				// Nadeel is dat we dan geen weergave krijgen wanneer user keyboard, muis,
				// of joystick gebruikt				
				//_RPT1(_CRT_WARN, "Test: %f\n", g_fY2);
				// pitch
				if (g_bRTPitchHold) {
					// do nothing
				} else {
					if (g_bRTPitchPull) {
						g_ly = int(g_fY2*32) + int(m_fCollective*32) + 19;
					} else {
						g_ly = int(-g_fY2*32) + int(-m_fCollective*32) + 19;
					}
				}
			}
			if (!g_bGrabR) {
				// roll
				g_rx = int(g_fRadsZ2*160) + 19;
				// nick
				g_ry = int(g_fRadsX2*160) + 19;
			}
			break;

		case 3:
			if (!g_bGrabL) {
				// roll
				g_lx = int(g_fRadsZ2*160) + 19;
				// nick
				g_ly = int(g_fRadsX2*160) + 19;
			}
			if (!g_bGrabR) {
				// yaw
				g_rx = int(g_fRadsY2*40) + 19;
				// pitch
				if (g_bRTPitchHold) {
					// do nothing
				} else {
					if (g_bRTPitchPull) {
						g_ry = int(g_fY2*32) + 19;
					} else {
						g_ry = int(-g_fY2*32) + 19;
					}

				}
			}
			break;

		case 4:
			if (!g_bGrabL) {
				// roll
				g_lx = int(g_fRadsZ2*160) + 19;
				// pitch
				if (g_bRTPitchHold) {
					// do nothing
				} else {						
					if (g_bRTPitchPull) {
						g_ly = int(g_fY2*32) + 19;
					} else {
						g_ly = int(-g_fY2*32) + 19;
					}
				}
			}
			if (!g_bGrabR) {
				// yaw
				g_rx = int(g_fRadsY2*40) + 19;
				// nick
				g_ry = int(g_fRadsX2*160) + 19;
			}
			break;
	}	
	//////////////////////////////////////////////////////////////////////////////



	// RT trims
	// Note: must do this after we have updated virtual tx sticks, otherwise sticks
	// will slide with trims
	if ( TabCtrl_GetCurSel(g_hTabControl) == 4 )
		GetVirtualTrims();

}



//-----------------------------------------------------------------------------
// Name: UpdateStatusBar()
// Desc: We are calling this from Render() to make sure status bar info gets
//		 updated when app is paused.
//-----------------------------------------------------------------------------
void CMyD3DApplication::UpdateStatusBar()
{

	// Status bar info ////////////////////////////////////////////////////////
	// NOTE: should only do this in windowed mode otherwise the windows
	// will be visible in fullscreen mode
	// TODO: this causes a memory leak. Fix it!!! It seems that sending SB_SETTEXT
	// in a gameloop causes a memory leak???
	// Reason: in the status bar's subclass proc we were handling WM_PAINT and
	// there we were doing CreateFont() without doing DeleteObject().
	// That's a lot of fonts....
	if (m_pDeviceInfo->bWindowed && g_hwndStatus) {

		// Playback/Record info
		// NOTE: this triggers WM_DRAWITEM
		// TODO: het is beter dit niet in de gameloop te doen want we krijgen flicker
		if (m_bPlayBack || m_bRecord) {
			SendMessage(g_hwndStatus, SB_SETTEXT, 2|SBT_OWNERDRAW, (LPARAM)0 );
		} else {
			//SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 1|SBT_NOBORDERS, (LPARAM)"" );
			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 2|0, (LPARAM)"" );
		}

		// FPS
		char stat1[20];
		sprintf(stat1, "FPS %i", (int)(g_fFPS+0.5f) ); // +0.5f is used for rounding, See C FAQ 14.6
		SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 5|0, (LPARAM)stat1 );

		// test
		// YES. Gotcha, you bugger. Sending SB_SETTEXT in a loop causes memory leak.
		// Reason: in the status bar's subclass proc we were handling WM_PAINT and
		// there we were doing CreateFont() without doing DeleteObject().
		// That's a lot of fonts....
		//for ( int i=0; i<1000; i++ )
		//	SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 3|0, (LPARAM)stat1 );


		// FrameCount/FrameTotal
		if (m_bPlayBack && g_iFrameTotal != 0) {
			char stat2[20];
			char stat3[20];
			sprintf(stat2, "Frame %i", g_iFrameCount );
			sprintf(stat3, "%i", g_iFrameTotal );
			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 3|0, (LPARAM)stat2 );
			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 4|0, (LPARAM)stat3 );
		}
		
		if (!m_bPlayBack) {
			// TODO: why is the f*cker not doing SBT_NOBORDERS, whereas SBT_POPOUT works fine
			//SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 3|SBT_NOBORDERS, (LPARAM)"" );
			//SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 4|SBT_NOBORDERS, (LPARAM)"" );

			//SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 3|SBT_POPOUT, (LPARAM)"" );
			//SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 4|SBT_POPOUT, (LPARAM)"" );

			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 3|0, (LPARAM)"" );
			SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 4|0, (LPARAM)"" );

			//UpdateWindow(g_hwndStatus);
			//UpdateWindow(m_hWnd);
			//SendMessage(g_hwndStatus, WM_ERASEBKGND, (WPARAM)GetDC(g_hwndStatus), (LPARAM)0 );
			//SendMessage(g_hwndStatus, WM_PAINT, (WPARAM)GetDC(g_hwndStatus), (LPARAM)0 );
		}


		// Progress bar
		// NOTE: beware of Divide by Zero
		if (m_bPlayBack && g_iFrameTotal != 0) {
			ShowWindow( g_hwndPB, SW_SHOW );
			if (g_bSmoothProgressBar) {
				//DrawBarEx( g_hwndPB, (g_iFrameCount*100)/g_iFrameTotal, true, COLOR_DARKBLUE, GetSysColor(COLOR_BTNFACE) );
				//DrawBarEx( g_hwndPB, MulDiv(g_iFrameCount,100,g_iFrameTotal), true, COLOR_GRAY/*GetSysColor(COLOR_SCROLLBAR)*/, COLOR_WHITE/*GetSysColor(COLOR_BTNFACE)*/ );
				//DrawBarEx( g_hwndPB, (g_iFrameCount*100)/g_iFrameTotal, true, COLOR_DARKGRAY, COLOR_WHITE );
				DrawBarEx( g_hwndPB, (g_iFrameCount*100)/g_iFrameTotal, true, g_crProgressBarColor, COLOR_WHITE );
			} else {			
				SendMessage( g_hwndPB, PBM_SETRANGE, (WPARAM)0, MAKELPARAM(1, g_iFrameTotal) );  
				SendMessage( g_hwndPB, PBM_SETPOS, (WPARAM)g_iFrameCount, (LPARAM)0 );
			}

			// Mooie grijze randjes om progress bar //////////
			// Is niet zo mooi
			//HDC hDC = GetDC(g_hwndPB);
			//HPEN hPen;
			//HPEN hPenOld;
			//hPen = CreatePen(PS_SOLID,1,GetSysColor(COLOR_MENU));
			//hPenOld = (HPEN)SelectObject(hDC, hPen);

			//MoveToEx(hDC,0,0,NULL);
			//LineTo	(hDC,100,0);
			//MoveToEx(hDC,0,13,NULL);
			//LineTo	(hDC,100,13);

			// Old Pen back
			//SelectObject(hDC, hPenOld);
			//DeleteObject(hPen);
			//ReleaseDC(g_hwndPB,hDC);
			//////////////////////////////////////////////////

		} else {
			ShowWindow( g_hwndPB, SW_HIDE );
		}
	}
	///////////////////////////////////////////////////////////////////////////

}






//-----------------------------------------------------------------------------
// Name: ResetFilePointer()
// Desc: Resets Flight Rec file pointer to [Frame g_iFrameCount]
// Note: Should probably use a mutex because this function seems non-reentrant.
// Note: from the Docs: If one thread is suspended by the Win32 scheduler while 
//		 executing the printf function, one of the program's other threads might start
//		 executing. If the second thread also calls printf, data might be corrupted. 
//		 To avoid this situation, access to static data used by the function must be 
//		 restricted to one thread at a time. 
//		 You do not need to serialize access to stack-based (automatic) variables 
//		 because each thread has a different stack. Therefore, a function that uses 
//		 only automatic (stack) variables is reentrant.
// Note: We are using only automatic vars here and still our chopper flies into space
//		 at times when play and pause are down and doing ff or rew or slider moves.
//		 We need a mutex on hFile2. Or easier: make hFile2 a Critical Section Object.
//		 Or easier still: use a bCanEnter variable.
// Note: Well...even that doesn't help. Maybe there is a bug in the file parse code.
// Note: from the Docs: You should be careful when setting the file pointer in a
//		 multithreaded application. For example, an application whose threads share
//		 a file handle, update the file pointer, and read from the file must protect
//		 this sequence by using a critical section object or mutex object. For more
//		 information about these objects, see Mutex Objects and Critical Section Objects.
// TODO: check FlightPlayBack()
//		 Waarschijnlijk is er dit aan de hand: FlightPlayBack() en ResetFilePointer()
//		 doen allebei ReadFile() gevolgd door SetFilePointer() operaties. Als ze
//		 dat doen op dezelfde file krijg je conflicten, e.g filepointers die "way off"
//		 zijn. Oplossing: toch een Critical Section Object of ReadFile() niet toelaten
//		 als de file al open is.
// NONO: Wat er aan de hand is is dat de parse code niet goed is. Het gaat fout doordat
//		 we ergens proberen de filepointer voor het begin van het bestand te zetten.
//		 Dat gebeurd zowel in FlightPlayBack() en ResetFilePointer(). Solve it!!!!
//		 Check alle SetFilePointers met FILE_CURRENT.
// NONO: Toch een mutex probleem: FlightPlayBack() en ResetFilePointer() sharen 
//		 weldegelijk de filehandle: via g_hFileFlightRec. Als ze dan tegelijk filepointer
//		 operaties doen gaat het fout. Dus we hebben een Mutex Object of Critical Section
//		 Object nodig voor g_hFileFlightRec.
// YESS: DIT IS ER AAN DE HAND: zoals we nu buffers inlezen kan het gebeuren
// 		 dat [Frame #] doormidden wordt geknipt: e.g. [Fra in de ene buffer en me #] in
// 		 de andere. [Frame #] zal dan niet gevonden worden, and we're hosed.
// 		 Dit is eigenlijk brugklas stof...Sukkel. Zie Michael Abrash's Blackbook voor
// 		 restartable blocks hoe je dit probleem ondervangt. See: Ch. 5 and 14.
//		 You must account for strings spanning blocks.
// TEST: In FrameMove() kunnen we voor het testen van ResetFilePointer() het beste
//		 het volgende doen (ResetFilePointer() werkt dan ook zonder playback):
//
//			if (g_bRTGrabFlightRecSlider || g_bFRGrabFlightRecSlider) {
//				/*if (m_bPlayBack)*/ ResetFilePointer(); <------ TEST
//			}
//
// TEST: In FlightRecProc() kunnen we voor testen van ResetFilePointer() het beste
//		 het volgende doen (ResetFilePointer() wordt dan slechts eenmaal aangeroepen):
//
//			// get/set slider position
//			if (g_bFRGrabFlightRecSlider) {
//				bResetFilePointer5 = true;
//			} else {
//				//SendDlgItemMessage( hDlg, IDC_SLIDER1, TBM_SETPOS, (WPARAM)TRUE, (LPARAM)g_iFrameCount );
//				if (bResetFilePointer5) {
//					bResetFilePointer5 = false;
//					if (g_pd3dApp->m_bPlayBack) {
//						//g_pd3dApp->ResetFilePointer(); <------ TEST
//					}
//				}
//			}
//
// Note: we are doing a simple memchr()/memcmp() search. For better performance we could 
//		 do a Boyer-Moore, Knuth-Morris-Pratt, Horspool, or whatever algorithm.
//		 See also: Bob Stout's Snippets or Sedgewick.
// Note: Buffer moet groter zijn dan het aantal bytes van een Frame.
// Note: We are using one of Bob Stout's Code Snippets to do a Find String In File.
//		 As this is using C file streams we must also open the flight file using
//		 the C file functions.
// Note: find_str_in_file() uses strncmp() to scan the file instead of memchr()/memcmp().
//		 This is the  brute-force technique: it is better to check for a potential match
//		 first with memchr().
// Note: Let's recap: ways to do file I/O:
//		 C Stream I/O:		 fopen()		 fclose()		  fread()	 fwrite()
//		 C++ Stream I/O:	 fstream::open() fstream::close() >>		 <<
//		 Windows Stream I/O: CreateFile()	 CloseHandle()	  ReadFile() WriteFile() 
//-----------------------------------------------------------------------------
void CMyD3DApplication::ResetFilePointer() {

	// mutex
	//if (!g_bCanEnter)
	//	return;
	//g_bCanEnter = false;


//	int i = 0;
//	//DWORD nBytesToRead = 0;
//	DWORD nBytesRead = 0;
//	//static bool bInitRec = true;
//	//static bool bInitPlay = true;
//
//	static TCHAR Buffer[512];
//
//	//char *pdest2 = NULL;
//	char *pdest2 = NULL;
//	int result2 = 0;
//
//	BOOL bResult = FALSE;
//	//DWORD dwResult;

	LARGE_INTEGER liFrequency;
	QueryPerformanceFrequency( &liFrequency ); 
 
	LARGE_INTEGER liCount1;
	LARGE_INTEGER liCount2;
	//QueryPerformanceCounter( &liCount1 );
	//QueryPerformanceCounter( &liCount2 );


	// get the global handle
	HANDLE hFile2 = g_hFileFlightRec;

	// Make sure to return if we have no file handle
	if ( hFile2 == NULL ) {
		return;
	}

	////////////////////////////////////////////////////////
	// C Stream I/O needed for string search functions
	static FILE *stream;
	char *filename = "..\\flights\\flight.temp";
	long startoffset = 0;
	//char *string = "";
	long lStatusFlag = 0;

	TCHAR frame[512];

	TCHAR msg1[512];
	static bool firsttime = true;



	// TODO: can't do this once otherwise you cannot load a new .fld file
	// DONE
	// Open for reading (only do this once)
	// Note: stream must be static!!!
	// Also could do this in FlightPlayBack(), or at application start-up
	//if (firsttime) {
	//	firsttime = false;
		// Note: we are opening in binary mode but might also use
		// text (translated) mode in our case.
		if( (stream = fopen(filename, "rb")) == NULL ) {
			sprintf(msg1, "The file %s was not opened\n", filename);
			MessageBox(NULL, msg1, "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
			return;
		} else {
			sprintf(msg1, "The file %s was opened\n", filename);
			//MessageBox(NULL,msg1,"QQQ",MB_OK);
		}

		// set the global file pointer
		//FILE *g_pFileFlightRec = stream;
	//}
	////////////////////////////////////////////////////////



//	// TEST: Frame Index Table /////////////////////////////////
//	static bool firsttime2 = true;
//	// create frame index table (only do this once) at app startup or
//	// when user loads a new .fld file or when a new recording was made
//	if (firsttime2) {
//		firsttime2 = false;
//		MessageBox(NULL,"Start: BuildFrameIndexTable()","QQQ",MB_OK);
//		BuildFrameIndexTable( stream );
//		MessageBox(NULL,"Ready: BuildFrameIndexTable()","QQQ",MB_OK);
//	}
//	////////////////////////////////////////////////////////////


	// NEW: set file pointer to begin of file
	fseek(stream, 0, SEEK_SET) ;


	// reset file pointer to [Frame g_iFrameCount] ////////////
	//MessageBox(NULL,"qqq!","QQQ!",MB_OK);
	//MessageBeep(-1);

	// check limits
	if (g_iFrameCount < 1) g_iFrameCount = 1;
	if (g_iFrameCount > g_iFrameTotal) g_iFrameCount = g_iFrameTotal;

	//TCHAR frame[512];
	sprintf( frame, "[Frame %i]", g_iFrameCount );


	// Request ownership of the critical section.
	__try 
	{
		EnterCriticalSection(&GlobalCriticalSection);

		
		//////////////////////////////////////////////////////////////////////
		// Find [Frame #] using restartable blocks and accounting for strings
		// that span blocks
		
		QueryPerformanceCounter( &liCount1 );

		//lStatusFlag = find_str_in_file(stream, startoffset, frame);
		lStatusFlag = SearchStringInFile(stream, frame);

		QueryPerformanceCounter( &liCount2 );
		
		sprintf( msg1, "Performance: %.1f ms\n", (float(int(liCount2.QuadPart - liCount1.QuadPart))/int(liFrequency.QuadPart))*1000 );
		//MessageBox(NULL, msg1, "QQQ", MB_OK);
		// Performance: (finding last frame in a 868 frame file)
		// Note: we are using SearchStringInFile() to do restartable blocks.
		
		// Michael Abrash's Blackbook 
		// SearchForString() using memchr()/memcmp(): 4.8 ms

		// Robert Sedgewick's Algorithms in C
		// bmsearch:		5.8 ms

		// Bob Stout's Code Snippets
		// find_str_in_file() using strncmp(): 103.4 ms
		// strsearch:		6.9 ms
		// bmh_search:		  ? ms

		// Thierry Lecroq's Exact String Matching Algorithms
		// SearchBF:	   30.2 ms
		// SearchAUT:		  ? ms
		// SearchBM:		6.5 ms
		// SearchTBM:		9.0 ms
		// SearchTUNEDBM:	5.5 ms
		// SearchKMP:	   17.2 ms
		// SearchHORSPOOL:	5.5 ms
		// SearchRC:	   10.8 ms
		// SearchNSN:	   11.3 ms
		// SearchZT:	   70.2 ms
		// SearchBR:	   71.3 ms
		// SearchQS:		7.3 ms
		// SearchSMITH:		8.1 ms
		// SearchGS:	   41.7 ms
		// SearchRAITA:	    5.3 ms

		// Conclusion: Michael Abrash rules. The only way to get faster performance
		// is by using assembler or by indexing the .fld file (i.e. building a table
		// with indexes to the frames)

		
//		// TEST: Frame Index Table /////////////////////////////
//		theIterator = theVector.begin() + g_iFrameCount;
//		SetFilePointer( hFile2, *theIterator, NULL, FILE_BEGIN );
//		////////////////////////////////////////////////////////



		switch(lStatusFlag) {
			case -1:
				sprintf(msg1, "%s not found\n", frame );
				MessageBox(NULL, msg1, "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
				break;
			case -2:
				printf(	"Not enough memory to perform search\n"	);
				MessageBox(NULL, msg1, "R/C Sim Sikorsky Error Message", MB_OK|MB_ICONERROR);
				break;
			default:
				sprintf(msg1, "%s found at position %d\n", frame, lStatusFlag );
				//MessageBox(NULL,msg1,"QQQ",MB_OK);
				//fseek(stream, lStatusFlag, SEEK_SET);
				SetFilePointer( hFile2, lStatusFlag, NULL, FILE_BEGIN );
		}		
		/////////////////////////////////////////////////////////////////
		
		
		// close file otherwise we cannot load a new .fld file
		fclose(stream);


		//dwResult = SetFilePointer( hFile2, 0L, NULL, FILE_CURRENT );
		//if (dwResult == 0xFFFFFFFF) {
		//	//TCHAR msg1[512];
		//	//sprintf(msg1, "%i ", result2);
		//	//MessageBox(NULL,msg1,"QQQ",MB_OK);
		//	ErrMsgBox();
		//
		//}

	}
	__finally 
	{
		// Release ownership of the critical section.
		LeaveCriticalSection(&GlobalCriticalSection);
	}
	////////////////////////////////////////////////////


	// reset view latency
	//if (g_bRTGrabFlightRecSlider || g_bRTButtonFastForwardDown || g_bRTButtonRewindDown) {
		//if (!g_bRTButtonPauseChecked)
			g_bResetLatency = true;
	//}


	// mutex
	//g_bCanEnter = true;
}




//-----------------------------------------------------------------------------
// Name: UpdateFlightRec()
// Desc: Update Flight Recorder input
// Note: must do this in Render(), not if FrameMove(), because we also want
//		 to do this during Pause
//-----------------------------------------------------------------------------
void CMyD3DApplication::UpdateFlightRec() {

	if (g_bRTButtonRewindDown || g_bFRButtonRewindDown) {
		if ( GetKeyState(VK_CONTROL) & 0x80 ) {
			g_iFrameCount -= 1;
		} else if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			g_iFrameCount -= 100;
		} else {
			g_iFrameCount -= 10;
		}
		// check limits
		if (g_iFrameCount < 1) g_iFrameCount = 1;
		if (g_iFrameCount > g_iFrameTotal) g_iFrameCount = g_iFrameTotal;

		ResetFilePointer();
	}

	if (g_bRTButtonFastForwardDown || g_bFRButtonFastForwardDown) {
		if ( GetKeyState(VK_CONTROL) & 0x80 ) {
			g_iFrameCount += 1;
		} else if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			g_iFrameCount += 100;
		} else {
			g_iFrameCount += 10;
		}		
		// check limits
		if (g_iFrameCount < 1) g_iFrameCount = 1;
		if (g_iFrameCount > g_iFrameTotal) g_iFrameCount = g_iFrameTotal;

		ResetFilePointer();
	}

	// TEST: Shuttle ////////
	if (g_iRTSliderShuttlePosition != 0) {		
		g_iFrameCount += g_iRTSliderShuttlePosition;
		
		// Kludge: during Play (without Pause) a shuttle position of 1 seems to slow down
		// playback. This fixes it.
		if (g_pd3dApp->m_bPlayBack && !g_bRTButtonPauseChecked && g_iRTSliderShuttlePosition == 1)
			g_iFrameCount+=1;

		// check limits
		if (g_iFrameCount < 1) g_iFrameCount = 1;
		if (g_iFrameCount > g_iFrameTotal) g_iFrameCount = g_iFrameTotal;

		ResetFilePointer();
	}
	/////////////////////////
}




//-----------------------------------------------------------------------------
// Name: InitModeMenu()
// Desc: Builds the list of Fullscreen modes for the Mode menu
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::InitModeMenu( HMENU hmenu )
{

	//D3DEnum_DeviceInfo** ppDeviceArg;
	D3DEnum_DeviceInfo* pCurrentDevice;
    DWORD dwCurrentMode;

	//HWND hwndMode;
	//DWORD dwModeItem;
	//DWORD dwMode;
	//TCHAR strMode[80];

	//DDSURFACEDESC2* pddsdMode;

	//DWORD dwWidthOld;
	//DWORD dwRGBBitCountOld;

	//UINT uPosition = 0;




	//static bool firsttime = true;
	//if (firsttime) {
	//	firsttime = false;
	//} else {
	//	return;
	//}


	// Delete all the menu items
	//int count = GetMenuItemCount(hmenu);
	//char msg[200];
	//sprintf( msg, "Count: %i", count );
	//MessageBox(NULL,msg,"QQQ",MB_OK);

	// Hell, this causes us to get irregular behaviour.
	// Probably because all items are deleted and then InsertMenuItem() loses count???
	//for (int item = 0; item < count; item++) {
		//RemoveMenu(hmenu, item, MF_BYPOSITION);
		//MessageBox(NULL,"qqq!","QQQ!",MB_OK);
	//}



	// Get the app's current device
	//ppDeviceArg = &g_pDeviceInfo;
	//if( NULL == ppDeviceArg )
	//	return FALSE;

	// Setup temp storage pointers for dialog
	//pCurrentDevice = (*ppDeviceArg);
	pCurrentDevice = g_pDeviceInfo;
	dwCurrentMode  = pCurrentDevice->dwCurrentMode;



	//MessageBeep(-1);
	//MessageBox(NULL,"qqq!","QQQ!",MB_OK);


    // Get access to the enumerated device list
    D3DEnum_DeviceInfo* pDeviceList;
    DWORD               dwNumDevices;
    D3DEnum_GetDevices( &pDeviceList, &dwNumDevices );

    // Access to UI controls
    //HWND hwndDevice         = GetDlgItem( hDlg, IDC_DEVICE_COMBO );
    //HWND hwndMode           = GetDlgItem( hDlg, IDC_MODE_COMBO );
    //HWND hwndWindowed       = GetDlgItem( hDlg, IDC_WINDOWED_CHECKBOX );
    //HWND hwndStereo         = GetDlgItem( hDlg, IDC_STEREO_CHECKBOX );
    //HWND hwndFullscreenText = GetDlgItem( hDlg, IDC_FULLSCREEN_TEXT );

    // Reset the content in each of the combo boxes
    //ComboBox_ResetContent( hwndDevice );
    //ComboBox_ResetContent( hwndMode );

    // Don't let non-GDI devices be windowed
    //if( FALSE == pCurrentDevice->bDesktopCompatible )
    //    bWindowed = FALSE;

    // Add a list of devices to the menu
    for( DWORD device = 0; device < dwNumDevices; device++ )
    {
        D3DEnum_DeviceInfo* pDevice = &pDeviceList[device];

		//MessageBeep(-1);
		// Add device name to the combo box
        //DWORD dwItem = ComboBox_AddString( hwndDevice, pDevice->strDesc );
        
        // Set the remaining UI states for the current device
        //if( 1/*pDevice == pCurrentDevice*/ )
		if( pDevice == pCurrentDevice )
        {
			//MessageBeep(-1);

			// save dwRGBBitCount if mode 0 into dwRGBBitCountOld
			// We need this to insert separators in the menu
			DDSURFACEDESC2* pddsdMode = &pDevice->pddsdModes[0];
			
			//dwWidthOld	 = pddsdMode->dwWidth;
			//dwRGBBitCountOld = pddsdMode->ddpfPixelFormat.dwRGBBitCount;



            // Build the list of fullscreen modes
            for( DWORD mode = 0; mode < pDevice->dwNumModes; mode++ )
            {
				
				//DDSURFACEDESC2* pddsdMode = &pDevice->pddsdModes[mode];
                pddsdMode = &pDevice->pddsdModes[mode];

                // Skip non-stereo modes, if the device is in stereo mode
                //if( 0 == (pddsdMode->ddsCaps.dwCaps2&DDSCAPS2_STEREOSURFACELEFT) )
                //    if( bStereo )
                //        continue;

                TCHAR strMode[80];
                wsprintf( strMode, _T("%ld x %ld x %ld"),
                          pddsdMode->dwWidth, pddsdMode->dwHeight,
                          pddsdMode->ddpfPixelFormat.dwRGBBitCount );


				



                // Add mode desc to the combo box
                //DWORD dwItem = ComboBox_AddString( hwndMode, strMode );

                // Set the item data to identify this mode
                //ComboBox_SetItemData( hwndMode, dwItem, mode );
				MENUITEMINFO miiMode;

				miiMode.cbSize = sizeof(MENUITEMINFO);
				miiMode.fMask = MIIM_CHECKMARKS | MIIM_DATA | MIIM_ID | MIIM_STATE | MIIM_SUBMENU | MIIM_TYPE;
				miiMode.fType = MFT_STRING;
				miiMode.fState = MFS_ENABLED; //MFS_GRAYED | MFS_CHECKED; 
				miiMode.wID = mode; //0; 
				miiMode.hSubMenu = NULL; 
				miiMode.hbmpChecked = NULL; 
				miiMode.hbmpUnchecked = NULL; 
				miiMode.dwItemData = mode; 
				miiMode.dwTypeData = strMode; 
				miiMode.cch = sizeof(strMode);

				if( mode == dwCurrentMode )
					miiMode.fState = MFS_CHECKED;

				//MessageBeep(-1);
				//MessageBox(NULL,"qqq!","QQQ!",MB_OK);
				// NOTE: we have to remove and then insert. We cannot remove all items
				// at once because it causes fucky behaviour. Probably because
				// InsertMenuItem() loses count if there are no items to start with.

				RemoveMenu(hmenu, mode, MF_BYPOSITION);
				InsertMenuItem(hmenu, mode, TRUE, &miiMode);				


		
				// No. let's not insert separators like this as cards report their modes
				// as follows:
				// mode 0: 320 x 200 x 16
				// mode 1: 320 x 200 x 32
				// etc.
				// We zouden dan elke keer een separator krijgen.
				// We moeten dus een manier vinden om de 16 en 32 bit modes bij elkaar
				// te zetten, e.g. in aparte submenus.
				// Of zo: 16 bits inserten vanaf begin, 32 bits vanaf end of menu (MF_BYPOSITION 
				// en uPosition == 0xFFFFFFFF). Maar wat als er 24 bit modes zijn???

				// Remove present menu item and insert the new one making sure to
				// insert a separator between different dwRGBBitCount
				//
				//if ( dwWidthOld == pddsdMode->dwWidth ) {
				//if ( dwRGBBitCountOld == pddsdMode->ddpfPixelFormat.dwRGBBitCount ) {					
				//	RemoveMenu(hmenu, uPosition, MF_BYPOSITION);
				//	InsertMenuItem(hmenu, uPosition, TRUE, &miiMode);
				//	uPosition += 1;
				//} else {
				//	RemoveMenu(hmenu, uPosition, MF_BYPOSITION);
				//	// NOTE: set uIDNewItem param to high value
				//	InsertMenu(hmenu, uPosition, MF_BYPOSITION|MF_SEPARATOR, 65535, NULL);				
				//
				//	RemoveMenu(hmenu, uPosition+1, MF_BYPOSITION);
				//	InsertMenuItem(hmenu, uPosition+1, TRUE, &miiMode);
				//
				//	uPosition += 2;
				//}

				// test
				//if (mode == 5) {				
				//	InsertMenu(hmenu, uPosition, MF_BYPOSITION|MF_SEPARATOR, 0, NULL);
				//	uPosition += 1;
				//}

				// save for next loop
				//dwWidthOld	 = pddsdMode->dwWidth;
				//dwRGBBitCountOld = pddsdMode->ddpfPixelFormat.dwRGBBitCount;
				
				

                // NO
				// Set the combobox selection on the current mode
                //if( mode == dwCurrentMode )
                //    ComboBox_SetCurSel( hwndMode, dwItem );

				// LET'S DO THIS INSTEAD
				// Set the combobox selection on the reg mode
				//TCHAR strRegMode[80];
                //wsprintf( strRegMode, _T("%ld x %ld x %ld"),
                //          g_dwRegModeWidth, g_dwRegModeHeight,
                //          g_dwRegModeRGBBitCount );

				//if( !strcmp(strMode, strRegMode) ) {
					//ComboBox_SetCurSel( hwndMode, dwItem );
				//}

            }
			//char msg[200];
			//sprintf( msg, "Modes added: %i", mode );
			//MessageBox(NULL,msg,"QQQ",MB_OK);
        }
    }
}




//-----------------------------------------------------------------------------
// Name: CreateTextures()
// Desc: Creates all textures
// Note: We call this function in InitDeviceObjects() to create all textures but
//		 also after every D3DTextr_RestoreAllTextures() to recreate those textures.
//		 D3DTextr_RestoreAllTextures() is necessary to restore textures in an .X file
//		 after it is loaded but also has the nasty habit to wipe out all textures
//		 loaded with D3DTextr_CreateTextureFromFile().
// Note: Beware the D3DTextr_RestoreAllTextures() beast!!!
// Note: NOTE NOTE NOTE: Read all about Access Violation in Debug Mode (F5) elsewhere
//		 caused by using a LPDIRECTDRAWSURFACE7
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::CreateTextures()
{
	HRESULT hr;

	// Create Lens Flare Textures /////////////////////////////////////////////
/*
    // Create textures 
	for(int i = 0; i < NUM_TEXTURES; i++)
	{
		D3DX_SURFACEFORMAT sf = D3DX_SF_UNKNOWN;
		char szTexName[512];

		sprintf(szTexName, "%s%s", g_szPath, g_szTexName[i]);
		//MessageBox(NULL,szTexName,"QQQ!",MB_OK);

		if(FAILED(hr = D3DXCreateTextureFromFile(g_pD3DDevice, 0, 0, 0, &sf, NULL,
			&g_ppTex[i], NULL, szTexName, D3DX_FT_LINEAR)))
		{
			g_ppTex[i] = NULL;
			//MessageBox(NULL,"Can't load lens flare textures!","QQQ!",MB_OK);
			//char errStr[256];
			//D3DXGetErrorString(hr, 256, errStr);
			//MessageBox(NULL, errStr, "D3DX Error", MB_OK);
		}
	}
	if(FAILED(hr)) MessageBox(NULL,"Can't load lens flare textures!","QQQ!",MB_OK);
*/

	
/*
	// moeten we 't weer zelf doen...
	// als we het zo doen zullen alle textures en hun ddraw surfaces 
	// netjes gedelete worden tijdens device change en app exit
	// we are already in media dir
	D3DTextr_CreateTextureFromFile( "flare0.bmp" );
    D3DTextr_CreateTextureFromFile( "flare1.bmp" );
    D3DTextr_CreateTextureFromFile( "flare2.bmp" );
    D3DTextr_CreateTextureFromFile( "flare3.bmp" );

	D3DTextr_RestoreAllTextures( m_pd3dDevice );

	g_ppTex[TEX_FLARE0] = D3DTextr_GetSurface( "flare0.bmp" );
	g_ppTex[TEX_FLARE1] = D3DTextr_GetSurface( "flare1.bmp" );
	g_ppTex[TEX_FLARE2] = D3DTextr_GetSurface( "flare2.bmp" );
	g_ppTex[TEX_FLARE3] = D3DTextr_GetSurface( "flare3.bmp" );
*/

	// NOTE: got to set the current directory right
	//
	// CD3DFile::Load() searches as follows:
	// 1. D3DUtil_GetDXSDKMediaPath()
	// 2. current directory
	//
	// D3DTextr_CreateTextureFromFile() searches are done by:
	// TextureContainer::LoadImageData() as follows:
	// 1. executable's resource (so it must be possible to put all the bmp's in the .exe)
	// 2. current directory/global texture path (can be set by D3DTextr_SetTexturePath(),
	//		initially set to current directory)
	// 3. D3DUtil_GetDXSDKMediaPath()


	// got our own media path
	strcpy( g_szPath, g_szRCSIMMediaPath );


	//CheckDDrawRefCount();
	// loopen is leuker...
	for (int i = 0; i < NUM_TEXTURES; i++)
    {
		char szTexName[512];
		sprintf(szTexName, "%s%s", g_szPath, g_szTexName[i]);

		hr = D3DTextr_CreateTextureFromFile( szTexName, 0, D3DTEXTR_TRANSPARENTBLACK );
		
		if( FAILED(hr) ) {
			//MessageBox(NULL,"qqq","QQQ",MB_OK);
			char msg[512];
			sprintf(msg, "Can't find texture: %s", szTexName);
			MessageBox(NULL,msg,"QQQ",MB_OK);
			//return E_FAIL;
		}

		D3DTextr_Restore( szTexName, m_pd3dDevice );
		g_ppTex[i] = D3DTextr_GetSurface( szTexName );
	}
	//CheckDDrawRefCount();


	g_pLensFlare->SetSource(17.0f, g_ppTex[TEX_FLARE0]);
	g_pLensFlare->SetFlare(0,  8.00f, 0.06f,  1.30f, g_ppTex[TEX_FLARE1]);
	g_pLensFlare->SetFlare(1, 12.00f, 0.04f,  1.00f, g_ppTex[TEX_FLARE2]);
	g_pLensFlare->SetFlare(2,  4.00f, 0.10f,  0.50f, g_ppTex[TEX_FLARE2]);
	g_pLensFlare->SetFlare(3,  8.00f, 0.08f, -0.30f, g_ppTex[TEX_FLARE2]);
	g_pLensFlare->SetFlare(4, 12.00f, 0.04f, -0.60f, g_ppTex[TEX_FLARE3]);
	g_pLensFlare->SetFlare(5, 30.00f, 0.04f, -1.00f, g_ppTex[TEX_FLARE1]);

	// check pixel format
	//DDPIXELFORMAT ddpf;
	//ZeroMemory(&ddpf, sizeof(ddpf));			// not necessary
	//ddpf.dwSize = sizeof(ddpf);					// required!!!
	//if (SUCCEEDED(m_pddsRenderTarget->GetPixelFormat(&ddpf))) {
	//	sprintf(msg1, "dwRBitMask: 0x%.8X", ddpf.dwRBitMask );
	//	sprintf(msg2, "dwGBitMask: 0x%.8X", ddpf.dwGBitMask);
	//	sprintf(msg3, "dwBBitMask: 0x%.8X", ddpf.dwBBitMask);
	//	MessageBox(NULL,"qqq","QQQ!",MB_OK);
	//}
	///////////////////////////////////////////////////////////////////////////



	// Create Heli Pad (Box) Textures ////////////////////////////////////////////////
	// NOTE NOTE NOTE: Read all about Access Violation in Debug Mode (F5) elsewhere
	// caused by using a LPDIRECTDRAWSURFACE7
	//
	// KLOTEZOOI: dit zorgt voor een fout. CRASH CRASH!!!!
	// When a new X file is loaded it does a D3DTextr_RestoreAllTextures()
	// and we then lose this texture. Therefore we have to recreate this in OpenFileDialog()
	// and after all other D3DTextr_RestoreAllTextures(). Basically, doing 
	// D3DTextr_RestoreAllTextures() is bad news.
	// We get crashes after OpenFileDialog(), OpenTerrainFileDialog(), OpenSkyFileDialog()
	// Huh: when doing a restore in OpenFileDialog() we also no longer crash after
	// the other dialogs??? But then again OpenFileDialog() has a texture fault in it
	// somewhere which still has to be solved.

	// NOTE: got to have the full path to the textures
	//char szTexName[512];

	// got our own media path
	//strcpy( g_szPath, g_szRCSIMMediaPath );
	//sprintf(szTexName, "%s%s", g_szPath, g_szTexNameHeliPad1);

	D3DTextr_CreateTextureFromFile( "helipad1.bmp" );
	D3DTextr_Restore( "helipad1.bmp", m_pd3dDevice );
	//g_pTexHeliPad1 = D3DTextr_GetSurface( szTexName ); // Nono

//	// Use D3DX to load textures OK. But note: have to do 
//	// a SAFE_RELEASE( g_pTexHeliPad1 ); in DeleteDeviceObjects()
//	D3DXInitialize();
//	D3DX_SURFACEFORMAT sf = D3DX_SF_UNKNOWN;
//	
//	if( FAILED(hr = D3DXCreateTextureFromFile( g_pd3dApp->m_pd3dDevice, NULL, 
//		0, 0, &sf, NULL, &g_pTexHeliPad1, NULL, MAKEINTRESOURCE(IDB_BITMAP4)/*szTexName*/,
//		D3DX_FT_DEFAULT )) )
//	{
//		g_pTexHeliPad1 = NULL;
//		MessageBox(NULL,"Can't load heli pad texture!","QQQ!",MB_OK);
//		char errStr[256];
//		D3DXGetErrorString(hr, 256, errStr);
//		MessageBox(NULL, errStr, "D3DX Error", MB_OK);
//	}



	//sprintf(szTexName, "%s%s", g_szPath, "eeb.dat");

	CopyFile("eeb.dat", "C:\\Windows\\Temp\\mdl.bmp", FALSE);
	D3DTextr_CreateTextureFromFile( "C:\\Windows\\Temp\\mdl.bmp" );
	D3DTextr_Restore( "C:\\Windows\\Temp\\mdl.bmp", m_pd3dDevice );
	//g_pTexHeliPad2 = D3DTextr_GetSurface( "C:\\Windows\\Temp\\mdl.bmp" ); // Nono
	///////////////////////////////////////////////////////////////////////////




	// Create tree textures /////////////////////////////////////////
	// NOTE NOTE NOTE: Read all about Access Violation in Debug Mode (F5) elsewhere
	// caused by using a LPDIRECTDRAWSURFACE7
	//
	SetCurrentDirectory("tree");

    D3DTextr_CreateTextureFromFile( "tree1.bmp", 0, D3DTEXTR_TRANSPARENTBLACK );
	D3DTextr_Restore( "tree1.bmp", m_pd3dDevice );
	//m_pTexTree = D3DTextr_GetSurface( "tree1.bmp" ); // Nono

	D3DTextr_CreateTextureFromFile( "shadow1.bmp", 0, D3DTEXTR_TRANSPARENTWHITE );
	D3DTextr_Restore( "shadow1.bmp", m_pd3dDevice );	
	//m_pTexTreeShadow = D3DTextr_GetSurface( "shadow1.bmp" ); // Nono

	SetCurrentDirectory("..");
	/////////////////////////////////////////////////////////////////

	
	return S_OK;

}




//-----------------------------------------------------------------------------
// Name: DrawTrees()
// Desc: From D3DIM Billboard Sample
// Note: Make sure to restore D3DTSS_ALPHAOP to D3DTOP_DISABLE otherwise we get
//		 a heli which turns invisible now and then!!! Nono: reset to D3DTOP_MODULATE.
//		 Well, this does not seem to be the culprit: we still get invisible flashing
//		 heli. Fix it!!! Odd thing is that with low tree count (10) we had no probs.
//		 Now with 200 trees we get that darned flashing again. It could be video card
//		 driver related: on Win98 no such probs.
// Note: When we leave active our initial code to draw trees using m_pTreeObject[]
//		 the texture will not be transparent: m_pTreeObject loads the x file
//		 that also uses Tree.bmp texture. m_pd3dDevice->SetTexture() then screws
//		 up in that it will set the texture, but not transparently. What hell!!!
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::DrawTrees()
{
    // NOTE: setting texture here makes heli disappear now and then
	// WHAT'S GOING ON???
	// THIS: we didn't restore D3DTSS_ALPHAOP to D3DTOP_DISABLE/D3DTOP_MODULATE
	
    // Set texture for drawing trees
	//m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface("Tree.bmp") );
	
	// NOTE NOTE NOTE!!!!!!!!!!!!!!!!!!!!!!!!!!! DEBUG ////////////////////////
	// NOTE: In Debug mode (F5) we get an Unhandled Exception: Access Violation here.
	// All our SetTexture() code does this. We probably do not have the
	// DDraw surfaces set properly. 
	//m_pd3dDevice->SetTexture( 0, NULL ); // no probs here, of course
	// OKOK: we've got the bugger: we cannot use the texture when the current dir is
	// not set to the dir the texture is in. Using a pointer like we did makes no difference:
	// We *must* have the directory set properly. This is gonna be a real pita!!!
	// Well, no: we do not need to set the dir: D3DTextr_GetSurface() will look in
	// the texture container for a texture with that name. Problem there is of course:
	// what will happen when there are two textures with the name same name? When
	// we use one \media dir there is no prob because there can never be textures with the
	// same name, but we are using different dirs...
	// Wat er aan de hand is is waarschijnlijk dit: een texture moet eerst restored worden
	// voordat je het kunt gebruiken in SetTexture(). Echter, wanneer er ergens een
	// D3DTextr_RestoreAllTextures() wordt gedaan, zal de pointer waarschijnlijk niet
	// meer geldig zijn (of zoiets) en krijgen we een Access Violation.
	// We moeten dus gewoon the texture naam met path gebruiken in D3DTextr_GetSurface()
	// en niet meer werken met een LPDIRECTDRAWSURFACE7
	// NOTE: texture name clashes kunnen worden opgelost door het volledige pad te gebruiken
	// bij:
	// * D3DTextr_CreateTextureFromFile()
	// * D3DTextr_Restore()
	// * D3DTextr_GetSurface()
	// NOTE: these functions search for the texture in the texture container *by string*
	// so if we provide the entire path we can always avoid name clashes. But it might not
	// be necessary: it is better to use unique texture names.
	// NOTE: only for D3DTextr_CreateTextureFromFile() do we need the correct current dir
	// if we use the texture filename only. D3DTextr_Restore() and D3DTextr_GetSurface()
	// look in the texture container *by string* so the current directory is irrelevant.
	// Let's recap:
	// D3DTextr_CreateTextureFromFile() looks for a .bmp file in the file system
	// D3DTextr_Restore() and D3DTextr_GetSurface() look for a texture in the texture container
	// *by string*
	// NOTE: here's 3 steps how to use textures:
	// 1. D3DTextr_CreateTextureFromFile( "tree1.bmp", 0, D3DTEXTR_TRANSPARENTBLACK ); // create texture from .bmp file
	// 2. D3DTextr_Restore( "tree1.bmp", m_pd3dDevice ); // restore texture (necessary) 
	// 3. m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface("tree1.bmp") ); // set texture
	//
	//
	//
	//SetCurrentDirectory("tree"); // Not necessary
	//m_pTexTree = D3DTextr_GetSurface( "tree1.bmp" ); // We'll get a totally different pointer than the original m_pTexTree
	m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface( "tree1.bmp" ) ); // OK
	//m_pd3dDevice->SetTexture( 0, m_pTexTree ); // WRONG: Access Violation
	//SetCurrentDirectory(".."); // Not necesarry
	//m_pd3dDevice->SetTexture( 0, m_pTexTree );


	

	// Set state for drawing trees

	// Note: in DX7, setting D3DRENDERSTATE_LIGHTING to FALSE is needed to 
    // turn off vertex lighting (and use the color in the D3DLVERTEX instead.)
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, FALSE );


	// BUGGER: ONE OF THESE
//	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG1, D3DTA_TEXTURE );
//	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLORARG2, D3DTA_DIFFUSE );
//	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP,   D3DTOP_MODULATE );

    // THIS ONE:
	// NOTE: Use this state but remember to restore D3DTSS_ALPHAOP to D3DTOP_DISABLE!!!
	// NONO: D3DTOP_MODULATE
	// Otherwise we'll get a heli that goes invisible every now and then.
    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAARG1, D3DTA_TEXTURE );
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP,   D3DTOP_SELECTARG1 );

//    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_LINEAR );
//    m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_LINEAR );
//    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DITHERENABLE, TRUE );


    // Billboards are in XY plane, so turn off texture perpective correction
    //m_pd3dDevice->SetRenderState( D3DRENDERSTATE_TEXTUREPERSPECTIVE, FALSE );

    // Trees should be using the zbuffer
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, TRUE );
    
    // Set diffuse blending for alpha set in vertices. 
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE,   TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND,  D3DBLEND_SRCALPHA );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA );

    // Enable alpha testing (avoids drawing pixels with less than a certain
    // alpha.)
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, m_bUseAlphaTest );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAREF,        0x08 );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAFUNC, D3DCMP_GREATEREQUAL );

    // Set up a rotation matrix to orient the billboard towards the camera.
    // This is for billboards that are fixed (to rotate about) about the Y-axis.
	// See note at top of file for more info.
//  D3DMATRIX matWorld;
//	D3DUtil_SetIdentityMatrix( matWorld );
//  D3DUtil_SetRotateYMatrix( matWorld, -m_fViewAngle );


	D3DMATRIX matTrans, matTrans2, matRotY, matAll;

    // Loop through and render all trees
	// TEST:
//	if ( GetKeyState(VK_SHIFT) & 0x80 ) {
//		if ( GetKeyState('L') & 0x80 ) g_iNumTrees+=10;
//		if ( GetKeyState('K') & 0x80 ) g_iNumTrees-=10;
//	} else {
//		if ( GetKeyState('L') & 0x80 ) g_iNumTrees++;
//		if ( GetKeyState('K') & 0x80 ) g_iNumTrees--;
//	}
//	// limit
//	if (g_iNumTrees<0) g_iNumTrees = 0;
//	if (g_iNumTrees>NUM_TREES) g_iNumTrees = NUM_TREES;

	// TEST: with one tree
	// DWORD i=4;    
	for( DWORD i=0; i<g_iNumTrees/*NUM_TREES*/; i++ )
    {
        // Translate and rotate the billboards into place

		// NOTE: we must reset to identity matrix every frame
		D3DUtil_SetIdentityMatrix( m_matTreeMatrix[i] );
		
		// Rotate about Y, then translate to position
		// NOTE: We are reseting the tree matrix to identity every frame move so we
		// need not translate to origin, rotate, translate back. Just rotate, translate.
		D3DUtil_SetTranslateMatrix(matTrans, m_TreePositions[i].x, -10.0f, m_TreePositions[i].z);
		//D3DUtil_SetTranslateMatrix(matTrans2, m_TreePositions[i].x, -10.0f, m_TreePositions[i].z);

		//static float f = 0.0f; f+=0.5f;
		//D3DUtil_SetRotateYMatrix(matRotY, f);
		float fViewAngle = 0.0f;
		

		// make billboards always face the camera
		// NOTE: m_matView._41, m_matView._42, m_matView._43 does not correspond to 
		// the x,y,z position of the camera.  Huh??? Well, m_fCamX, m_fCamY, m_fCamZ do.
		// When we move cam X, m_matView._43 also changes. Odd. 
//		if (m_bRCView) {
//			if (m_TreePositions[i].x < m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/(m_TreePositions[i].x-m_fCamX) );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x > m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/(m_fCamX-m_TreePositions[i].x) );
//				D3DUtil_SetRotateYMatrix(matRotY, fViewAngle+g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x == m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/0.000001f );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//		}
//
//		//sprintf( msg12, "ViewAngle: %.2f", D3DXToDegree(fViewAngle) );
//		//sprintf( msg12, "m_matView._41: %.2f, m_matView._42: %.2f, m_matView._43: %.2f", m_matView._41, m_matView._42, m_matView._43 );
//		//sprintf( msg12, "m_fCamX: %.2f, m_fCamY: %.2f, m_fCamZ: %.2f", m_fCamX, m_fCamY, m_fCamZ );
//		//sprintf( msg12, "m_fCamRadsX: %.2f, m_fCamRadsY: %.2f, m_fCamRadsZ: %.2f", m_fCamRadsX, m_fCamRadsY, m_fCamRadsZ );
//		
//
//		if (m_bInModelView) {
//			if (m_TreePositions[i].x < m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/(m_TreePositions[i].x-m_matFileObjectMatrix._41) );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x > m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/(m_matFileObjectMatrix._41-m_TreePositions[i].x) );
//				D3DUtil_SetRotateYMatrix(matRotY, fViewAngle+g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x == m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/0.000001f );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//		}
//
//
//		if (m_bChaseView) {
//			if (m_TreePositions[i].x < m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/(m_TreePositions[i].x-m_matFileObjectMatrix._41) );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x > m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/(m_matFileObjectMatrix._41-m_TreePositions[i].x) );
//				D3DUtil_SetRotateYMatrix(matRotY, fViewAngle+g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x == m_matFileObjectMatrix._41) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_matFileObjectMatrix._43)/0.000001f );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//		}
//
//
//		if (m_bFollowView) {
//			if (m_TreePositions[i].x < m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/(m_TreePositions[i].x-m_fCamX) );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x > m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/(m_fCamX-m_TreePositions[i].x) );
//				D3DUtil_SetRotateYMatrix(matRotY, fViewAngle+g_PI_DIV_2);
//			}
//			if (m_TreePositions[i].x == m_fCamX) {
//				fViewAngle = atanf( (m_TreePositions[i].z-m_fCamZ)/0.000001f );
//				D3DUtil_SetRotateYMatrix(matRotY, -fViewAngle-g_PI_DIV_2);
//			}
//		}



		// TODO: need different rotation algo for InModel, Chase, and Follow views
		// Well, rotating the billboard towards the camera is a pain with all those
		// camera modes. Moreover, you will always see the trees rotating from certain
		// cam angles and from high positions you see the flat tree. Instead it's more
		// robust and simple to make a two-faced billboard and put it in a cross.
		// Or possibly in a hex.
		// We then no longer have to rotate the billboard. Drawback however is that
		// the trunk of the tree does not look perfect and that we must take extra care
		// to place the tree in the vertical center of the bitmap. But the advantages far
		// outweigh this drawback: no odd and complicated rotations and the tree looks good
		// even from high positions.

		// Show both sides
		DWORD dwCullModeOld;
		m_pd3dDevice->GetRenderState( D3DRENDERSTATE_CULLMODE, &dwCullModeOld );
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_NONE );

		
		//D3DUtil_SetRotateYMatrix(matRotY, 0.0f);
		D3DUtil_SetRotateYMatrix(matRotY, g_PI); // rotate 180 for better alignment with shadow

		// Order!!!
		D3DMath_MatrixMultiply(m_matTreeMatrix[i],matRotY,matTrans);
		//D3DMath_MatrixMultiply(matAll,matAll,matTrans2);		
		//D3DMath_MatrixMultiply(m_matTreeMatrix[i], m_matTreeMatrix[i], matAll);

				
		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTreeMatrix[i] );

        // Render the billboards polygons
        m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_LVERTEX,
                                     m_TreeMesh, 4, 0 );

		
		// TEST:
		// cross
		D3DUtil_SetRotateYMatrix(matRotY, g_PI_DIV_2);
		D3DMath_MatrixMultiply(m_matTreeMatrix[i],matRotY,matTrans);
		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTreeMatrix[i] );
		m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_LVERTEX,
                                     m_TreeMesh, 4, 0 );

//		// hex
//		D3DUtil_SetRotateYMatrix(matRotY, g_PI_DIV_4);
//		D3DMath_MatrixMultiply(m_matTreeMatrix[i],matRotY,matTrans);
//		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTreeMatrix[i] );
//		m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_LVERTEX,
//                                     m_TreeMesh, 4, 0 );
//
//		D3DUtil_SetRotateYMatrix(matRotY, 3*g_PI_DIV_4);
//		D3DMath_MatrixMultiply(m_matTreeMatrix[i],matRotY,matTrans);
//		m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matTreeMatrix[i] );
//		m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_LVERTEX,
//                                     m_TreeMesh, 4, 0 );


		// Restore cull mode
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, dwCullModeOld );
    }

    // Restore state
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE,  FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, FALSE );

	//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_TEXTUREPERSPECTIVE, TRUE );
	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, TRUE );

	//m_pd3dDevice->SetTextureStageState( 0, D3DTSS_COLOROP, D3DTOP_MODULATE );
	m_pd3dDevice->SetTextureStageState( 0, D3DTSS_ALPHAOP, D3DTOP_MODULATE );


	// Restore texture to no texture
	m_pd3dDevice->SetTexture( 0, NULL );

    return S_OK;
}





//-----------------------------------------------------------------------------
// Name: DrawTreeShadows()
// Desc: From D3DIM Billboard Sample
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::DrawTreeShadows()
{
    // Set texture for rendering shadows
	//m_pd3dDevice->SetTexture( 0, m_pTexTreeShadow ); // WRONG: Access Violation in Debug Mode (F5)	
    m_pd3dDevice->SetTexture( 0, D3DTextr_GetSurface("shadow1.bmp") ); // OK


	// Note: in DX7, setting D3DRENDERSTATE_LIGHTING to FALSE is needed to 
    // turn off vertex lighting (and use the color in the D3DLVERTEX instead.)
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, FALSE );


	// Set state for rendering shadows
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_TEXTUREPERSPECTIVE, TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE,   TRUE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND,  D3DBLEND_ZERO );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_SRCCOLOR );

    // Enable alpha testing (avoids drawing pixels with less than a certain
    // alpha.)
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, m_bUseAlphaTest );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAREF,        0x08 );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHAFUNC, D3DCMP_GREATEREQUAL );

    // Rotate the world matrix, to lay the shadow on the ground
	D3DMATRIX matTrans, matTrans2, matRotX, matRotY, matAll;
    D3DMATRIX matWorld;
    D3DUtil_SetRotateXMatrix( matRotX, g_PI/2.0f);
	D3DUtil_SetRotateYMatrix( matRotY, -D3DXToRadian(m_iSunDirection)-g_PI_DIV_2);

	// Order!!!
	D3DMath_MatrixMultiply(matWorld,matRotX,matRotY);	

    // Loop through the trees rendering the shadows
    for( WORD i=0; i<g_iNumTrees/*NUM_TREES*/; i++ )
    {
        // Tranlate the world matrix to move the shadow into place
        matWorld._41 = m_TreePositions[i].x;
        matWorld._42 = m_TreePositions[i].y+0.03f;
        matWorld._43 = m_TreePositions[i].z;

        m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &matWorld );

		// longer shadows depending on sun elevation
		// NOTE: at 90 deg we will have no shadow but that's of course one of the
		// drawbacks of the billboarding technique. We might kludge to remedy this.
		// It would involve drawing the tree's top shadow (we'll need a separate
		// bitmap for that) under the tree. But it's not really necessary.
		D3DLVERTEX  TreeMeshShadow[4];
		TreeMeshShadow[0] = m_TreeMesh[0];
		TreeMeshShadow[1] = m_TreeMesh[1];
		TreeMeshShadow[2] = m_TreeMesh[2];
		TreeMeshShadow[3] = m_TreeMesh[3];
	
		TreeMeshShadow[1].y = TreeMeshShadow[1].y/tanf(D3DXToRadian(m_iSunElevation));		
		TreeMeshShadow[3].y = TreeMeshShadow[3].y/tanf(D3DXToRadian(m_iSunElevation));


		// Show both sides
		DWORD dwCullModeOld;
		m_pd3dDevice->GetRenderState( D3DRENDERSTATE_CULLMODE, &dwCullModeOld );
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, D3DCULL_NONE );

        // Render the shadow's polygons
        m_pd3dDevice->DrawPrimitive( D3DPT_TRIANGLESTRIP, D3DFVF_LVERTEX,
                                     TreeMeshShadow, 4, 0 );

		// Restore cull mode
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_CULLMODE, dwCullModeOld );
    }

    // Restore state
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE,  FALSE );
    m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, FALSE );

	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_LIGHTING, TRUE );

	// Restore texture to no texture
	m_pd3dDevice->SetTexture( 0, NULL );

    return S_OK;
}





//-----------------------------------------------------------------------------
// Name: RenderProxy()
// Desc: 
// Note:
//-----------------------------------------------------------------------------
HRESULT CMyD3DApplication::RenderProxy()
{

	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, TRUE );

//	if (g_fProxyAlpha < 1.0f/*g_bTransparentProxy*/) {
//		//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, TRUE );
//
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, true);
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_SRCBLEND, D3DBLEND_ONE);
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_DESTBLEND, D3DBLEND_ONE);
//	}

	// TEST:
	//g_fProxyAlpha = 0.5f;

	// check alpha
	if (g_fProxyAlpha<0.0f) g_fProxyAlpha = 0.0f;
	if (g_fProxyAlpha>1.0f) g_fProxyAlpha = 1.0f;
	
	D3DMATERIAL7 mtrl;
	D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, g_fProxyAlpha );
	//D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, 0.7f );		
	m_pd3dDevice->SetMaterial( &mtrl );

	DWORD dwLight = 0x00404040;
	m_pd3dDevice->SetRenderState(  D3DRENDERSTATE_AMBIENT, dwLight );

//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &m_matFileObjectMatrix );
//	m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
//										g_pvCube, NUM_CUBE_VERTICES,
//										g_pwCubeIndices, NUM_CUBE_INDICES, NULL );
	//m_pd3dDevice->DrawIndexedPrimitive( D3DPT_TRIANGLELIST, D3DFVF_VERTEX,
	//									g_pvCube2, NUM_CUBE_VERTICES_2,
	//									g_pwCubeIndices, NUM_CUBE_INDICES, NULL );

	
	// proxy composite
	D3DXMATRIX matTrans;
	D3DXMATRIX matRot;
	D3DXQUATERNION qRot;

	// Fuselage (box)
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[0] );
	
	g_pBox[0]->Draw();

	// Canopy (capped cylinder)
	// NOTE: D3DXCreateCylinder() creates the cylinder with one end at (0,0,0)
	// ODE creates the ccylinder geom with the ccylinder's center at (0,0,0)
	// Therefore we have to translate back by half the cylinder length
	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, -proxyCanopy.length*0.5 );
	D3DMath_MatrixMultiply( g_matProxy[1], matTrans, g_matProxy[1] ); // order is crucial!!!
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[1] );

	g_pCylinder[0]->Draw();
	g_pSphere[0]->Draw();

	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyCanopy.length );
	D3DMath_MatrixMultiply( g_matProxy[1], matTrans, g_matProxy[1] );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[1] );
	
	g_pSphere[1]->Draw();
	
	// Tail (capped cylinder)
	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, -proxyTail.length*0.5 );
	D3DMath_MatrixMultiply( g_matProxy[2], matTrans, g_matProxy[2] ); // order is crucial!!!
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[2] );

	g_pCylinder[1]->Draw();
	g_pSphere[2]->Draw();		

	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyTail.length );
	D3DMath_MatrixMultiply( g_matProxy[2], matTrans, g_matProxy[2] );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[2] );

	g_pSphere[3]->Draw();

	
	// Left Skid (capped cylinder)
	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, -proxyLSkid.length*0.5 );
	D3DMath_MatrixMultiply( g_matProxy[3], matTrans, g_matProxy[3] ); // order is crucial!!!
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[3] );

	g_pCylinder[2]->Draw();
	g_pSphere[4]->Draw();		

	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyLSkid.length );
	D3DMath_MatrixMultiply( g_matProxy[3], matTrans, g_matProxy[3] );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[3] );

	g_pSphere[5]->Draw();

	// Right Skid (capped cylinder)
	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, -proxyRSkid.length*0.5 );
	D3DMath_MatrixMultiply( g_matProxy[4], matTrans, g_matProxy[4] ); // order is crucial!!!
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[4] );

	g_pCylinder[3]->Draw();
	g_pSphere[6]->Draw();		

	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyRSkid.length );
	D3DMath_MatrixMultiply( g_matProxy[4], matTrans, g_matProxy[4] );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &g_matProxy[4] );

	g_pSphere[7]->Draw();

	// TEST: fast/slow rotors
	if (g_pd3dApp->m_bRotaryWing) {
		if (g_pd3dApp->m_fThrottle>7.00f) {
			// NOTE: We draw the fast rotor proxies always transparently
			// Hell, but if we do this Show Exhaust Smoke still affects the rotor transparency
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, TRUE );

			// rotors
			// TODO: make heli pads and heli visible through rotors. We can see the heli proxy
			// allright through the alpha proxy rotors, but not the heli pads and heli.
			// All objects drawn *after* transparent proxy rotors are not visible through the
			// rotors.
			// Add option to draw entire proxy (not only rotors) in alpha
			// Make sure rest of the scene shines through properly...
			// Solution: in order to achieve this you have two options:
			// 1. Make sure transparent primitives are drawn after opaque primitives
			// 2. Turn off the Z-buffer when drawing transparent primitives
			// 
			// DONE: Changed drawing order. We are now drawing the proxy last which enables us
			// to see all objects through the transparent proxy. There is one problem though:
			// We cannot see the proxy through the red proto heli's rotors. Cause:
			// The red proto heli has transparent rotors. Since the heli is drawn before the
			// proxy, these rotors will not allow the proxy to shine through. A kludge would be
			// to draw all meshes in the heli .X file separately (except mesh-mrotor and
			// mesh-trotor) and draw the mesh-mrotor and mesh-trotor after the proxy.
			// We can also draw the proxy (except for the transparent proxy rotors) before
			// the red proto heli. This however would not give us the option to draw the
			// entire proxy transparently (because the heli would not be visible through the
			// proxy because it is drawn later than the proxy).
			D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, 0.7f );
			m_pd3dDevice->SetMaterial( &mtrl );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, TRUE );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHATESTENABLE, TRUE );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ALPHABLENDENABLE, TRUE );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_SRCBLEND, D3DBLEND_SRCALPHA );
			//m_pd3dDevice->SetRenderState( D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA );
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, TRUE);
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_SRCBLEND, D3DBLEND_SRCCOLOR);
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_DESTBLEND, D3DBLEND_INVSRCALPHA);
			 
			// NOTE1: gotto flip the entire D3DX cylinder 90 deg. along X-axis because ODE cylinder
			// is aligned along Y-axis (unlike ccylinder which is aligned along Z-axis).
			// NOTE2: gotto flip one polygon (flat cap) 180 deg. along
			// And this one also goes for cylinders:
			// NOTE: D3DXCreateCylinder() creates the cylinder with one end at (0,0,0)
			// ODE creates the ccylinder geom with the ccylinder's center at (0,0,0)
			// Therefore we have to translate back by half the cylinder length
			// This time up along Y-axis, though.
			// Main Rotor
			D3DXMatrixTranslation( &matTrans, 0.0f, proxyMRotor.length*0.5, 0.0f );
			D3DXMatrixRotationYawPitchRoll(&matRot, 0.0f, g_PI_DIV_2, 0.0f); // flip cylinder
			D3DMath_MatrixMultiply( matTrans, matRot, matTrans );
			D3DMath_MatrixMultiply( proxyMRotor.mat, matTrans, proxyMRotor.mat ); // order is crucial!!!
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyMRotor.mat );

			g_pRotorCylinder[0]->Draw();
			g_pRotorPolygon[0]->Draw();		
		
			D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyMRotor.length );
			D3DXMatrixRotationYawPitchRoll(&matRot, 0.0f, g_PI, 0.0f); // flip polygon
			D3DMath_MatrixMultiply( matTrans, matRot, matTrans );
			D3DMath_MatrixMultiply( proxyMRotor.mat, matTrans, proxyMRotor.mat );
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyMRotor.mat );

			g_pRotorPolygon[1]->Draw();

			// Tail Rotor
			D3DXMatrixTranslation( &matTrans, 0.0f, proxyTRotor.length*0.5, 0.0f );
			D3DXMatrixRotationYawPitchRoll(&matRot, 0.0f, g_PI_DIV_2, 0.0f); // flip cylinder
			D3DMath_MatrixMultiply( matTrans, matRot, matTrans );
			D3DMath_MatrixMultiply( proxyTRotor.mat, matTrans, proxyTRotor.mat ); // order is crucial!!!
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyTRotor.mat );

			g_pRotorCylinder[1]->Draw();
			g_pRotorPolygon[2]->Draw();		

			D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyTRotor.length );
			D3DXMatrixRotationYawPitchRoll(&matRot, 0.0f, g_PI, 0.0f); // flip polygon
			D3DMath_MatrixMultiply( matTrans, matRot, matTrans );
			D3DMath_MatrixMultiply( proxyTRotor.mat, matTrans, proxyTRotor.mat );
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyTRotor.mat );

			g_pRotorPolygon[3]->Draw();

			// restore render states
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, FALSE);
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_BLENDENABLE, FALSE);
			//m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ZWRITEENABLE, TRUE);
		} else {
			D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, g_fProxyAlpha );
			m_pd3dDevice->SetMaterial( &mtrl );

			// slow rotors
			// Slow MRotor (box)
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyMRotorSlow.mat );	
			g_pSlowRotorBox[0]->Draw();

			// Slow TRotor (box)
			m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyTRotorSlow.mat );	
			g_pSlowRotorBox[1]->Draw();
		}
	}

	D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, g_fProxyAlpha );
	m_pd3dDevice->SetMaterial( &mtrl );

	// Wheels
	D3DXMatrixRotationYawPitchRoll(&matRot, g_PI_DIV_2, 0.0f, 0.0f );
	D3DMath_MatrixMultiply( proxyLFrontWheel.mat, matRot, proxyLFrontWheel.mat );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyLFrontWheel.mat );
	g_pWheelSphere[0]->Draw();

	D3DXMatrixRotationYawPitchRoll(&matRot, g_PI_DIV_2, 0.0f, 0.0f );
	D3DMath_MatrixMultiply( proxyRFrontWheel.mat, matRot, proxyRFrontWheel.mat );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyRFrontWheel.mat );
	g_pWheelSphere[1]->Draw();

	D3DXMatrixRotationYawPitchRoll(&matRot, g_PI_DIV_2, 0.0f, 0.0f );
	D3DMath_MatrixMultiply( proxyLBackWheel.mat, matRot, proxyLBackWheel.mat );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyLBackWheel.mat );
	g_pWheelSphere[2]->Draw();

	D3DXMatrixRotationYawPitchRoll(&matRot, g_PI_DIV_2, 0.0f, 0.0f );
	D3DMath_MatrixMultiply( proxyRBackWheel.mat, matRot, proxyRBackWheel.mat );
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyRBackWheel.mat );
	g_pWheelSphere[3]->Draw();


	// Windsock (capped cylinder)
//	D3DUtil_InitMaterial( mtrl, 1.0f, 1.0f, 1.0f, g_fProxyAlpha );
//	m_pd3dDevice->SetMaterial( &mtrl );
//
//	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, -proxyWindsock.length*0.5 );
//	D3DMath_MatrixMultiply( proxyWindsock.mat, matTrans, proxyWindsock.mat ); // order is crucial!!!
//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyWindsock.mat );
//
//	g_pWindsockCylinder->Draw();
//	g_pWindsockSphere[0]->Draw();		
//
//	D3DXMatrixTranslation( &matTrans, 0.0f, 0.0f, proxyWindsock.length );
//	D3DMath_MatrixMultiply( proxyWindsock.mat, matTrans, proxyWindsock.mat );
//	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyWindsock.mat );
//
//	g_pWindsockSphere[1]->Draw();

	// Windsock (box)
	m_pd3dDevice->SetTransform( D3DTRANSFORMSTATE_WORLD, &proxyWindsock.mat );
	
	if (g_bWindsock) 
		g_pWindsockBox->Draw();
	/////////////////////////


//	// restore render states
//	if (g_fProxyAlpha < 1.0f/*g_bTransparentProxy*/) {
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE, false);
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_SRCBLEND, D3DBLEND_ONE);   // Set in InitDeviceObjects()
//		m_pd3dDevice->SetRenderState(D3DRENDERSTATE_DESTBLEND, D3DBLEND_ZERO); // Set in InitDeviceObjects()
//	}


	m_pd3dDevice->SetRenderState( D3DRENDERSTATE_BLENDENABLE, FALSE );

	
	return S_OK;
}



//-----------------------------------------------------------------------------
// Name: SaveFileObjectOrigMeshVertices()
// Desc: 
// Note: We need this for mesh morphing (rotor coning, skid spring, crashes)
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::SaveFileObjectOrigMeshVertices()
{
	D3DVERTEX* pVertices;
	DWORD      dwNumVertices;


	// fill vertex array skid
	//D3DVERTEX* pVertices;
	//DWORD      dwNumVertices;
	if ( SUCCEEDED( m_pFileObject->GetMeshVertices( "mesh-skid", &pVertices, &dwNumVertices ) ) )
	{
		delete[] g_pVerticesOrigSkid;
		g_pVerticesOrigSkid = new D3DVERTEX[dwNumVertices];
		
		for ( DWORD i=0; i<dwNumVertices; i++ ) {
			g_pVerticesOrigSkid[i] = pVertices[i];
		}
	} else {		
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	}


	// fill vertex array mrotor
	//D3DVERTEX* pVertices;
	//DWORD      dwNumVertices;
	if ( SUCCEEDED( m_pFileObject->GetMeshVertices( "mesh-mrotor", &pVertices, &dwNumVertices ) ) )
	{
		delete[] g_pVerticesOrigMRotor;
		g_pVerticesOrigMRotor = new D3DVERTEX[dwNumVertices];

		for ( DWORD i=0; i<dwNumVertices; i++ ) {
			g_pVerticesOrigMRotor[i] = pVertices[i];
		}
	} else {		
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	}


	// fill vertex array trotor
	//D3DVERTEX* pVertices;
	//DWORD      dwNumVertices;
	if ( SUCCEEDED( m_pFileObject->GetMeshVertices( "mesh-trotor", &pVertices, &dwNumVertices ) ) )
	{
		delete[] g_pVerticesOrigTRotor;
		g_pVerticesOrigTRotor = new D3DVERTEX[dwNumVertices];

		for ( DWORD i=0; i<dwNumVertices; i++ ) {
			g_pVerticesOrigTRotor[i] = pVertices[i];
		}
	} else {		
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	}


	// fill vertex array shaft
	//D3DVERTEX* pVertices;
	//DWORD      dwNumVertices;
	if ( SUCCEEDED( m_pFileObject->GetMeshVertices( "mesh-shaft", &pVertices, &dwNumVertices ) ) )
	{
		delete[] g_pVerticesOrigShaft;
		g_pVerticesOrigShaft = new D3DVERTEX[dwNumVertices];

		for ( DWORD i=0; i<dwNumVertices; i++ ) {
			g_pVerticesOrigShaft[i] = pVertices[i];
		}
	} else {		
		//MessageBox(NULL,"No such mesh in this X file","QQQ",MB_OK);	
		//return E_FAIL;
	}


}




//-----------------------------------------------------------------------------
// Name: LoadScenery()
// Desc: Loads the panorama bitmaps and creates the textures
// Note: Called from InitDeviceObjects()
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::LoadScenery()
{
	HRESULT hr;

	// PANORAMA //////////////////////////////////////////
	//D3DTextr_CreateTextureFromFile( "pan1.bmp" );
	//D3DTextr_Restore( "pan1.bmp", m_pd3dDevice );

	//D3DTextr_CreateTextureFromFile( "pan2.bmp" );
	//D3DTextr_Restore( "pan2.bmp", m_pd3dDevice );

	//IDirectDrawSurface7 *g_ptexPanorama;
    //m_pd3dDevice = m_pd3dx->GetD3DDevice();

	// TEST:
	// We seem to be getting two WM_SIZE after resizing and the mouse seems
	// to stick to the window edge. This is a bore since these textures 
	// take a long time to load.
	// Could it be we get that extra WM_SIZE from the client window??? No!
	// It is because when resizing, a flood of WM_SIZE messages are sent. And
	// every time we get a WM_SIZE InitDeviceObjects() is called. Because that
	// has become slow due to all the texture loading it looks like we are getting
	// two messages and the mouse seems to stick. But this is normal behaviour.
	// Try it: comment out the texture loading to make InitDeviceObjects() fast:
	// now the mouse does not stick and we get many WM_SIZE during resizing.
	// What we could do is to call InitDeviceObjects() only after the user has
	// released the left mouse-button or something like that, but that would be
	// a kludge.
	//MessageBox(m_hWnd,"InitDeviceObjects()","QQQ!",MB_OK);
	//MessageBeep(MB_ICONEXCLAMATION);
 
	D3DXInitialize();

	//D3DX_SURFACEFORMAT sf = D3DX_SF_UNKNOWN;

//    if( FAILED( hr = D3DXCreateTextureFromFile(
//		m_pd3dDevice,
//		NULL,                   // dwFlags
//		NULL,                   // auto-width
//		NULL,                   // auto-height
//		NULL,                   // auto-surface type
//		NULL,                   // pointer to Palette
//		&g_ptexPanorama1,        // returned pointer to texture
//		NULL,                   // returned number of mipmaps
//		"pan1.bmp",				// file name for texture
//		D3DX_FT_DEFAULT)))      // default scaling
//	{	
//		MessageBox(NULL,"Can't load panorama texture!","QQQ!",MB_OK);
//		char errStr[256];
//		D3DXGetErrorString(hr, 256, errStr);
//		MessageBox(NULL, errStr, "D3DX Error", MB_OK);
//	}
//
//	if( FAILED( hr = D3DXCreateTextureFromFile(
//		m_pd3dDevice,
//		NULL,                   // dwFlags
//		NULL,                   // auto-width
//		NULL,                   // auto-height
//		NULL,                   // auto-surface type
//		NULL,                   // pointer to Palette
//		&g_ptexPanorama2,        // returned pointer to texture
//		NULL,                   // returned number of mipmaps
//		"pan2.bmp",				// file name for texture
//		D3DX_FT_DEFAULT)))      // default scaling
//	{	
//		MessageBox(NULL,"Can't load panorama texture!","QQQ!",MB_OK);
//		char errStr[256];
//		D3DXGetErrorString(hr, 256, errStr);
//		MessageBox(NULL, errStr, "D3DX Error", MB_OK);
//	}


	// TEST:
	if( '\0' == g_strSceneryFilePath[0] ) {
		SetCurrentDirectory( "scenery\\The Wings\\" );
	} else {
		SetCurrentDirectory( g_strSceneryFilePath );
	}


	// Check if pan0.bmp file exists. If not, we assume bitmaps were not created
	// before and will return
	HANDLE hFile = CreateFile( "pan0.bmp", GENERIC_READ, FILE_SHARE_READ, NULL,
		OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	
	if ( hFile == INVALID_HANDLE_VALUE ) {
		// pan0.bmp not present
		return;
	}

	
	// threadless
//	// NOTE: comment out if we want a thread
//	if (g_bShowProgressDialog) {
//		// TEST: show progress dialog
//		g_hwndProgressDialog = CreateDialog( (HINSTANCE)GetWindowLong( m_hWnd, GWL_HINSTANCE ),
//			MAKEINTRESOURCE(IDD_PROGRESS2), m_hWnd,
//			(DLGPROC)ProgressProc );		
//		SetWindowText(g_hwndProgressDialog, "Loading Scenery...");
//	} else {
//		ShowWindow( g_hwndPB, SW_SHOW );
//	}

	
	int iProgress = 0;


	// Set progress bar range
	SendMessage( g_hwndPB, PBM_SETRANGE, (WPARAM)0, MAKELPARAM(1, 28) );
	SendDlgItemMessage( g_hwndProgressDialog, IDC_PROGRESS1, PBM_SETRANGE, (WPARAM)0, MAKELPARAM(1, 28) );
	
	iProgress++;
	SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 0|SBT_NOBORDERS, 
        (LPARAM) "Loading scenery..." );
	SendMessage( g_hwndPB, PBM_SETPOS, (WPARAM)iProgress++, (LPARAM)0 );

	SendDlgItemMessage( g_hwndProgressDialog, IDC_STATIC1, WM_SETTEXT, (WPARAM)0, 
        (LPARAM) "Loading scenery..." );
	SendDlgItemMessage( g_hwndProgressDialog, IDC_PROGRESS1, PBM_SETPOS, (WPARAM)iProgress, (LPARAM)0 );

	


	for (int i = 0; i < NUM_PAN; i++) {

		// let user cancel
//		if (g_bCancelLoadScenery) {
//			g_bCancelLoadScenery = false;
//			//g_bLoadSceneryFromThreadFinished = true;
//			return;
//		}

		TCHAR msg[512];
		lstrcpy(msg, "Loading ");
		lstrcat(msg, g_szPanName[i]);
		lstrcat(msg, "...");		

		iProgress++;
		SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 0|SBT_NOBORDERS, 
			(LPARAM) msg );
		SendMessage( g_hwndPB, PBM_SETPOS, (WPARAM)iProgress, (LPARAM)0 );

		SendDlgItemMessage( g_hwndProgressDialog, IDC_STATIC1, WM_SETTEXT, (WPARAM)0, 
			(LPARAM) msg );
		SendDlgItemMessage( g_hwndProgressDialog, IDC_PROGRESS1, PBM_SETPOS, (WPARAM)iProgress, (LPARAM)0 );



		if( FAILED( hr = D3DXCreateTextureFromFile(
			g_pd3dApp->m_pd3dDevice,
			NULL,                   // dwFlags
			NULL,                   // auto-width
			NULL,                   // auto-height
			NULL,                   // auto-surface type
			NULL,                   // pointer to Palette
			&g_ptexPan[i],			// returned pointer to texture
			NULL,                   // returned number of mipmaps
			g_szPanName[i],			// file name for texture
			D3DX_FT_DEFAULT)))      // default scaling
		{	
			MessageBox(NULL,"Can't load panorama texture!","QQQ!",MB_OK);
			char errStr[256];
			D3DXGetErrorString(hr, 256, errStr);
			MessageBox(NULL, errStr, "D3DX Error", MB_OK);
		}
	}

	SetCurrentDirectory( g_szRCSIMMediaPath );

	SendMessage(g_hwndStatus, SB_SETTEXT, (WPARAM) 0|SBT_NOBORDERS, 
        (LPARAM) "Ready" );
	SendMessage( g_hwndPB, PBM_SETPOS, (WPARAM)28, (LPARAM)0 );

	SendDlgItemMessage( g_hwndProgressDialog, IDC_STATIC1, WM_SETTEXT, (WPARAM)0, 
        (LPARAM) "Ready" );
	SendDlgItemMessage( g_hwndProgressDialog, IDC_PROGRESS1, PBM_SETPOS, (WPARAM)28, (LPARAM)0 );


	if (g_bShowProgressDialog) {
		// Close progress dialog
		if ( DestroyWindow( g_hwndProgressDialog ) ) {
			g_hwndProgressDialog = NULL;
		}
	} else {
		ShowWindow( g_hwndPB, SW_HIDE );
	}




//	D3DXLoadTextureFromFile(
//			m_pd3dDevice,
//			g_ptexPanorama,             // destination
//			D3DX_DEFAULT,               // all mip levels
//			"pan1.bmp",                 // source
//			NULL,                       // entire source
//			NULL,                       // entire destination
//			D3DX_FT_DEFAULT); 
	///////////////////////////////////////////////////////////

}




//-----------------------------------------------------------------------------
// Name: RenderScenery()
// Desc: 
// Note: Renders the panoramic scenery
//-----------------------------------------------------------------------------
VOID CMyD3DApplication::RenderScenery()
{

		// PANORAMA /////////////////////////////////////////////////////
		static int x = -8160;
		static int y = -200;

		// move panorama
		if ( GetKeyState(VK_SHIFT) & 0x80 ) {
			if ( GetKeyState('K') & 0x80 ) x--;
			if ( GetKeyState('O') & 0x80 ) y--;
		} else {
			if ( GetKeyState('K') & 0x80 ) x++;
			if ( GetKeyState('O') & 0x80 ) y++;	
		}

	
		// TEST:
//		g_bHelipad = 0;		
//		g_bTrees = 0;
//		g_bField = 0;
//		g_bRunway = 0;



//		DrawSquare( m_pd3dDevice, NULL, 
//					0-x, 0, 640-x, 480, 255, false );
//		DrawSquare( m_pd3dDevice, NULL, 
//					320-x, 0, 640, 480, 255, false );

		DDSURFACEDESC2 ddsd;
		ddsd.dwSize = sizeof(ddsd);
		ddsd.dwFlags = DDSD_HEIGHT | DDSD_WIDTH;
		g_pd3dApp->m_pddsRenderTarget->GetSurfaceDesc(&ddsd);

		DWORD dwRenderWidth  = ddsd.dwWidth;
		DWORD dwRenderHeight = ddsd.dwHeight;
		
		//float x2 = -(g_fCosAngle*700.0f);
		//float y2 = g_fSinAngle*700.0f;

		float x2 =  (g_fCamPanAngle *8160)/g_2_PI;
		float y2 = -(g_fCamTiltAngle*3060)/g_PI;

		// NOTE: When RECT is passed to the FillRect function, the rectangle is filled up to,
		// but not including, the right column and bottom row of pixels.
		// This means we must use 0-1024 to indicate 0-1023 (which is the real pixel range
		// of our 1024x1024 texture bitmap. But how is that with our square???


		
		D3DXVECTOR4 square1[4];
//		square1[0]=D3DXVECTOR4( 0   +x+x2,0+y+y2,		0.99999f,1.0f);
//		square1[1]=D3DXVECTOR4( 1024+x+x2,0+y+y2,		0.99999f,1.0f);
//		square1[2]=D3DXVECTOR4( 1024+x+x2,1024+y+y2,	0.99999f,1.0f);
//		square1[3]=D3DXVECTOR4( 0   +x+x2,1024+y+y2,	0.99999f,1.0f);

		RECT rect1;
		rect1.left = 0/*+x*/;
		rect1.top = 0/*+y*/;
		rect1.right = 1024/*+x*/;
		rect1.bottom = 1024/*+y*/;


//		D3DXVECTOR4 square2[4];
//		square2[0]=D3DXVECTOR4( 1024+x+x2,0+y+y2,		0.99999f,1.0f);
//		square2[1]=D3DXVECTOR4( 2048+x+x2,0+y+y2,		0.99999f,1.0f);
//		square2[2]=D3DXVECTOR4( 2048+x+x2,1024+y+y2,	0.99999f,1.0f);
//		square2[3]=D3DXVECTOR4( 1024+x+x2,1024+y+y2,	0.99999f,1.0f);
//
//		RECT rect2;
//		rect2.left = 0/*+x*/;
//		rect2.top = 0/*+y*/;
//		rect2.right = 1024/*+x*/;
//		rect2.bottom = 1024/*+y*/;

		// NOTE: we must use texture created by D3DXCreateTextureFromFile()
		// If we use texture created by D3DTextr_GetSurface() it will not be sampled
		// correctly and show banding. We need the exact bitmap.
		// Well, even D3DXCreateTextureFromFile() does not give the exact bitmap...
		//
		//D3DXDrawSprite3D( D3DTextr_GetSurface( "pan1.bmp" ), m_pd3dDevice,
		//	square1, 1.0f, &rect1);

		// Yyyyyeeeesssss!!! We got to turn on point-sampling to get the exact bitmap.
		// Note however that with point-sampling we have a perfect bitmap when square1
		// and rect1 have the same dimensions, but a not so perfect bitmap when they differ.
		// With linear sampling we have an intermediate quality for all dimensions.
		// We can circumvent this by always keeping square1 and rect1 the same dimension.
		// This however asks for compensation of the panorama positioning when the user
		// switches resolution or resizes the window.
		// NOTE: we should not use textures larger that 1024x1024. When reaching the end of
		// a panorama texture we should show a new one which takes it from there.
		// Note that, contrary to DrawPrimitive() with D3DTLVERTEX, we can move square1 to
		// e.g. -10, -10. Well, that's also possible with DrawPrimitive() with D3DTLVERTEX if
		// we use the D3DDP_DONOTCLIP flag. Note: D3DXDrawSprite3D() just calls DrawPrimitive().
		// This is important for sliding in the next panorama texture.
		// Note: now that we can fully slide square1, it is probably better to slide
		// that instead of sliding rect1. That way we can keep the textures small.
		// NOTE: Reflex uses one giant .jpeg panorama file of 8160x3060. We can get that into
		// 8x1024=8192 and 3x1024=3072. So we need 8x3=24 textures of 1024x1024.
		// Reflex seems to use an overlap of two pixels: 
		// 8192-8160=32, 32/8=4, 4/2=2 pixels overlap horizontally left and right
		// 3072-3060=12, 12/3=4, 4/2=2 pixels overlap vertically above and below
		// Doing this overlap prevents the stitches from being visible when doing
		// linear texture sampling for the panorama (we always use point-sampling for
		// the panorama so our stitches were not visible in the first place)
		// It would be nice if we could:
		// * Load the .jpeg directly instead of having to convert to a .bmp file
		// * Keep the giant panorama intact, load it into memory, and from memory load
		//   1024x1024 parts into texture surfaces
		// Also nice: since the R/C View camera controls are not much use when using
		// panoramas we might rewire them to make the pilot look round and up/down.
		// This option we could also retain in normal 3D scenery.
		// NOTE: we must align the terrain's 0-360 deg. (0-2pi) with the panorama's
		// 0-8192. The terrain's horizon should always be aligned with/parallel to the
		// panorama's horizon.
		//
		// 0	1024	4080	6120	8160	panorama x
		// 0	90		180		270		360		camera pan
		//
		// camera tilt	panorama y
		// 0			0
		// 90			1530
		// 180			3060
		//
		// NOTE: Direct3D uses the current viewport parameters (the dwX, dwY, dwWidth, and
		// dwHeight members of the D3DVIEWPORT7 structure) to clip D3DTLVERTEX vertices. The
		// system always clips z-coordinates to [0, 1]. To prevent the system from clipping
		// these vertices, use the D3DDP_DONOTCLIP flag when calling rendering methods.
		// NOTE: From the DirectX 9.0 SDK docs:
		// For the best performance when using D3DXCreateTextureFromFile: 
		// Doing image scaling and format conversion at load time can be slow. Store images
		// in the format and resolution they will be used. If the target hardware requires
		// power of two dimensions, create and store images using power of two dimensions.
		// NOTE: Don't call D3DXPrepareDeviceForSprite() as that will set linear texture
		// filtering.
		// TODO:
		// * Solve issue related to panoramic engine: sometimes the panoramic textures are 
		// flickering. Could this be caused by Z-fighting? Maybe we should use DrawPrimitive()
		// instead of D3DXDrawSprite3D()
		// DONE: use m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, FALSE );
		// * Solve issue related to panoramic engine: maybe we should use a pilot proxy that
		// prevents the heli from flying through the pilot, thus preventing the panoramic hole
		// at the ground from being visible
		// * Solve issue related to panoramic engine: on WM_SIZE the mouse sticks to the window
		// border and we go through InitDeviceObjects() twice. This was already an issue but
		// now it is very noticable as the panoramic textures take a long time to create
		// DONE: this is normal behaviour: many WM_SIZE are sent when resizing, not just two
		// See: remarks in MsgProc() at WM_SIZE and InitDEviceObjects()
		// * Solve issue related to panoramic engine: sometimes Screenshot2() creates all-black
		// .bmp files. DrawImage2() seems to be the culprit
		//
		// Save old values
		DWORD dwMagFilter;
		DWORD dwMinFilter;
		m_pd3dDevice->GetTextureStageState( 0, D3DTSS_MAGFILTER, &dwMagFilter );
		m_pd3dDevice->GetTextureStageState( 0, D3DTSS_MINFILTER, &dwMinFilter );

		// Set point-sampling for the panorama to get exact bitmap
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, D3DTFG_POINT );
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, D3DTFN_POINT );

		// Maybe this solves the flickering? Yes! Well, most of it is gone
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, FALSE );


		// Draw panorama
		int x3 = 0;
		int y3 = 0;

		for (int i = 0; i < NUM_PAN; i++) {
			square1[0]=D3DXVECTOR4( (x3		+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[1]=D3DXVECTOR4( (x3+1024+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);
			square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);

			if (i>15) {
				square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				rect1.bottom = 1012;
			}

			D3DXDrawSprite3D( g_ptexPan[i], m_pd3dDevice,
				square1, 0.1f, &rect1);

			x3 += 1024;
			if (i==7)  {x3 = 0; y3 = 1024;}
			if (i==15) {x3 = 0; y3 = 2048;}
		};

		// Wrap around: stitch first three vertical textures at the end
		// i.e. pan0, pan8, pan16
		x3 = 8160;
		y3 = 0;
		rect1.bottom = 1024;
		for (i = 0; i < NUM_PAN; i+=8) {
			square1[0]=D3DXVECTOR4( (x3		+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[1]=D3DXVECTOR4( (x3+1024+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);
			square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);

			if (i>15) {
				square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				rect1.bottom = 1012;
			}

			D3DXDrawSprite3D( g_ptexPan[i], m_pd3dDevice,
				square1, 1.0f, &rect1);

			if (i==0) {y3 = 1024;}
			if (i==8) {y3 = 2048;}
		}

		// Wrap some more: an extra strip for resolutions bigger than 1024x768
		// i.e. pan1, pan9, pan17
		x3 = 8160+1024;
		y3 = 0;
		rect1.bottom = 1024;
		for (i = 1; i < NUM_PAN; i+=8) {
			square1[0]=D3DXVECTOR4( (x3		+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[1]=D3DXVECTOR4( (x3+1024+x2)+x, (y3		+y2)+y,	0.99999f,1.0f);
			square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);
			square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1024+y2)+y,	0.99999f,1.0f);

			if (i>15) {
				square1[2]=D3DXVECTOR4( (x3+1024+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				square1[3]=D3DXVECTOR4( (x3		+x2)+x, (y3+1012+y2)+y,	0.99999f,1.0f);
				rect1.bottom = 1012;
			}

			D3DXDrawSprite3D( g_ptexPan[i], m_pd3dDevice,
				square1, 1.0f, &rect1);

			if (i==1) {y3 = 1024;}
			if (i==9) {y3 = 2048;}
		}


		// Restore old values
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MAGFILTER, dwMagFilter );
		m_pd3dDevice->SetTextureStageState( 0, D3DTSS_MINFILTER, dwMinFilter );
		
		m_pd3dDevice->SetRenderState( D3DRENDERSTATE_ZENABLE, TRUE );
		//////////////////////////////////////////////////////////////////////

}








//////////////////////////////////////////
// helper function
bool IsPower2(int x)
{
	for (int i=1; i+=i; i<=2048)
		if (x==i) return true;

	return false;
}


//////////////////////////////////////////
// helper function
// NOTE: Not necessary: MSVC++ already has _strrev()
// But what a cool algorithm.
// Reverse a string in-place.
char *my_strrev(char *string) { 
	
	char *original = string; 
	char *forward = string; 
	char temp; 
	
	while(*string) 
	{ 
		string++; 
	}	
	
	while(forward<string) 
	{ 
		temp=*(--string); 
		*string=*forward; 
		*forward++=temp; 
	}	
	
	return (original); 
} 


//////////////////////////////////////////
// ErrMsgBox
//      Displays a MsgBox with the error message for GetLastError.
//      Uses FormatMessage to retrieve the message.
//      The code was taken from the FormatMessage help topic.
//
void ErrMsgBox()
{
    LPVOID lpMsgBuf;

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM,
        NULL,
        GetLastError(),
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), // Default language
        (LPTSTR) &lpMsgBuf,
        0,
        NULL
    );

    // Display the string.
    MessageBox(NULL, (char*)lpMsgBuf, "GetLastError", MB_OK|MB_ICONINFORMATION);

    // Free the buffer.
    LocalFree( lpMsgBuf );
}



// PANORAMA
//-----------------------------------------------------------------------------
// Name: DrawSquare()
// Desc: Draws a textured square which can be used for fade-in/fade out.
// Note: We must be in a scene.
// Note: Must use D3DDP_DONOTCLIP flag to be able to move square1 to e.g. -10, -10
//		 and get the same functionality as D3DXDrawSprite3D().
//-----------------------------------------------------------------------------
void DrawSquare( LPDIRECT3DDEVICE7 pdev, LPDIRECTDRAWSURFACE7 pTex, 
					WORD x, WORD y, WORD x2, WORD y2, BYTE factor, bool clear )
{
	D3DTLVERTEX square1[4];
	//D3DLVERTEX square1[4];
//	square1[0]=D3DTLVERTEX(D3DVECTOR(x,y,0.0),  1.0,RGBA_MAKE(255,255,255,factor),0,0,0);
//	square1[1]=D3DTLVERTEX(D3DVECTOR(x2,y,0.0), 1.0,RGBA_MAKE(255,255,255,factor),0,1,0);
//	square1[2]=D3DTLVERTEX(D3DVECTOR(x,y2,0.0), 1.0,RGBA_MAKE(255,255,255,factor),0,0,1);
//	square1[3]=D3DTLVERTEX(D3DVECTOR(x2,y2,0.0),1.0,RGBA_MAKE(255,255,255,factor),0,1,1);

	// we zetten de square zover mogelijk naar achter zodat PP1 ervoor blijft
	// NOTE: The largest allowable value for dvSZ is 0.99999 if you want the vertex to be 
	// within the range of z-values that are displayed
	// Z-values are in device space rather than camera space
	square1[0]=D3DTLVERTEX( D3DVECTOR(x,y,  0.99999f), 1.0,RGBA_MAKE(255,255,255,factor),0,0,0);
	square1[1]=D3DTLVERTEX( D3DVECTOR(x2,y, 0.99999f), 1.0,RGBA_MAKE(255,255,255,factor),0,1,0);
	square1[2]=D3DTLVERTEX( D3DVECTOR(x,y2, 0.99999f), 1.0,RGBA_MAKE(255,255,255,factor),0,0,1);
	square1[3]=D3DTLVERTEX( D3DVECTOR(x2,y2,0.99999f), 1.0,RGBA_MAKE(255,255,255,factor),0,1,1);


//	square1[0]=D3DLVERTEX( D3DVECTOR(x,y,  0.99999f), RGBA_MAKE(255,255,255,factor),0,0,0);
//	square1[1]=D3DLVERTEX( D3DVECTOR(x2,y, 0.99999f), RGBA_MAKE(255,255,255,factor),0,1,0);
//	square1[2]=D3DLVERTEX( D3DVECTOR(x,y2, 0.99999f), RGBA_MAKE(255,255,255,factor),0,0,1);
//	square1[3]=D3DLVERTEX( D3DVECTOR(x2,y2,0.99999f), RGBA_MAKE(255,255,255,factor),0,1,1);


//	D3DXMATRIX matProjection;
//	D3DXMatrixPerspectiveLH(&matProjection, 1.0f, 1.0f, 1.0f, 1000.0f);
//	pdev->SetTransform( D3DTRANSFORMSTATE_PROJECTION, (_D3DMATRIX *)&matProjection );

//	D3DMATRIX matIdentity;
//	D3DUtil_SetIdentityMatrix( matIdentity );
//	pdev->SetTransform( D3DTRANSFORMSTATE_WORLD, &matIdentity );



	if (clear==true) pdev->Clear(0,NULL,D3DCLEAR_TARGET,0,0.0f,0);

	pdev->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE,true);
	pdev->SetRenderState(D3DRENDERSTATE_SRCBLEND,D3DBLEND_SRCALPHA);
    pdev->SetRenderState(D3DRENDERSTATE_DESTBLEND,D3DBLEND_INVSRCALPHA);
	pdev->SetTextureStageState(0,D3DTSS_ALPHAOP,D3DTOP_MODULATE);

	// Panorama should not be using the zbuffer, or should it
	//pdev->SetRenderState( D3DRENDERSTATE_ZENABLE, FALSE );
	
	pdev->SetTexture(0,pTex);	
	pdev->SetTexture( 0, D3DTextr_GetSurface( "pan1.bmp" ) );	
	//pdev->SetTexture( 0, g_ptexPanorama );	
	

	//if( SUCCEEDED( pdev->BeginScene() ) ) {


// This has no use for TLVERTEX
//		D3DMATRIX matScale;
//		D3DUtil_SetScaleMatrix(matScale, 1.0f, 1.0f, 1.0f);		
//		D3DMATRIX matTrans;
//		D3DUtil_SetTranslateMatrix(matTrans, 0.0f, 0.0f, -120.0f);		
//		D3DMATRIX matRot;
//		D3DUtil_SetRotateYMatrix(matRot, 3.14f);	
//		D3DMATRIX matAll;
//		D3DMath_MatrixMultiply(matAll, matScale, matTrans);
//		D3DMath_MatrixMultiply(matAll, matAll, matRot);
//
//		pdev->SetTransform( D3DTRANSFORMSTATE_WORLD, &matAll );
//	
//
//		// enable and apply texture coordinate transformation 
//		pdev->SetTextureStageState( 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_COUNT2);
//		pdev->SetTransform(D3DTRANSFORMSTATE_TEXTURE0, &matAll);

	
	
		
		pdev->DrawPrimitive(D3DPT_TRIANGLESTRIP,D3DFVF_TLVERTEX,square1,4,0);
		//pdev->DrawPrimitive(D3DPT_TRIANGLESTRIP,D3DFVF_LVERTEX,square1,4,0);
		
		pdev->SetTexture(0,NULL);	
		pdev->SetRenderState(D3DRENDERSTATE_ALPHABLENDENABLE,false);

		//pdev->SetRenderState( D3DRENDERSTATE_ZENABLE, TRUE );
		pdev->SetTextureStageState( 0, D3DTSS_TEXTURETRANSFORMFLAGS, D3DTTFF_DISABLE);
		
		// Restore texture to no texture
		//pdev->SetTexture( 0, NULL );
		
	//	pdev->EndScene();
	//}
}

