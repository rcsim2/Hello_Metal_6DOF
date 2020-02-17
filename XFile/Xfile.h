//-----------------------------------------------------------------------------
// File: XFile.h
//
// Desc: R/C Sim Sikorsky: an R/C heli simulator
//		 This also shows how to load .X files in D3DIM.
//		 XFile.h is the master header for the project.
//
// Note: This code uses the D3D Framework helper library.
//
//
// Copyright (c) 2000 Black Sphere Corp.
//-----------------------------------------------------------------------------
#pragma once
#ifndef __XFILE_H
#define __XFILE_H


//#include <afxwin.h>
// NOTE: From afxv_w32.h:  MFC apps must not #include <windows.h>
// Can we use MFC in an SDK app???


// NOTE NOTE NOTE: not defining STRICT can cause LNK2001 Errors!!!
//
// From the docs: Enabling STRICT Type Checking 
// Note: If you are writing a C++ application, you don't have the option of applying
// STRICT to only some of your source files. Because of the way C++ "type safe linking"
// works, mixing STRICT and non-STRICT source files in your application can cause 
// linking errors.
//
// So instead of in every source file, it would be better to define STRICT in 
// Project|Settings|C/C++|General|Preprocessor Definitions


///////////////////////////////////////////////////////////////////////////////
// NOTE: We are now using a precompiled header file (precompiled.h) which #includes
// all the system header files and project header files that hardly ever change.
//
// NOTE2: Idempotency of header files means they cannot be inserted twice or more IN THE
// SAME SOURCE FILE. It does not mean that they cannot be inserted twice or more in the
// project. For instance, if one source file #includes windows.h, then windows.h will
// only be included once in that source file. Another source file in the project, however,
// can still #include windows.h (note: windows.h is idempotent).
// This means that projects in which many source files #include windows.h take a long time
// to build because windows.h will have to be compiled many times. The solution is to use a
// precompiled header file which includes all the system headers only once, and then let
// the project's source files use this precompiled header. Here's how to do it:
//
// 1. Add two files to the project: precompiled.h and precompiled.cpp
// 2. #include all the system header files and project header files that hardly ever 
//	  change into precompiled.h
// 3. #include precompiled.h in precompiled.cpp
// 4. In Project|Settings choose Settings For: All Configurations
// 5. Select XFile|Source Files|precompiled.cpp
// 6. In C/C++|Precompiled Headers choose Create precompiled header file (.pch)
// 7. Specify Through header: precompiled.h
// 8. Select XFile
// 9. In C/C++|Precompiled Headers choose Use precompiled header file (.pch)
// 10. Specify Through header: precompiled.h
// 11. Click OK
// 12. In all the project's source files put #include precompiled.h as the first
//	   non-comment line
//
// See: http://www.cygnus-software.com/papers/precompiledheaders.html
//
// NOTE3: The build of the precompiled header takes long but after that's done
// builds are very fast.
//
// NOTE4: All headers (except precompiled.h) should not #include anything.
// All source files should #include precompiled.h and their own header, and other
// headers they need which are not in precompiled.h
//
// NOTE5: We are also #including all our project headers. This is far from optimal:
// When we change something in a header the entire .pch has to be rebuilt. We should
// do a better physical design of our project: precompiled.h should only #include
// all system headers and project headers that hardly ever change. It should not
// #include project headers that change frequently.
// For instance: when dynamics.h/dynamics.cpp is under development we should not
// #include dynamics.h in precompiled.h. Those source files that need dynamics.h 
// (like xfile.cpp) should #include dynamics.h themselves. This way when a change 
// is made in dynamics.h only two files have to be rebuilt (dynamics.cpp and xfile.cpp).
// This greatly increases build time.
//
// NOTE6: If you have small header files that don't include much, there is no advantage
// to putting them in the precompiled header file. If you have a large header file that
// is only included from a few places you shouldn't put it in your precompiled header 
// file because this will cause more total rebuilds. Finally, if you have a large header
// file that is needed everywhere then you need to fix the physical design of your 
// program - and you can't do that if you are including this file from your precompiled
// header file.
//
// NOTE7: Most important is to avoid rebuiling the pch. So never put headers
// that change often in precompiled.h
//
// NOTE8: resource.h has //{{NO_DEPENDENCIES}} as the first line. This means the pch will
// not be rebuilt when resource.h changes. Remove this line in resource.h
// Of course, when resource.h changes a lot it's better not to #include it in precompiled.h
// Well, it seems that even after removing the line in resource.h there is never a rebuilt of
// files that #include resource.h when resource.h changes.
///////////////////////////////////////////////////////////////////////////////




//#define STRICT
//
//#include <windows.h>
//#include <windowsx.h>
//#include <prsht.h>
//#include <commdlg.h>
//#include <commctrl.h>
//#include <math.h>
//#include <stdio.h>
//#include <time.h>
//#include <process.h>
//#include <tchar.h>
//#include <mmsystem.h>
//#include <vfw.h >
//#include <d3dx.h>
//#include <d3dxmath.h>
//#include <htmlhelp.h>
//#include <shlobj.h>



//#include "D3DApp.h"
//#include "D3DTextr.h"
//#include "D3DUtil.h"
//#include "D3DMath.h"
//#include "D3DFile.h"
//
//
//#include "LimitSingleInstance.h"
//
//
//#include "resource.h"
//#include "input.h"
//#include "clensflare.h"
//#include "propsheet.h"
//#include "proppage.h"
//#include "rtconfig.h"
//#include "config.h"
//#include "tabdlg.h"
//#include "about.h"
//#include "dxverify.h"
//#include "vxd.h"
//#include "wdm.h"
//#include "screenshot.h"
//#include "dialog.h"
//#include "statbar.h"
//#include "shareware.h"
//#include "registry.h"
//#include "tooltip.h"
//#include "calibration.h"
//#include "flightrec.h"
//#include "fsif.h"
//#include "search.h"
//#include "frameindex.h"
//#include "parse.h"
//#include "dynamics.h"
//
////#include "ctxhelp.h"
//
//#include "Play3DSound.h"
//
//#include "ParticleEng.h"
//
//#include "loadxrm.h"




extern int test;


///////////////////////////////////////////////////////////////////////////////
// .h header/interface										// declarations
// .cpp source/implementation (module, translation unit)	// definitions
//
// We hebben nogal wat overbodige declaraties laten staan, e.g.:
//
// xfile.h						rtconfig.h
// 
// extern int a;				extern int a;				// declaration
// int f(int x);				int f(int x);				// declaration
//
// xfile.cpp					rtconfig.cpp
//
// #include "rtconfig.h"		#include "xfile.h"
// #include "xfile.h"			#include "rtconfig.h"
//
// int a = 0;					int a = 0;					// error: multiple definition
// int f(int x);				int f(int x);				// declaration		
//
// int f(int x) { return x; }	int f(int x) { return x; }	// error: multiple definition
//
//
// Op zich zijn deze dubbele of zelfs vierdubbele declaraties van dezelfde
// variabele of functie geen probleem. (Dubbele definities zijn natuurlijk fout.)
// Maar het zou voldoende zijn om b.v. in xfile.h int f(int x); te declareren en
// in xfile.cpp int f(int x) { return x; } te definieren. Vervolgens zou 
// #include "xfile.h" genoeg zijn om f() te kunnen gebruiken in rtconfig.cpp.
//
//
// extern int a;				// declaration
// int a;						// definition
// int a = 0;					// definition
// extern int a = 0;			// definition
//
// int f(int x);				// declaration
// int f(int x) { return x; };	// definition
///////////////////////////////////////////////////////////////////////////////





//-----------------------------------------------------------------------------
// Global variables and functions
//-----------------------------------------------------------------------------

// demo
// NOTE: better do this in Project Settings, Preprocessor definitions and
// add a separate Demo Configuration
//#define DEMO


// R/C Sim Sikorsky Registry key
extern char g_szRCSIMRegistryKey[512];

// R/C Sim Sikorsky program directory (set by InstallShield)
extern char g_szRCSIMProgramPath[512];

// our media path
extern char g_szRCSIMMediaPath[512];


// initial model position/orientation
extern float INIT_X;
extern float INIT_Y; // grounded == -8.5f;
extern float INIT_Z;
extern float INIT_RADS_Y; // nose-in==0.0f nose-out==g_PI

// initial collective and weight, etc.
extern float INIT_COLLECTIVE;
extern float INIT_WEIGHT;
extern float INIT_TORQUE;


// pilot positions
// add 8.5f to get correct Y values
extern float PILOTPOSITION1_X;
extern float PILOTPOSITION1_Y;
extern float PILOTPOSITION1_Z;

extern float PILOTPOSITION2_X;
extern float PILOTPOSITION2_Y;
extern float PILOTPOSITION2_Z;

extern float PILOTPOSITION3_X;
extern float PILOTPOSITION3_Y;
extern float PILOTPOSITION3_Z;

extern float PILOTPOSITION4_X;
extern float PILOTPOSITION4_Y;
extern float PILOTPOSITION4_Z;

extern int g_iPPMarkSize;

extern bool g_bShowPP1Mark;
extern bool g_bShowPP2Mark;
extern bool g_bShowPP3Mark;
extern bool g_bShowPP4Mark;

extern bool g_bDrawPPMarkShadows;




// use nifty helis
extern bool g_bBell;
extern bool g_bCobra;
extern bool g_bCougar;

// for direct input
#define WM_SYNCACQUIRE (WM_USER+1)


// for lens flare ////////////////////////////////////////////////////////////
// NOTE: on the Voodoo5 5500 PCI lensflare causes a crash
// On the SiS 6326 AGP lensflare works fine, also on the S3 Virge PCI
// So it is not a programming error by us
// Probably, the bug is in D3DIM700.DLL
// Hoewel...: alle lensflare Samples werken OK onder de Voodoo...

//LPDIRECT3D7       g_pD3D         = NULL;
extern LPDIRECT3DDEVICE7 g_pD3DDevice;

extern D3DDEVICEDESC7 g_D3DDeviceDesc;

//
// Textures
//

extern char g_szPath[512];

enum {
    TEX_FLARE0, TEX_FLARE1, TEX_FLARE2, TEX_FLARE3,
    NUM_TEXTURES
};

extern char *g_szTexName[NUM_TEXTURES];

extern LPDIRECTDRAWSURFACE7 g_ppTex[NUM_TEXTURES];


//
// Light source
//

extern D3DXCOLOR g_colorLight;
extern D3DXVECTOR4 g_vecLight;

//
// Lens Flare
//

extern BOOL g_bCapsLensFlare;
extern BOOL g_bDrawLensFlare;
extern CLensFlare *g_pLensFlare;

extern D3DXMATRIX g_matPosition;
extern D3DXMATRIX g_matIdentity;

extern BOOL g_bSun;
extern BOOL g_bFlare;

///////////////////////////////////////////////////////////////////////////////


// for a wonderful kludge
extern char msg1[512];
extern char msg2[512];
extern char msg3[512];

extern char msg4[512];
extern char msg5[512];
extern char msg6[512];

extern char msg7[512];
extern char msg8[512];
extern char msg9[512];

extern char msg10[512];
extern char msg11[512];


extern bool g_bShowValues;





// for god mode dialog box
BOOL CALLBACK GodModeProc( HWND, UINT, WPARAM, LPARAM );

extern bool g_bGodModeOn;

// for console dialog box
BOOL CALLBACK ConsoleProc( HWND, UINT, WPARAM, LPARAM );

// for preferences dialog box
BOOL CALLBACK PreferencesProc( HWND, UINT, WPARAM, LPARAM );


// for flight info
extern COLORREF g_crTextColor;
extern int g_iAirSpeedUnit;
extern int g_iAltitudeUnit;


// VxD
#define VMYXD_APIFUNC_1 1
#define VMYXD_APIFUNC_2 2

extern HANDLE  g_hVxD;

// use int instead of DWORD: DWORD is unsigned causing signed/unsigned probs
// or use __int32 (synonymous with int) to make it clear it is 32 bits wide
extern DWORD	cbBytesReturned;
extern int dwErrorCode;
//DWORD   RetInfo[13];	// 1 Sync Pulse + 10 Channels + 1 IntCount + 1 PulseLength
extern int RetInfo[88]; 	// big sucker to test

// TODO: calibrate
extern int RetInfoMax[11];
extern int RetInfoMin[11];

extern int IntCount;
extern int ChannelTotal;

extern CHAR*   strVxDFileName;
extern CHAR    lpBuffer[MAX_PATH];
extern CHAR    strVxDFilePath[MAX_PATH];

extern bool LoadVxD();
extern bool UnLoadVxD();
// NOTE: don't give the user the option to load/unload the VxD during app session.
// Only load at app start and unload at app exit. It may be a dynaload VxD but
// too much loading/unloading will only introduce potential instability.
// We can give the user the option to load or not to load the VxD during app start.
// (On the basis of that option, which must be a registry value, we should
// enable/disable the Control->Transmitter menu item}.


// WDM
// I/O Control Codes
#define IOCTL_TXINTPAR_OPEN \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x800, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )

#define IOCTL_TXINTPAR_CLOSE \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x801, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )
         
#define IOCTL_TXINTPAR_FREE \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x802, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )

#define IOCTL_TXINTPAR_INIT \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x803, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )

#define IOCTL_TXINTPAR_READ \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x804, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )

#define IOCTL_TXINTPAR_WRITE \
   CTL_CODE(FILE_DEVICE_PARALLEL_PORT, 0x805, METHOD_BUFFERED, \
         FILE_ANY_ACCESS )

extern HANDLE  g_hWDM;

extern bool LoadWDM();
extern bool UnLoadWDM();

extern bool g_bShowDriverVersionInfo;

	

// smooth controls
extern D3DXVECTOR3 g_vecVelocity;
extern D3DXVECTOR3 g_vecAngularVelocity;

extern float g_fSpeed;
extern float g_fAngularSpeed;

extern bool g_bFirstFrame;


// smooth cam controls
extern D3DXVECTOR3 g_vecCamVelocity;
extern D3DXVECTOR3 g_vecAngularCamVelocity;

extern float g_fCamSpeed;
extern float g_fAngularCamSpeed;



// always handy
extern FLOAT GetFPS();
extern float g_fFPS;
extern float g_fFPSLimit;
extern float g_fFrameDelay;
extern bool g_bAutoFrameRate;
extern bool g_bNewFPSLimitSet;

// motion
extern float g_fMotionRate;
extern float g_fVibration;
extern float g_fTurbulence;


// reset latency arrays
extern bool g_bResetLatency;

// for view latencies
#define MAXLATENCY 100

// for R/C view latency
extern int g_iLat;
extern D3DMATRIX g_matOld[MAXLATENCY+3];

// for Chase view latency
extern int g_iLat2;
extern D3DMATRIX g_matOld2[MAXLATENCY+3];

// for Follow view latency
extern int g_iLat3;
extern D3DMATRIX g_matOld3[MAXLATENCY+3];




// for registry window sizing
// TODO: must make this a reg value when this becomes a user option
// DONE
// NOTE: this is a registry value!!!
// Let's make it a DWORD instead of bool to prevent C4800 warnings
extern DWORD g_bRegWindowSize;
extern DWORD g_bRegFullScreen;
extern DWORD g_bRegSplashScreen;

extern DWORD SPLASHSCREEN;

// For the Preferences Dialog Box
VOID UpdateDialogControls( HWND hDlg, D3DEnum_DeviceInfo* pCurrentDevice,
                                  DWORD dwCurrentMode );

// for registry initial fullscreen mode
extern DWORD g_dwRegModeWidth;
extern DWORD g_dwRegModeHeight;
extern DWORD g_dwRegModeRGBBitCount;


// for the configuration property sheet
extern HINSTANCE g_hinst;
VOID DoPropertySheet(HWND hwndOwner);
int CALLBACK ConfigurationProc( HWND hwndDlg, UINT uMsg, LPARAM lParam );

// property sheets won't center by themselves so we subclass
#define WM_PROPSHEETCENTER (WM_USER+2)
extern WNDPROC wpOrigPropSheetProc;
LRESULT APIENTRY PropSheetSubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );

// VxD
// reg vars for the VxD
extern DWORD g_bRegLoadVxD;
extern DWORD g_dwRegWelcomeVxD;
extern DWORD g_dwRegGoodbyeVxD;

extern bool g_bLoadedVxD;

// WDM
extern DWORD g_bRegLoadWDM;

extern bool g_bLoadedWDM;
	

// DialogProc for prop pages
BOOL CALLBACK  PropPageWindowProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageWDMProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageVxDProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPagePositionProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageAltitudeProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageControlsProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageCameraProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageZoomProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageViewProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageSkyProc		( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageTerrainProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageSoundProc	( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageFlightRecProc( HWND, UINT, WPARAM, LPARAM );
BOOL CALLBACK  PropPageFPSProc		( HWND, UINT, WPARAM, LPARAM );





#define COLOR_BLUE 	RGB(0,0,255)
#define COLOR_BLACK	RGB(0,0,0)
#define COLOR_RED	RGB(255,0,0)
#define COLOR_WHITE	RGB(255,255,255)
#define COLOR_DARKBLUE 	RGB(0,0,127)
#define COLOR_GRAY	RGB(192,192,192)
#define COLOR_LIGHTGRAY	RGB(245,245,245)
#define COLOR_DARKGRAY	RGB(128,128,128)


// need this one in Window prop page
extern D3DEnum_DeviceInfo*  g_pDeviceInfo;


// TODO: fix bugs (they are not mine but the framework's)
// 1. Load xfile second time in opendialog will show no textures
// 2. When mouse control in fullscreen and going back to windowed gives
//    no mouse cursor when going to keyboard or joystick control

// TODO: implement zoom a la Mortyr


// for zoom
extern bool g_bZoomIn;
extern bool g_bZooming;
extern float g_fZoomValue;


// for landing
extern bool g_bLanded;



// all this crap to get spiffy sliders
extern WNDPROC wpOrigTrackBar1Proc;
extern WNDPROC wpOrigTrackBar2Proc;
extern WNDPROC wpOrigTrackBar3Proc;
extern WNDPROC wpOrigTrackBar4Proc;
extern WNDPROC wpOrigTrackBar5Proc;
extern WNDPROC wpOrigTrackBar6Proc;

LRESULT APIENTRY TrackBar1SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar2SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar3SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar4SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar5SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam); 
LRESULT APIENTRY TrackBar6SubclassProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);

extern int g_iDelta;

extern int iMargin1;
extern int iMargin2;
extern int iMargin3;
extern int iMargin4;
extern int iMargin5;
extern int iMargin6;

extern int iSliderPos1;
extern int iSliderPos2;
extern int iSliderPos3;
extern int iSliderPos4;
extern int iSliderPos5;
extern int iSliderPos6;

extern int iOldSliderPos1;
extern int iOldSliderPos2;
extern int iOldSliderPos3;
extern int iOldSliderPos4;
extern int iOldSliderPos5;
extern int iOldSliderPos6;

extern int iPreviousSliderPos1;
extern int iPreviousSliderPos2;
extern int iPreviousSliderPos3;
extern int iPreviousSliderPos4;
extern int iPreviousSliderPos5;
extern int iPreviousSliderPos6;

extern bool bCtrlSliders123Locked;
extern bool bCtrlSliders456Locked;
extern bool bCamSliders123Locked;
extern bool bCamSliders456Locked;
extern bool bZoomSliders12Locked;

// controls sensitivity
extern float CTRL_X_SENS;
extern float CTRL_Y_SENS;
extern float CTRL_Z_SENS;
extern float CTRL_RADS_X_SENS;
extern float CTRL_RADS_Y_SENS;
extern float CTRL_RADS_Z_SENS;


// camera sensitivity
extern float CAM_X_SENS;
extern float CAM_Y_SENS;
extern float CAM_Z_SENS;
extern float CAM_RADS_X_SENS;
extern float CAM_RADS_Y_SENS;
extern float CAM_RADS_Z_SENS;


// zoom sensitivity
extern float ZOOM_IN_SENS;
extern float ZOOM_OUT_SENS;



// altitude warning
extern bool g_bAltitudeWarning ;
extern bool g_bAltitudeWarningSound;
extern bool g_bAltitudeWarningSpeaker;
extern float ALTITUDE_WARNING_Y;


// sound file
extern TCHAR g_strSoundFileName[512];
extern TCHAR g_strSoundFilePath[512];
extern HRESULT OpenSoundFileDialog(HWND hWnd);


// sky texture
extern TCHAR g_strTextureFileName[512];
extern TCHAR g_strTextureFilePath[512];
extern HRESULT OpenTextureFileDialog(HWND hWnd);

bool IsPower2(int x);


// heli file
extern TCHAR g_strHeliFileName[512];
extern TCHAR g_strHeliFilePath[512];
extern TCHAR g_strHeliFileOld[512];
extern TCHAR g_strHeliFileTextrName[512];


// sky file
extern TCHAR g_strSkyFileName[512];
extern TCHAR g_strSkyFilePath[512];
extern TCHAR g_strSkyFileOld[512];
extern TCHAR g_strSkyFileTextrName[512];

extern HRESULT OpenSkyFileDialog(HWND hWnd);

extern BOOL g_bShowSkyFile;


// terrain file
extern TCHAR g_strTerrainFileName[512];
extern TCHAR g_strTerrainFilePath[512];
extern TCHAR g_strTerrainFileOld[512];
extern TCHAR g_strTerrainFileTextrName[512];

extern HRESULT OpenTerrainFileDialog(HWND hWnd);


// flight file
extern TCHAR g_strFlightFileName[512];
extern TCHAR g_strFlightFilePath[512];
extern TCHAR g_strFlightFileOld[512];

HRESULT OpenFlightFileDialog(HWND hWnd);
HRESULT SaveFlightFileDialog(HWND hWnd);


// calibration file
extern TCHAR g_strCalibrationFileName[512];
extern TCHAR g_strCalibrationFilePath[512];
extern TCHAR g_strCalibrationFileOld[512];

HRESULT OpenCalibrationFileDialog(HWND hWnd);
HRESULT SaveCalibrationFileDialog(HWND hWnd);


// scenery file
extern TCHAR g_strSceneryFileName[512];
extern TCHAR g_strSceneryFilePath[512];
extern TCHAR g_strSceneryFileOld[512];

HRESULT OpenSceneryFileDialog(HWND hWnd);


// flight rec skin
extern bool g_bFlightRecShowSkin;
extern TCHAR g_strFlightRecSkinDir[MAX_PATH];
int CALLBACK BrowseCallbackProc( HWND hwnd, UINT uMsg, LPARAM lParam, LPARAM lpData );
#define WM_FLIGHTRECSKINNEW  (WM_USER+5)

// flight rec tooltips
extern bool g_bFlightRecShowTooltips;


// say cheese
extern bool Screenshot (LPCTSTR FileName, LPDIRECTDRAWSURFACE7 lpDDS);
extern bool Screenshot2(LPCTSTR FileName, HDC hDC);
extern bool Screenshot3(LPCTSTR FileName, HWND hWnd);

extern bool g_bScreenshotIncludeWindowBorder;


// RT Config Toolwindow
BOOL CALLBACK RTConfigurationProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM );
void CenterDownDialog(HWND hdlg);
extern HWND g_hWndTools;
extern bool bShowingTools;
extern HINSTANCE g_hInst;


extern DLGTEMPLATE* pTabPage[5];
extern HWND g_hTabControl;
extern HWND g_hTabCurrent;


void CreateTabControl(HWND hWnd);
void TabChange(HWND hWnd);
void TabCenter(HWND hWnd);




extern int g_iRT1;
extern int g_iRT2;
extern int g_iRT3;


extern HHOOK hHook;
LRESULT CALLBACK MouseHookProc( int code, WPARAM wParam, LPARAM lParam );
extern bool g_bMouseDown;
extern bool g_bOverButton;
//extern bool g_bCaptured;


// tooltips for collapse/restore button
extern HWND g_hwndTT;		// handle of the ToolTip control 
extern HWND g_hwndDlg;		// handle of the dialog box 
extern HHOOK g_hhk;			// handle of the hook procedure 

// hook
LRESULT CALLBACK GetMsgProc(int nCode, WPARAM wParam, LPARAM lParam);




// reset values after new file load
extern bool g_bResetValues;



// TODO: channel mapping
// We'll read this in from the registry at start-up or after having received
// a message from Bigpush.exe that a new mapping was set.
extern int g_iMapPitchCh;
extern int g_iMapRollCh;
extern int g_iMapNickCh;
extern int g_iMapYawCh;

extern int g_iMapThrottleCh;

extern int g_iMapForwardCh;
extern int g_iMapBankCh;


// TODO: channel inversion, etc.
// Let's make it a DWORD instead of bool to prevent C4800 warnings
// when reading from registry
// NOTE: DWORD is unsigned!!!
extern DWORD g_bChMute[11];

extern DWORD g_bChInv[11];
extern DWORD g_bChExp[11];

// ???
extern int g_iChSens[11];


// macros to convert RGB values from D3D's D3DCOLOR to GDI's COLORREF, and back
// NOTE: D3D D3DCOLOR: 0xaarrggbb
//       GDI COLORREF: 0x00bbggrr
#define RGB_TOCOLORREF( d3dcolor ) (RGB( (RGB_GETRED  (d3dcolor)), \
									     (RGB_GETGREEN(d3dcolor)), \
									     (RGB_GETBLUE (d3dcolor)) )) 


#define RGB_TOD3DCOLOR( colorref ) (D3DRGB( ((float(GetRValue(colorref))) / 255), \
										    ((float(GetGValue(colorref))) / 255), \
										    ((float(GetBValue(colorref))) / 255) ))


// registry g_hWndCalibration
void RegistryRead3();
void RegistryWrite3();

// Registry value allowing R/C Sim to check whether Calibration is already running
// and then Restore/SetForeground the window or Launch the app
extern DWORD g_hWndCalibration;


// registry g_hWndSikorsky
void RegistryRead4();
void RegistryWrite4();

// Registry value allowing Calibration (or R/C Sim Sikorsky itself) to check whether Sikorsky
// is already running and then Restore/SetForeground the window or Launch the app
extern DWORD g_hWndSikorsky;



// DirectSound
extern unsigned int g_uCountBeforeMaxVolume;

// light
extern D3DLIGHT7 g_Light1;
extern D3DLIGHT7 g_Light2;


// Flight Record and Playback
extern g_iFrameCount;
extern g_iFrameTotal;

extern HANDLE g_hFileFlightRec;
//void ResetFilePointer();


// status bar
extern bool g_bStatusBar;
extern HWND g_hwndStatus;
extern UINT IDC_STATUSBAR;

// progress bar on status bar
extern HWND g_hwndPB;
extern bool g_bSmoothProgressBar;
extern COLORREF g_crProgressBarColor;


// client window
extern bool g_bClientWindow;
extern HWND g_hwndClient;


// shareware
extern bool g_bShareWareCheck;
extern int g_iShareWareDaysUsed;

extern int g_iShareWareRegisterCode;


// particle engine
extern TSnowfall* g_pSnowfall;   // Snow particle system
extern TSmoke* g_pSmoke;         // Smoke particle system

extern int g_iExhaustSmokeDensity;
extern float g_fExhaustSmokeVolume;

extern bool g_bExhaustSmokeBlack;

extern D3DVECTOR g_vExhaustSmokeOutput;


// authentic heli sound
extern BOOL g_bUseAuthenticHeliSound;


// Windows 95/98/Me - NT/2K/XP
extern OSVERSIONINFO osvinfo;
extern bool g_bWindowsNT;


// channel assignment
extern int g_iChAssignedThrottle;
extern int g_iChAssignedRoll;
extern int g_iChAssignedNick;
extern int g_iChAssignedYaw;
extern int g_iChAssignedPitch;
extern int g_iChAssignedGyro;

extern int g_iChMaxThrottle;
extern int g_iChMaxRoll;
extern int g_iChMaxNick;
extern int g_iChMaxYaw;
extern int g_iChMaxPitch;
extern int g_iChMaxGyro;

extern BOOL g_bUpdateChannelAssignment;

void RegistryRead5();
void RegistryWrite5();


// flight and calibration
void RegistryRead7();
void RegistryWrite7();


// error checking
void ErrMsgBox();



// small rec/play toolwindow
BOOL CALLBACK FlightRecProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );
BOOL CALLBACK FlightRecProc2( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );

extern HWND g_hWndFlightRec;
extern bool g_bShowingFlightRec;


// boxes (heli pads)
extern float BOXPOSITION1_X;
extern float BOXPOSITION1_Y;
extern float BOXPOSITION1_Z;

extern float BOXPOSITION2_X;
extern float BOXPOSITION2_Y;
extern float BOXPOSITION2_Z;

extern float BOXPOSITION3_X;
extern float BOXPOSITION3_Y;
extern float BOXPOSITION3_Z;

extern float BOXPOSITION4_X;
extern float BOXPOSITION4_Y;
extern float BOXPOSITION4_Z;

extern int g_iBoxSize;

extern bool g_bShowBox1;
extern bool g_bShowBox2;
extern bool g_bShowBox3;
extern bool g_bShowBox4;

extern BOOL g_bShowBoxes;


// heli pad (boxes) texture
extern char *g_szTexNameHeliPad1;
extern char *g_szTexNameHeliPad2;
//extern LPDIRECTDRAWSURFACE7 g_pTexHeliPad1;
//extern LPDIRECTDRAWSURFACE7 g_pTexHeliPad2;
extern int g_iTexHeliPad;


// frame index table
extern INTVECTOR theVector;
extern INTVECTOR::iterator theIterator;
HRESULT BuildFrameIndexTable( FILE *fpFile );


// R/C Script parser
bool ParseScript( char* script );



// Hook proc for Open Model File Dialog 
UINT APIENTRY OFNHookProc1( HWND hdlg, UINT uiMsg, WPARAM wParam, LPARAM lParam );

// Hook proc for Open Scenery File Dialog 
UINT APIENTRY OFNHookProc2( HWND hdlg, UINT uiMsg, WPARAM wParam, LPARAM lParam );


// FMS
extern float g_fScaleVal;

// mrotor axis
extern D3DVECTOR g_vAxisMRotor;

// trotor axis
extern D3DVECTOR g_vAxisTRotor;


// used for saving skid vertices
//extern D3DVERTEX g_pVerticesOrigSkid[6666];

// Hell, typedef vector<D3DVERTEX> D3DVERTEXVECTOR; is not accepted by STL
//// used for saving mesh vertices
//using namespace std;
////typedef vector<D3DVERTEX> D3DVERTEXVECTOR;
////typedef vector<int> D3DVERTEXVECTOR;
////typedef vector<D3DVECTOR> D3DVERTEXVECTOR;
//
//// used for saving mesh vertices
//// Dynamically allocated vector begins with 0 elements.
//extern D3DVERTEXVECTOR theVectorSkid;
//extern D3DVERTEXVECTOR theVectorMRotor;
//extern D3DVERTEXVECTOR theVectorTRotor;
//
//// Iterator is used to loop through the vector.
//extern D3DVERTEXVECTOR::iterator theIteratorSkid;
//extern D3DVERTEXVECTOR::iterator theIteratorMRotor;
//extern D3DVERTEXVECTOR::iterator theIteratorTRotor;

// used for saving mesh vertices
extern D3DVERTEX* g_pVerticesOrigSkid;
extern D3DVERTEX* g_pVerticesOrigMRotor;
extern D3DVERTEX* g_pVerticesOrigTRotor;
extern D3DVERTEX* g_pVerticesOrigShaft;


// variation on Follow mode
extern bool g_bFollowTail;


// The one and only CLimitSingleInstance object
extern CLimitSingleInstance g_SingleInstanceObj;


// wind
extern bool g_bWind;
extern float g_fWindSpeed;
extern float g_fWindDirection;
extern float g_fWindSpeedTolerance;
extern float g_fWindDirectionTolerance; 
extern float g_fWindSpeedVariation;
extern float g_fWindDirectionVariation;

// windsock
extern BOOL g_bWindsock;
extern bool g_bFlag;

extern float WINDSOCK_X;
extern float WINDSOCK_Y;
extern float WINDSOCK_Z;


// helipad
extern BOOL g_bHelipad;

extern float HELIPAD_X;
extern float HELIPAD_Y;
extern float HELIPAD_Z;


// runway
extern BOOL g_bRunway;

extern float RUNWAY_X;
extern float RUNWAY_Y;
extern float RUNWAY_Z;


// field
extern BOOL g_bField;

extern float FIELD_X;
extern float FIELD_Y;
extern float FIELD_Z;


// trees
#define NUM_TREES 1000
#define MAX_WIDTH 400.0f

inline FLOAT RandomPos()   { return (MAX_WIDTH*(FLOAT)(rand()-rand()))/RAND_MAX; }

extern BOOL g_bTrees;
extern int g_iNumTrees;

//extern float TREE_X[NUM_TREES];
//extern float TREE_Y[NUM_TREES];
//extern float TREE_Z[NUM_TREES];


// for m_fSpeedFactor
extern float g_acc;
extern unsigned long g_count;


// kludge for exhaust smoke
extern bool g_bResetExhaustSmoke;


// skid spring/suspension
extern float g_fSkidCFM;
extern float g_fSkidERP;
extern bool g_bSkidSpring;


// PANORAMA
extern BOOL g_bPanorama;
extern bool g_bRecreatePanoramaBitmaps;

extern BOOL g_bShowProgressDialog;
extern bool g_bLoadScenery;
extern bool g_bCancelLoadScenery;

#define NUM_PAN 24

extern char *g_szPanName[NUM_PAN];

extern LPDIRECTDRAWSURFACE7 g_ptexPan[NUM_PAN];

BOOL CALLBACK ProgressProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );
extern HWND g_hwndProgressDialog;







//-----------------------------------------------------------------------------
// Name: class CMyD3DApplication
// Desc: Application class. The base class provides just about all the
//       functionality we want, so we're just supplying stubs to interface with
//       the non-C++ functions of the app.
//-----------------------------------------------------------------------------
class CMyD3DApplication : public CD3DApplication
{
	FLOAT m_fRadsY;
	FLOAT m_fRadsX;
	FLOAT m_fRadsZ;

	FLOAT m_fX, m_fY, m_fZ;

	FLOAT m_fCamRadsY;
	FLOAT m_fCamRadsX;
	FLOAT m_fCamRadsZ;

	FLOAT m_fCamX, m_fCamY, m_fCamZ;

	FLOAT m_fThrottle;
	FLOAT m_fRevs;
	FLOAT m_fValueKey;
	FLOAT m_fSpeed;
	FLOAT m_fAirSpeed;
	FLOAT m_fSpeedDescent;
	FLOAT m_fAltitude;
	FLOAT m_fDistance;

	FLOAT m_fSpeedFactor;
	BOOL  m_bResetSpeedFactor;

	FLOAT m_fSpeedX;
	FLOAT m_fSpeedY;
	FLOAT m_fSpeedZ;

	FLOAT m_fCollective;
	FLOAT m_fWeight;
	FLOAT m_fTorque;	
	
	FLOAT m_fRotorConeY;
	FLOAT m_fRotorTiltX;
	FLOAT m_fRotorTiltZ;
	FLOAT m_fRotorConeX;

	FLOAT m_fSkidSpringY;

	FLOAT m_fFogStart;
	FLOAT m_fFogEnd;
	DWORD m_dwFogColor;

	DWORD m_dwBackGroundColor;

    D3DMATRIX m_matSave;
	D3DMATRIX m_matSave2;
    D3DMATRIX m_matRotate;

	BOOL m_bRCView;
	BOOL m_bInModelView;
	BOOL m_bChaseView;
	BOOL m_bFollowView;

	UINT m_uPilotPosition;

	BOOL m_bRotaryWing;
	BOOL m_bFixedWing;
	BOOL m_b6DOF;

	BOOL m_bText;
    BOOL m_bFog;
    BOOL m_bCull;
    BOOL m_bFlat;
	BOOL m_bSpec;	
	BOOL m_bSolid;
    BOOL m_bWire;
	BOOL m_bPoint;

	INT m_iTextureFilter;

	BOOL m_bActive;
	
	BOOL m_bFirstTime;
	BOOL m_bReAcquire;
	BOOL m_bFrameMoving;
	BOOL m_bFrameStepping;

	BOOL m_bMixCyclicTail;
	BOOL m_bLeftRotating;

	BOOL m_bShowFlightInfo;
	BOOL m_bShowChannels;

	BOOL m_bDrawShadow;
	
	BOOL m_bCapsShadow;
	BOOL m_bDrawShadowVolume;

	BOOL m_bSplashScreenShowing;

	BOOL m_bSmoothControls;
	BOOL m_bSmoothCamControls;

	BOOL m_bExhaustSmoke;

	BOOL m_bSound;
	BOOL m_bActiveSound;

	BOOL m_bShowPPMarks;

	BOOL m_bVibration;
	BOOL m_bTurbulence;

	BOOL m_bRecord;
	BOOL m_bPlayBack;

	BOOL m_bUseAlphaTest;

	BOOL m_bEscExits;


	INT m_iSunElevation;
	INT m_iSunDirection;
	INT m_iSunIntensity;
	FLOAT m_fShadowAlpha;

	CD3DFile* m_pTerrainObject;          // X file of terrain
    D3DMATRIX m_matTerrainMatrix;        // Matrix to position terrain

	CD3DFile* m_pSkyObject;				 // X file of terrain
    D3DMATRIX m_matSkyMatrix;			 // Matrix to position sky

	CD3DFile* m_pWindsockObject;          // X file of windsock
    D3DMATRIX m_matWindsockMatrix;        // Matrix to position windsock

	CD3DFile* m_pHelipadObject;          // X file of helipad
    D3DMATRIX m_matHelipadMatrix;        // Matrix to position helipad

	CD3DFile* m_pRunwayObject;          // X file of runway
    D3DMATRIX m_matRunwayMatrix;        // Matrix to position runway

	CD3DFile* m_pFieldObject;			// X file of field
    D3DMATRIX m_matFieldMatrix;			// Matrix to position field

	//CD3DFile* m_pTreeObject[NUM_TREES];		// X file of tree
    D3DMATRIX m_matTreeMatrix[NUM_TREES];		// Matrix to position tree


	CD3DFile* m_pFileObject;             // X file object to render
    D3DMATRIX m_matFileObjectMatrix;     // Matrix to animate X file object

	CD3DFile* m_pFileObject2;             // X file object to render
    D3DMATRIX m_matFileObjectMatrix2;     // Matrix to animate X file object

	D3DMATRIX m_matIdentityMatrix;


	CD3DFile* m_pPP1Object;
	CD3DFile* m_pPP2Object;
	CD3DFile* m_pPP3Object;
	CD3DFile* m_pPP4Object;

    D3DMATRIX m_matPP1Matrix;
	D3DMATRIX m_matPP2Matrix;
	D3DMATRIX m_matPP3Matrix;
	D3DMATRIX m_matPP4Matrix;

	CD3DFile* m_pSphere1Object;


	D3DVERTEX* m_pFileObjectVertices;
    DWORD      m_dwNumFileObjectVertices;

	LPWORD	   m_pFileObjectIndices;
    DWORD      m_dwNumFileObjectIndices;

	D3DVERTEX* m_pFileObjectVertices2;
    DWORD      m_dwNumFileObjectVertices2;

	D3DMATRIX m_matView;

	D3DVECTOR m_vRight;
	D3DVECTOR m_vUp;
	D3DVECTOR m_vForward;

	LPDIRECTDRAWSURFACE7 m_pddsDepthBuffer;

	LPDIRECTDRAWSURFACE7 m_pddsBackBufferShadow; // render light view to this buffer
	LPDIRECTDRAWSURFACE7 m_pddsZBufferShadow;    // this will contain our shadow map

	D3DLVERTEX  m_TreeMesh[4];
    D3DVECTOR   m_TreePositions[NUM_TREES];
	//LPDIRECTDRAWSURFACE7 m_pTexTree;
	//LPDIRECTDRAWSURFACE7 m_pTexTreeShadow;


	HRESULT OpenFileDialog();
	HRESULT LoadFile( TCHAR* strFilename );

	static BOOL CalcFileObjectSizeCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext );	// use static to use callback as member
	
	static BOOL GetNumFileObjectVerticesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext );
	static BOOL GetFileObjectVerticesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext );
	
	static BOOL GetNumFileObjectIndicesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext );
	static BOOL GetFileObjectIndicesCB( CD3DFileObject* pObject, D3DMATRIX* pmat,
                           VOID* pContext );

	HRESULT CreateTextures();
	
	HRESULT CreateWindowedShadowBuffers();
	HRESULT CreateFullscreenShadowBuffers( DDSURFACEDESC2* );

	HRESULT CreateStencilBuffer();
	HRESULT DrawShadow();
    HRESULT RenderShadow();
	static HRESULT ConfirmDevice( DDCAPS* pddDriverCaps,
                                  D3DDEVICEDESC7* pd3dDeviceDesc );
	static HRESULT WINAPI EnumZBufferFormatsCallback( DDPIXELFORMAT* pddpf,
                                                      VOID* pddpfDesired );

    HRESULT RenderPlanarShadow();
	HRESULT RenderLensFlare();
	HRESULT RenderExhaustSmoke();

	HRESULT RenderProxy();

	HRESULT RenderSplashScreen();
	HRESULT DoScreenShot();

    HRESULT DrawTrees();
	HRESULT DrawTreeShadows();

	HRESULT FlightRecord();
	HRESULT FlightPlayBack();
	void ResetFilePointer();
	void UpdateFlightRec();

	void RegistryRead();
	void RegistryWrite();

	void SetRCViewMatrix();
	void SetInModelViewMatrix();
	void SetChaseViewMatrix();
	void SetFollowViewMatrix();

	VOID SaveFileObjectOrigMeshVertices();

	VOID LoadScenery();
	VOID RenderScenery();

protected:
    HRESULT OneTimeSceneInit();
    HRESULT InitDeviceObjects();
    HRESULT DeleteDeviceObjects();
    HRESULT Render();
    HRESULT FrameMove( FLOAT fTimeKey );
    HRESULT FinalCleanup();
	

	// NOTE: all inline functions must be defined in xfile.h
	inline void PitchUp();
	inline void PitchDown();
	inline void RollLeft();
	inline void RollRight();
	inline void YawLeft();
	inline void YawRight();

	inline void Pitch( LONG lJoyYAxis );
	inline void Roll( LONG lJoyXAxis );
	inline void Yaw( LONG lJoyZAxis );

	inline void XUp( LONG lJoyAxis );
	inline void YUp( LONG lJoyAxis );
	inline void ZUp( LONG lJoyAxis );


	inline void XUp();
	inline void XDown();
	inline void YUp();
	inline void YDown();
	inline void ZUp();
	inline void ZDown();

	inline void XUpIncremental();
	inline void XDownIncremental();
	inline void YUpIncremental();
	inline void YDownIncremental();
	inline void ZUpIncremental();
	inline void ZDownIncremental();

	inline void ThrottleUp();
	inline void ThrottleDown();

	inline void CamXUp();
	inline void CamXDown();
	inline void CamYUp();
	inline void CamYDown();
	inline void CamZUp();
	inline void CamZDown();

	inline void CamPitchUp();
	inline void CamPitchDown();
	inline void CamRollLeft();
	inline void CamRollRight();
	inline void CamYawLeft();
	inline void CamYawRight();

	inline void FogStartUp();
	inline void FogStartDown();
	inline void FogEndUp();
	inline void FogEndDown();

	void GetSticks();
	void GetVirtualSticks();
	void GetVirtualTrims();
	void GetSliders();
	void GetKeys();
	void GetInput();
	void GetGodMode();

	void ResetCam();
	void ResetHeli();

	BOOL CollisionDetect();

	void CheckD3DDeviceRefCount();
	void CheckDDrawRefCount();

	void ShowFlightInfo();
	void ShowChannels();

	VOID OutputTextEx( DWORD x, DWORD y, TCHAR* str );

	void DoSmoothControls();
	void DoSmoothCamControls();
	//void DoDynamics();

	void DoRTConfigPosition();
	void UpdateRTChannels();
	void UpdateRTVirtualTx();

	void UpdateStatusBar();

	VOID SetMenuItems(HMENU hmenu);
	VOID InitModeMenu( HMENU hmenu );

public:
    CMyD3DApplication();

    LRESULT MsgProc( HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam );


	
	// let's make some friends
	friend BOOL CALLBACK ConfigurationProc2		( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );
	friend BOOL CALLBACK PropPageWindowProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageVxDProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPagePositionProc	( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageAltitudeProc	( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageControlsProc	( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageCameraProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageZoomProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageViewProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageSkyProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageTerrainProc	( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageSoundProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageBoxesProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageFogProc		( HWND, UINT, WPARAM, LPARAM );
	friend BOOL CALLBACK PropPageFlightInfoProc	( HWND, UINT, WPARAM, LPARAM );
	
	
	friend BOOL CALLBACK RTConfigurationProc( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam );
	friend BOOL CALLBACK Tab1DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab2DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab3DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab4DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab5DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab6DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab7DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab8DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab9DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab10DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab11DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab12DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab13DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab14DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK Tab15DlgProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);


	friend BOOL CALLBACK FlightRecProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend BOOL CALLBACK FlightRecProc2(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);

	friend HRESULT OpenFlightFileDialog(HWND hWnd);
	friend HRESULT OpenSceneryFileDialog(HWND hWnd);

	friend DWORD WINAPI ThreadFuncLoadScenery( LPVOID lpParam );
	
	friend BOOL CALLBACK AboutProc3( HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM );
	friend void GameLoopBsc();

	friend HWND DoCreateStatusWindow( HWND hwndParent, HINSTANCE hinst );
	friend HWND DoCreateClientWindow( HWND hwndParent, HINSTANCE hinst );

	friend VOID ProcessCommandLine( LPSTR strCmdLine );
	friend VOID HandleDragDrop( HWND hWnd, HDROP hDrop, LPSTR strCmdLine );

	friend BOOL CALLBACK ConsoleProc(HWND hDlg, UINT uMsg, WPARAM wParam, LPARAM lParam);
	friend bool ParseScript( char* script );

	// ODE
	friend void InitDynamics();
	friend void DoDynamics();
	friend void DestroyDynamics();

	friend static void nearCallback (void *data, dGeomID o1, dGeomID o2);

	// NOTE NOTE NOTE: very strange indeed: we wanted to make these friends too. Doing
	// so however causes the compiler to hang on xfile.cpp or dynamics.cpp. It is probably
	// because these functions use DDraw objects. Or stack probs because of ODE????
	// Or is there a limit to the number of friends a class can have???
	// Well, we can now make these just friends (instead of member functions). We have
	// removed all proxy code from dynamics.cpp to a separate file (proxy.cpp) and now
	// the compiler no longer hangs. Could it be that dynamics.cpp was too large/complex???
	friend void CreateProxy();
	friend void DestroyProxy();

	// ClothSim 
	friend BOOL CreateCloth(char *ftex, unsigned int *faces, int faceCount, float *vertices, int vertexCount, BOOL makechild);
	friend void UpdateCloth(unsigned int *faces, int faceCount, float *vertices, int vertexCount);
	friend void CopyWindsockVertices(void);

	// .DAT file
	friend HRESULT ParseDatFile( TCHAR* strFilename );

	
	//friend void Test();
	//friend void Test2();

	// RM
	//friend int CreateRM(AppInfo* info);	

	friend class TSmoke;

};




// Sorry Mr. Strouptrup (or is this OK?)
extern CMyD3DApplication* g_pd3dApp;




// Controls /////////////////////////////////////////////////////
// NOTE: we have speed-factored all controls but since we already speed-factor ODE's
// worldstep we undo the speed factor later in FrameMove() for ODE input. Leaving in the
// speed factor for the controls gives us frame-rate independent motion with plain 6DOF
// flight dynamics.
// Rotation
// NOTE: Pitch (graphics lingo) == Nick (R/C heli lingo)
inline void CMyD3DApplication::PitchUp()
{
	m_fRadsX = +(0.05f)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsX = +(0.05f/2)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsX = +(0.05f*2)*CTRL_RADS_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::PitchDown()
{
	m_fRadsX = -(0.05f)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsX = -(0.05f/2)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)   & 0x80 ) m_fRadsX = -(0.05f*2)*CTRL_RADS_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::RollLeft()
{
	m_fRadsZ = -(0.05f)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsZ = -(0.05f/2)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)   & 0x80 ) m_fRadsZ = -(0.05f*2)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::RollRight()
{
	m_fRadsZ = +(0.05f)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsZ = +(0.05f/2)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsZ = +(0.05f*2)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::YawLeft()
{
	m_fRadsY = -(0.20f)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsY = -(0.20f/2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)   & 0x80 ) m_fRadsY = -(0.20f*2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::YawRight()
{
	m_fRadsY = +(0.20f)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsY = +(0.20f/2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsY = +(0.20f*2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
}



// linear controls for joystick
// NOTE: Pitch == Nick
inline void CMyD3DApplication::Pitch( LONG lJoyYAxis )
{
	m_fRadsX = (lJoyYAxis*0.00005f)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsX = (lJoyYAxis*0.00005f/2)*CTRL_RADS_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsX = (lJoyYAxis*0.00005f*2)*CTRL_RADS_X_SENS*m_fSpeedFactor; 
}

inline void CMyD3DApplication::Roll( LONG lJoyXAxis )
{
	m_fRadsZ = (lJoyXAxis*0.00005f)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsZ = (lJoyXAxis*0.00005f/2)*CTRL_RADS_Z_SENS*m_fSpeedFactor; 
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsZ = (lJoyXAxis*0.00005f*2)*CTRL_RADS_Z_SENS*m_fSpeedFactor;
}


inline void CMyD3DApplication::Yaw( LONG lJoyZAxis )
{
	m_fRadsY = (lJoyZAxis*0.00005f)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fRadsY = (lJoyZAxis*0.00005f/2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fRadsY = (lJoyZAxis*0.00005f*2)*CTRL_RADS_Y_SENS*m_fSpeedFactor;
}



inline void CMyD3DApplication::XUp( LONG lJoyAxis )
{
	m_fX = -(lJoyAxis*0.001f)*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fX = -(lJoyAxis*0.001f/2)*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fX = -(lJoyAxis*0.001f*2)*CTRL_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::YUp( LONG lJoyAxis )
{
	m_fY = +(lJoyAxis*0.001f)*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fY = +(lJoyAxis*0.001f/2)*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fY = +(lJoyAxis*0.001f*2)*CTRL_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::ZUp( LONG lJoyAxis )
{
	m_fZ = -(lJoyAxis*0.001f)*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fZ = -(lJoyAxis*0.001f/2)*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fZ = -(lJoyAxis*0.001f*2)*CTRL_Z_SENS*m_fSpeedFactor;
}




// Position ///////////////////////////////////////////////////
inline void CMyD3DApplication::XUp()
{
	m_fX = -0.25f*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fX = -(0.25f/2)*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fX = -(0.25f*2)*CTRL_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::XDown()
{
	m_fX = +0.25f*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fX = +(0.25f/2)*CTRL_X_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fX = +(0.25f*2)*CTRL_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::YUp()
{
	m_fY = +0.25f*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fY = +(0.25f/2)*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fY = +(0.25f*2)*CTRL_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::YDown()
{
	m_fY = -0.25f*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fY = -(0.25f/2)*CTRL_Y_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fY = -(0.25f*2)*CTRL_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::ZUp()
{
	m_fZ = -0.25f*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fZ = -(0.25f/2)*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fZ = -(0.25f*2)*CTRL_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::ZDown()
{
	m_fZ = +0.25f*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_CONTROL) & 0x80 ) m_fZ = +(0.25f/2)*CTRL_Z_SENS*m_fSpeedFactor;
	if ( GetKeyState(VK_SHIFT)	 & 0x80 ) m_fZ = +(0.25f*2)*CTRL_Z_SENS*m_fSpeedFactor;
}



// Position incremental  ////////////////////////////////////
inline void CMyD3DApplication::XUpIncremental()
{

}

inline void CMyD3DApplication::XDownIncremental()
{

}

inline void CMyD3DApplication::YUpIncremental()
{
	// collective incremental stepping
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fCollective+=(0.002f/10)*CTRL_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fCollective+=(0.002f*10)*CTRL_Y_SENS*m_fSpeedFactor;
	else									   m_fCollective+=(0.002f)*CTRL_Y_SENS*m_fSpeedFactor;	
}

inline void CMyD3DApplication::YDownIncremental()
{
	// collective incremental stepping
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fCollective-=(0.002f/10)*CTRL_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fCollective-=(0.002f*10)*CTRL_Y_SENS*m_fSpeedFactor;
	else									   m_fCollective-=(0.002f)*CTRL_Y_SENS*m_fSpeedFactor;	
}
 
inline void CMyD3DApplication::ZUpIncremental()
{
	// speed incremental stepping
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fSpeed+=(0.01f/10)*CTRL_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fSpeed+=(0.01f*10)*CTRL_Z_SENS*m_fSpeedFactor;
	else									   m_fSpeed+=(0.01f)*CTRL_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::ZDownIncremental()
{
	// speed incremental stepping
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fSpeed-=(0.01f/10)*CTRL_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fSpeed-=(0.01f*10)*CTRL_Z_SENS*m_fSpeedFactor;
	else									   m_fSpeed-=(0.01f)*CTRL_Z_SENS*m_fSpeedFactor;
}


// Engine ////////////////////////////////////////////
inline void CMyD3DApplication::ThrottleUp()
{
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fThrottle+=(0.1f/10);
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fThrottle+=(0.1f*10);
	else									   m_fThrottle+=(0.1f);
}

inline void CMyD3DApplication::ThrottleDown()
{
	if		( GetKeyState(VK_CONTROL) & 0x80 ) m_fThrottle-=(0.1f/10);
	else if ( GetKeyState(VK_SHIFT)	  & 0x80 ) m_fThrottle-=(0.1f*10);
	else									   m_fThrottle-=(0.1f);
}





// Cam Position ///////////////////////////////////////////////////
// Note: because the Shift key toggles the numerical keyboard functions, and 
// the Alt key has Windows menu functions, the camera has its own modifier keys:
// Numerical0: increased values
// NumericalDot: decreased values
inline void CMyD3DApplication::CamXUp()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 ) 
		m_fCamX+=(0.5f/2)*CAM_X_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 ) 
		m_fCamX+=(0.5f*2)*CAM_X_SENS*m_fSpeedFactor;
	else									  
		m_fCamX+=(0.5f)*CAM_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamXDown()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 ) 
		m_fCamX-=(0.5f/2)*CAM_X_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 )
		m_fCamX-=(0.5f*2)*CAM_X_SENS*m_fSpeedFactor;
	else									   
		m_fCamX-=(0.5f)*CAM_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamYUp()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamY == 0 )
		m_fCamY+=(0.5f/2)*CAM_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamY == 2 )
		m_fCamY+=(0.5f*2)*CAM_Y_SENS*m_fSpeedFactor;
	else									   
		m_fCamY+=(0.5f)*CAM_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamYDown()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamY == 0 )
		m_fCamY-=(0.5f/2)*CAM_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamY == 2 )
		m_fCamY-=(0.5f*2)*CAM_Y_SENS*m_fSpeedFactor;
	else	
		m_fCamY-=(0.5f)*CAM_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamZUp()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamZ == 0 )
		m_fCamZ+=(0.5f/2)*CAM_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamZ == 2 )
		m_fCamZ+=(0.5f*2)*CAM_Z_SENS*m_fSpeedFactor;
	else									   
		m_fCamZ+=(0.5f)*CAM_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamZDown()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamZ == 0 )
		m_fCamZ-=(0.5f/2)*CAM_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamZ == 2 )
		m_fCamZ-=(0.5f*2)*CAM_Z_SENS*m_fSpeedFactor;
	else									   
		m_fCamZ-=(0.5f)*CAM_Z_SENS*m_fSpeedFactor;
}


// Cam Orientation ////////////////////////////////////////////////////////
inline void CMyD3DApplication::CamPitchUp()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 )
		m_fCamRadsX+=(0.02f/2)*CAM_RADS_X_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 )
		m_fCamRadsX+=(0.02f*2)*CAM_RADS_X_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsX+=(0.02f)*CAM_RADS_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamPitchDown()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamX == 0 )
		m_fCamRadsX-=(0.02f/2)*CAM_RADS_X_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamX == 2 )
		m_fCamRadsX-=(0.02f*2)*CAM_RADS_X_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsX-=(0.02f)*CAM_RADS_X_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamRollLeft()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamZ == 0 )
		m_fCamRadsZ+=(0.02f/2)*CAM_RADS_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamZ == 2 )
		m_fCamRadsZ+=(0.02f*2)*CAM_RADS_Z_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsZ+=(0.02f)*CAM_RADS_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamRollRight()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamZ == 0 )
		m_fCamRadsZ-=(0.02f/2)*CAM_RADS_Z_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamZ == 2 )
		m_fCamRadsZ-=(0.02f*2)*CAM_RADS_Z_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsZ-=(0.02f)*CAM_RADS_Z_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamYawLeft()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamY == 0 )
		m_fCamRadsY+=(0.02f/2)*CAM_RADS_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamY == 2 )
		m_fCamRadsY+=(0.02f*2)*CAM_RADS_Y_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsY+=(0.02f)*CAM_RADS_Y_SENS*m_fSpeedFactor;
}

inline void CMyD3DApplication::CamYawRight()
{
	if		( GetKeyState(VK_DECIMAL) & 0x80 || g_iRTSliderCamY == 0 )
		m_fCamRadsY-=(0.02f/2)*CAM_RADS_Y_SENS*m_fSpeedFactor;
	else if ( GetKeyState(VK_NUMPAD0) & 0x80 || g_iRTSliderCamY == 2 )
		m_fCamRadsY-=(0.02f*2)*CAM_RADS_Y_SENS*m_fSpeedFactor;
	else									   
		m_fCamRadsY-=(0.02f)*CAM_RADS_Y_SENS*m_fSpeedFactor;
}


// Fog functions //////////////////////////////////////
inline void CMyD3DApplication::FogStartUp()
{
	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFogStart == 1 ) {
		m_fFogStart += 10.0f;
	} else  {
		m_fFogStart += 1.0f;
	}
}

inline void CMyD3DApplication::FogStartDown()
{
	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFogStart == 1 ) {
		m_fFogStart -= 10.0f;
	} else  {
		m_fFogStart -= 1.0f;
	}
}

inline void CMyD3DApplication::FogEndUp()
{
	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFogEnd == 1 ) {
		m_fFogEnd += 10.0f;
	} else  {
		m_fFogEnd += 1.0f;
	}
}

inline void CMyD3DApplication::FogEndDown()
{
	if ( GetKeyState(VK_SHIFT) & 0x80 || g_iRTSliderFogEnd == 1 ) {
		m_fFogEnd -= 10.0f;
	} else  {
		m_fFogEnd -= 1.0f;
	}
}




#endif // __XFILE_H