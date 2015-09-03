
// PhidgetCloseLoopDlg.cpp : implementation file
//

#include "stdafx.h"
#include "PhidgetCloseLoop.h"
#include "PhidgetCloseLoopDlg.h"

#include "Stepper-simple.h"
#include "afxdialogex.h"
#include "ChartLineSerie.h"
#include "ChartPointsSerie.h"

#include "math.h"
#include <iostream>
#include <windows.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

using namespace std;


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CPhidgetCloseLoopDlg dialog




CPhidgetCloseLoopDlg::CPhidgetCloseLoopDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CPhidgetCloseLoopDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CPhidgetCloseLoopDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_CHARTCTRL, m_ChartCtrl);
	DDX_Control(pDX, IDC_STATUSBAR, m_StatusBarControl);
	DDX_Control(pDX, IDC_FPS, m_FPSControl);
}

BEGIN_MESSAGE_MAP(CPhidgetCloseLoopDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BUTTON1, &CPhidgetCloseLoopDlg::OnBnClickedButton1)
END_MESSAGE_MAP()


// CPhidgetCloseLoopDlg message handlers

BOOL CPhidgetCloseLoopDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	XValues = new double[MAX_BUFFER];
	YValuesO = new double[MAX_BUFFER];
	YValuesI = new double[MAX_BUFFER];
	dataready=false;

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	CChartStandardAxis* pBottomAxis = 
		m_ChartCtrl.CreateStandardAxis(CChartCtrl::BottomAxis);
	pBottomAxis->SetMinMax(0, 10);
	CChartStandardAxis* pLeftAxis =
		m_ChartCtrl.CreateStandardAxis(CChartCtrl::LeftAxis);
	pLeftAxis->SetMinMax(-5, 5);
	CChartStandardAxis* pTopAxis =
		m_ChartCtrl.CreateStandardAxis(CChartCtrl::TopAxis);
	pTopAxis->SetMinMax(0, 10);
	CChartStandardAxis* pRightAxis =
		m_ChartCtrl.CreateStandardAxis(CChartCtrl::RightAxis);
	pRightAxis->SetMinMax(-5, 5);

	pLineSeriesO = m_ChartCtrl.CreateLineSerie(1, 1);
	pLineSeriesO->SetWidth(1);
	pLineSeriesO->SetPenStyle(1);
	pLineSeriesI = m_ChartCtrl.CreateLineSerie(1, 1);
	pLineSeriesI->SetWidth(1);
	pLineSeriesI->SetPenStyle(2);

	m_FPSControl.SetWindowText(CString("1000"));
	m_StatusBarControl.SetWindowText(CString("IDLE"));

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CPhidgetCloseLoopDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CPhidgetCloseLoopDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

int CPhidgetCloseLoopDlg::stepper_simple(int num, __int64 targetsteps)
{
	int result;
	__int64 curr_pos;
	const char *err;
	double minAccel, maxAccel, maxVel;
	int stopped;
	char FPS[50];
	

	//Declare an stepper handle
	CPhidgetStepperHandle stepper = 0;

	//create the stepper object
	CPhidgetStepper_create(&stepper);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)stepper, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)stepper, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)stepper, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetStepper_set_OnPositionChange_Handler(stepper, PositionChangeHandler, NULL);

	//open the device for connections
	CPhidget_open((CPhidgetHandle)stepper, -1);

	//get the program to wait for an stepper device to be attached
	TRACE("Waiting for Phidget to be attached....\n");
	/*text=CString(_T("Waiting for Phidget to be attached...."));
	m_StatusBarControl.SetWindowText(text);*/
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)stepper, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		m_StatusBarControl.SetWindowText(CString(_T("Problem waiting for attachment: %s\n", err)));
		return 0;
	}
	
	//Display the properties of the attached device
	display_properties(stepper);

	//read event data
	TRACE("Reading.....\n");
	/*text=CString(_T("Reading....."));
	m_StatusBarControl.SetWindowText(text);*/

	//Set up some initial acceleration and velocity values
	CPhidgetStepper_getAccelerationMin(stepper, num, &minAccel);
	CPhidgetStepper_getAccelerationMax(stepper, num, &maxAccel);
	CPhidgetStepper_setAcceleration(stepper, num, maxAccel);
	CPhidgetStepper_getVelocityMax(stepper, num, &maxVel);
	CPhidgetStepper_setVelocityLimit(stepper, num, 5000);//maxVel);

	//display current motor position if available
	if(CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos) == EPHIDGET_OK){
			TRACE("Motor: %i > Starting Position: %lld Target: %i\n", num, curr_pos, targetsteps);
			/*text=CString(_T("Motor: %i > Current Position: %lld\n", num, curr_pos));
			m_StatusBarControl.SetWindowText(text);*/
			CPhidgetStepper_setCurrentPosition(stepper, num, 0);
	}


	//Sleep(1000);

	CPhidgetStepper_setCurrentPosition(stepper, num, 0);
	CPhidgetStepper_setEngaged(stepper, num, 1);
	Sleep(100);


	TRACE("SINUS\n");
	int start = GetTickCount();int elapsed=0;int target=0;int freq=0;int i=0;
	while((double)elapsed/1000.0<10.0 && i<MAX_BUFFER)		//sinus test for 10 sec.
	{
		elapsed = GetTickCount()-start;
		target = rad2steps(PI/2*sin(2*(double)elapsed/1000.0));
		CPhidgetStepper_setTargetPosition (stepper, num, target);
		CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
		YValuesI[i]=PI/2*sin(2*(double)elapsed/1000.0);
		YValuesO[i]=steps2rad(curr_pos);XValues[i]=(double)elapsed/1000.0;
		freq = GetTickCount()-start-elapsed;
		if(freq!=0)
			sprintf(FPS,"%3.3f", 1/((double)freq/1000.0));
		else
			sprintf(FPS,"<1000");
		m_FPSControl.SetWindowText(CString(FPS));
		i++;
		pLineSeriesO->SetPoints(XValues,YValuesO,i);
		pLineSeriesI->SetPoints(XValues,YValuesI,i);
	}

	TRACE("STRAIGHT\n");
	CPhidgetStepper_setCurrentPosition(stepper, num, 0);
	CPhidgetStepper_setTargetPosition (stepper, num, targetsteps);
	CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
	i=0;
	while(curr_pos!=targetsteps){
		//dataready=false;
		CPhidgetStepper_getCurrentPosition(stepper, num, &curr_pos);
		//TRACE("Motor: %i > Current Position: %lld\n (%i)", num, curr_pos, GetTickCount());
		//YValues[i]=steps2rad(curr_pos);XValues[i]=(double)i/10.0;
		//i++;
		//dataready=true;
		//pLineSeries->SetPoints(XValues,YValues,1000);
		//Sleep(10);
	}

	stopped = PFALSE;
	while(!stopped)
	{
		CPhidgetStepper_getStopped(stepper, num, &stopped);
		//usleep(100000);
	}

	CPhidgetStepper_setEngaged(stepper, num, 0);

	//printf("Press any key to end\n");
	//getchar();

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	TRACE("Closing...\n");
	/*text=CString(_T("Closing..."));
	m_StatusBarControl.SetWindowText(text);*/
	CPhidget_close((CPhidgetHandle)stepper);
	CPhidget_delete((CPhidgetHandle)stepper);

	//all done, exit
	return 0;
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CPhidgetCloseLoopDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CPhidgetCloseLoopDlg::OnBnClickedButton1()
{
	// TODO: Add your control notification handler code here
	m_StatusBarControl.SetWindowText(CString(_T("Starting Stepper routine.")));
	stepper_simple(0,rad2steps(2*3.1416));
	m_StatusBarControl.SetWindowText(CString(_T("Stepper routine done.")));
}