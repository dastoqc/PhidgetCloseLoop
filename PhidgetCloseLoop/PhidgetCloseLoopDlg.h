
// PhidgetCloseLoopDlg.h : header file
//

#pragma once

#include "ChartCtrl.h"
#include "ColourPicker.h"
#include "ChartLineSerie.h"

#define MAX_BUFFER 10000

// CPhidgetCloseLoopDlg dialog
class CPhidgetCloseLoopDlg : public CDialogEx
{
// Construction
public:
	CPhidgetCloseLoopDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_PHIDGETCLOSELOOP_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;
	double* XValues;
	double* YValuesO;
	double* YValuesI;
	CChartLineSerie* pLineSeriesO;
	CChartLineSerie* pLineSeriesI;
	bool dataready;

	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()

	int stepper_simple(int num, __int64 target);

public:
	CChartCtrl m_ChartCtrl;
	CStatic m_StatusBarControl; 
	CStatic m_FPSControl; 
	afx_msg void OnBnClickedButton1();
	afx_msg void OnStnClickedFps2();
};
