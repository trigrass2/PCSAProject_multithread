#include "cameracapture.h"
using  std::cout;
using  std::endl;
CameraCapture::CameraCapture()
	
{
	ui.setupUi(this);
	cameraStatus = false;

	
}

CameraCapture::~CameraCapture()
{
	
}

HObject CameraCapture::captureImage()
{
	
	//获取图像
	GrabImageStart(hv_AcqHandle, -1);
	sendPLC();

	GrabImageAsync(&ho_Image, hv_AcqHandle, -1);
	//Image Acquisition 01: Do something
	//CloseFramegrabber(hv_AcqHandle);
	return ho_Image;
}

double CameraCapture::RateToSpeed(HTuple hv_RateValue)
{
	double speed_mm = hv_RateValue * 0.02;
	cout << "speed_mm" << speed_mm << endl;
	//double speed_count = speed_mm * MMTOCOUNT;
	return speed_mm;
}

void CameraCapture::initializationCamera()
{
	try
	{
		hv_CalibFile = "gear.cfg";

		OpenFile(hv_CalibFile, "input", &hv_FileHandle);
		hv_variableName = HTuple();
		hv_variableValue = HTuple();
		for (hv_Index = 0; hv_Index <= 2; hv_Index += 1)
		{
			FreadLine(hv_FileHandle, &hv_OutLine, &hv_IsEOF);
		}
		while (0 != 1)
		{
			FreadLine(hv_FileHandle, &hv_OutLine, &hv_IsEOF);
			TupleRegexpReplace(hv_OutLine, "\\t", " ", &hv_OutLine);
			TupleRegexpReplace(hv_OutLine, "\\n", "", &hv_OutLine);
			TupleSplit(hv_OutLine, " ", &hv_Substrings);
			TupleNumber(hv_Substrings, &hv_HigherOrderCoefficients);
			TupleLength(hv_Substrings, &hv_Length);
			if (0 != (hv_Length == 0))
			{
				break;
			}
			else
			{
				hv_variableName = hv_variableName.TupleConcat(HTuple(hv_HigherOrderCoefficients[0]));
				hv_variableValue = hv_variableValue.TupleConcat(HTuple(hv_HigherOrderCoefficients[1]));
			}
		}
		CloseFile(hv_FileHandle);

		
		InfoFramegrabber("GigEVision", "device", &hv_Information, &hv_ValueList);
		TupleRegexpSelect(hv_ValueList, "C5", &hv_Device);
		OpenFramegrabber("GigEVision", 0, 0, 0, 0, 0, 0, "progressive", 16, "default",
			-1, "false", "default", hv_Device, 0, -1, &hv_AcqHandle);
		hv_num = hv_variableValue.TupleLength();
		{
			HTuple end_val28 = hv_num - 1;
			HTuple step_val28 = 1;
			for (hv_i = 0; hv_i.Continue(end_val28, step_val28); hv_i += step_val28)
			{
				SetFramegrabberParam(hv_AcqHandle, HTuple(hv_variableName[hv_i]), HTuple(hv_variableValue[hv_i]));
			}
		}
		SetFramegrabberParam(hv_AcqHandle, "grab_timeout", 50000);
		//set_framegrabber_param (AcqHandle, 'EnableDC0', 0)
		//set_framegrabber_param (AcqHandle, 'EnableDC1', 0)
		//SetFramegrabberParam(hv_AcqHandle, "EnableDC2", 1);

		GetFramegrabberParam(hv_AcqHandle, "AcquisitionLineRate", &hv_RateValue);
		cameraStatus = true;
		cout << "相机初始化" << endl;
		sendInfo(QStringLiteral("相机打开"),0);
	}
	// catch (Exception) 
	catch (HalconCpp::HException &HDevExpDefaultException)
	{
		HDevExpDefaultException.ToHTuple(&hv_Exception);
		cameraStatus = false;
		cout << "相机打开失败" << endl;
		sendInfo(QStringLiteral("相机打开失败"),1);
	}

	
}

void CameraCapture::on_openButton_clicked()
{
	try
	{
		hv_CalibFile = "gear.cfg";

		OpenFile(hv_CalibFile, "input", &hv_FileHandle);
		hv_variableName = HTuple();
		hv_variableValue = HTuple();
		for (hv_Index = 0; hv_Index <= 2; hv_Index += 1)
		{
			FreadLine(hv_FileHandle, &hv_OutLine, &hv_IsEOF);
		}
		while (0 != 1)
		{
			FreadLine(hv_FileHandle, &hv_OutLine, &hv_IsEOF);
			TupleRegexpReplace(hv_OutLine, "\\t", " ", &hv_OutLine);
			TupleRegexpReplace(hv_OutLine, "\\n", "", &hv_OutLine);
			TupleSplit(hv_OutLine, " ", &hv_Substrings);
			TupleNumber(hv_Substrings, &hv_HigherOrderCoefficients);
			TupleLength(hv_Substrings, &hv_Length);
			if (0 != (hv_Length == 0))
			{
				break;
			}
			else
			{
				hv_variableName = hv_variableName.TupleConcat(HTuple(hv_HigherOrderCoefficients[0]));
				hv_variableValue = hv_variableValue.TupleConcat(HTuple(hv_HigherOrderCoefficients[1]));
			}
		}
		CloseFile(hv_FileHandle);


		InfoFramegrabber("GigEVision", "device", &hv_Information, &hv_ValueList);
		TupleRegexpSelect(hv_ValueList, "C5", &hv_Device);
		OpenFramegrabber("GigEVision", 0, 0, 0, 0, 0, 0, "progressive", 16, "default",
			-1, "false", "default", hv_Device, 0, -1, &hv_AcqHandle);
		hv_num = hv_variableValue.TupleLength();
		{
			HTuple end_val28 = hv_num - 1;
			HTuple step_val28 = 1;
			for (hv_i = 0; hv_i.Continue(end_val28, step_val28); hv_i += step_val28)
			{
				SetFramegrabberParam(hv_AcqHandle, HTuple(hv_variableName[hv_i]), HTuple(hv_variableValue[hv_i]));
			}
		}
		SetFramegrabberParam(hv_AcqHandle, "grab_timeout", 50000);
		//set_framegrabber_param (AcqHandle, 'EnableDC0', 0)
		//set_framegrabber_param (AcqHandle, 'EnableDC1', 0)
		//SetFramegrabberParam(hv_AcqHandle, "EnableDC2", 1);

		GetFramegrabberParam(hv_AcqHandle, "AcquisitionLineRate", &hv_RateValue);
		cameraStatus = true;
		cout << "相机初始化" << endl;
		sendInfo(QStringLiteral("相机打开"), 0);
	}
	// catch (Exception) 
	catch (HalconCpp::HException &HDevExpDefaultException)
	{
		HDevExpDefaultException.ToHTuple(&hv_Exception);
		cameraStatus = false;
		cout << "相机打开失败" << endl;
		sendInfo(QStringLiteral("相机打开失败"), 1);
	}
}

void CameraCapture::on_closeButton_clicked()
{
	if (cameraStatus == true){
		CloseFramegrabber(hv_AcqHandle);
		sendInfo(QStringLiteral("相机关闭"),0);
		cameraStatus = false;
	}
	else{
		sendInfo(QStringLiteral("相机已经关闭，请勿重复关闭"),1);
	}
}

void CameraCapture::sendPLC()
{
	QString str = "05 30 30 46 46 42 54 30 30 31 58 30 30 31 30 31";
	QByteArray Bstr = str.toLatin1();
	QByteArray Byte = QByteArray::fromHex(Bstr);
	emit toPLCInfo(Byte);
}
